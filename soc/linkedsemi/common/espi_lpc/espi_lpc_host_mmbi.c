#define DT_DRV_COMPAT linkedsemi_ls_host_mmbi

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "espi_lpc_common.h"

LOG_MODULE_REGISTER(mmbi, CONFIG_ESPI_LOG_LEVEL);

#define MMBI_DESC_SIGNATURE "#MMBI$"

struct host_ros {
    uint32_t b2h_wp;
    uint32_t h2b_rp;
};

struct host_rws {
    uint32_t h2b_wp;
    uint32_t b2h_rp;
};

struct mmbi_desc {
    uint8_t signature[6];
    uint8_t version;
    uint8_t os_use;
    uint32_t b2h_ba;
    uint32_t h2b_ba;
    uint32_t b2h_l;
    uint32_t h2b_l;
    uint8_t buft;
    uint8_t reserved0[7];
    uint32_t h_ros_p;
    uint32_t h_rws_p;
    uint8_t h_int_t;
    uint8_t h_int_l;
    uint8_t reserved1[3];
    uint8_t h_int_v;
    uint8_t bmc_int_t;
    uint32_t bmc_int_l;
    uint8_t reserved2[4];
    uint8_t bmc_int_v;
    uint8_t reserved3[8];
} __packed;

struct mmbi_ls_data {
    struct mmbi_info info;
    struct peri_mem mem;
};

struct mmbi_ls_config {
    const struct device *espi_lpc;
    struct host_bmc_msg_exch hb_exch;
    struct peri_mem_content content;
    uint64_t host_map_addr;
};

static uint8_t *host_mmbi_mem_ptr(struct peri_mem_content *mem, uint64_t addr, uint16_t size)
{
    struct device *dev = mem->ctx;
    const struct mmbi_ls_config *dev_cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;
    uint64_t offset;

    if (size == 0 || dev_data->info.size == 0 || addr < dev_cfg->host_map_addr) {
        return NULL;
    }

    offset = addr - dev_cfg->host_map_addr;
    if (offset >= dev_data->info.size || size > dev_data->info.size - offset) {
        return NULL;
    }

    if (offset > UINTPTR_MAX - dev_data->info.base) {
        return NULL;
    }

    return (uint8_t *)((uintptr_t)dev_data->info.base + (uintptr_t)offset);
}

static bool host_mmbi_mem_read(struct peri_mem_content *mem, uint64_t addr, uint16_t size, void *buf)
{
    uint8_t *src = host_mmbi_mem_ptr(mem, addr, size);

    if (src == NULL || buf == NULL) {
        return false;
    }

    memcpy(buf, src, size);
    return true;
}

static bool host_mmbi_mem_write(struct peri_mem_content *mem, uint64_t addr, uint16_t size, uint8_t *data)
{
    struct device *dev = mem->ctx;
    const struct mmbi_ls_config *dev_cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;
    struct mmbi_msg msg = {
        .type = MMBI_HOST_RWS_WR,
    };
    uint8_t *dst = host_mmbi_mem_ptr(mem, addr, size);
    uint32_t offset;
    uint32_t rws_offset;
    uint32_t rws_end;

    if (dst == NULL || data == NULL) {
        return false;
    }

    memcpy(dst, data, size);

    offset = (uint32_t)(addr - dev_cfg->host_map_addr);
    rws_offset =
        ((const struct mmbi_desc *)(uintptr_t)dev_data->info.base)->h_rws_p * 8U;
    rws_end = rws_offset + sizeof(struct host_rws);
    if (offset < rws_end && offset + size > rws_offset) {
        mmbi_msg_send(&dev_cfg->hb_exch, &msg);
    }

    return true;
}

static int mmbi_ls_init(const struct device *dev)
{
    const struct mmbi_ls_config *cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;
    if(!device_is_ready(cfg->espi_lpc))
    {
        LOG_DBG("%s device not ready", cfg->espi_lpc->name);
        return -ENODEV;
    }
    dev_data->mem.content = (struct peri_mem_content *)&cfg->content;
    host_bmc_msg_exch_init(&cfg->hb_exch);

    return 0;
}

void host_mmbi_rx_callback(const struct device *dev,void *msg)
{
    const struct mmbi_ls_config *cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;
    struct mmbi_msg *m_msg = msg;
    if(m_msg->type == MMBI_INFO)
    {
        dev_data->info = m_msg->u.info;
        espi_lpc_add_mem(cfg->espi_lpc, &dev_data->mem);
    }else if(m_msg->type == MMBI_HOST_INT)
    {
        if(m_msg->u.host_int.vw_idx == 0x6 && m_msg->u.host_int.vw_data == 0x11)
        {
            espi_send_sci(cfg->espi_lpc);
        }else if(m_msg->u.host_int.vw_idx == 0 || m_msg->u.host_int.vw_idx == 1)
        {
            espi_lpc_raise_edge_irq(cfg->espi_lpc,m_msg->u.host_int.vw_data&0x7f);
        }else
        {
            __ASSERT_NO_MSG(0);
        }
    }else
    {
        __ASSERT_NO_MSG(0);
    }
}

#if !defined(CONFIG_ESPI_LPC_MBOX)
void __weak bmc_mmbi_rx_callback(const struct device *dev, void *msg)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(msg);
}
#endif

#define LS_MMBI_INIT(idx)                                                            \
    static struct mmbi_ls_data mmbi_ls_data_##idx;                                   \
    static struct mmbi_ls_config mmbi_ls_cfg_##idx = {                               \
        .espi_lpc = DEVICE_DT_GET(DT_INST_PHANDLE(idx,espi_lpc)),\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,host_mmbi_rx_callback,bmc_mmbi_rx_callback),\
        .content = {                               \
            .mem_read = host_mmbi_mem_read,                                  \
            .mem_write = host_mmbi_mem_write,                                \
            .ctx = (void *)DEVICE_DT_INST_GET(idx),                          \
        },                                                                   \
        .host_map_addr = (uint64_t)DT_INST_PROP(idx,host_map_addr_hi)<<32|DT_INST_PROP(idx,host_map_addr_lo),\
    };                                                                               \
    DEVICE_DT_INST_DEFINE(idx,                                                       \
                          &mmbi_ls_init,                                             \
                          NULL,                                                      \
                          &mmbi_ls_data_##idx,                                       \
                          &mmbi_ls_cfg_##idx,                                        \
                          POST_KERNEL,                                               \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                        \
                          0);

DT_INST_FOREACH_STATUS_OKAY(LS_MMBI_INIT)
