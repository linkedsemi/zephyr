#define DT_DRV_COMPAT linkedsemi_ls_host_mmbi

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include "espi_lpc_common.h"

LOG_MODULE_REGISTER(mmbi, CONFIG_ESPI_LOG_LEVEL);

#define MMBI_DESC_SIGNATURE "#MMBI$"

/* DSP0282 1.1.0: up to 7 channels (CH0..CH6) per MMBI interface */
#define MMBI_MAX_CH 7

struct host_ros {
    uint32_t b2h_wp;
    uint32_t h2b_rp;
};

struct host_rws {
    uint32_t h2b_wp;
    uint32_t b2h_rp;
};

/* Per-channel definition, identical to MMBI_Desc bytes 8..39 (DSP0282 Table 1) */
struct mmbi_desc_ch {
    uint32_t b2h_ba;
    uint32_t h2b_ba;
    uint32_t b2h_l;
    uint32_t h2b_l;
    uint8_t buft;
    uint8_t h_int_v_ch;
    uint8_t bmc_int_v_ch;
    uint8_t reserved0[5];
    uint32_t h_ros_p;
    uint32_t h_rws_p;
} __packed;

struct mmbi_desc {
    uint8_t signature[6];
    uint8_t version;
    uint8_t noi_os_use;             /* [7] reserved, [6:4] NOI, [3:1] reserved, [0] OS use */
    struct mmbi_desc_ch ch0;        /* channel 0 definition (bytes 8..39) */
    uint8_t h_int_t;
    uint8_t h_int_l;
    uint8_t reserved1[3];
    uint8_t h_int_v;
    uint8_t bmc_int_t;
    uint32_t bmc_int_l;
    uint8_t reserved2[4];
    uint8_t bmc_int_v;
    uint8_t reserved3[8];
    struct mmbi_desc_ch ch[MMBI_MAX_CH - 1];   /* channel 1..6 definitions (bytes 64..255) */
} __packed;

/* MMBI_Desc layout per DSP0282 1.1.0 Table 1 */
BUILD_ASSERT(sizeof(struct mmbi_desc_ch) == 32);
BUILD_ASSERT(sizeof(struct mmbi_desc) == 256);
BUILD_ASSERT(offsetof(struct mmbi_desc, ch0) == 8);
BUILD_ASSERT(offsetof(struct mmbi_desc, h_int_t) == 40);
BUILD_ASSERT(offsetof(struct mmbi_desc, bmc_int_v) == 55);
BUILD_ASSERT(offsetof(struct mmbi_desc, ch) == 64);
BUILD_ASSERT(offsetof(struct mmbi_desc_ch, h_ros_p) == 24);
BUILD_ASSERT(offsetof(struct mmbi_desc_ch, h_rws_p) == 28);

struct mmbi_ls_data {
    struct mmbi_info info;
    struct peri_mem mem;
    uint8_t ch_num;
};

struct mmbi_ls_config {
    const struct device *espi_lpc;
    struct host_bmc_msg_exch hb_exch;
    struct peri_mem_content content;
    uint64_t host_map_addr;
};

static const struct mmbi_desc_ch *mmbi_desc_ch_get(const struct mmbi_desc *desc, uint8_t ch)
{
    return (ch == 0U) ? &desc->ch0 : &desc->ch[ch - 1U];
}

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
    const struct mmbi_desc *desc =
        (const struct mmbi_desc *)(uintptr_t)dev_data->info.base;
    struct mmbi_msg msg = {
        .type = MMBI_HOST_RWS_WR,
    };
    uint8_t *dst = host_mmbi_mem_ptr(mem, addr, size);
    uint32_t offset;
    uint8_t ch;

    if (dst == NULL || data == NULL) {
        return false;
    }

    memcpy(dst, data, size);

    offset = (uint32_t)(addr - dev_cfg->host_map_addr);
    for (ch = 0U; ch < dev_data->ch_num; ch++) {
        const struct mmbi_desc_ch *ch_desc = mmbi_desc_ch_get(desc, ch);
        uint32_t rws_offset = ch_desc->h_rws_p * 8U;
        uint32_t rws_end = rws_offset + sizeof(struct host_rws);

        if (offset < rws_end && offset + size > rws_offset) {
            msg.u.host_rws_wr.ch = ch;
            mmbi_msg_send(&dev_cfg->hb_exch, &msg);
            break;
        }
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
        const struct mmbi_desc *desc =
            (const struct mmbi_desc *)(uintptr_t)m_msg->u.info.base;
        uint8_t noi = (desc->noi_os_use >> 4U) & 0x7U;

        dev_data->info = m_msg->u.info;
        if (noi >= MMBI_MAX_CH) {
            LOG_WRN("Invalid MMBI NOI %u, using legacy mode", noi);
            noi = 0U;
        }
        dev_data->ch_num = noi + 1U;
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
