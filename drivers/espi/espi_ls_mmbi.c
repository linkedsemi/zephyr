#define DT_DRV_COMPAT linkedsemi_ls_mmbi

#include <zephyr/kernel.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <espi_lpc_common.h>

LOG_MODULE_REGISTER(mmbi, CONFIG_ESPI_LOG_LEVEL);

#ifndef PAGE_SIZE
    #define PAGE_SIZE 4096
#endif
#define PORT_NUM            1
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

struct mmbi_struct {
    struct mmbi_desc desc;
    struct host_ros ros;
    struct host_rws rws;
    uint32_t buf[];
};

struct mmbi_ls_data {
    struct mmbi_struct *mmbi;
    uint32_t size;
};

struct mmbi_ls_config {
    const struct device *parent;
    uint32_t host_addr;
    uint32_t size_order;
    struct peri_mem mem[PORT_NUM];
};

static bool host_mmbi_mem_read(struct peri_mem_content *mem, uint32_t addr, uint8_t size, void *res)
{
    struct device *dev = mem->ctx;
    const struct mmbi_ls_config *dev_cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;

    if (dev_cfg->host_addr > addr || dev_cfg->host_addr + dev_data->size <= addr) {
        return false;
    }
    switch (size) {
    case 1:
        *(uint8_t *)res = *((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr);
        break;
    case 2:
        *(uint16_t *)res = *(uint16_t *)((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr);
        break;
    case 4:
        *(uint32_t *)res = *(uint32_t *)((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr);
        break;
    default:
        __ASSERT_NO_MSG(0);
        break;
    }

    return true;
}

static bool host_mmbi_mem_write(struct peri_mem_content *mem, uint32_t addr, uint8_t size, uint8_t *data)
{
    struct device *dev = mem->ctx;
    const struct mmbi_ls_config *dev_cfg = dev->config;
    struct mmbi_ls_data *dev_data = dev->data;

    if (dev_cfg->host_addr > addr || dev_cfg->host_addr + dev_data->size <= addr) {
        return false;
    }
    switch (size) {
    case 1:
        *((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr) = data[0];
        break;
    case 2:
        *(uint16_t *)((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr) = data[1] << 8 | data[0];
        break;
    case 4:
        *(uint32_t *)((uint8_t *)dev_data->mmbi + addr - dev_cfg->host_addr) = data[3] << 24 | data[2] << 16 | data[1] << 8 | data[0];
        break;
    default:
        __ASSERT_NO_MSG(0);
        break;
    }
    return true;
}

static int mmbi_ls_init(const struct device *dev)
{
    const struct mmbi_ls_config *dev_cfg = dev->config;

    if (!device_is_ready(dev_cfg->parent)) {
        __ASSERT(0, "%s device not ready", dev_cfg->parent->name);
        return -ENODEV;
    }

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        espi_lpc_add_mem(dev_cfg->parent, (struct peri_mem *)(&dev_cfg->mem[i]));
    }

    return 0;
}

#define LS_MMBI_INIT(idx)                                                            \
    static uint8_t mmbi_buf##idx[PAGE_SIZE * (1 << DT_INST_PROP(idx, size_order))] = \
                                    {0x24, 0x4d, 0x4d, 0x42, 0x49, 0x24, 0x01, 0x00, \
                                     0x00, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, \
                                     0x08, 0x80, 0x00, 0x00, 0x00, 0x08, 0x00, 0x08, \
                                     0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                     0x40, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, \
                                     0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb9};\
    static struct mmbi_ls_data mmbi_ls_data_##idx = {                                \
        .mmbi = (struct mmbi_struct *)mmbi_buf##idx,                                 \
        .size = PAGE_SIZE * (1 << DT_INST_PROP(idx, size_order)),                    \
    };                                                                               \
    static const struct mmbi_ls_config mmbi_ls_cfg_##idx = {                         \
        .mem = {                                                                     \
            [0] = {                                                                  \
                .content = &(struct peri_mem_content){                               \
                    .mem_read = host_mmbi_mem_read,                                  \
                    .mem_write = host_mmbi_mem_write,                                \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                          \
                },                                                                   \
            },                                                                       \
        },                                                                           \
        .parent = DEVICE_DT_GET(DT_INST_PARENT(idx)),                                \
        .host_addr = DT_INST_PROP(idx, host_addr),                                   \
        .size_order = DT_INST_PROP(idx, size_order),                                 \
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
