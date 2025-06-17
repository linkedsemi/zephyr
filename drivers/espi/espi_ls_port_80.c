#define DT_DRV_COMPAT linkedsemi_ls_port_80

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <espi_lpc_common.h>
#include <core_rv32.h>

LOG_MODULE_REGISTER(port80, CONFIG_ESPI_LOG_LEVEL);

#define PORT_NUM 1

struct port80_ls_data {
    mem_addr_t reg;
};

struct port80_ls_config {
    const struct device *parent;
    uint16_t hostport80_reg;
    struct peri_ioport ioport[PORT_NUM];
};

static void hostport80_reg_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    struct port80_ls_data *dev_data = dev->data;

    // printk("hostport80_reg_read\n");
    *((uint8_t *)res) = dev_data->reg;
}

static void hostport80_reg_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct device *dev = ioport->ctx;
    struct port80_ls_data *dev_data = dev->data;

    // printk("hostport80_reg_write\n");
    dev_data->reg = *((uint8_t *)data);
}

static int port80_ls_init(const struct device *dev)
{
    const struct port80_ls_config *dev_cfg = dev->config;

    if (!device_is_ready(dev_cfg->parent)) {
        __ASSERT(0, "%s device not ready", dev_cfg->parent->name);
        return -ENODEV;
    }

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        espi_lpc_add_ioport(dev_cfg->parent, (struct peri_ioport *)(&dev_cfg->ioport[i]));
    }

    return 0;
}

#define LSPORT80_INIT(idx)                                         \
    static struct port80_ls_data port80_ls_data_##idx;             \
    static const struct port80_ls_config port80_ls_cfg_##idx = {   \
        .ioport = {                                                \
            [0] = {                                                \
                .content = &(struct peri_ioport_content){          \
                    .io_read = hostport80_reg_read,                \
                    .io_write = hostport80_reg_write, \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),        \
                    .addr = DT_INST_PROP(idx, port),               \
                },                                                 \
            },                                                     \
        },                                                         \
        .parent = DEVICE_DT_GET(DT_INST_PARENT(idx)),              \
        .hostport80_reg = DT_INST_PROP(idx, port),                 \
    };                                                             \
    DEVICE_DT_INST_DEFINE(idx,                                     \
                          &port80_ls_init,                         \
                          NULL,                                    \
                          &port80_ls_data_##idx,                   \
                          &port80_ls_cfg_##idx,                    \
                          POST_KERNEL,                             \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,      \
                          0);

DT_INST_FOREACH_STATUS_OKAY(LSPORT80_INIT)
