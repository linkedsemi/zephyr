#define DT_DRV_COMPAT linkedsemi_ls_sio

#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <espi_lpc_common.h>

LOG_MODULE_REGISTER(sio, CONFIG_ESPI_LOG_LEVEL);

#define PORT_NUM 2

struct ls_ldn_dev {
    uint8_t base_addr_lsb;
    uint8_t base_addr_msb;
    uint8_t enable;
};

struct ls_sio {
    struct ls_espi_lpc_struct *parent;
    struct peri_ioport_content io_2e;
    struct peri_ioport_content io_2f;
    struct ls_ldn_dev ldn_2;
    struct ls_ldn_dev ldn_3;
    struct ls_ldn_dev ldn_4;
    struct ls_ldn_dev ldn_7;
    struct ls_ldn_dev ldn_b;
    struct ls_ldn_dev ldn_c;
    struct ls_ldn_dev ldn_d;
    struct ls_ldn_dev ldn_e;
    struct ls_ldn_dev ldn_f;
    struct ls_ldn_dev *ldn;
    uint8_t index;
    uint8_t ldn_num;
    uint8_t reg_2x[16];
};

struct sio_ls_data {
    bool is_lcr_avoid;
};

struct sio_ls_config {
    const struct device *parent;
    uint16_t host_sio_reg;
    mem_addr_t reg;
    struct peri_ioport ioport[PORT_NUM];
};

static void select_ldn(struct ls_sio *sio, uint8_t ldn_num)
{
    switch (ldn_num) {
    case 0x2:
        sio->ldn = &sio->ldn_2;
        break;
    case 0x3:
        sio->ldn = &sio->ldn_3;
        break;
    case 0x4:
        sio->ldn = &sio->ldn_4;
        break;
    case 0x7:
        sio->ldn = &sio->ldn_7;
        break;
    case 0xb:
        sio->ldn = &sio->ldn_b;
        break;
    case 0xc:
        sio->ldn = &sio->ldn_c;
        break;
    case 0xd:
        sio->ldn = &sio->ldn_d;
        break;
    case 0xe:
        sio->ldn = &sio->ldn_e;
        break;
    case 0xf:
        sio->ldn = &sio->ldn_f;
        break;
    }
}

static void io_2e_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct ls_sio *priv = ioport->ctx;
    uint8_t *val = res;

    *val = priv->index;
}

static void io_2e_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct ls_sio *priv = ioport->ctx;

    priv->index = *data;
}

static uint8_t ldn_dev_read(struct ls_ldn_dev *ldn, uint8_t index)
{
    uint8_t val = 0;

    switch (index) {
    case 0x30:
        val = ldn->enable;
        break;
    case 0x60:
        val = ldn->base_addr_msb;
        break;
    case 0x61:
        val = ldn->base_addr_lsb;
        break;
    }
    return val;
}

static void io_2f_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct ls_sio *sio = ioport->ctx;
    uint8_t *val = res;

    if (sio->index == 0x7) {
        *val = sio->ldn_num;
    } else if (sio->index == 0x20) {
        *val = 0x26;
    } else if (sio->index >= 0x20 && sio->index < 0x30) {
        *val = sio->reg_2x[sio->index & 0xf];
    } else {
        __ASSERT_NO_MSG(sio->ldn != NULL);
        *val = ldn_dev_read(sio->ldn, sio->index);
    }
}

static void ldn_dev_write(struct ls_ldn_dev *ldn, uint8_t index, uint8_t val)
{
    switch (index) {
    case 0x30:
        ldn->enable = val;
        break;
    case 0x60:
        ldn->base_addr_msb = val;
        break;
    case 0x61:
        ldn->base_addr_lsb = val;
        break;
    }
}

static void io_2f_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct ls_sio *sio = ioport->ctx;

    if (sio->index == 0x7) {
        sio->ldn_num = *data;
        select_ldn(sio, sio->ldn_num);
    } else if (sio->index >= 0x20 && sio->index < 0x30) {
        sio->reg_2x[sio->index & 0xf] = *data;
    } else {
        __ASSERT_NO_MSG(sio->ldn != NULL);
        ldn_dev_write(sio->ldn, sio->index, *data);
    }
}

static int sio_ls_init(const struct device *dev)
{
    const struct sio_ls_config *dev_cfg = dev->config;

    if (!device_is_ready(dev_cfg->parent)) {
        __ASSERT(0, "%s device not ready", dev_cfg->parent->name);
        return -ENODEV;
    }

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        espi_lpc_add_ioport(dev_cfg->parent, (struct peri_ioport *)(&dev_cfg->ioport[i]));
    }

    return 0;
}

#define LS_SIO_INIT(idx)                                      \
    static struct sio_ls_data sio_ls_data_##idx;              \
    static const struct sio_ls_config sio_ls_cfg_##idx = {    \
        .ioport = {                                           \
            [0] = {                                           \
                .content = &(struct peri_ioport_content){     \
                    .io_read = io_2e_read,                    \
                    .io_write = io_2e_write,                  \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),   \
                    .addr = DT_INST_PROP(idx, port),          \
                },                                            \
            },                                                \
            [1] = {                                           \
                .content = &(struct peri_ioport_content){     \
                    .io_read = io_2f_read,                    \
                    .io_write = io_2f_write,                  \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),   \
                    .addr = DT_INST_PROP(idx, port) + 1,      \
                },                                            \
            },                                                \
        },                                                    \
        .parent = DEVICE_DT_GET(DT_INST_PARENT(idx)),         \
        .host_sio_reg = DT_INST_PROP(idx, port),              \
    };                                                        \
    DEVICE_DT_INST_DEFINE(idx,                                \
                          &sio_ls_init,                       \
                          NULL,                               \
                          &sio_ls_data_##idx,                 \
                          &sio_ls_cfg_##idx,                  \
                          POST_KERNEL,                        \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
                          0);

DT_INST_FOREACH_STATUS_OKAY(LS_SIO_INIT)
