#define DT_DRV_COMPAT linkedsemi_ls_vuart

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <espi_lpc_common.h>
#include <core_rv32.h>

LOG_MODULE_REGISTER(vuart, CONFIG_ESPI_LOG_LEVEL);

#define PORT_NUM 8

#define LCR_DLAB 0x80 /* divisor latch access enable */

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct vuart_ls_data {
    bool is_lcr_avoid;
    struct peri_ioport ioport[PORT_NUM];
};

struct vuart_ls_config {
    const struct device *parent;
    uint16_t host_vuart_reg;
    uint16_t irq;
    mem_addr_t reg;
    const struct upstream_irq_type *up_irq;
    irq_cfg_func_t irq_config_func;
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

static void ls_vuart_isr(struct device *dev)
{
    const struct vuart_ls_config *dev_cfg = dev->config;

    irq_disable(dev_cfg->irq);
    if (dev_cfg->up_irq->type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH)) {
        espi_lpc_set_level_irq(dev_cfg->parent, dev_cfg->up_irq->idx, 1);
    } else {
        espi_lpc_raise_edge_irq(dev_cfg->parent, dev_cfg->up_irq->idx);
    }
}

static void irq_state_update(struct device *dev)
{
    const struct vuart_ls_config *dev_cfg = dev->config;
    bool masked;
    bool pending;

    masked = irq_is_enabled(dev_cfg->irq) > 0 ? true : false;
    if (!masked) {
        return;
    }

    irq_disable(dev_cfg->irq);
    pending = csi_vic_get_pending_irq(dev_cfg->irq) > 0 ? true : false;
    if (!pending) {
        if (dev_cfg->up_irq->type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH)) {
            espi_lpc_set_level_irq(dev_cfg->parent, dev_cfg->up_irq->idx, 0);
        }
        irq_enable(dev_cfg->irq);
    }
}

static void host_vuart_reg_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct vuart_ls_config *dev_cfg = dev->config;
    uint8_t offset = (ioport->addr - dev_cfg->host_vuart_reg) * 4;

    *((uint8_t *)res) = sys_read8(dev_cfg->reg + offset);
    irq_state_update(dev);
}

static void host_vuart_reg_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct device *dev = ioport->ctx;
    const struct vuart_ls_config *dev_cfg = dev->config;
    uint8_t offset = (ioport->addr - dev_cfg->host_vuart_reg) * 4;

    sys_write8(*data, dev_cfg->reg + offset);
    irq_state_update(dev);
}

static void host_vuart_reg_is_lcr_avoid_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct device *dev = ioport->ctx;
    const struct vuart_ls_config *dev_cfg = dev->config;
    const struct vuart_ls_data *dev_data = dev->data;
    uint8_t offset = (ioport->addr - dev_cfg->host_vuart_reg) * 4;

    if (!dev_data->is_lcr_avoid) {
        sys_write8(*data, dev_cfg->reg + offset);
    } else {
        /* 75M clock 115200 buadrate magic */
        if (0 == offset) {
            sys_write8(0x29, dev_cfg->reg + offset);
        } else if (0x4 == offset) {
            sys_write8(0x0, dev_cfg->reg + offset);
        }
    }
    irq_state_update(dev);
}

static void host_vuart_reg3_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *data)
{
    struct device *dev = ioport->ctx;
    const struct vuart_ls_config *dev_cfg = dev->config;
    struct vuart_ls_data *dev_data = dev->data;
    uint8_t offset = (ioport->addr - dev_cfg->host_vuart_reg) * 4;

    if (*data & LCR_DLAB) {
        dev_data->is_lcr_avoid = true;
    } else {
        dev_data->is_lcr_avoid = false;
    }
    sys_write8(*data, dev_cfg->reg + offset);
    irq_state_update(dev);
}

static int vuart_ls_init(const struct device *dev)
{
    const struct vuart_ls_config *dev_cfg = dev->config;
    const struct vuart_ls_data *dev_data = dev->data;
    int ret = 0;

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_cfg->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_cfg->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_cfg->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_cfg->reset.dev != NULL) {
        if (!device_is_ready(dev_cfg->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_cfg->reset.dev, dev_cfg->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_cfg->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_cfg->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_cfg->ccfg);
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_DBG("%s: Could not configure pins", dev->name);
    }
#endif

    dev_cfg->irq_config_func(dev);

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        espi_lpc_add_ioport(dev_cfg->parent, (struct peri_ioport *)(&dev_data->ioport[i]));
    }

    return 0;
}

#define LS_VUART_INIT(idx)                                                                                    \
    static void vuart_ls_irq_config_func_##idx(const struct device *dev)                                      \
    {                                                                                                         \
        IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), ls_vuart_isr, DEVICE_DT_INST_GET(idx), 0); \
        irq_enable(DT_INST_IRQN(idx));                                                                        \
    }                                                                                                         \
    IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(idx)));                                                 \
    IF_ENABLED(DT_HAS_UP_IRQ(idx), (UPSTREAM_IRQ_DT_INST_DEFINE(idx)))                                        \
    static struct vuart_ls_data vuart_ls_data_##idx = {                                                       \
        .ioport = {                                                                                           \
            [0] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_is_lcr_avoid_write,                                            \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port),                                                          \
                },                                                                                            \
            },                                                                                                \
            [1] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_is_lcr_avoid_write,                                            \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 1,                                                      \
                },                                                                                            \
            },                                                                                                \
            [2] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_write,                                                         \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 2,                                                      \
                },                                                                                            \
            },                                                                                                \
            [3] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg3_write,                                                        \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 3,                                                      \
                },                                                                                            \
            },                                                                                                \
            [4] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_write,                                                         \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 4,                                                      \
                },                                                                                            \
            },                                                                                                \
            [5] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_write,                                                         \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 5,                                                      \
                },                                                                                            \
            },                                                                                                \
            [6] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_write,                                                         \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 6,                                                      \
                },                                                                                            \
            },                                                                                                \
            [7] = {                                                                                           \
                .content = &(struct peri_ioport_content){                                                     \
                    .io_read = host_vuart_reg_read,                                                           \
                    .io_write = host_vuart_reg_write,                                                         \
                    .ctx = (void *)DEVICE_DT_INST_GET(idx),                                                   \
                    .addr = DT_INST_PROP(idx, port) + 7,                                                      \
                },                                                                                            \
            },                                                                                                \
        },                                                                                                    \
    };                                                                                                        \
    static const struct vuart_ls_config vuart_ls_cfg_##idx = {                                                \
        .irq = DT_INST_IRQN(idx),                                                                             \
        .irq_config_func = vuart_ls_irq_config_func_##idx,                                                    \
        .parent = DEVICE_DT_GET(DT_INST_PARENT(idx)),                                                         \
        .reg = DT_INST_REG_ADDR(idx),                                                                         \
        .host_vuart_reg = DT_INST_PROP(idx, port),                                                            \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx), ))                           \
        IF_ENABLED(DT_HAS_CLOCKS(idx), (.ccfg = LS_DT_CLK_CFG_ITEM(idx), ))                                   \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(idx, resets), (.reset = RESET_DT_SPEC_INST_GET(idx), ))              \
        IF_ENABLED(DT_HAS_UP_IRQ(idx), (.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(idx),))                     \
    };                                                                                                        \
    DEVICE_DT_INST_DEFINE(idx,                                                                                \
                          &vuart_ls_init,                                                                     \
                          NULL,                                                                               \
                          &vuart_ls_data_##idx,                                                               \
                          &vuart_ls_cfg_##idx,                                                                \
                          POST_KERNEL,                                                                        \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                                 \
                          0);

DT_INST_FOREACH_STATUS_OKAY(LS_VUART_INIT)
