#define DT_DRV_COMPAT linkedsemi_dma

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/dma.h>
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(dma_dw, CONFIG_DMA_LOG_LEVEL);

#include <soc.h>
#include <soc_dma.h>
#include "dma_dw_common.h"

/* Device constant configuration parameters */
struct dw_dma_cfg {
    struct dw_dma_dev_cfg dw_cfg;
    void (*irq_config)(void);
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

static int dw_dma_init(const struct device *dev)
{
    const struct dw_dma_cfg *const dev_config = dev->config;
    int ret;

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

    /* Disable all channels and Channel interrupts */
    ret = dw_dma_setup(dev);

    if (ret != 0) {
        LOG_ERR("failed to initialize DW DMA %s", dev->name);
        goto out;
    }

    /* Configure interrupts */
    dev_config->irq_config();

    LOG_INF("Device %s initialized", dev->name);

out:
    return ret;
}

int linkedsemi_dma_config(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
    const struct dw_dma_cfg *const dev_config = dev->config;

    if (cfg->channel_direction != MEMORY_TO_MEMORY) {
        soc_dma_channel_handshake_set(dev_config->dw_cfg.base, channel, cfg->dma_slot);
    }

    return dw_dma_config(dev, channel, cfg);
}

static const struct dma_driver_api dw_dma_driver_api = {
    .config = linkedsemi_dma_config,
    .start = dw_dma_start,
    .stop = dw_dma_stop,
    .suspend = dw_dma_suspend,
    .resume = dw_dma_resume,
    .get_status = dw_dma_get_status,
};

#define DW_DMAC_INIT(inst)                                                                            \
                                                                                                      \
    static struct dw_drv_plat_data dmac##inst = {                                                     \
        .chan[0] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[1] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[2] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[3] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[4] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[5] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[6] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
        .chan[7] = {                                                                                  \
            .class = 6,                                                                               \
            .weight = 0,                                                                              \
        },                                                                                            \
    };                                                                                                \
                                                                                                      \
    static void dw_dma##inst##_irq_config(void);                                                      \
                                                                                                      \
    static const struct dw_dma_cfg dw_dma##inst##_config = {                                          \
        .dw_cfg = {                                                                                   \
            .base = DT_INST_REG_ADDR(inst),                                                           \
        },                                                                                            \
        .irq_config = dw_dma##inst##_irq_config,                                                      \
        IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                         \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))    \
    };                                                                                                \
                                                                                                      \
    static struct dw_dma_dev_data dw_dma##inst##_data = {                                             \
        .channel_data = &dmac##inst,                                                                  \
    };                                                                                                \
                                                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                                                       \
                          &dw_dma_init,                                                               \
                          NULL,                                                                       \
                          &dw_dma##inst##_data,                                                       \
                          &dw_dma##inst##_config,                                                     \
                          POST_KERNEL,                                                                \
                          CONFIG_DMA_INIT_PRIORITY,                                                   \
                          &dw_dma_driver_api);                                                        \
                                                                                                      \
    static void dw_dma##inst##_irq_config(void)                                                       \
    {                                                                                                 \
        IRQ_CONNECT(DT_INST_IRQN(inst),                                                               \
                    DT_INST_IRQ(inst, priority),                                                      \
                    dw_dma_isr,                                                                       \
                    DEVICE_DT_INST_GET(inst),                                                         \
                    0);                                                                               \
        irq_enable(DT_INST_IRQN(inst));                                                               \
    }

DT_INST_FOREACH_STATUS_OKAY(DW_DMAC_INIT)
