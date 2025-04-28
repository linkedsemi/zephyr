/*
 * Driver for Synopsys DesignWare MAC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define LOG_MODULE_NAME dwmac_plat
#define LOG_LEVEL       CONFIG_ETHERNET_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

/* be compatible with the HAL-based driver here */
#define DT_DRV_COMPAT linkedsemi_ethernet

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/net/ethernet.h>
#include "eth.h"
#include <zephyr/irq.h>

#include "eth_dwmac_priv.h"

#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif

#define M_10_1000M

struct eth_linkedsemi_config {
    void (*irq_config_func)(const struct device *dev);
    void (*irq_deconfig_func)(const struct device *dev);
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

int dwmac_bus_init(struct dwmac_priv *p)
{
    const struct device *const dev = p->dev;
    const struct eth_linkedsemi_config *dev_config = dev->config;
    __maybe_unused int ret;

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

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Could not configure pins");
    }
#endif

    return 0;
}

void dwmac_platform_init(struct dwmac_priv *p)
{
    const struct device *const dev = p->dev;
    const struct eth_linkedsemi_config *dev_config = dev->config;

#if defined(M_10_1000M)
    REG_WRITE(MAC_CONF,
              MAC_CONF_PS | MAC_CONF_DM);
#elif defined(M_100_2500M)
    REG_WRITE(MAC_CONF,
              MAC_CONF_PS | MAC_CONF_FES | MAC_CONF_DM);
#else
    #error not define speed
#endif

    REG_WRITE(DMA_SYSBUS_MODE, DMA_SYSBUS_MODE_AAL | DMA_SYSBUS_MODE_FB);

    /* set up IRQs (still masked for now) */
    dev_config->irq_config_func(dev);

    /* create MAC address */
    gen_random_mac(p->mac_addr, 0x00, 0x80, 0xE1);
}

void dwmac_platform_deinit(const struct device *const dev)
{
    /* basic configuration for this platform */
    const struct eth_linkedsemi_config *dev_config = dev->config;
    dev_config->irq_deconfig_func(dev);
}

BUILD_ASSERT(CONFIG_NOCACHE_MEMORY, "descriptors are placed in nocache section");
#define __desc_mem __nocache __aligned(4)

#define LINKEDSEMI_ETH_IRQ_HANDLER(index)                                          \
    static void eth_linkedsemi_irq_config_func_##index(const struct device *dev)   \
    {                                                                              \
        IRQ_CONNECT(DT_INST_IRQN(index),                                           \
                    DT_INST_IRQ(index, priority),                                  \
                    dwmac_isr,                                                     \
                    DEVICE_DT_INST_GET(index),                                     \
                    0);                                                            \
        irq_enable(DT_INST_IRQN(index));                                           \
    }                                                                              \
    static void eth_linkedsemi_irq_deconfig_func_##index(const struct device *dev) \
    {                                                                              \
        ARG_UNUSED(dev);                                                           \
        irq_disable(DT_INST_IRQN(index));                                          \
    }

#define LINKEDSEMI_ETH_INIT(index)                                                                   \
    static struct dwmac_dma_desc dwmac_tx_descs_##index[NB_TX_DESCS] __desc_mem;                     \
    static struct dwmac_dma_desc dwmac_rx_descs_##index[NB_RX_DESCS] __desc_mem;                     \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));                                     \
    LINKEDSEMI_ETH_IRQ_HANDLER(index)                                                                \
    static const struct eth_linkedsemi_config eth_linkedsemi_cfg_##index = {                         \
        .irq_config_func = eth_linkedsemi_irq_config_func_##index,                                   \
        .irq_deconfig_func = eth_linkedsemi_irq_deconfig_func_##index,                               \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), ))                \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), ))                      \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), )) \
    };                                                                                               \
    static struct dwmac_priv dwmac_instance_##index = {                                              \
        .base_addr = (uint32_t)DT_INST_REG_ADDR(index),                                              \
        .tx_descs = dwmac_tx_descs_##index,                                                          \
        .rx_descs = dwmac_rx_descs_##index,                                                          \
    };                                                                                               \
    ETH_NET_DEVICE_DT_INST_DEFINE(index,                                                             \
                                  dwmac_init,                                                        \
                                  NULL,                                                              \
                                  &dwmac_instance_##index,                                           \
                                  &eth_linkedsemi_cfg_##index,                                       \
                                  CONFIG_ETH_INIT_PRIORITY,                                          \
                                  &dwmac_api,                                                        \
                                  NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(LINKEDSEMI_ETH_INIT)
