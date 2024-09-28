/*
 * Driver for Synopsys DesignWare MAC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define LOG_MODULE_NAME dwmac_plat
#define LOG_LEVEL CONFIG_ETHERNET_LOG_LEVEL
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
PINCTRL_DT_INST_DEFINE(0);
static const struct pinctrl_dev_config *eth0_pcfg =
    PINCTRL_DT_INST_DEV_CONFIG_GET(0);
#endif

int dwmac_bus_init(struct dwmac_priv *p)
{
#if defined(CONFIG_PINCTRL)
    int ret;
    ret = pinctrl_apply_state(eth0_pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Could not configure ethernet pins");
        return ret;
    }
#endif

    p->base_addr = DT_INST_REG_ADDR(0);

    return 0;
}

#define DESCS_NONCACHEABLE_MEM     DT_INST_PROP(0, descs)
/* Descriptor rings in uncached memory */
static struct dwmac_dma_desc *dwmac_tx_descs = (struct dwmac_dma_desc *)(DESCS_NONCACHEABLE_MEM);
static struct dwmac_dma_desc *dwmac_rx_descs = (struct dwmac_dma_desc *)(DESCS_NONCACHEABLE_MEM \
                                                 + (sizeof(struct dwmac_dma_desc) * NB_TX_DESCS));

void dwmac_platform_init(struct dwmac_priv *p)
{
    p->tx_descs = dwmac_tx_descs;
    p->rx_descs = dwmac_rx_descs;

    /* basic configuration for this platform */
    REG_WRITE(MAC_CONF,
          MAC_CONF_PS |
          MAC_CONF_FES |
          MAC_CONF_DM);
    REG_WRITE(DMA_SYSBUS_MODE,
          DMA_SYSBUS_MODE_AAL |
          DMA_SYSBUS_MODE_FB);

    /* set up IRQs (still masked for now) */
    IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), dwmac_isr,
            DEVICE_DT_INST_GET(0), 0);
    irq_enable(DT_INST_IRQN(0));

    /* create MAC address */
    gen_random_mac(p->mac_addr, 0x00, 0x80, 0xE1);
}

/* Our private device instance */
static struct dwmac_priv dwmac_instance;

ETH_NET_DEVICE_DT_INST_DEFINE(0,
                  dwmac_probe,
                  NULL,
                  &dwmac_instance,
                  NULL,
                  CONFIG_ETH_INIT_PRIORITY,
                  &dwmac_api,
                  NET_ETH_MTU);
