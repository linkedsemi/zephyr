#define LOG_MODULE_NAME board
#define LOG_LEVEL       CONFIG_KERNEL_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <field_manipulate.h>
#include <reg_sysc_app_cpu.h>

#if defined(CONFIG_ETH_DWMAC_LINKEDSEMI)
extern void dwmac_10M_100M_speed_cofig(const struct device *const dev);
extern void dwmac_1000M_2500M_speed_cofig(const struct device *const dev);
extern void dwmac_tx_unlock(const struct device *dev);
#endif

void board_late_init_hook(void)
{
#if defined(CONFIG_ETH_DWMAC_LINKEDSEMI)
    const struct device *const eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth1));
#if 1
    /* tx delay */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_RGMII_TX_DELAY_SEL, 0x2);
    /* rx delay */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_RGMII_RX_DELAY_SEL, 0x2);

    dwmac_1000M_2500M_speed_cofig(eth_dev);
#else
    /* tx delay */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_RGMII_TX_DELAY_SEL, 0x7);
    /* rx delay */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_RGMII_RX_DELAY_SEL, 0x7);

    dwmac_10M_100M_speed_cofig(eth_dev);
#endif
#endif
}
