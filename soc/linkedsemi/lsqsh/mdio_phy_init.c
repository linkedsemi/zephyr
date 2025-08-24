#define LOG_MODULE_NAME mdio_phy_init
#define LOG_LEVEL       CONFIG_MDIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>

#define BMCR                       0x0
#define BMCR_RESET_MASK            0x8000
#define BMCR_FULL_DUPLEX_OPERATION BIT(8)

#define BMSR                    0x1
#define BMSR_LINK_STATUS_MASK   0x4
#define BMSR_LINK_STATUS_POS    2
#define BMSR_LINK_STATUS_LINKED 0x1
#define AUTO_NEGOTIATION_COMPLETE_MASK 0x20
#define AUTO_NEGOTIATION_COMPLETE_POS 5

#define PHYSR                    0x1a
#define PHYSR_SPEED_STATUS_MASK  0x30
#define PHYSR_SPEED_STATUS_POS   4
#define PHYSR_SPEED_STATUS_1000M 0x2
#define PHYSR_SPEED_STATUS_100M  0x1
#define PHYSR_SPEED_STATUS_10M   0x0

#define PHYID1                   0x2
#define PHYID2                   0x3

extern void dwmac_10M_100M_speed_cofig(const struct device *const dev);
extern void dwmac_1000M_2500M_speed_cofig(const struct device *const dev);
extern void dwmac_tx_unlock(const struct device *dev);

static int mdio_set_phy(void)
{
    const struct device *const eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth1));

    dwmac_1000M_2500M_speed_cofig(eth_dev);
    return 0;
}

SYS_INIT(mdio_set_phy, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
