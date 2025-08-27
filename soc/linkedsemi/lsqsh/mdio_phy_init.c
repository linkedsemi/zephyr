#define LOG_MODULE_NAME mdio_phy_init
#define LOG_LEVEL       CONFIG_MDIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>

#if defined(RTL8211F)
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
#endif

#define PORT_CURRENT             0x15

#define PORT_STATUS_REG          0x0

#define LINK_MASK                BIT(11)
#define DUPLEX_MASK              BIT(10)
#define SPEED_MASK               (BIT(9) | BIT(8))

#define LINK_UP_MASK             BIT(11)
#define FULL_DUPLEX_MASK         BIT(10)
#define SPEED_100M_200M_MASK     BIT(8)
#define SPEED_1000M_MASK         BIT(9)

extern void dwmac_10M_100M_speed_cofig(const struct device *const dev);
extern void dwmac_1000M_2500M_speed_cofig(const struct device *const dev);
extern void dwmac_tx_unlock(const struct device *dev);

static int mdio_set_phy(void)
{
    const struct device *const mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio1));
    const struct device *const eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth1));
    uint16_t val = 0;
    int rc = 0;

    while(1) {
        rc = mdio_read(mdio_dev, PORT_CURRENT, PORT_STATUS_REG, &val);
        if (val & LINK_UP_MASK) {
            break;
        } else {
            k_msleep(100);
        }
    }

    if (SPEED_1000M_MASK == (val & SPEED_1000M_MASK)) {
        dwmac_1000M_2500M_speed_cofig(eth_dev);
        LOG_INF("1000Mbps link up");
    } else if (SPEED_100M_200M_MASK == (val & SPEED_100M_200M_MASK)) {
        dwmac_10M_100M_speed_cofig(eth_dev);
        LOG_INF("100Mbps link up");
    } else {
        dwmac_10M_100M_speed_cofig(eth_dev);
        LOG_INF("10Mbps link up");
    }

    if (val & FULL_DUPLEX_MASK) {
        dwmac_1000M_2500M_speed_cofig(eth_dev);
        LOG_INF("full duplex");
    } else {
        dwmac_10M_100M_speed_cofig(eth_dev);
        LOG_INF("half duplex");
    }

    return 0;
}

SYS_INIT(mdio_set_phy, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
