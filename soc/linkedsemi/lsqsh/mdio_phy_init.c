#define LOG_MODULE_NAME mdio_phy_init
#define LOG_LEVEL       CONFIG_MDIO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>

#define BMCR            0x0
#define BMCR_RESET_MASK 0x8000

#define BMSR                    0x1
#define BMSR_LINK_STATUS_MASK   0x4
#define BMSR_LINK_STATUS_POS    2
#define BMSR_LINK_STATUS_LINKED 0x1

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

static int mdio_set_phy(void)
{
    const struct device *const mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio1));
    const struct device *const eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth1));
    uint16_t val = 0;
    uint16_t addr = 0x0;
    uint16_t reg = 0x0;
    int rc = 0;

    for (int i = 0; i < 32; i++) {
        if (mdio_read(mdio_dev, i, 0x0, &val) >= 0 && val != UINT16_MAX) {
            addr = i;
            break;
        }
    }

    reg = BMCR;
    rc = mdio_write(mdio_dev, addr, reg, BMCR_RESET_MASK);

    do {
        k_msleep(100);
        reg = BMCR;
        rc = mdio_read(mdio_dev, addr, reg, &val);
        LOG_DBG("phy addr: %d  reg: %x : %4.4x\n", addr, reg, val);

        reg = BMSR;
        rc = mdio_read(mdio_dev, addr, reg, &val);
        LOG_DBG("phy addr: %d  reg: %x : %4.4x\n", addr, reg, val);
    } while (((val & BMSR_LINK_STATUS_MASK) >> BMSR_LINK_STATUS_POS) != BMSR_LINK_STATUS_LINKED);

    reg = PHYID1;
    rc = mdio_read(mdio_dev, addr, reg, &val);
    LOG_DBG("phy addr: %d  reg: %x : %4.4x\n", addr, reg, val);
    reg = PHYID2;
    rc = mdio_read(mdio_dev, addr, reg, &val);
    LOG_DBG("phy addr: %d  reg: %x : %4.4x\n", addr, reg, val);

    /* force 10Mbps */
    // sys_clear_bits((mem_addr_t)&val, BIT(6) | BIT(12) | BIT(13));
    // rc = mdio_write(mdio_dev, addr, reg, val);

    /* force 10Mbps */
    // reg = BMCR;
    // rc = mdio_write(mdio_dev, addr, reg, 0);

    /* force 100Mbps */
    // reg = BMCR;
    // rc = mdio_write(mdio_dev, addr, reg, BIT(13));

    /* force 1Gbps */
    // reg = BMCR;
    // rc = mdio_write(mdio_dev, addr, reg, BIT(6) | BIT(13));

    reg = PHYSR;
    rc = mdio_read(mdio_dev, addr, reg, &val);
    LOG_DBG("phy addr: %d  reg: %x : %4.4x\n", addr, reg, val);

    val = (val & PHYSR_SPEED_STATUS_MASK) >> PHYSR_SPEED_STATUS_POS;
    if (PHYSR_SPEED_STATUS_1000M == val) {
        dwmac_1000M_2500M_speed_cofig(eth_dev);
        LOG_INF("1000Mbps link up\n");
    } else if (PHYSR_SPEED_STATUS_100M == val) {
        dwmac_10M_100M_speed_cofig(eth_dev);
        LOG_INF("100Mbps link up\n");
    } else if (PHYSR_SPEED_STATUS_10M == val) {
        dwmac_10M_100M_speed_cofig(eth_dev);
        LOG_INF("10Mbps link up\n");
    }

    return rc;
}

SYS_INIT(mdio_set_phy, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
