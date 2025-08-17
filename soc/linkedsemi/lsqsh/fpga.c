#if defined(CONFIG_MDIO)

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>

static int mdio_set_phy(void)
{
    const struct device *const mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio1));

    uint16_t val;
    int rc = 0;

    for (uint16_t addr = 0x0; addr < 0x2; addr++) {
        uint16_t reg = 0x0;
        printk("phy addr: %d  id: ", addr);
        rc = mdio_read(mdio_dev, addr, 0x2, &val);
        printk("%4.4x", val);
        rc = mdio_read(mdio_dev, addr, 0x3, &val);
        printk("%4.4x\n", val);

        printk("phy addr: %d  reg: %d :", addr, reg);
        rc = mdio_read(mdio_dev, addr, reg, &val);
        printk("%4.4x\n", val);

        /* 10Mbps */
        // sys_clear_bits((mem_addr_t)&val, BIT(6) | BIT(12) | BIT(13));
        // rc = mdio_write(mdio_dev, 0x0, reg, val);

        /* 10Mbps */
        // rc = mdio_write(mdio_dev, 0x0, reg, 0);

        /* 100Mbps */
        rc = mdio_write(mdio_dev, 0x0, reg, BIT(13));

        // // /* 1Gbps */
        // rc = mdio_write(mdio_dev, 0x0, reg, BIT(6) | BIT(13));

        printk("phy addr: %d  reg: %d :", addr, reg);
        rc = mdio_read(mdio_dev, addr, reg, &val);
        printk("%4.4x\n", val);

        reg = 0x1;
        printk("phy addr: %d  reg: %d :", addr, reg);
        rc = mdio_read(mdio_dev, addr, reg, &val);
        printk("%4.4x\n", val);
    }

    return rc;
}

SYS_INIT(mdio_set_phy, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif
