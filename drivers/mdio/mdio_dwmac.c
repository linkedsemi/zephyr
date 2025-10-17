/*
 * Copyright (c) 2024 linkedmdio_mutexi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dwmac_mdio

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#include <zephyr/drivers/gpio.h>
#endif

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

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_dwmac, CONFIG_MDIO_LOG_LEVEL);

#define MAC_MDIO_ADDRESS 0x0200
#define MAC_MDIO_DATA    0x0204

#define DMA_MODE         0x1000
#define DMA_MODE_SWR     BIT(0)

/* MDC Clock Selection define*/
#define STMMAC_CSR_60_100M  0x0 /* MDC = clk_scr_i/42 */
#define STMMAC_CSR_100_150M 0x1 /* MDC = clk_scr_i/62 */
#define STMMAC_CSR_20_35M   0x2 /* MDC = clk_scr_i/16 */
#define STMMAC_CSR_35_60M   0x3 /* MDC = clk_scr_i/26 */
#define STMMAC_CSR_150_250M 0x4 /* MDC = clk_scr_i/102 */
#define STMMAC_CSR_250_300M 0x5 /* MDC = clk_scr_i/124 */
#define STMMAC_CSR_300_500M 0x6 /* MDC = clk_scr_i/204 */
#define STMMAC_CSR_500_800M 0x7 /* MDC = clk_scr_i/324 */

typedef union mdio_address {
    uint32_t value;
    struct {
        uint32_t
            GB : 1,             /*[0]*/
            C45E : 1,           /*[1]*/
            GOC_0 : 1,          /*[2]*/
            GOC_1 : 1,          /*[3]*/
            SKAP : 1,           /*[4]*/
            Reserved_7_5 : 3,   /*[5-7]*/
            CR : 4,             /*[8-11]*/
            NTC : 3,            /*[12-14]*/
            Reserved_15 : 1,    /*[15]*/
            RDA : 5,            /*[16-20]*/
            PA : 5,             /*[21-25]*/
            BTB : 1,            /*[26]*/
            PSE : 1,            /*[27]*/
            Reserved_31_28 : 4; /*[28-31]*/
    };
} mdio_address_t;

typedef union mdio_data {
    uint32_t value;
    struct {
        uint32_t
            GD : 16, /*[0-15]*/
            RA : 16; /*[16-31]*/
    };
} mdio_data_t;

struct mdio_dwmac_data {
    uint8_t divider;
    struct k_mutex mdio_mutex;
};

struct mdio_dwmac_config {
    mem_addr_t base;
    uint32_t clock_frequency;
    IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios), (const struct gpio_dt_spec reset_gpio;))
    IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_hold_ms), (uint32_t reset_hold_ms;))
    IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(circuits_set_ms), (uint32_t circuits_set_ms;))
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
    bool mac_enabled[1];
};

static bool check_busy(const struct device *dev)
{
    const struct mdio_dwmac_config *const dev_config = dev->config;
    mdio_address_t mdio_address;

    mdio_address.value = sys_read32(dev_config->base + MAC_MDIO_ADDRESS);

    /* Return the busy bit */
    return mdio_address.GB;
}

static int mdio_dwmac_transfer(const struct device *dev,
                               uint8_t prtad,
                               uint8_t devad,
                               uint16_t regad,
                               uint16_t *data,
                               enum mdio_opcode op)
{
    const struct mdio_dwmac_config *const dev_config = dev->config;
    struct mdio_dwmac_data *const dev_data = dev->data;
    bool is_c45 = ((op == MDIO_OP_C22_READ) || (op == MDIO_OP_C22_WRITE)) ? false : true;
    bool is_write = ((op == MDIO_OP_C22_READ) || (op == MDIO_OP_C45_READ)) ? false : true;
    int ret = 0;
    mdio_data_t mdio_data;
    mdio_address_t mdio_address = {
        .Reserved_31_28 = 0,
        .PSE = 0,
        .BTB = 0,
        .PA = prtad,
        .RDA = is_c45 ? devad : regad,
        .Reserved_15 = 0,
        .NTC = 0,
        .CR = dev_data->divider,
        .Reserved_7_5 = 0,
        .SKAP = 0,
        .GOC_1 = is_write ? 0 : 1,
        .GOC_0 = 1,
        .C45E = is_c45 ? 1 : 0,
        .GB = 1,
    };

    k_mutex_lock(&dev_data->mdio_mutex, K_FOREVER);

    if (is_write) {
        mdio_data.RA = is_c45 ? regad : 0;
        mdio_data.GD = *data;
        sys_write32(mdio_data.value, dev_config->base + MAC_MDIO_DATA);
    }

    sys_write32(mdio_address.value, dev_config->base + MAC_MDIO_ADDRESS);

    ret = -ETIMEDOUT;
    for (int i = CONFIG_MDIO_SNPS_DWMAC_RECHECK_COUNT; i > 0; i--) {
        if (!check_busy(dev)) {
            ret = 0;
            break;
        }
        k_busy_wait(CONFIG_MDIO_SNPS_DWMAC_RECHECK_TIME);
    }

    if (ret) {
        LOG_ERROR("MDIO transaction timed out");
        goto done;
    }

    if (!is_write) {
        mdio_data.value = sys_read32(dev_config->base + MAC_MDIO_DATA);
        *data = mdio_data.GD;
    }

done:
    k_mutex_unlock(&dev_data->mdio_mutex);

    if (ret) {
        return -EIO;
    }

    return ret;
}

static int mdio_dwmac_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
    return mdio_dwmac_transfer(dev, prtad, 0, regad, data, MDIO_OP_C22_READ);
}

static int mdio_dwmac_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
    return mdio_dwmac_transfer(dev, prtad, 0, regad, &data, MDIO_OP_C22_WRITE);
}

static int mdio_dwmac_read_c45(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t *data)
{
    return mdio_dwmac_transfer(dev, prtad, devad, regad, data, MDIO_OP_C45_READ);
}

static int mdio_dwmac_write_c45(const struct device *dev, uint8_t prtad, uint8_t devad, uint16_t regad, uint16_t data)
{
    return mdio_dwmac_transfer(dev, prtad, devad, regad, &data, MDIO_OP_C45_WRITE);
}

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
static int phy_reset(const struct device *dev)
{
    const struct mdio_dwmac_config *const dev_config = dev->config;
    int ret = 0;

    if (dev_config->reset_gpio.port) {
        /* Configure reset pin */
        if (dev_config->reset_gpio.port) {
            ret = gpio_pin_configure_dt(&dev_config->reset_gpio, GPIO_OUTPUT_ACTIVE);
            if (ret) {
                return ret;
            }
        }

        /* Start reset */
        ret = gpio_pin_set_dt(&dev_config->reset_gpio, 0);
        if (ret) {
            return ret;
        }

        if (dev_config->reset_hold_ms) {
            k_busy_wait(USEC_PER_MSEC * dev_config->reset_hold_ms);
        } else {
            /* Hold reset for the minimum time specified by datasheet */
            k_busy_wait(USEC_PER_MSEC * 10);
        }

        /* Reset over */
        ret = gpio_pin_set_dt(&dev_config->reset_gpio, 1);
        if (ret) {
            return ret;
        }

        if (dev_config->circuits_set_ms) {
            k_busy_wait(USEC_PER_MSEC * dev_config->circuits_set_ms);
        } else {
            /* Wait another 30 ms (circuits settling time) before accessing registers */
            k_busy_wait(USEC_PER_MSEC * 30);
        }
    }

    return ret;
}
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

static int mdio_dwmac_init(const struct device *dev)
{
    const struct mdio_dwmac_config *const dev_config = dev->config;
    struct mdio_dwmac_data *const dev_data = dev->data;
    k_timepoint_t timeout;
    __maybe_unused int ret;

    k_mutex_init(&dev_data->mdio_mutex);

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
    phy_reset(dev);
#endif /* DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios) */

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
            LOG_ERROR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERROR("toggle reset line failed");
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
        LOG_WRN("pinctrl_apply_state fail");
    }
#endif

    if (dev_config->mac_enabled[0]) {
        /* resets all of the MAC internal registers and logic */
        sys_write32(DMA_MODE_SWR, dev_config->base + DMA_MODE);
        timeout = sys_timepoint_calc(K_MSEC(1000));
        while (sys_read32(dev_config->base + DMA_MODE) & DMA_MODE_SWR) {
            if (sys_timepoint_expired(timeout)) {
                __ASSERT(0, "unable to reset hardware");
                return -EIO;
            }
        }
    }

    if (dev_config->clock_frequency < MHZ(35)) {
        dev_data->divider = STMMAC_CSR_20_35M;
    } else if ((dev_config->clock_frequency >= MHZ(35)) && (dev_config->clock_frequency < MHZ(60))) {
        dev_data->divider = STMMAC_CSR_35_60M;
    } else if ((dev_config->clock_frequency >= MHZ(60)) && (dev_config->clock_frequency < MHZ(100))) {
        dev_data->divider = STMMAC_CSR_60_100M;
    } else if ((dev_config->clock_frequency >= MHZ(100)) && (dev_config->clock_frequency < MHZ(150))) {
        dev_data->divider = STMMAC_CSR_100_150M;
    } else if ((dev_config->clock_frequency >= MHZ(150)) && (dev_config->clock_frequency < MHZ(250))) {
        dev_data->divider = STMMAC_CSR_150_250M;
    } else if ((dev_config->clock_frequency >= MHZ(250)) && (dev_config->clock_frequency < MHZ(300))) {
        dev_data->divider = STMMAC_CSR_250_300M;
    } else if ((dev_config->clock_frequency >= MHZ(300)) && (dev_config->clock_frequency < MHZ(500))) {
        dev_data->divider = STMMAC_CSR_300_500M;
    } else if ((dev_config->clock_frequency >= MHZ(500)) && (dev_config->clock_frequency <= MHZ(800))) {
        dev_data->divider = STMMAC_CSR_500_800M;
    } else {
        dev_data->divider = 0xf;
    }

    return 0;
}

static const struct mdio_driver_api mdio_dwmac_api = {
    .read = mdio_dwmac_read,
    .write = mdio_dwmac_write,
    .read_c45 = mdio_dwmac_read_c45,
    .write_c45 = mdio_dwmac_write_c45,
};

#define CHECK_MAC_CHILD_(child) DT_NODE_HAS_COMPAT(child, snps_designware_ethernet)

#define MDIO_DWMAC_DEVICE(inst)                                                                                                          \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                                                                          \
                                                                                                                                         \
    static struct mdio_dwmac_data mdio_dwmac_data_##inst = {};                                                                           \
    static struct mdio_dwmac_config mdio_dwmac_config_##inst = {                                                                         \
        .base = (uint32_t)DT_INST_REG_ADDR(inst),                                                                                        \
        .clock_frequency = COND_CODE_1(                                                                                                  \
            DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),                                                            \
            (DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),                                                                    \
            (DT_INST_PROP(inst, clock_frequency))),                                                                                      \
        .mac_enabled = {DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst, CHECK_MAC_CHILD_, (||))},                                            \
        IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios), (.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),))     \
        IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_hold_ms), (.reset_hold_ms = DT_INST_PROP_OR(inst, reset_hold_ms, {0}),))       \
        IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(circuits_set_ms), (.circuits_set_ms = DT_INST_PROP_OR(inst, circuits_set_ms, {0}),)) \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))                                                     \
        IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                                                            \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))                                       \
    };                                                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst,                                                                                                          \
                          &mdio_dwmac_init,                                                                                              \
                          NULL,                                                                                                          \
                          &mdio_dwmac_data_##inst,                                                                                       \
                          &mdio_dwmac_config_##inst,                                                                                     \
                          POST_KERNEL,                                                                                                   \
                          CONFIG_MDIO_INIT_PRIORITY,                                                                                     \
                          &mdio_dwmac_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_DWMAC_DEVICE)
