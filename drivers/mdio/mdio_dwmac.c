/*
 * Copyright (c) 2024 linkedmdio_mutexi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/drivers/mdio.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_dwmac, CONFIG_MDIO_LOG_LEVEL);

#define DT_DRV_COMPAT snps_dwmac_mdio

#define MAC_MDIO_ADDRESS 0x0200
#define MAC_MDIO_DATA    0x0204

union mdio_address {
    uint32_t reg;
    struct {
        uint32_t GB : 1,
            C45E : 1,
            GOC_0 : 1,
            GOC_1 : 1,
            SKAP : 1,
            Reserved_7_5 : 3,
            CR : 4,
            NTC : 3,
            Reserved_15 : 1,
            RDA : 5,
            PA : 5,
            BTB : 1,
            PSE : 1,
            Reserved_31_28 : 4;
    } st;
} __attribute__((packed));

union mdio_data {
    uint32_t reg;
    struct {
        uint32_t GD : 16,
            RA : 16;
    } st;
} __attribute__((packed));

struct mdio_dwmac_data {
    mem_addr_t base;
    uint8_t divider;
    struct k_mutex mdio_mutex;
};

struct mdio_dwmac_config {
    const struct pinctrl_dev_config *pincfg;
};

static bool check_busy(const struct device *dev)
{
    struct mdio_dwmac_data *const dev_data = dev->data;
    union mdio_address mdio_address_un;

    mdio_address_un.reg = sys_read32(dev_data->base + MAC_MDIO_ADDRESS);

    /* Return the busy bit */
    return mdio_address_un.st.GB;
}

static int mdio_dwmac_transfer(const struct device *dev,
                               uint8_t prtad,
                               uint8_t devad,
                               uint16_t regad,
                               uint16_t *data,
                               enum mdio_opcode op)
{
    struct mdio_dwmac_data *const dev_data = dev->data;
    bool is_c45 = ((op == MDIO_OP_C22_READ) || (op == MDIO_OP_C22_WRITE)) ? false : true;
    bool is_write = ((op == MDIO_OP_C22_READ) || (op == MDIO_OP_C45_READ)) ? false : true;
    int ret = 0;
    union mdio_data mdio_data_un;
    union mdio_address mdio_address_un = {
        .st = {
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
        },
    };

    k_mutex_lock(&dev_data->mdio_mutex, K_FOREVER);

    if (is_write) {
        mdio_data_un.st.RA = is_c45 ? regad : 0,
        mdio_data_un.st.GD = *data,
        sys_write32(mdio_data_un.reg, dev_data->base + MAC_MDIO_DATA);
    }

    sys_write32(mdio_address_un.reg, dev_data->base + MAC_MDIO_ADDRESS);

    ret = -ETIMEDOUT;
    for (int i = CONFIG_MDIO_SNPS_DWMAC_RECHECK_COUNT; i > 0; i--) {
        if (!check_busy(dev)) {
            ret = 0;
            break;
        }
        k_busy_wait(CONFIG_MDIO_SNPS_DWMAC_RECHECK_TIME);
    }

    if (ret) {
        LOG_ERR("MDIO transaction timed out");
        goto done;
    }

    if (!is_write) {
        mdio_data_un.reg = sys_read32(dev_data->base + MAC_MDIO_DATA);
        *data = mdio_data_un.st.GD;
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

static int mdio_dwmac_init(const struct device *dev)
{
    struct mdio_dwmac_data *const dev_data = dev->data;
    const struct mdio_dwmac_config *const config = dev->config;
    uint32_t per_clk_rate = DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency);

    k_mutex_init(&dev_data->mdio_mutex);

#if defined(CONFIG_PINCTRL)
    int ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }
#endif

    per_clk_rate /= 1000000;
    if (per_clk_rate >= 20 && per_clk_rate < 35) {
        dev_data->divider = 2;
    } else if (per_clk_rate < 60) {
        dev_data->divider = 3;
    } else if (per_clk_rate < 100) {
        dev_data->divider = 0;
    } else if (per_clk_rate < 150) {
        dev_data->divider = 1;
    } else if (per_clk_rate < 250) {
        dev_data->divider = 4;
    } else {
        LOG_ERR("ENET QOS clk rate does not allow MDIO");
        return -ENOTSUP;
    }

    return 0;
}

static const struct mdio_driver_api mdio_dwmac_api = {
    .read = mdio_dwmac_read,
    .write = mdio_dwmac_write,
    .read_c45 = mdio_dwmac_read_c45,
    .write_c45 = mdio_dwmac_write_c45,
};

#define MDIO_DWMAC_DEVICE(inst)                                                        \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                        \
                                                                                       \
    static struct mdio_dwmac_data mdio_dwmac_data_##inst = {                           \
        .base = DT_REG_ADDR(DT_INST_PARENT(inst)),                                     \
    };                                                                                 \
    static struct mdio_dwmac_config mdio_dwmac_config_##inst = {                       \
        IF_ENABLED(CONFIG_PINCTRL, (.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), )) \
    };                                                                                 \
    DEVICE_DT_INST_DEFINE(inst,                                                        \
                          &mdio_dwmac_init,                                            \
                          NULL,                                                        \
                          &mdio_dwmac_data_##inst,                                     \
                          &mdio_dwmac_config_##inst,                                   \
                          POST_KERNEL,                                                 \
                          CONFIG_ETH_INIT_PRIORITY,                                    \
                          &mdio_dwmac_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_DWMAC_DEVICE)
