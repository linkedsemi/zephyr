/*
 * Copyright (c) 2025 linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_rctl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/dt-bindings/reset/lsqsh_reset.h>
#include <reg_base_addr.h>

struct rctl_linkedsemi_config {
    mm_reg_t base;
};

typedef struct __packed {
    union {
        uint32_t value;
        struct {
            uint32_t
                reg : 22,    /*[0-21]*/
                set_bit : 5, /*[22-26]*/
                clr_bit : 5; /*[27-31]*/
        };
    };
} rstctrl_soc_rst_t;

static int reset_linkedsemi_status(const struct device *dev, uint32_t id, uint8_t *status)
{
    const struct rctl_linkedsemi_config *const dev_config = dev->config;
    rstctrl_soc_rst_t *rst = (rstctrl_soc_rst_t *)&id;

    *status = !sys_test_bit(dev_config->base + rst->reg, rst->set_bit);

    return 0;
}

static int reset_linkedsemi_line_assert(const struct device *dev, uint32_t id)
{
    const struct rctl_linkedsemi_config *const dev_config = dev->config;
    rstctrl_soc_rst_t *rst = (rstctrl_soc_rst_t *)&id;

    sys_write32(BIT(rst->clr_bit), dev_config->base + rst->reg);

    return 0;
}

static int reset_linkedsemi_line_deassert(const struct device *dev, uint32_t id)
{
    const struct rctl_linkedsemi_config *const dev_config = dev->config;
    rstctrl_soc_rst_t *rst = (rstctrl_soc_rst_t *)&id;

    sys_write32(BIT(rst->set_bit), dev_config->base + rst->reg);

    return 0;
}

static int reset_linkedsemi_line_toggle(const struct device *dev, uint32_t id)
{
    reset_linkedsemi_line_assert(dev, id);
    reset_linkedsemi_line_deassert(dev, id);

    return 0;
}

static const struct reset_driver_api reset_linkedsemi_driver_api = {
    .status = reset_linkedsemi_status,
    .line_assert = reset_linkedsemi_line_assert,
    .line_deassert = reset_linkedsemi_line_deassert,
    .line_toggle = reset_linkedsemi_line_toggle,
};

#define LINKEDSEMI_CCTL_INIT(index)                                                       \
    static const struct rctl_linkedsemi_config rctl_linkedsemi_config_##index = { \
        .base = DT_INST_REG_ADDR(index),                                          \
    };                                                                            \
                                                                                  \
    DEVICE_DT_INST_DEFINE(index,                                                  \
                          NULL,                                                   \
                          NULL,                                                   \
                          NULL,                                                   \
                          &rctl_linkedsemi_config_##index,                        \
                          PRE_KERNEL_1,                                           \
                          CONFIG_RESET_INIT_PRIORITY,                             \
                          &reset_linkedsemi_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LINKEDSEMI_CCTL_INIT)
