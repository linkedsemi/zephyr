#include "zephyr/arch/common/sys_bitops.h"
#define DT_DRV_COMPAT linkedsemi_le501x_cctl

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_le501x, LOG_LEVEL_DBG);

struct cctl_le501x_cfg {
    uint32_t reg;
};

static inline int le501x_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
    ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    const struct cctl_le501x_cfg *const config = dev->config;

    sys_set_bit(config->reg + clk_cfg->reset, clk_cfg->pos);
    sys_clear_bit(config->reg + clk_cfg->reset, clk_cfg->pos);
    sys_set_bit(config->reg + clk_cfg->bus, clk_cfg->pos);

    return 0;
}

static inline int le501x_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
    ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    const struct cctl_le501x_cfg *const config = dev->config;
    sys_clear_bit(config->reg + clk_cfg->bus, clk_cfg->pos);

    return 0;
}

/* Clock controller driver registration */
static const struct clock_control_driver_api ls_clock_control_api = {
    .on = le501x_clock_control_on,
    .off = le501x_clock_control_off,
};

#define LE501X_CCTL_INIT(index)            \
static const struct cctl_le501x_cfg cctl_le501x_cfg_##index = { \
    .reg = DT_INST_REG_ADDR(index),   \
}; \
                                \
DEVICE_DT_INST_DEFINE(index,          \
            NULL,                        \
            NULL,                        \
            NULL, &cctl_le501x_cfg_##index, \
            PRE_KERNEL_1,                      \
            CONFIG_CLOCK_CONTROL_INIT_PRIORITY, \
            &ls_clock_control_api);
DT_INST_FOREACH_STATUS_OKAY(LE501X_CCTL_INIT)