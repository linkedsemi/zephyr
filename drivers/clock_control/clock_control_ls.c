#define DT_DRV_COMPAT linkedsemi_ls_cctl

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_ls, LOG_LEVEL_DBG);

#define LS_CLK_SET(base, n)   (*(volatile uint32_t *)((base) + (n)))

struct cctl_ls_cfg {
	uint32_t reg;
};

static inline int ls_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
	struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
	const struct cctl_ls_cfg *const config = dev->config;
	LS_CLK_SET(config->reg, clk_cfg->cctl_addr_offest) = BIT(clk_cfg->set_bit);
	return 0;
}

static inline int ls_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
	const struct cctl_ls_cfg *const config = dev->config;
	LS_CLK_SET(config->reg, clk_cfg->cctl_addr_offest) = BIT(clk_cfg->clr_bit);
	return 0;
}

/* Clock controller driver registration */
static const struct clock_control_driver_api ls_clock_control_api = {
	.on = ls_clock_control_on,
	.off = ls_clock_control_off,
};

#define LS_CCTL_INIT(index)				\
static const struct cctl_ls_cfg cctl_ls_cfg_##index = {	\
	.reg = DT_INST_REG_ADDR(index),   \
};	\
								\
DEVICE_DT_INST_DEFINE(index,          \
		    NULL,							\
		    NULL,							\
		    NULL, &cctl_ls_cfg_##index,		\
		    PRE_KERNEL_1,						\
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
		    &ls_clock_control_api);
DT_INST_FOREACH_STATUS_OKAY(LS_CCTL_INIT)