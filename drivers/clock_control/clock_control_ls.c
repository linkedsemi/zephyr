#define DT_DRV_COMPAT linkedsemi_ls_cctl

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_ls, LOG_LEVEL_DBG);

/* Clock controller local functions */
static inline int ls_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    uint32_t *cctl_base_addr = (uint32_t *)clk_cfg->cctl_base_addr;
    *cctl_base_addr = BIT(clk_cfg->set_bit);
	return 0;
}

static inline int ls_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    uint32_t *cctl_base_addr = (uint32_t *)clk_cfg->cctl_base_addr;
    *cctl_base_addr = BIT(clk_cfg->clr_bit);
	return 0;
}

/* Clock controller driver registration */
static const struct clock_control_driver_api ls_clock_control_api = {
	.on = ls_clock_control_on,
	.off = ls_clock_control_off,
};

DEVICE_DT_INST_DEFINE(0,
		    NULL,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &ls_clock_control_api);