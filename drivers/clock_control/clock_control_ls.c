#define DT_DRV_COMPAT linkedsemi_ls_cctl

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <zephyr/dt-bindings/clock/ls101x_clock.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_ls, LOG_LEVEL_DBG);

#define LS_CLK_SET(base, n)   (*(volatile uint32_t *)((base) + (n)))

struct ls_cctl_config {
    uintptr_t base_sysc_per;
    uintptr_t base_sysc_cpu;
};

/* Clock controller local functions */
static inline int ls_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    const uint32_t base_sysc_per = ((const struct ls_cctl_config *)dev->config)->base_sysc_per;
    const uint32_t base_sysc_cpu = ((const struct ls_cctl_config *)dev->config)->base_sysc_cpu;
    switch(clk_cfg->offest){
        case LS_PD_CPU_CLKG:
            LS_CLK_SET(base_sysc_cpu, clk_cfg->offest) = BIT(clk_cfg->bit);
            break;
        case LS_PD_PER_CLKG0:
        case LS_PD_PER_CLKG1:
        case LS_PD_PER_CLKG2:
        case LS_PD_PER_CLKG3:
            LS_CLK_SET(base_sysc_per, clk_cfg->offest) = BIT(clk_cfg->bit);
            break;
        default:
            LOG_ERR("Error: Invalid clock offset value\n");
            return -EINVAL;
            break;
    }
	return 0;
}

static inline int ls_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	ARG_UNUSED(dev);
    struct ls_clk_cfg *clk_cfg = (struct ls_clk_cfg *)(sub_system);
    const uint32_t base_sysc_per = ((const struct ls_cctl_config *)dev->config)->base_sysc_per;
    const uint32_t base_sysc_cpu = ((const struct ls_cctl_config *)dev->config)->base_sysc_cpu;
    switch(clk_cfg->offest){
        case LS_PD_CPU_CLKG:
            LS_CLK_SET(base_sysc_cpu, clk_cfg->offest) = BIT(clk_cfg->bit + 1);
            break;
        case LS_PD_PER_CLKG0:
        case LS_PD_PER_CLKG1:
        case LS_PD_PER_CLKG2:
        case LS_PD_PER_CLKG3:
            LS_CLK_SET(base_sysc_per, clk_cfg->offest) = BIT(clk_cfg->bit + 1);
            break;
        default:
            LOG_ERR("Error: Invalid clock offset value\n");
            return -EINVAL;
            break;
    }
	return 0;
}

const struct ls_cctl_config cctl_config = {
    .base_sysc_per = DT_INST_REG_ADDR_BY_NAME(0, sysc_per),
    .base_sysc_cpu = DT_INST_REG_ADDR_BY_NAME(0, sysc_cpu),
};

/* Clock controller driver registration */
static const struct clock_control_driver_api ls_clock_control_api = {
	.on = ls_clock_control_on,
	.off = ls_clock_control_off,
};

DEVICE_DT_INST_DEFINE(0,
		    NULL,
		    NULL,
		    NULL, &cctl_config,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &ls_clock_control_api);