#define DT_DRV_COMPAT linkedsemi_ls_cctl

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <zephyr/logging/log.h>
#include "field_manipulate.h"

LOG_MODULE_REGISTER(clock_control_ls, LOG_LEVEL_DBG);

#define LS_CLK_SET(base, n)   (*(volatile uint32_t *)((base) + (n)))

#if(CONFIG_SOC_LS1010)
#include <zephyr/dt-bindings/clock/ls101x_clock.h>
#include "reg_sysc_awo.h"
#define CPU_FREQ DT_PROP(DT_NODELABEL(cpu0), clock_frequency)
#endif

#if(CONFIG_SOC_LSQSH)
#include <zephyr/dt-bindings/clock/lsqsh_clock.h>
#include "reg_sysc_sec_awo.h"
#define CPU_FREQ DT_PROP(DT_PATH(cpus, cpu_1), clock_frequency)
#endif

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

#if(CONFIG_SOC_LS1010)
static int ls_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sub_system, uint32_t *rate)
{
	ARG_UNUSED(dev);
	uint32_t *clock_source = (uint32_t *)(sub_system);
	switch (*clock_source) {
	case CLK_SRC_PBUS1:
		*rate = CPU_FREQ / (REG_FIELD_RD(SYSC_AWO->PD_AWO_CLK_CTRL, SYSC_AWO_CLK_PBUS1_DIV4) + 1);
		break;
	case CLK_SRC_PBUS2:
		*rate = CPU_FREQ / (REG_FIELD_RD(SYSC_AWO->PD_AWO_CLK_CTRL, SYSC_AWO_CLK_SEL_PBUS2) + 1);;
		break;
	case CLK_SRC_PBUS3:
		*rate = CPU_FREQ / (REG_FIELD_RD(SYSC_AWO->PD_AWO_CLK_CTRL, SYSC_AWO_CLK_SEL_PBUS3) + 1);
		break;
	case CLK_SRC_PBUS4:
		*rate = CPU_FREQ  / (REG_FIELD_RD(SYSC_AWO->PD_AWO_CLK_CTRL, SYSC_AWO_CLK_SEL_PBUS4) + 1);
		break;
	case CLK_SRC_HBUS:
		*rate = CPU_FREQ;
		break;
	default:
		*rate = 0U;
		return -EINVAL;
	}
	return 0;
}
#endif

#if(CONFIG_SOC_LSQSH)
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
static int ls_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sub_system, uint32_t *rate)
{
	ARG_UNUSED(dev);
	uint32_t *clock_source = (uint32_t *)(sub_system);
	switch (*clock_source) {
	case CLK_SRC_PBUS0:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1));
		break;
	case CLK_SRC_PBUS1:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1)) / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_SEL_PBUS1) + 1);
		break;
	case CLK_SRC_PBUS2:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1));
		break;
	case CLK_SRC_PBUS3:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1)) / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_SEL_PBUS3) + 1);
		break;
	case CLK_SRC_PBUS4:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1)) / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_SEL_PBUS4) + 1);
		break;
	case CLK_SRC_HBUS:
		*rate = (CPU_FREQ / (REG_FIELD_RD(SYSC_SEC_AWO->PD_AWO_CLK_CTRL1, SYSC_SEC_AWO_CLK_DIV_HBUS) + 1));
		break;
	default:
		*rate = 0U;
		return -EINVAL;
	}
	return 0;
}
#else
static int ls_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sub_system, uint32_t *rate)
{
	ARG_UNUSED(dev);
	uint32_t *clock_source = (uint32_t *)(sub_system);
	switch (*clock_source) {
	case CLK_SRC_PBUS0:
	case CLK_SRC_PBUS1:
	case CLK_SRC_PBUS2:
	case CLK_SRC_PBUS3:
	case CLK_SRC_HBUS:
		*rate = MHZ(300);
		break;
	case CLK_SRC_PBUS4:
		*rate = MHZ(75);
		break;
	default:
		*rate = 0U;
		return -EINVAL;
	}
	return 0;
}
#endif
#endif

/* Clock controller driver registration */
static const struct clock_control_driver_api ls_clock_control_api = {
	.on = ls_clock_control_on,
	.off = ls_clock_control_off,
	.get_rate = ls_clock_control_get_rate,
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