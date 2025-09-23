#define DT_DRV_COMPAT linkedsemi_ls_pwm

#include <errno.h>
#include <zephyr/spinlock.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>
#include <zephyr/dt-bindings/clock/lsqsh_clock.h>
#include <zephyr/devicetree.h>
#include "reg_pwm_type.h"

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

#if defined(CONFIG_PWM_LS_V2)
#define CHAN_CONT 16
#else
#define CHAN_CONT 8
#endif
LOG_MODULE_REGISTER(pwm_ls, LOG_LEVEL_DBG);

struct pwm_data {
	struct k_spinlock pwm_spinlock;
};

struct ls_pwm_config {
	reg_pwm_t *const reg;
	uint8_t prescaler;
	uint32_t clock_frequency;
	IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
};

static int ls_pwm_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			     uint32_t pulse_cycles, pwm_flags_t flags)
{
	struct pwm_data *const data = dev->data;
	const struct ls_pwm_config *config = dev->config;
	if (channel >= CHAN_CONT) {
		return -EINVAL;
	}
	if (pulse_cycles > period_cycles) {
		return -EINVAL;
	}
	if ((pulse_cycles == 0) || (period_cycles == 0)) {
		return -EINVAL;
	}
	if ((period_cycles > 0xFFFFu) || (pulse_cycles > 0xFFFFu)) {
		return -EINVAL;
	}
	k_spinlock_key_t key = k_spin_lock(&data->pwm_spinlock);
	if (flags & PWM_POLARITY_INVERTED == 0) {
		config->reg->PWM_CTRL[channel] &= ~PWM_POL_MASK;
	} else {
		config->reg->PWM_CTRL[channel] |= PWM_POL_MASK;
	}
	config->reg->PWM_PARAM[channel].PWM_CYC = period_cycles;
	config->reg->PWM_PARAM[channel].PWM_HIGH = pulse_cycles;
	config->reg->PWM_EN |= (1 << channel);
	k_spin_unlock(&data->pwm_spinlock, key);
	return 0;
}

static int ls_pwm_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycle)
{
	const struct ls_pwm_config *config = dev->config;
	if (channel >= CHAN_CONT) {
		return -EINVAL;
	}
	*cycle = config->clock_frequency / (config->reg->PWM_PRE_DIV + 1);
	return 0;
}

static int ls_pwm_init(const struct device *dev)
{
	const struct ls_pwm_config *const cfg = dev->config;
	reg_pwm_t *pwm = (reg_pwm_t *)cfg->reg;
	int ret;
	if (!pwm) {
		return -EIO;
	}
#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev) {
		const struct device *clk_dev = cfg->ccfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_DBG("%s device not ready", clk_dev->name);
			return -ENODEV;
		}
		clock_control_off(clk_dev, (clock_control_subsys_t)&cfg->ccfg);
	}
#endif

#if defined(CONFIG_RESET)
	if (cfg->reset.dev != NULL) {
		if (!device_is_ready(cfg->reset.dev)) {
			LOG_ERR("%s: Reset controller device is not ready", dev->name);
			return -ENODEV;
		}

		ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
		if (ret != 0) {
			LOG_ERR("%s: toggle reset line failed", dev->name);
			return ret;
		}
	}
#endif

#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev) {
		const struct device *clk_dev = cfg->ccfg.cctl_dev;
		clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->ccfg);
	}
#endif

#if defined(CONFIG_PINCTRL)
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Failed to apply 'pin' state (%d)", ret);
		return ret;
	}
#endif
	for (int channel = 0; channel < CHAN_CONT; channel++) {
		pwm->PWM_CTRL[channel] = 0;
	}
	pwm->PWM_PRE_DIV = cfg->prescaler;
	pwm->INTR_CLR = 0xffffff;
	return 0;
}

static const struct pwm_driver_api ls_pwm_driver_api = {
	.set_cycles = ls_pwm_set_cycles,
	.get_cycles_per_sec = ls_pwm_get_cycles_per_sec,
};

#define LS_PWM_DEVICE_INIT(n)                                                                      \
	static struct pwm_data pwm_data_##n;                                                       \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(n);))                                                                                 \
	static const struct ls_pwm_config ls_pwm_config_##n = {                                    \
		.reg = (reg_pwm_t *)DT_INST_REG_ADDR(n),                                           \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), ))                                                                         \
		IF_ENABLED(DT_HAS_CLOCKS(n), (.ccfg = LS_DT_CLK_CFG_ITEM(n), ))                                                     \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n), )) };                  \
	DEVICE_DT_INST_DEFINE(n, ls_pwm_init, NULL, &pwm_data_##n, &ls_pwm_config_##n,             \
			      POST_KERNEL, CONFIG_PWM_INIT_PRIORITY, &ls_pwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LS_PWM_DEVICE_INIT)
