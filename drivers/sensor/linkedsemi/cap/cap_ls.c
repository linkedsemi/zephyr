#define DT_DRV_COMPAT linkedsemi_ls_cap

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/mutex.h>
#include <stdio.h>
#include <zephyr/dt-bindings/clock/lsqsh_clock.h>
#include <zephyr/dt-bindings/sensor/linkedsemi-cap.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/devicetree.h>
#include "reg_cap_type.h"
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

#define CHAN_CONT 8

LOG_MODULE_REGISTER(cap_ls, LOG_LEVEL_DBG);

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct cap_channel_data {
	uint16_t cap_low;
	uint16_t cap_high;
	uint8_t data_err;
};

struct cap_data {
	struct k_sem data_sem[CHAN_CONT];
	struct k_mutex ch_mutex[CHAN_CONT];
	struct k_spinlock cap_spinlock;
	struct cap_channel_data ch_data[CHAN_CONT];
};

struct cap_config {
	reg_cap_t *const regs;
	uint8_t prescaler;
	uint8_t channels[CHAN_CONT];
	irq_cfg_func_t irq_config_func;
	uint32_t hclk_hz;
	IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
};

uint32_t get_freq(uint32_t hclk_mhz, uint16_t cap_low, uint16_t cap_high, uint32_t prescaler)
{
	uint32_t divisor = cap_low + cap_high;
	if (divisor == 0) {
		LOG_ERR("Divisor is zero cap_low %d cap_high %d\n", cap_low, cap_high);
		return 0;
	}

	return ((hclk_mhz) / ((prescaler) + 1) / divisor);
}

static uint32_t get_apb_val(const struct cap_data *data, const struct cap_config *cfg,
			    uint8_t channel)
{
	uint32_t apb_val = 0;
	if (!data->ch_data[channel].data_err) {
		uint16_t high = data->ch_data[channel].cap_high;
		uint16_t low = data->ch_data[channel].cap_low;
		apb_val = get_freq(cfg->hclk_hz, low, high, cfg->prescaler);
	} else {
		LOG_DBG("Channel %u data is abnormal, high or low pulse error [error %u]\n",
			channel, data->ch_data[channel].data_err);
	}
	return apb_val;
}

static void cap_intr_mask_atomic(struct cap_data *data, reg_cap_t *cap, uint8_t channel,
				 bool enable)
{
	k_spinlock_key_t key = k_spin_lock(&data->cap_spinlock);

	if (enable) {
		cap->INTR_MSK |= BIT(channel) | BIT(channel + 8) | BIT(channel + 16);
	} else {
		cap->INTR_MSK &= ~(BIT(channel) | BIT(channel + 8) | BIT(channel + 16));
	}

	k_spin_unlock(&data->cap_spinlock, key);
}

void ls_cap_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct cap_config *const cfg = dev->config;
	struct cap_data *const data = dev->data;
	reg_cap_t *const cap = cfg->regs;
	uint32_t isrflags = cap->INTR_STT;

	for (uint8_t channel = 0; channel < CHAN_CONT; channel++) {

		if (isrflags & BIT(channel)) {
			cap->INTR_CLR = BIT(channel);
			cap_intr_mask_atomic(data, cap, channel, false);
			data->ch_data[channel].cap_high = cap->CAP_COUNT[channel * 2];
			data->ch_data[channel].cap_low = cap->CAP_COUNT[channel * 2 + 1];
			data->ch_data[channel].data_err = 0;
			k_sem_give(&data->data_sem[channel]);

		} else if (BIT(channel) & (isrflags >> 8)) {
			cap->INTR_CLR = BIT(channel + 8);
			cap_intr_mask_atomic(data, cap, channel, false);
			data->ch_data[channel].data_err = 1;
			k_sem_give(&data->data_sem[channel]);
		} else if (BIT(channel) & (isrflags >> 16)) {
			cap->INTR_CLR = BIT(channel + 16);
			cap_intr_mask_atomic(data, cap, channel, false);
			data->ch_data[channel].data_err = 2;
			k_sem_give(&data->data_sem[channel]);
		}
	}
}

static int cap_init(const struct device *dev)
{
	const struct cap_config *const cfg = dev->config;
	reg_cap_t *cap = (reg_cap_t *)cfg->regs;
	struct cap_data *data = dev->data;

	int ret;
	if (!cap) {
		return -EIO;
	}

	if (data) {
		for (uint8_t i = 0; i < CHAN_CONT; i++) {
			k_sem_init(&data->data_sem[i], 0, 1);
			k_mutex_init(&data->ch_mutex[i]);
			data->ch_data[i].cap_low = 0;
			data->ch_data[i].cap_high = 0;
			data->ch_data[i].data_err = 0;
		}
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
	cfg->irq_config_func(dev);

	cap->CAP_CNT_EN = 1;
	cap->CAP_PRE_DIV = cfg->prescaler;
	cap->INTR_CLR = 0xffffff;

	for (size_t i = 0; i < CHAN_CONT; i++) {
		cap->CAP_CTRL[i] = cfg->channels[i];
	}

	return 0;
}

static int cap_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct cap_data *const data = dev->data;
	const struct cap_config *const cfg = dev->config;
	reg_cap_t *const cap = cfg->regs;
	int ret;

	if (chan < SENSOR_CHAN_CAP_01 || chan > SENSOR_CHAN_CAP_08) {
		LOG_WRN("Can't accept this commond. chan=%d, CAP_00=%d, CAP_07=%d", chan,
			SENSOR_CHAN_CAP_01, SENSOR_CHAN_CAP_08);
		return -ENOTSUP;
	}

	uint8_t channel = chan - SENSOR_CHAN_CAP_01;

	if (cap->CAP_CTRL[channel] == 0) {
		LOG_WRN("Channel %d is not enabled in CAP_CTRL register", channel);
		return -ENODEV;
	}

	k_mutex_lock(&data->ch_mutex[channel], K_FOREVER);

	cap_intr_mask_atomic(data, cap, channel, true);

	ret = k_sem_take(&data->data_sem[channel], K_FOREVER);
	if (ret != 0) {
		LOG_DBG("CAP timeout on channel %d", channel);
		k_mutex_unlock(&data->ch_mutex[channel]);
		return -EAGAIN;
	}

	k_mutex_unlock(&data->ch_mutex[channel]);
	return 0;
}

static int cap_channel_get(const struct device *dev, enum sensor_channel chan,
			   struct sensor_value *val)
{
	struct cap_data *const data = dev->data;
	const struct cap_config *const cfg = dev->config;

	if (chan < SENSOR_CHAN_CAP_01 || chan > SENSOR_CHAN_CAP_08) {
		LOG_WRN("Can't accept this commond. chan=%d, CAP_01=%d, CAP_08=%d", chan,
			SENSOR_CHAN_CAP_01, SENSOR_CHAN_CAP_08);
		return -ENOTSUP;
	}

	uint8_t channel = chan - SENSOR_CHAN_CAP_01;
	val->val1 = get_apb_val(data, cfg, channel);
	val->val2 = 0;
	return 0;
}

static const struct sensor_driver_api cap_driver_api = {
	.sample_fetch = cap_sample_fetch,
	.channel_get = cap_channel_get,
};

#define CAP_DEFINE(inst)                                                                           \
	static struct cap_data cap_data_##inst;                                                    \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                              \
	static void cap_irq_config_##inst(const struct device *dev)                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), ls_cap_isr,           \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct cap_config cap_config_##inst = {                                       \
		.regs = (reg_cap_t *)DT_INST_REG_ADDR(inst),                                       \
		.prescaler = DT_INST_PROP(inst, prescaler),                                        \
		.channels = DT_INST_PROP(inst, channels),                                          \
		.hclk_hz = DT_INST_PROP(inst, clock_frequency),                                    \
		.irq_config_func = cap_irq_config_##inst,                                          \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), )) \
        IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), )) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), )) \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, cap_init, NULL, &cap_data_##inst, &cap_config_##inst,   \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &cap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAP_DEFINE)
