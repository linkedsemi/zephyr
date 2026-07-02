#define DT_DRV_COMPAT linkedsemi_ls_watchdog

#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <errno.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/dt-bindings/clock/lsqsh_clock.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>
#include <stdbool.h>
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include "reg_iwdgv2_type.h"
#include "field_manipulate.h"
#include "ls_msp_iwdg.h"
LOG_MODULE_REGISTER(wdt_iwdg_ls, LOG_LEVEL_DBG);

#define WDT_CTRL_EN       BIT(0)
#define WDT_CTRL_RST_EN   BIT(1)
#define WDT_CTRL_INTR_CLR BIT(2)

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct iwdt_ls_config {
	reg_iwdg_t *iwdg_reg;
	irq_cfg_func_t irq_config_func;
	uint32_t hclk_hz;
	bool quick_enable;
	uint32_t quick_timeout_ms;
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct iwdt_ls_data {
	struct k_spinlock lock;
	wdt_callback_t callback;
};

static uint32_t iwdt_calculate_load(uint32_t timeout_ms, uint32_t frequency)
{
	uint64_t ticks = timeout_ms * frequency / 1000;

	return (ticks > UINT32_MAX) ? UINT32_MAX : (uint32_t)ticks;
}

static int iwdt_ls_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);
	struct iwdt_ls_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	const struct iwdt_ls_config *const config = dev->config;
	MODIFY_REG(config->iwdg_reg->IWDT_CTRL, IWDT_CNT_CLR_MASK | IWDT_INTR_CLR_MASK,
		   1 << IWDT_CNT_CLR_POS | 1 << IWDT_INTR_CLR_POS);
	MODIFY_REG(config->iwdg_reg->IWDT_CTRL, IWDT_CNT_CLR_MASK | IWDT_INTR_CLR_MASK, 0);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int iwdt_ls_setup(const struct device *dev, uint8_t options)
{
	struct iwdt_ls_data *data = dev->data;
	const struct iwdt_ls_config *const config = dev->config;

	if (!config->iwdg_reg->IWDT_LOAD) {
		LOG_INF("wdt_ls_setup: timeout not installed");
		return -ENOTSUP;
	}

	if (config->iwdg_reg->IWDT_CTRL) {
		LOG_INF("wdt_ls_setup: interrupt already configured");
		return -EBUSY;
	}

	if (options & WDT_OPT_PAUSE_IN_SLEEP) {
		LOG_INF("wdt_ls_setup: unsupported options\n");
		return -ENOTSUP;
	}

	if (options & WDT_OPT_PAUSE_HALTED_BY_DBG) {
		HAL_IWDG_MSP_DEBUG(config->iwdg_reg, 1);
	} else {
		HAL_IWDG_MSP_DEBUG(config->iwdg_reg, 0);
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);
	config->iwdg_reg->IWDT_CTRL = FIELD_BUILD(IWDT_RST_EN, 1) | FIELD_BUILD(IWDT_EN, 1);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int iwdt_ls_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct iwdt_ls_data *data = dev->data;
	const struct iwdt_ls_config *const config = dev->config;

	if (cfg->window.min != 0U || cfg->window.max == 0U) {
		LOG_INF("wdt_ls_install_timeout: invalid window");
		return -EINVAL;
	}

	if(!HAL_IWDG_IS_MATCH(config->iwdg_reg, cfg->flags))
	{
		LOG_INF("wdt_ls_install_timeout: flags error or not support  %d",cfg->flags);
		return -EINVAL;
	}


	if (cfg->callback != NULL) {
		config->irq_config_func(dev);
		data->callback = cfg->callback;
	} else {
		data->callback = NULL;
	}

	config->iwdg_reg->IWDT_CTRL = 0x0;

	uint32_t ticks = iwdt_calculate_load(cfg->window.max, config->hclk_hz);

	config->iwdg_reg->IWDT_LOAD = ticks;

	return 0;
}

static int iwdt_ls_disable(const struct device *dev)
{
	const struct iwdt_ls_config *const config = dev->config;
	config->iwdg_reg->IWDT_CTRL = 0x0;
	return 0;
}

static void iwdt_ls_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct iwdt_ls_data *data = dev->data;
	if (data->callback) {
		data->callback(dev, 0);
	}
}

static int iwdt_init(const struct device *dev)
{
	const struct iwdt_ls_config *const cfg = dev->config;
	struct iwdt_ls_data *data = dev->data;
	cfg->iwdg_reg->IWDT_CTRL &= ~WDT_CTRL_RST_EN;

#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev) {
		const struct device *clk_dev = cfg->ccfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_ERR("%s: %s device not ready", dev->name, clk_dev->name);
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

		int ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
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

	if (cfg->quick_enable) {
		if (cfg->quick_timeout_ms == 0U) {
			LOG_ERR("%s: quick-timeout-ms must be > 0", dev->name);
			return -EINVAL;
		}

		uint32_t ticks = iwdt_calculate_load(cfg->quick_timeout_ms, cfg->hclk_hz);
		cfg->iwdg_reg->IWDT_LOAD = ticks;
		k_spinlock_key_t key = k_spin_lock(&data->lock);
		cfg->iwdg_reg->IWDT_CTRL = FIELD_BUILD(IWDT_RST_EN, 1) | FIELD_BUILD(IWDT_EN, 1);
		k_spin_unlock(&data->lock, key);
		LOG_INF("%s: Quick start enabled with %d ms timeout", dev->name, cfg->quick_timeout_ms);
	}

	return 0;
}

static const struct wdt_driver_api iwdt_ls_api = {
	.setup = iwdt_ls_setup,
	.disable = iwdt_ls_disable,
	.install_timeout = iwdt_ls_install_timeout,
	.feed = iwdt_ls_feed,
};

#define IWDT_LS_DEFINE(inst)                                                                       \
	static struct iwdt_ls_data iwdt_ls_data##inst;                                             \
	static void iwdt_irq_config_##inst(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), iwdt_ls_isr,          \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct iwdt_ls_config iwdt_ls_config##inst = {                                \
		.iwdg_reg = (reg_iwdg_t *)DT_INST_REG_ADDR(inst),                                  \
		.irq_config_func = iwdt_irq_config_##inst,                                         \
		.hclk_hz = DT_INST_PROP(inst, clock_frequency),                                    \
		.quick_enable = DT_INST_PROP(inst, quick_enable),                                  \
		.quick_timeout_ms = DT_INST_PROP_OR(inst, quick_timeout_ms, 1000),                 \
		IF_ENABLED(CONFIG_CLOCK_CONTROL, (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))              \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), )) \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, iwdt_init, NULL, &iwdt_ls_data##inst, &iwdt_ls_config##inst,   \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &iwdt_ls_api);

DT_INST_FOREACH_STATUS_OKAY(IWDT_LS_DEFINE)