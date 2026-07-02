#define DT_DRV_COMPAT linkedsemi_ls_dwtrng

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/mutex.h>
#include <zephyr/irq.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#if defined(CONFIG_RESET)
#include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#endif
#include "reg_dwtrng_type.h"

#define CMD_GEN_NOISE       0x1
#define CMD_CREATE_STATE    0x3
#define CMD_GEN_RANDOM      0x6
#define CMD_ADVANCE_STATE   0x7

#define DWTRNG_STAT_BUSY_Msk   (1u << 31)
#define DWTRNG_ISTAT_DONE_Msk  (1u << 4)
#define DWTRNG_IE_GLBL_Msk     (1u << 31)
#define DWTRNG_IE_DONE_Msk     (1u << 4)

#define GEN_BITS_PER_CMD      128u
#define GEN_BYTES_PER_CMD     (GEN_BITS_PER_CMD / 8u)
#define MAX_BITS_PER_REQUEST  (1u << 19)

#define LS_TRNG_POOL_SIZE 128

struct trng_ls_data {
	struct k_pipe pool;
	uint8_t pool_buf[LS_TRNG_POOL_SIZE];
	uint32_t req_bits;
	bool skip_next;
	struct k_mutex mutex;
};

struct trng_ls_config {
	reg_dwtrng_t *const regs;
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
	void (*irq_config_func)(const struct device *dev);
};

static inline void trng_irq_enable(reg_dwtrng_t *regs)
{
	unsigned int key = irq_lock();

	regs->IE |= DWTRNG_IE_DONE_Msk;
	irq_unlock(key);
}

static inline void trng_irq_disable(reg_dwtrng_t *regs)
{
	unsigned int key = irq_lock();

	regs->IE &= ~DWTRNG_IE_DONE_Msk;
	irq_unlock(key);
}

static inline void send_cmd_busywait(reg_dwtrng_t *regs, uint8_t cmd)
{
	while (regs->STAT & DWTRNG_STAT_BUSY_Msk) {
	}

	regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
	regs->CTRL = cmd;

	while ((regs->ISTAT & DWTRNG_ISTAT_DONE_Msk) == 0) {
	}
	regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
}

static void trng_issue_next(reg_dwtrng_t *regs, struct trng_ls_data *data)
{
	if (data->req_bits + GEN_BITS_PER_CMD > MAX_BITS_PER_REQUEST) {
		regs->CTRL = CMD_ADVANCE_STATE;
		data->req_bits = 0;
		data->skip_next = true;
	} else {
		regs->CTRL = CMD_GEN_RANDOM;
	}
}

static void trng_ls_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;
	reg_dwtrng_t *regs = cfg->regs;

	if ((regs->ISTAT & DWTRNG_ISTAT_DONE_Msk) == 0) {
		return;
	}

	if (data->skip_next) {
		regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
		data->skip_next = false;
		if ((regs->STAT & DWTRNG_STAT_BUSY_Msk) == 0) {
			regs->CTRL = CMD_GEN_RANDOM;
		}
		return;
	}

	uint32_t r[4] = {regs->RAND[0], regs->RAND[1],
			 regs->RAND[2], regs->RAND[3]};
	regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;

	size_t written = 0;
	k_pipe_put(&data->pool, r, sizeof(r), &written, sizeof(r),
		   K_NO_WAIT);
	data->req_bits += GEN_BITS_PER_CMD;

	if (written == 0) {
		regs->IE &= ~DWTRNG_IE_DONE_Msk;
	}

	if ((regs->STAT & DWTRNG_STAT_BUSY_Msk) == 0) {
		trng_issue_next(regs, data);
	}
}

static int ls_trng_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	if (!buf || len == 0) {
		return -EINVAL;
	}

	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;
	uint16_t copied = 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	while (copied < len) {
		size_t got = 0;

		trng_irq_enable(cfg->regs);

		(void)k_pipe_get(&data->pool, &buf[copied], len - copied,
				 &got, 1, K_FOREVER);
		copied += got;
	}

	k_mutex_unlock(&data->mutex);
	
	return 0;
}

static int ls_trng_get_entropy_isr(const struct device *dev, uint8_t *buf, uint16_t len, uint32_t flags)
{
	if (!buf || len == 0) {
		return -EINVAL;
	}

	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;

	size_t got = 0;
	(void)k_pipe_get(&data->pool, buf, len, &got, 1, K_NO_WAIT);

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		trng_irq_enable(cfg->regs);
		return (int)got;
	}

	uint16_t i = (uint16_t)got;
	if (i == len) {
		return len;
	}

	trng_irq_disable(cfg->regs);
	cfg->regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
	data->skip_next = false;

	while (i < len) {
		if (data->req_bits + GEN_BITS_PER_CMD > MAX_BITS_PER_REQUEST) {
			send_cmd_busywait(cfg->regs, CMD_ADVANCE_STATE);
			data->req_bits = 0;
		}

		send_cmd_busywait(cfg->regs, CMD_GEN_RANDOM);

		uint32_t r[4] = {cfg->regs->RAND[0], cfg->regs->RAND[1],
				 cfg->regs->RAND[2], cfg->regs->RAND[3]};
		uint16_t take = MIN((uint16_t)sizeof(r), len - i);
		memcpy(&buf[i], r, take);
		i += take;
		data->req_bits += GEN_BITS_PER_CMD;
	}

	if ((cfg->regs->STAT & DWTRNG_STAT_BUSY_Msk) == 0) {
		trng_issue_next(cfg->regs, data);
	}

	trng_irq_enable(cfg->regs);

	return len;
}

static int ls_trng_init(const struct device *dev)
{
	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;

#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev) {
		const struct device *clk_dev = cfg->ccfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			return -ENODEV;
		}
		clock_control_off(clk_dev, (clock_control_subsys_t)&cfg->ccfg);
	}
#endif
#if defined(CONFIG_RESET)
	if (cfg->reset.dev != NULL) {
		if (!device_is_ready(cfg->reset.dev)) {
			return -ENODEV;
		}
		int ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
		if (ret) {
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

	k_pipe_init(&data->pool, data->pool_buf, sizeof(data->pool_buf));
	k_mutex_init(&data->mutex);
	data->req_bits = 0;
	data->skip_next = false;

	send_cmd_busywait(cfg->regs, CMD_GEN_NOISE);
	send_cmd_busywait(cfg->regs, CMD_CREATE_STATE);

	cfg->regs->IE |= (DWTRNG_IE_GLBL_Msk | DWTRNG_IE_DONE_Msk);
	if (cfg->irq_config_func) {
		cfg->irq_config_func(dev);
	}

	cfg->regs->CTRL = CMD_GEN_RANDOM;

	return 0;
}

static const struct entropy_driver_api ls_trng_api = {
	.get_entropy = ls_trng_get_entropy,
	.get_entropy_isr = ls_trng_get_entropy_isr,
};

#define LS_TRNG_INIT(inst)                                                                         \
	static struct trng_ls_data trng_ls_data##inst;                                             \
	static void trng_irq_config_##inst(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), trng_ls_isr,          \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
	static const struct trng_ls_config trng_ls_config_##inst = {                               \
		.regs = (reg_dwtrng_t *)DT_INST_REG_ADDR(inst),                                    \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, clocks), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                                                                         \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))   \
		.irq_config_func = trng_irq_config_##inst,                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ls_trng_init, NULL, &trng_ls_data##inst,                       \
			      &trng_ls_config_##inst, POST_KERNEL,                                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ls_trng_api);

DT_INST_FOREACH_STATUS_OKAY(LS_TRNG_INIT)
