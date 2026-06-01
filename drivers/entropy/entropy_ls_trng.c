#define DT_DRV_COMPAT linkedsemi_ls_dwtrng

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <limits.h>
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
#define CMD_RENEW_STATE     0x4
#define CMD_REFRESH_ADDIN   0x5
#define CMD_GEN_RANDOM      0x6
#define CMD_ADVANCE_STATE   0x7
#define CMD_RUN_KAT         0x8
#define CMD_ZEROIZE         0xF
#define DWTRNG_STAT_BUSY_Msk   (1u << 31)
#define DWTRNG_ISTAT_DONE_Msk  (1u << 4)
#define DWTRNG_IE_GLBL_Msk     (1u << 31)
#define DWTRNG_IE_DONE_Msk     (1u << 4)

#define GEN_BITS_PER_CMD      128u
#define GEN_BYTES_PER_CMD     (GEN_BITS_PER_CMD / 8u)
#define MAX_BITS_PER_REQUEST  (1u << 19)

LOG_MODULE_REGISTER(trng_ls, LOG_LEVEL_DBG);

#ifndef FLEXIBLE_ARRAY_DECLARE
#define FLEXIBLE_ARRAY_DECLARE(type, name) type name[]
#endif

#ifndef CONFIG_ENTROPY_LS_POOL_SIZE
#define CONFIG_ENTROPY_LS_POOL_SIZE 128
#endif
#ifndef CONFIG_ENTROPY_LS_POOL_THRESHOLD
#define CONFIG_ENTROPY_LS_POOL_THRESHOLD 64
#endif

BUILD_ASSERT((CONFIG_ENTROPY_LS_POOL_SIZE & (CONFIG_ENTROPY_LS_POOL_SIZE - 1)) == 0,
	     "CONFIG_ENTROPY_LS_POOL_SIZE must be power of 2");
BUILD_ASSERT(CONFIG_ENTROPY_LS_POOL_THRESHOLD < CONFIG_ENTROPY_LS_POOL_SIZE,
	     "CONFIG_ENTROPY_LS_POOL_THRESHOLD must be less than pool size");

struct rng_pool {
	uint8_t first_alloc;
	uint8_t first_read;
	uint8_t last;
	uint8_t mask;
	uint8_t threshold;
	FLEXIBLE_ARRAY_DECLARE(uint8_t, buffer);
};
#define RNG_POOL_DEFINE(name, len) uint8_t name[sizeof(struct rng_pool) + (len)]

enum trng_state {
	TRNG_S_INIT_NOISE = 0,
	TRNG_S_INIT_CREATE,
	TRNG_S_READY,
};

enum trng_busy
{
	TRNG_S_NOWAIT = 0,
	TRNG_S_WAIT
};

struct trng_ls_data {
	uint32_t req_bits;
	RNG_POOL_DEFINE(pool_mem, CONFIG_ENTROPY_LS_POOL_SIZE);
	struct k_sem sem_data;
	struct k_sem sem_cmd;
	enum trng_state state;
};

struct trng_ls_config {
	reg_dwtrng_t *const regs;
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
	void (*irq_config_func)(const struct device *dev);
};
static inline uint8_t rng_pool_avail(struct rng_pool *rngp)
{
	unsigned int key = irq_lock();
	uint8_t first = rngp->first_alloc;
	uint8_t last  = rngp->last;
	uint8_t mask  = rngp->mask;
	uint8_t avail = (last - first) & mask;
	irq_unlock(key);
	return avail;
}

static bool no_data_cmd(const struct trng_ls_config *cfg,struct trng_ls_data *data, uint8_t cmd,enum trng_busy state)
{
	reg_dwtrng_t *regs = cfg->regs;
	if(state == TRNG_S_NOWAIT && (regs->STAT & DWTRNG_STAT_BUSY_Msk)){
		return false;
	}
	while (regs->STAT & DWTRNG_STAT_BUSY_Msk) {}
	regs->CTRL = cmd;
	return true;
}


static inline void trigger_gen_random(const struct trng_ls_config *cfg, struct trng_ls_data *data,enum trng_busy state)
{

	if(data->state != TRNG_S_READY){
		return ;
	}
	if (data->req_bits + GEN_BITS_PER_CMD > MAX_BITS_PER_REQUEST) {
  	 
		if(!no_data_cmd(cfg, data, CMD_ADVANCE_STATE,state))return ;
		data->req_bits = 0; 
	}
   no_data_cmd(cfg, data, CMD_GEN_RANDOM,state);

}

static int rng_pool_put(struct rng_pool *rngp, uint8_t byte)
{
	uint8_t first = rngp->first_read;
	uint8_t last  = rngp->last;
	uint8_t mask  = rngp->mask;

	if (((last - first) & mask) == mask) {
		return -ENOBUFS;
	}
	rngp->buffer[last] = byte;
	rngp->last = (last + 1) & mask;
	return 0;
}

static void rng_pool_init(struct rng_pool *rngp, uint16_t size, uint8_t threshold)
{
	rngp->first_alloc = 0U;
	rngp->first_read  = 0U;
	rngp->last        = 0U;
	rngp->mask        = size - 1;
	rngp->threshold   = threshold;
}

static uint16_t rng_pool_get(struct rng_pool *rngp, uint8_t *buf, uint16_t len, const struct trng_ls_config *cfg,struct trng_ls_data *data)
{
	uint32_t mask = rngp->mask;
	uint8_t *dst = buf;
	uint32_t first, available;
	uint32_t other_read_in_progress;
	unsigned int key;

	key = irq_lock();
	first = rngp->first_alloc;
	other_read_in_progress = (rngp->first_read ^ first);
	available = (rngp->last - first) & mask;
	if (available < len) {
		len = available;
	}
	rngp->first_alloc = (first + len) & mask;
	irq_unlock(key);

	while (len--) {
		*dst++ = rngp->buffer[first];
		first = (first + 1) & mask;
	}

	if (!other_read_in_progress) {
		key = irq_lock();
		rngp->first_read = rngp->first_alloc;
		irq_unlock(key);
	}

	uint16_t copied = (uint16_t)(dst - buf);

	available = (available >= copied) ? (available - copied) : 0U;
	if (available <= rngp->threshold) {
		trigger_gen_random(cfg, data,TRNG_S_NOWAIT);
	}
	return copied;
}



static inline void send_cmd_busywait(reg_dwtrng_t *regs, uint8_t cmd)
{
	while (regs->STAT & DWTRNG_STAT_BUSY_Msk) {}
	regs->CTRL = cmd;
	while ((regs->ISTAT & DWTRNG_ISTAT_DONE_Msk) == 0) {}
	regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
}


static void trng_ls_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;
	reg_dwtrng_t *regs = cfg->regs;

	if (regs->ISTAT & DWTRNG_ISTAT_DONE_Msk) {
		__ASSERT(!(regs->STAT & DWTRNG_STAT_BUSY_Msk), "TRNG Core BUSY");

		if (data->state == TRNG_S_INIT_NOISE) {
			regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
			data->state = TRNG_S_INIT_CREATE;
			regs->CTRL = CMD_CREATE_STATE;
			return;
		} else if (data->state == TRNG_S_INIT_CREATE) {
			regs->ISTAT = DWTRNG_ISTAT_DONE_Msk;
			data->state = TRNG_S_READY;
			regs->CTRL = CMD_GEN_RANDOM;
			k_sem_give(&data->sem_cmd);
			return;
		}

		regs->ISTAT = DWTRNG_ISTAT_DONE_Msk; 
		uint32_t r0 = regs->RAND[0];
		uint32_t r1 = regs->RAND[1];
		uint32_t r2 = regs->RAND[2];
		uint32_t r3 = regs->RAND[3];

		uint8_t blk[GEN_BYTES_PER_CMD];
		memcpy(&blk[0],  &r0, 4);
		memcpy(&blk[4],  &r1, 4);
		memcpy(&blk[8],  &r2, 4);
		memcpy(&blk[12], &r3, 4);

		for (int i = 0; i < GEN_BYTES_PER_CMD; i++) {
			(void)rng_pool_put((struct rng_pool *)data->pool_mem, blk[i]);
		}
		data->req_bits += GEN_BITS_PER_CMD;

		k_sem_give(&data->sem_data);

		struct rng_pool *pool = (struct rng_pool *)data->pool_mem;
		if (rng_pool_avail(pool) <= pool->threshold) {
			if (data->req_bits + GEN_BITS_PER_CMD <= MAX_BITS_PER_REQUEST) {
				if ((regs->STAT & DWTRNG_STAT_BUSY_Msk) == 0) {
					regs->CTRL = CMD_GEN_RANDOM; 
				}
			}
		}
	}
}

static int ls_trng_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	if (!buf || len == 0) {
		return -EINVAL;
	}

	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;
	struct rng_pool *pool = (struct rng_pool *)data->pool_mem;
	uint16_t copied = 0;

	
	if(data->state != TRNG_S_READY){
			k_sem_take(&data->sem_cmd, K_FOREVER);
	}


	while (copied < len) {
		uint16_t got = rng_pool_get(pool, &buf[copied], len - copied, cfg, data);
		if (got) {
			copied += got;
      continue;
		}

		trigger_gen_random(cfg, data,TRNG_S_WAIT);
		k_sem_take(&data->sem_data, K_FOREVER);
	}
	return 0;
}

static int ls_trng_get_entropy_isr(const struct device *dev, uint8_t *buf, uint16_t len, uint32_t flags)
{
	if (!buf || len == 0) {
		return -EINVAL;
	}

	const struct trng_ls_config *cfg = dev->config;
	struct trng_ls_data *data = dev->data;

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		return rng_pool_get((struct rng_pool *)data->pool_mem, buf, len, cfg, data);
	}

	if (data->state != TRNG_S_READY) {
		send_cmd_busywait(cfg->regs, CMD_GEN_NOISE);
		send_cmd_busywait(cfg->regs, CMD_CREATE_STATE);
		data->state = TRNG_S_READY;
	}

	uint16_t remaining = len;
	while (remaining) {
		if (data->req_bits + GEN_BITS_PER_CMD > MAX_BITS_PER_REQUEST) {
			send_cmd_busywait(cfg->regs, CMD_ADVANCE_STATE);
			data->req_bits = 0;
		}

		send_cmd_busywait(cfg->regs, CMD_GEN_RANDOM);

		uint32_t r0 = cfg->regs->RAND[0];
		uint32_t r1 = cfg->regs->RAND[1];
		uint32_t r2 = cfg->regs->RAND[2];
		uint32_t r3 = cfg->regs->RAND[3];

		uint8_t blk[GEN_BYTES_PER_CMD];
		memcpy(&blk[0],  &r0, 4);
		memcpy(&blk[4],  &r1, 4);
		memcpy(&blk[8],  &r2, 4);
		memcpy(&blk[12], &r3, 4);

		uint16_t take = MIN((uint16_t)GEN_BYTES_PER_CMD, remaining);
		memcpy(&buf[len - remaining], blk, take);
		remaining -= take;
		data->req_bits += GEN_BITS_PER_CMD;
	}
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

	rng_pool_init((struct rng_pool *)data->pool_mem, CONFIG_ENTROPY_LS_POOL_SIZE, CONFIG_ENTROPY_LS_POOL_THRESHOLD);
	data->req_bits = 0;

	k_sem_init(&data->sem_data, 0, UINT_MAX);
	k_sem_init(&data->sem_cmd, 0, UINT_MAX);

	cfg->regs->IE |= (DWTRNG_IE_GLBL_Msk  | DWTRNG_IE_DONE_Msk);
	if (cfg->irq_config_func) {
		cfg->irq_config_func(dev);
	}

	data->state = TRNG_S_INIT_NOISE;
	while (cfg->regs->STAT & DWTRNG_STAT_BUSY_Msk) {}
	cfg->regs->CTRL = CMD_GEN_NOISE;
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