/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 LINKEDSEMI Technology Inc.
 */

#define DT_DRV_COMPAT linkedsemi_sha256

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "crypto_linkedsemi_sha256.h"
#include "field_manipulate.h"

#define IS_4BYTE_ALIGNED(ptr) ((((uintptr_t)(ptr)) & (sizeof(uint32_t) - 1)) == 0)

LOG_MODULE_REGISTER(sha256_linkedsemi, LOG_LEVEL_DBG);

static uint32_t u8tou32(uint8_t *in)
{
	return in[0] | in[1] << 8 | in[2] << 16 | in[3] << 24;
}

void sha256_linkedsemi_isr(const struct device *dev)
{
	const struct sha256_linkedsemi_config *cfg = dev->config;
    struct sha256_linkedsemi_data *dev_data = dev->data;
	reg_sha_t *reg = cfg->reg;

	if (reg->INTR_S & SHA_FSM_END_INTR_MASK) {
		reg->INTR_C = SHA_FSM_END_INTR_MASK;
		k_sem_give(&dev_data->fsm_end_sem);
	}
}

static void sha256_data_output(struct hash_pkt *pkt)
{
	const struct device *dev = pkt->ctx->device;
	const struct sha256_linkedsemi_config *cfg = dev->config;
	struct sha256_linkedsemi_data *dev_data = dev->data;
	reg_sha_t *reg = cfg->reg;

	uint8_t result_len = (dev_data->algo == CRYPTO_HASH_ALGO_SHA224) ? 7 : 8;
	for (uint8_t i = 0; i < result_len; ++i) {
		uint32_t val = reg->SHA_RSLT[i];
		val = BSWAP_32(val);
		memcpy(pkt->out_buf + i * sizeof(uint32_t), &val, sizeof(uint32_t));
	}

	dev_data->buf_idx = 0;
	dev_data->total_len = 0;
}

static void sha256_calc(struct hash_ctx *ctx, uint8_t *in, uint32_t block_number)
{
	const struct device *dev = ctx->device;
	const struct sha256_linkedsemi_config *cfg = dev->config;
	struct sha256_linkedsemi_data *dev_data = dev->data;
	reg_sha_t *reg = cfg->reg;

	k_mutex_lock(&dev_data->sha256_engine_mutex, K_FOREVER);

	reg->SHA_CTRL = FIELD_BUILD(SHA_SHA_LEN, block_number - 1) |
			FIELD_BUILD(SHA_FST_DAT, dev_data->total_len == 0) |
			FIELD_BUILD(SHA_CALC_SM3, dev_data->algo == CRYPTO_HASH_ALGO_SM3) |
			FIELD_BUILD(SHA_CALC_SHA224, dev_data->algo == CRYPTO_HASH_ALGO_SHA224);
	reg->INTR_C = SHA_FSM_END_INTR_MASK;
	reg->INTR_M = SHA_FSM_END_INTR_MASK;
	reg->SHA_START = SHA_FSM_START_MASK;

	for (uint32_t i = 0; i < block_number * SHA256_BLOCK_WORD_SIZE; i++) {
		reg->FIFO_DAT = u8tou32(in);
		in += sizeof(uint32_t);
	}

	k_sem_take(&dev_data->fsm_end_sem, K_FOREVER);

	k_mutex_unlock(&dev_data->sha256_engine_mutex);
	dev_data->total_len += block_number * SHA256_BLOCK_BYTE_SIZE;
}

static void sha256_buffer_write_byte(struct hash_ctx *ctx, uint8_t byte)
{
	const struct device *dev = ctx->device;
	struct sha256_linkedsemi_data *dev_data = dev->data;

	dev_data->buffer[dev_data->buf_idx++] = byte;
	if (dev_data->buf_idx == SHA256_BLOCK_BYTE_SIZE) {
		dev_data->buf_idx = 0;
		sha256_calc(ctx, dev_data->buffer, 1);
	}
}

static int sha256_linkedsemi_sha(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	const struct device *dev = ctx->device;
	struct sha256_linkedsemi_data *dev_data = dev->data;

	for (uint32_t i = 0; i < pkt->in_len; i++) {
		sha256_buffer_write_byte(ctx, pkt->in_buf[i]);
	}

	if (finish) {
		uint64_t bit_cnt = (dev_data->total_len + dev_data->buf_idx) * 8;

		sha256_buffer_write_byte(ctx, SHA256_PADDING_BYTE);

		while (dev_data->buf_idx != SHA256_FIANL_LENGTH) {
			sha256_buffer_write_byte(ctx, SHA256_PADDING_ZERO);
		}

		for (int i = 7; i >= 0; i--) {
			dev_data->buffer[dev_data->buf_idx++] = (uint8_t)(bit_cnt >> (8 * i));
		}
		sha256_calc(ctx, dev_data->buffer, 1);

		sha256_data_output(pkt);
	}

	return 0;
}

static int sha256_linkedsemi_init(const struct device *dev)
{
	const struct sha256_linkedsemi_config *const dev_config = dev->config;
	struct sha256_linkedsemi_data *dev_data = dev->data;
    __maybe_unused int ret;

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

	dev_config->irq_config_func(dev);

	k_mutex_init(&dev_data->sha256_engine_mutex);
	k_sem_init(&dev_data->fsm_end_sem, 0, K_SEM_MAX_LIMIT);
	return 0;
}

static int sha256_linkedsemi_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return SHA256_LINKEDSEMI_HASH_CAPS;
}

static int sha256_linkedsemi_cipher_begin_session(const struct device *dev, struct hash_ctx *ctx,
						  enum hash_algo algo)
{
	struct sha256_linkedsemi_data *dev_data = dev->data;

	if (ctx->flags & ~(SHA256_LINKEDSEMI_HASH_CAPS)) {
		LOG_ERR("Unsupported flag");
		return -ENOTSUP;
	}

	switch (algo) {
	case CRYPTO_HASH_ALGO_SM3:
	case CRYPTO_HASH_ALGO_SHA224:
	case CRYPTO_HASH_ALGO_SHA256:
		dev_data->algo = algo;
		break;
	case CRYPTO_HASH_ALGO_SHA384:
	case CRYPTO_HASH_ALGO_SHA512:
	default:
		LOG_ERR("Unsupported algo");
		return -ENOTSUP;
	}
	
	dev_data->buf_idx = 0;
	dev_data->total_len = 0;

	ctx->hash_hndlr = sha256_linkedsemi_sha;

	return 0;
}

static int sha256_linkedsemi_cipher_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	return 0;
}

static struct crypto_driver_api sha256_driver_api = {
	.query_hw_caps = sha256_linkedsemi_query_caps,
	.hash_begin_session = sha256_linkedsemi_cipher_begin_session,
	.hash_free_session = sha256_linkedsemi_cipher_free_session,
	.hash_async_callback_set = NULL,
	.cipher_begin_session = NULL,
	.cipher_free_session = NULL,
	.cipher_async_callback_set = NULL,
};

#define LS_SHA256_INIT(idx)                                                                        \
	static void sha256_linkedsemi_irq_config_func_##idx(const struct device *dev)              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), sha256_linkedsemi_isr,  \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}                                                                                          \
	static struct sha256_linkedsemi_data sha256_linkedsemi_data_##idx;                         \
	static const struct sha256_linkedsemi_config sha256_linkedsemi_config_##idx = {            \
		.reg = (void *)DT_INST_REG_ADDR(idx),                                              \
		.irq_config_func = sha256_linkedsemi_irq_config_func_##idx,                        \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), ))                       \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), ))  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, sha256_linkedsemi_init, NULL, &sha256_linkedsemi_data_##idx,    \
			      &sha256_linkedsemi_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&sha256_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_SHA256_INIT)