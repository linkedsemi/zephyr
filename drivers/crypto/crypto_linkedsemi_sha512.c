/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2024 LINKEDSEMI Technology Inc.
 */

#define DT_DRV_COMPAT linkedsemi_sha512

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "crypto_linkedsemi_sha512.h"
#include "field_manipulate.h"

#define IS_4BYTE_UNALIGNED(ptr) ((((uintptr_t)(ptr)) & (sizeof(uint32_t) - 1)) != 0)

LOG_MODULE_REGISTER(sha512_linkedsemi, LOG_LEVEL_DBG);


void sha512_linkedsemi_isr(const struct device *dev)
{
	const struct sha512_linkedsemi_config *cfg = dev->config;
	struct sha512_linkedsemi_data *dev_data = dev->data;
	reg_sha512_t *reg = cfg->reg;

	if (reg->INTR_STT & SHA512_INTR_CALC_END_MASK) {
		reg->INTR_MSK &= ~SHA512_INTR_CALC_END_MASK;
		k_sem_give(&dev_data->calc_end_sem);
	}
}

static void sha512_data_output(struct hash_pkt *pkt)
{
	const struct device *dev = pkt->ctx->device;
	const struct sha512_linkedsemi_config *cfg = dev->config;
	struct sha512_linkedsemi_data *dev_data = dev->data;
	reg_sha512_t *reg = cfg->reg;

	for (uint8_t i = 0; i < dev_data->result_word_num; i++) {
		uint32_t val = reg->DIGEST[15 - i];
		val = BSWAP_32(val);
		memcpy(pkt->out_buf + i * sizeof(uint32_t), &val, sizeof(uint32_t));
	}

	dev_data->buf_idx = 0;
	dev_data->total_len = 0;
}

static void sha512_calc(struct hash_ctx *ctx, uint32_t *in, uint32_t block_number)
{
	const struct device *dev = ctx->device;
	struct sha512_linkedsemi_data *dev_data = dev->data;
	const struct sha512_linkedsemi_config *cfg = dev->config;
	const uint32_t data_len = SHA512_BLOCK_BYTE_SIZE * block_number;
	reg_sha512_t *reg = cfg->reg;

	sys_cache_data_flush_range(in, data_len);

	reg->INTR_MSK = 0;
	reg->ADDR = (uint32_t)in;
	unsigned int key = irq_lock();
	reg->CTRL = FIELD_BUILD(SHA512_CTRL_START, 1) |
		    FIELD_BUILD(SHA512_CTRL_INIT_CALC, dev_data->total_len == 0) |
		    FIELD_BUILD(SHA512_CTRL_MODE, dev_data->algo) |
		    FIELD_BUILD(SHA512_CTRL_BLOCK_NUM, block_number - 1);
	reg->INTR_CLR = SHA512_INTR_CALC_END_MASK;
	while (reg->INTR_RAW & SHA512_INTR_CALC_END_MASK) ;
	irq_unlock(key);

	reg->INTR_MSK = SHA512_INTR_CALC_END_MASK;

	k_sem_take(&dev_data->calc_end_sem, K_FOREVER);

	dev_data->total_len += data_len;
}

static void sha512_buffer_write_byte(struct hash_ctx *ctx, uint8_t byte)
{
	const struct device *dev = ctx->device;
	struct sha512_linkedsemi_data *dev_data = dev->data;

	((uint8_t *)dev_data->buffer)[dev_data->buf_idx++] = byte;
	if (dev_data->buf_idx == SHA512_BLOCK_BYTE_SIZE) {
		dev_data->buf_idx = 0;
		sha512_calc(ctx, dev_data->buffer, 1);
	}
}

static int sha512_linkedsemi_sha(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	const struct device *dev = ctx->device;
	struct sha512_linkedsemi_data *dev_data = dev->data;
	uint8_t *pBuffer = (uint8_t *)dev_data->buffer;
	uint8_t *in_buf = pkt->in_buf;
	uint32_t in_len = pkt->in_len;

	if (IS_4BYTE_UNALIGNED(pkt->in_buf) || IS_4BYTE_UNALIGNED(pkt->in_len)) {
		return EINVAL;
	}

	k_mutex_lock(&dev_data->sha512_engine_mutex, K_FOREVER);

	if (dev_data->buf_idx) {
		while ((dev_data->buf_idx != 0) && (in_len != 0)) {
			sha512_buffer_write_byte(ctx, *in_buf++);
			in_len--;
		}
	}

	if (in_len / SHA512_BLOCK_BYTE_SIZE) {
		sha512_calc(ctx, (uint32_t *)in_buf, in_len / SHA512_BLOCK_BYTE_SIZE);
	}

	in_len = in_len % SHA512_BLOCK_BYTE_SIZE;
	while (in_len--) {
		sha512_buffer_write_byte(ctx, *in_buf++);
	}

	if (finish) {
		uint64_t bit_cnt = (dev_data->total_len + dev_data->buf_idx) * 8;

		sha512_buffer_write_byte(ctx, SHA512_PADDING_BYTE);

		while (dev_data->buf_idx != SHA512_FIANL_LENGTH) {
			sha512_buffer_write_byte(ctx, SHA512_PADDING_ZERO);
		}

		memset(&pBuffer[dev_data->buf_idx], SHA512_PADDING_ZERO, 8);
		dev_data->buf_idx += 8;

		for (int i = 7; i >= 0; i--) {
			pBuffer[dev_data->buf_idx++] = (uint8_t)(bit_cnt >> (8 * i));
		}
		sha512_calc(ctx, dev_data->buffer, 1);

		sha512_data_output(pkt);
	}

	k_mutex_unlock(&dev_data->sha512_engine_mutex);
	return 0;
}

static int sha512_linkedsemi_init(const struct device *dev)
{
	const struct sha512_linkedsemi_config *const cfg = dev->config;
	struct sha512_linkedsemi_data *dev_data = dev->data;

	if (cfg->cctl_cfg.cctl_dev) {
		const struct device *clk_dev = cfg->cctl_cfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_DBG("%s device not ready", clk_dev->name);
			return -ENODEV;
		}
		clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->cctl_cfg);
	}

	cfg->irq_config_func(dev);

	k_mutex_init(&dev_data->sha512_engine_mutex);
	k_sem_init(&dev_data->calc_end_sem, 0, K_SEM_MAX_LIMIT);

	return 0;
}

static int sha512_linkedsemi_query_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return SHA512_LINKEDSEMI_HASH_CAPS;
}

static int sha512_linkedsemi_cipher_begin_session(const struct device *dev, struct hash_ctx *ctx,
						  enum hash_algo algo)
{
	struct sha512_linkedsemi_data *dev_data = dev->data;

	if (ctx->flags & ~(SHA512_LINKEDSEMI_HASH_CAPS)) {
		LOG_ERR("Unsupported flag");
		return -ENOTSUP;
	}

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA384:
		dev_data->algo = SHA512_MODE_SHA384;
		dev_data->result_word_num = SHA384_RESULT_WORD_NUM;
		break;
	case CRYPTO_HASH_ALGO_SHA512:
		dev_data->algo = SHA512_MODE_SHA512;
		dev_data->result_word_num = SHA512_RESULT_WORD_NUM;
		break;
	default:
		LOG_ERR("Unsupported algo");
		return -ENOTSUP;
	}

	dev_data->buf_idx = 0;
	dev_data->total_len = 0;

	ctx->hash_hndlr = sha512_linkedsemi_sha;

	return 0;
}

static int sha512_linkedsemi_cipher_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ctx);

	return 0;
}

static struct crypto_driver_api sha512_driver_api = {
	.query_hw_caps = sha512_linkedsemi_query_caps,
	.hash_begin_session = sha512_linkedsemi_cipher_begin_session,
	.hash_free_session = sha512_linkedsemi_cipher_free_session,
	.hash_async_callback_set = NULL,
	.cipher_begin_session = NULL,
	.cipher_free_session = NULL,
	.cipher_async_callback_set = NULL,
};

#define LS_SHA512_INIT(idx)                                                                        \
	static void sha512_linkedsemi_irq_config_func_##idx(const struct device *dev)              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), sha512_linkedsemi_isr,  \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}                                                                                          \
	static struct sha512_linkedsemi_data sha512_linkedsemi_data_##idx;                         \
	static const struct sha512_linkedsemi_config sha512_linkedsemi_config_##idx = {            \
		.reg = (void *)DT_INST_REG_ADDR(idx),                                              \
		.irq_config_func = sha512_linkedsemi_irq_config_func_##idx,                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, sha512_linkedsemi_init, NULL, &sha512_linkedsemi_data_##idx,    \
			      &sha512_linkedsemi_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&sha512_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_SHA512_INIT)