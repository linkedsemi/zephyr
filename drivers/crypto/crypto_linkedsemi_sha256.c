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
#include <assert.h>

LOG_MODULE_REGISTER(sha256_linkedsemi, LOG_LEVEL_DBG);

static struct hash_ctx * ls_sha_ctx = NULL;

static void sha_variable_init(struct sha256_linkedsemi_data *dev_data)
{
    dev_data->total_length = 0;
    dev_data->current_block_bytes = 0;
    dev_data->current_word = 0;
}

static void sha_byte_update(struct sha256_linkedsemi_data *dev_data, reg_sha_t *reg,
                const uint8_t val)
{
    switch (dev_data->current_block_bytes % sizeof(uint32_t)) {
    case 0:
        MODIFY_REG(dev_data->current_word, 0xff, val);
        break;
    case 1:
        MODIFY_REG(dev_data->current_word, 0xff00, val << 8);
        break;
    case 2:
        MODIFY_REG(dev_data->current_word, 0xff0000, val << 16);
        break;
    case 3:
        MODIFY_REG(dev_data->current_word, 0xff000000, val << 24);
        break;
    }
    dev_data->current_block_bytes++;
    if (dev_data->current_block_bytes % sizeof(uint32_t) == 0) {
        reg->FIFO_DAT = dev_data->current_word;
    }
}

static void sha_block_start(struct sha256_linkedsemi_data *dev_data, reg_sha_t *reg, bool end)
{
    if (dev_data->current_block_bytes == SHA256_BLOCK_BYTE_SIZE) {
        dev_data->current_block_bytes = 0;
        while ((reg->INTR_R & SHA_FSM_END_INTR_MASK) == 0);
        reg->INTR_C = SHA_FSM_END_INTR_MASK;
        if (!end) {
            reg->SHA_START = 1;
        }
    }
}

static int sha256_linkedsemi_sha(struct hash_ctx *ctx, struct hash_pkt *pkt, bool final)
{
    const struct device *dev = ctx->device;
    struct sha256_linkedsemi_data *dev_data = dev->data;
    const struct sha256_linkedsemi_config *cfg = dev->config;
    reg_sha_t *reg = cfg->reg;
    reg->INTR_M = 0;
    if(!ctx->started) {
        k_mutex_lock(&dev_data->sha256_engine_mutex, K_FOREVER);
        ls_sha_ctx = ctx;
        ctx->started = true;
        if (!dev_data->first_update) {
            reg->SHA_START = 1;
            reg->SHA_CTRL &= ~SHA_FST_DAT_MASK;
            dev_data->first_update = true;
        }
    }
    assert(ls_sha_ctx == ctx);

    dev_data->total_length += (uint64_t)pkt->in_len * 8;
    const uint8_t *data = pkt->in_buf;
    uint32_t length = pkt->in_len;
    while (length) {
        do {
            sha_byte_update(dev_data, reg, *data);
            data++;
            length--;
        } while (dev_data->current_block_bytes != SHA256_BLOCK_BYTE_SIZE && length);
        sha_block_start(dev_data, reg, false);
    }
    
    if(final){
        sha_byte_update(dev_data, reg, 0x80);
        while (dev_data->current_block_bytes != SHA_PADDING_MOD) {
            sha_block_start(dev_data, reg, false);
            sha_byte_update(dev_data, reg, 0x00);
        }
        sha_byte_update(dev_data, reg, dev_data->total_length >> 56);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 48);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 40);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 32);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 24);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 16);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 8);
        sha_byte_update(dev_data, reg, dev_data->total_length >> 0);
        sha_block_start(dev_data, reg, true);

        uint8_t count = (dev_data->algo == CRYPTO_HASH_ALGO_SHA224) ? SHA224_WORDS_NUM : SHA256_SM3_WORDS_NUM;
        uint8_t *out = pkt->out_buf;

        for (uint8_t i = 0; i < count; ++i) {
            uint32_t val = reg->SHA_RSLT[i];
            *out++ = val >> 24;
            *out++ = val >> 16;
            *out++ = val >> 8;
            *out++ = val;
        }
        sha_variable_init(dev_data);
        ls_sha_ctx = NULL;
        dev_data->first_update = false;
        k_mutex_unlock(&dev_data->sha256_engine_mutex);
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

    k_mutex_init(&dev_data->sha256_engine_mutex);
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
    const struct sha256_linkedsemi_config *cfg = dev->config;
    reg_sha_t *reg = cfg->reg;

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

    reg->SHA_CTRL = FIELD_BUILD(SHA_FST_DAT, 1) |
        FIELD_BUILD(SHA_CALC_SHA224, dev_data->algo == CRYPTO_HASH_ALGO_SHA224) |
        FIELD_BUILD(SHA_CALC_SM3, dev_data->algo == CRYPTO_HASH_ALGO_SM3);
    dev_data->first_update = false;
    sha_variable_init(dev_data);

    ctx->started = false;
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
    static struct sha256_linkedsemi_data sha256_linkedsemi_data_##idx;                         \
    static const struct sha256_linkedsemi_config sha256_linkedsemi_config_##idx = {            \
        .reg = (void *)DT_INST_REG_ADDR(idx),                                              \
        IF_ENABLED(DT_HAS_CLOCKS(idx), (.ccfg = LS_DT_CLK_CFG_ITEM(idx), ))                       \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(idx, resets), (.reset = RESET_DT_SPEC_INST_GET(idx), ))  \
    };                                                                                         \
    DEVICE_DT_INST_DEFINE(idx, sha256_linkedsemi_init, NULL, &sha256_linkedsemi_data_##idx,    \
                  &sha256_linkedsemi_config_##idx, POST_KERNEL,                        \
                  CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&sha256_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_SHA256_INIT)