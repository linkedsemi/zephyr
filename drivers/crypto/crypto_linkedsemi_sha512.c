/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2026 LINKEDSEMI Technology Inc.
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
#include <core_rv32.h>

#define SHA384_SHA512_WAIT_TIMEOUT_MS 100000

LOG_MODULE_REGISTER(sha512_linkedsemi, LOG_LEVEL_DBG);

static struct hash_ctx *ls_sha512_ctx = NULL;

static inline bool is_sram_address(uint32_t addr) {
    return ((addr >= 0x10000000 && addr < 0x10140000)
            || (addr >= 0x30000000 && addr < 0x30140000));
}

void sha512_linkedsemi_isr(const struct device *dev)
{
    const struct sha512_linkedsemi_config *cfg = dev->config;
    struct sha512_linkedsemi_data *dev_data = dev->data;
    reg_sha512_t *reg = cfg->reg;

    if (reg->INTR_STT & SHA512_INTR_CALC_END_MASK) {
        reg->INTR_CLR = SHA512_INTR_CALC_END_MASK;
        reg->INTR_MSK = 0;
        k_sem_give(&dev_data->calc_end_sem);
    }
}

static void sha512_calc(const struct device *dev, uint32_t addr, uint32_t block_number)
{
    const struct sha512_linkedsemi_config *cfg = dev->config;
    struct sha512_linkedsemi_data *dev_data = dev->data;
    reg_sha512_t *reg = cfg->reg;

    while ((reg->STATUS & 0x1) != 0x1);
    REG_FIELD_WR(reg->CTRL, SHA512_CTRL_BLOCK_NUM, (block_number - 1));
    reg->ADDR = addr & (~BIT(29));

    __ASSERT_NO_MSG(((uint32_t)addr % 4) == 0);
    csi_dcache_clean_range((void *)addr, block_number*SHA512_BLOCK_BYTE_SIZE);

    irq_disable(cfg->irqn);

    if (dev_data->first_update) {
        REG_FIELD_WR(reg->CTRL, SHA512_CTRL_INIT_CALC, 1);
        REG_FIELD_WR(reg->CTRL, SHA512_CTRL_START, 1);
        dev_data->first_update = false;
    } else {
        REG_FIELD_WR(reg->CTRL, SHA512_CTRL_INIT_CALC, 0);
        REG_FIELD_WR(reg->CTRL, SHA512_CTRL_START, 1);
    }

    reg->INTR_CLR = SHA512_INTR_DMA_END_MASK | SHA512_INTR_CALC_END_MASK;

    irq_enable(cfg->irqn);

    reg->INTR_MSK = SHA512_INTR_CALC_END_MASK;

    k_sem_take(&dev_data->calc_end_sem, K_MSEC(SHA384_SHA512_WAIT_TIMEOUT_MS));
}

static int sha512_linkedsemi_sha(struct hash_ctx *ctx, uint8_t *msg, uint32_t length, struct hash_pkt *pkt, bool final)
{
    const struct device *dev = ctx->device;
    struct sha512_linkedsemi_data *dev_data = dev->data;
    const struct sha512_linkedsemi_config *cfg = dev->config;
    reg_sha512_t *reg = cfg->reg;

    if (!ctx->started) {
        k_mutex_lock(&dev_data->sha512_engine_mutex, K_FOREVER);
        ls_sha512_ctx = ctx;
        ctx->started = true;
    }

    __ASSERT_NO_MSG(ls_sha512_ctx == ctx);
    __ASSERT_NO_MSG(is_sram_address((uint32_t)msg) && ((uint32_t)msg % 4) == 0);

    dev_data->total_length += length;

    if(length) {
        if(dev_data->buffer_idx) {
            if ((length + dev_data->buffer_idx) < SHA512_BLOCK_BYTE_SIZE) {
                memcpy(dev_data->buffer + dev_data->buffer_idx, msg, length);
                dev_data->buffer_idx += length;
                if(!final) {
                    return 0;
                }
                goto do_final;
            } else {
                uint32_t wr_len = SHA512_BLOCK_BYTE_SIZE - dev_data->buffer_idx;
                memcpy((uint8_t *)dev_data->buffer + dev_data->buffer_idx, msg, wr_len);
                sha512_calc(dev, (uint32_t)dev_data->buffer, 1);
                dev_data->buffer_idx = 0;
                length -= wr_len;
                msg += wr_len;
            }
        }
    }

    uint32_t block_number = length / SHA512_BLOCK_BYTE_SIZE;
    if (block_number) {
        if ((uint32_t)msg % 4) {
            for (uint32_t i = 0; i < block_number / MAX_BLOCK_SIZE; i++) {
                memcpy(dev_data->temp_sram_buffer, msg, sizeof(dev_data->temp_sram_buffer));
                sha512_calc(dev, (uint32_t)dev_data->temp_sram_buffer, MAX_BLOCK_SIZE);
                msg += sizeof(dev_data->temp_sram_buffer);
            }
            if (block_number % MAX_BLOCK_SIZE) {
                memcpy(dev_data->temp_sram_buffer, msg, (block_number % MAX_BLOCK_SIZE) * SHA512_BLOCK_BYTE_SIZE);
                sha512_calc(dev, (uint32_t)dev_data->temp_sram_buffer, block_number % MAX_BLOCK_SIZE);
                msg += (block_number % MAX_BLOCK_SIZE) * SHA512_BLOCK_BYTE_SIZE;
            }
        } else {
            sha512_calc(dev, (uint32_t)msg, block_number);
            msg += block_number * SHA512_BLOCK_BYTE_SIZE;
        }
    }

    if (length % SHA512_BLOCK_BYTE_SIZE) {
        memcpy((uint8_t *)dev_data->buffer, msg, length % SHA512_BLOCK_BYTE_SIZE);
        dev_data->buffer_idx = length % SHA512_BLOCK_BYTE_SIZE;
    }

do_final:
    if(final) {
        uint8_t *p_buffer = dev_data->buffer;
        uint64_t bit_cnt = dev_data->total_length * 8;
        p_buffer[dev_data->buffer_idx++] = SHA512_PADDING_BYTE;

        if (dev_data->buffer_idx == SHA512_BLOCK_BYTE_SIZE) {
            sha512_calc(dev, (uint32_t)dev_data->buffer, 1);
            dev_data->buffer_idx = 0;
        }

        while (dev_data->buffer_idx != (SHA512_BLOCK_BYTE_SIZE - SHA512_TOTAL_LEN_BYTE)) {
            p_buffer[dev_data->buffer_idx++] = SHA512_PADDING_ZERO;
            if (dev_data->buffer_idx == SHA512_BLOCK_BYTE_SIZE) {
                sha512_calc(dev, (uint32_t)dev_data->buffer, 1);
                dev_data->buffer_idx = 0;
            }
        }

        memset(&p_buffer[dev_data->buffer_idx], 0x0, 8);
        dev_data->buffer_idx += 8;

        for (uint8_t i = 0; i < 8; i++) {
            p_buffer[dev_data->buffer_idx + (7 - i)] = (uint8_t)(bit_cnt >> (8 * i));
        }
        dev_data->buffer_idx += 8;
        sha512_calc(dev, (uint32_t)dev_data->buffer, 1);

        uint8_t *out = pkt->out_buf;
        for (uint8_t j = 0; j < dev_data->result_word_num; j++) {
            uint32_t val = reg->DIGEST[15 - j];
            *out++ = val >> 24;
            *out++ = val >> 16;
            *out++ = val >> 8;
            *out++ = val;
        }

        dev_data->buffer_idx = 0;
        dev_data->total_length = 0;
        dev_data->first_update = false;
        ls_sha512_ctx = NULL;
        ctx->started = false;
        k_mutex_unlock(&dev_data->sha512_engine_mutex);
    }
    return 0;
}

static int sha512_linkedsemi_sha_align(struct hash_ctx *ctx, struct hash_pkt *pkt, bool final)
{
    const struct device *dev = ctx->device;
    struct sha512_linkedsemi_data *dev_data = dev->data;

    if (is_sram_address((uint32_t)pkt->in_buf) && ((uint32_t)pkt->in_buf % 4) == 0) {
        sha512_linkedsemi_sha(ctx, pkt->in_buf, pkt->in_len, pkt, final);
    } else {
        uint8_t *current = pkt->in_buf;
        uint32_t remain = pkt->in_len % sizeof(dev_data->temp_sram_buffer);

        for (uint32_t blk = 0; blk < (pkt->in_len / sizeof(dev_data->temp_sram_buffer)); blk++) {
            memcpy(dev_data->temp_sram_buffer, current, sizeof(dev_data->temp_sram_buffer));
            sha512_linkedsemi_sha(ctx, dev_data->temp_sram_buffer, sizeof(dev_data->temp_sram_buffer), pkt, final);
            current += sizeof(dev_data->temp_sram_buffer);
        }
        if (remain || (final && !pkt->in_len)) {
            memcpy(dev_data->temp_sram_buffer, current, remain);
            sha512_linkedsemi_sha(ctx, dev_data->temp_sram_buffer, remain, pkt, final);
        }
    }
    return 0;
}

static int sha512_linkedsemi_init(const struct device *dev)
{
    const struct sha512_linkedsemi_config *const dev_config = dev->config;
    struct sha512_linkedsemi_data *dev_data = dev->data;
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

    k_mutex_init(&dev_data->sha512_engine_mutex);
    k_sem_init(&dev_data->calc_end_sem, 0, K_SEM_MAX_LIMIT);
    dev_config->irq_config_func(dev);
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
    const struct sha512_linkedsemi_config *cfg = dev->config;
    reg_sha512_t *reg = cfg->reg;

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

    REG_FIELD_WR(reg->CTRL, SHA512_CTRL_MODE, dev_data->algo);
    dev_data->buffer_idx = 0;
    dev_data->total_length = 0;
    dev_data->first_update = true;
    ctx->started = false;
    ctx->hash_hndlr = sha512_linkedsemi_sha_align;
    return 0;
}

static int sha512_linkedsemi_cipher_free_session(const struct device *dev, struct hash_ctx *ctx)
{
    const struct sha512_linkedsemi_config *const dev_config = dev->config;
    struct sha512_linkedsemi_data *dev_data = dev->data;
    ctx->started = false;
    ls_sha512_ctx = NULL;
    k_mutex_unlock(&dev_data->sha512_engine_mutex);
    irq_disable(dev_config->irqn);
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
    __attribute__((aligned(32))) static struct sha512_linkedsemi_data                          \
    sha512_linkedsemi_data_##idx;                                                              \
    static const struct sha512_linkedsemi_config sha512_linkedsemi_config_##idx = {            \
        .reg = (void *)DT_INST_REG_ADDR(idx),                                              \
        .irq_config_func = sha512_linkedsemi_irq_config_func_##idx,                        \
        .irqn = DT_INST_IRQN(idx),                                                         \
        IF_ENABLED(DT_HAS_CLOCKS(idx), (.ccfg = LS_DT_CLK_CFG_ITEM(idx), ))                       \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(idx, resets),                                             \
               (.reset = RESET_DT_SPEC_INST_GET(idx), ))                                   \
    };                                                                                         \
    DEVICE_DT_INST_DEFINE(idx, sha512_linkedsemi_init, NULL, &sha512_linkedsemi_data_##idx,    \
              &sha512_linkedsemi_config_##idx, POST_KERNEL,                                  \
              CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&sha512_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_SHA512_INIT)