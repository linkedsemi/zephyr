/*
 * Copyright (c) 2021 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/byteorder.h>
#include <ls_hal_sha.h>
#include <ls_hal_sm4.h>
#include <ls_hal_otbn.h>
#include <ls_msp_otbn.h>
#include "crypto_linkedsemi.h"

LOG_MODULE_REGISTER(crypto_linkedsem);

#define DT_DRV_COMPAT linkedsemi_crypto

#define CRYP_SUPPORT (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

#define CRYPTO_LINKEDSEMI_AES_MAX_KEY_LEN_BIT  512
#define CRYPTO_LINKEDSEMI_AES_MAX_KEY_LEN_BYTE 32
#define AES_BLOCK_LEN_BYTE                     16
#define IV_LEN_BYTE                            AES_BLOCK_LEN_BYTE

struct crypto_linkedsemi_data {
    void *user_data;
    const struct device *dev;
    uint32_t data;
    struct k_mutex crypto_mutex;
    struct k_sem device_sync_sem;
};

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct crypto_linkedsemi_config {
    mem_addr_t reg_calc_sha;
    mem_addr_t reg_calc_crc;
    mem_addr_t reg_calc;
    mem_addr_t reg_crypt;
    mem_addr_t reg_calc_sm4;
    uint32_t data;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    irq_cfg_func_t irq_config_func;
};

static void linkedsemi_crypto_isr(const struct device *dev)
{
    struct crypto_linkedsemi_data *dev_data = dev->data;
    const struct crypto_linkedsemi_config *dev_config = dev->config;

    union crypto_reg_sr sr_un;
    sr_un.value = sys_read32(dev_config->reg_crypt + CRYPT_SR);
    if (sr_un.field.AESRIF) {
        union crypto_reg_icfr crypto_reg_icfr_un = {
            .field = {
                .AESIF = 1,
            },
        };
        sys_write32(crypto_reg_icfr_un.value, dev_config->reg_crypt + CRYPT_ICFR);
        k_sem_give(&dev_data->device_sync_sem);
    } else if (sr_un.field.DESRIF) {
        __ASSERT(0, "TODO");
    }
}

static void linkedsemi_sha_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    LSSHA_IRQHandler();
}

static void linkedsemi_sm4_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_SM4_IRQHandler();
}

static void linkedsemi_otbn_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_OTBN_IRQHandler();
}

static void linkedsemi_sysc_otbn_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_OTBN_SYSC_IRQHandler();
}

static int crypto_linkedsemi_single_block(const struct device *dev,
                                          const uint8_t *ctx_key_bit_stream,
                                          uint16_t ctx_keylen,
                                          uint8_t *pkt_in_buf,
                                          uint8_t *pkt_out_buf,
                                          int pkt_in_len,
                                          bool is_encrypt,
                                          enum cipher_mode mode,
                                          uint8_t *iv)
{
    struct crypto_linkedsemi_data *dev_data = dev->data;
    const struct crypto_linkedsemi_config *dev_config = dev->config;
    bool is_cbc = (mode == CRYPTO_CIPHER_MODE_CBC) ? true : false;
    bool is_iv_exist = iv ? true : false;
    uint32_t u32_key[CRYPTO_LINKEDSEMI_AES_MAX_KEY_LEN_BYTE];
    uint32_t u32_iv[4];
    union crypto_reg_cr crypto_reg_cr_un;
    int ret = 0;

    __ASSERT(pkt_in_len % AES_BLOCK_LEN_BYTE == 0, "padding before crypto");

    sys_memcpy_swap(u32_key, ctx_key_bit_stream, ctx_keylen);

    k_mutex_lock(&dev_data->crypto_mutex, K_FOREVER);

    switch (ctx_keylen) {
    case 32:
        sys_write32(u32_key[7], dev_config->reg_crypt + CRYPT_KEY7);
        sys_write32(u32_key[6], dev_config->reg_crypt + CRYPT_KEY6);
        __fallthrough;
    case 24:
        sys_write32(u32_key[5], dev_config->reg_crypt + CRYPT_KEY5);
        sys_write32(u32_key[4], dev_config->reg_crypt + CRYPT_KEY4);
        __fallthrough;
    case 16:
        sys_write32(u32_key[3], dev_config->reg_crypt + CRYPT_KEY3);
        sys_write32(u32_key[2], dev_config->reg_crypt + CRYPT_KEY2);
        sys_write32(u32_key[1], dev_config->reg_crypt + CRYPT_KEY1);
        sys_write32(u32_key[0], dev_config->reg_crypt + CRYPT_KEY0);
        break;
    default:
        __ASSERT(0, "key len error");
        break;
    };

    if (is_iv_exist) {
        sys_memcpy_swap(u32_iv, iv, 16);
        sys_write32(u32_iv[0], dev_config->reg_crypt + CRYPT_IVR0);
        sys_write32(u32_iv[1], dev_config->reg_crypt + CRYPT_IVR1);
        sys_write32(u32_iv[2], dev_config->reg_crypt + CRYPT_IVR2);
        sys_write32(u32_iv[3], dev_config->reg_crypt + CRYPT_IVR3);
    }

    sys_write32(BSWAP_32(((uint32_t *)pkt_in_buf)[0]), dev_config->reg_crypt + CRYPT_DATA3);
    sys_write32(BSWAP_32(((uint32_t *)pkt_in_buf)[1]), dev_config->reg_crypt + CRYPT_DATA2);
    sys_write32(BSWAP_32(((uint32_t *)pkt_in_buf)[2]), dev_config->reg_crypt + CRYPT_DATA1);
    sys_write32(BSWAP_32(((uint32_t *)pkt_in_buf)[3]), dev_config->reg_crypt + CRYPT_DATA0);

    crypto_reg_cr_un = (union crypto_reg_cr){
        .field = {
            .GO = 1,
            .ENCS = is_encrypt, /* is_enc */
            .AESKS = (ctx_keylen - 1) >> 4, /* 00: 128 bits   01: 192 bits   10: 256 bits */
            .MODE = is_cbc, /* is_cbc */
            .IVREN = is_iv_exist, /* is_iv_exist */
            .IE = 1,
            .TYPE = 0,
            .TDES = 0,
            .DESKS = 0,
            .FIFOEN = 0,
            .FIFOODR = 0,
            .DMAEN = 0,
            .RESERVED0 = 0,
            .CRYSEL = 0,
        },
    };

    sys_write32(crypto_reg_cr_un.value, dev_config->reg_crypt + CRYPT_CR);

    k_sem_take(&dev_data->device_sync_sem, K_FOREVER);

    uint32_t out[] = {
        [0] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES3)),
        [1] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES2)),
        [2] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES1)),
        [3] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES0)),
    };

    memcpy(pkt_out_buf, out, sizeof(out));

    k_mutex_unlock(&dev_data->crypto_mutex);

    return ret;
}

static int crypto_linkedsemi_multiple_block(const struct device *dev,
                                            const uint8_t *ctx_key_bit_stream,
                                            uint16_t ctx_keylen,
                                            uint8_t *pkt_in_buf,
                                            uint8_t *pkt_out_buf,
                                            uint32_t pkt_in_len,
                                            uint32_t *pkt_out_len,
                                            bool is_encrypt,
                                            enum cipher_mode mode,
                                            uint8_t *iv)
{
    int ret;

    *pkt_out_len = 0;
    for (uint32_t i = 0; i < pkt_in_len / AES_BLOCK_LEN_BYTE; i++) {
        ret = crypto_linkedsemi_single_block(dev,
                                             ctx_key_bit_stream,
                                             ctx_keylen,
                                             pkt_in_buf + i * AES_BLOCK_LEN_BYTE,
                                             pkt_out_buf + i * AES_BLOCK_LEN_BYTE,
                                             AES_BLOCK_LEN_BYTE,
                                             is_encrypt,
                                             mode,
                                             (i == 0) ? iv : NULL);
        if (ret == 0) {
            *pkt_out_len += AES_BLOCK_LEN_BYTE;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            break;
        }
    }

    return ret;
}

static int crypto_linkedsemi_ecb_encrypt(struct cipher_ctx *ctx,
                                         struct cipher_pkt *pkt)
{
    int ret;

    ret = crypto_linkedsemi_multiple_block(ctx->device,
                                           ctx->key.bit_stream,
                                           ctx->keylen,
                                           pkt->in_buf,
                                           pkt->out_buf,
                                           pkt->in_len,
                                           &pkt->out_len,
                                           true,
                                           CRYPTO_CIPHER_MODE_ECB,
                                           NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
    }

    return ret;
}

static int crypto_linkedsemi_ecb_decrypt(struct cipher_ctx *ctx,
                                         struct cipher_pkt *pkt)
{
    int ret;

    ret = crypto_linkedsemi_multiple_block(ctx->device,
                                           ctx->key.bit_stream,
                                           ctx->keylen,
                                           pkt->in_buf,
                                           pkt->out_buf,
                                           pkt->in_len,
                                           &pkt->out_len,
                                           false,
                                           CRYPTO_CIPHER_MODE_ECB,
                                           NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
    }

    return ret;
}

static int crypto_linkedsemi_cbc_encrypt(struct cipher_ctx *ctx,
                                         struct cipher_pkt *pkt,
                                         uint8_t *iv)
{
    int ret;
    uint32_t out_offset = 0;

    if (!(ctx->flags & CAP_NO_IV_PREFIX)) {
        /* Prefix IV to ciphertext unless CAP_NO_IV_PREFIX is set. */
        memcpy(pkt->out_buf, iv, IV_LEN_BYTE);
        out_offset = IV_LEN_BYTE;
    }

    ret = crypto_linkedsemi_multiple_block(ctx->device,
                                           ctx->key.bit_stream,
                                           ctx->keylen,
                                           pkt->in_buf,
                                           pkt->out_buf + out_offset,
                                           pkt->in_len,
                                           &pkt->out_len,
                                           true,
                                           CRYPTO_CIPHER_MODE_CBC,
                                           iv);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
    }

    pkt->out_len += out_offset;

    return ret;
}

static int crypto_linkedsemi_cbc_decrypt(struct cipher_ctx *ctx,
                                         struct cipher_pkt *pkt,
                                         uint8_t *iv)
{
    int ret;
    uint32_t in_offset = 0;

    if (!(ctx->flags & CAP_NO_IV_PREFIX)) {
        /* Prefix IV to ciphertext unless CAP_NO_IV_PREFIX is set. */
        in_offset = IV_LEN_BYTE;
    }

    ret = crypto_linkedsemi_multiple_block(ctx->device,
                                           ctx->key.bit_stream,
                                           ctx->keylen,
                                           pkt->in_buf + in_offset,
                                           pkt->out_buf,
                                           pkt->in_len - in_offset,
                                           &pkt->out_len,
                                           false,
                                           CRYPTO_CIPHER_MODE_CBC,
                                           iv);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
    }

    return ret;
}

static int crypto_linkedsemi_begin_session(const struct device *dev,
                                           struct cipher_ctx *ctx,
                                           enum cipher_algo algo,
                                           enum cipher_mode mode,
                                           enum cipher_op op_type)
{
    if (ctx->flags & ~(CRYP_SUPPORT)) {
        LOG_ERR("Unsupported flag");
        return -ENOTSUP;
    }

    if (algo != CRYPTO_CIPHER_ALGO_AES) {
        LOG_ERR("Unsupported algo");
        return -ENOTSUP;
    }

    switch (mode) {
    case CRYPTO_CIPHER_MODE_ECB:
    case CRYPTO_CIPHER_MODE_CBC:
        break;
    default:
        LOG_ERR("Unsupported mode");
        return -ENOTSUP;
    }

    switch (ctx->keylen) {
    case 16:
    case 24:
    case 32:
        break;
    default:
        LOG_ERR("Unsupported");
        return -ENOTSUP;
    }

    if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
        switch (mode) {
        case CRYPTO_CIPHER_MODE_ECB:
            ctx->ops.block_crypt_hndlr = crypto_linkedsemi_ecb_encrypt;
            break;
        case CRYPTO_CIPHER_MODE_CBC:
            ctx->ops.cbc_crypt_hndlr = crypto_linkedsemi_cbc_encrypt;
            break;
        default:
            LOG_ERR("Unsupported");
            return -ENOTSUP;
        }
    } else {
        switch (mode) {
        case CRYPTO_CIPHER_MODE_ECB:
            ctx->ops.block_crypt_hndlr = crypto_linkedsemi_ecb_decrypt;
            break;
        case CRYPTO_CIPHER_MODE_CBC:
            ctx->ops.cbc_crypt_hndlr = crypto_linkedsemi_cbc_decrypt;
            break;
        default:
            LOG_ERR("Unsupported");
            return -ENOTSUP;
        }
    }

    return 0;
}

static int crypto_linkedsemi_free_session(const struct device *dev,
                                          struct cipher_ctx *ctx)
{
    return 0;
}

static int crypto_linkedsemi_query_caps(const struct device *dev)
{
    return CRYP_SUPPORT;
}

static int crypto_linkedsemi_init(const struct device *dev)
{
    struct crypto_linkedsemi_data *dev_data = dev->data;
    const struct crypto_linkedsemi_config *cfg = dev->config;

    k_mutex_init(&dev_data->crypto_mutex);
    k_sem_init(&dev_data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
    cfg->irq_config_func(dev);

    return 0;
}

static struct crypto_driver_api crypto_enc_funcs = {
    .cipher_begin_session = crypto_linkedsemi_begin_session,
    .cipher_free_session = crypto_linkedsemi_free_session,
    .cipher_async_callback_set = NULL,
    .query_hw_caps = crypto_linkedsemi_query_caps,
};

#define CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, irq_name)              \
    do {                                                            \
        IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, irq_name, irq),      \
                    DT_INST_IRQ_BY_NAME(index, irq_name, priority), \
                    linkedsemi_##irq_name##_isr,                    \
                    DEVICE_DT_INST_GET(index),                      \
                    0);                                             \
        irq_enable(DT_INST_IRQ_BY_NAME(index, irq_name, irq));      \
    } while (false)

#define CRYPTO_LINKEDSEMI_IRQ_HANDLER(index)                                        \
    static void crypto_linkedsemi_irq_config_func_##index(const struct device *dev) \
    {                                                                               \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, crypto);                               \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sha);                                  \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sm4);                                  \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, otbn);                                 \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sysc_otbn);                            \
    }

#define CRYPTO_LINKEDSEMI_INIT(index)                                               \
    CRYPTO_LINKEDSEMI_IRQ_HANDLER(index)                                            \
    static const struct crypto_linkedsemi_config crypto_linkedsemi_cfg_##index = {  \
        .reg_calc_sha = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(index, calc_sha),      \
        .reg_calc_crc = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(index, calc_crc),      \
        .reg_calc = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(index, calc),              \
        .reg_crypt = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(index, crypt),            \
        .reg_calc_sm4 = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(index, calc_sm4),      \
        .irq_config_func = crypto_linkedsemi_irq_config_func_##index,               \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(index), )) \
    };                                                                              \
    static struct crypto_linkedsemi_data crypto_linkedsemi_dev_data_##index;        \
    DEVICE_DT_INST_DEFINE(index,                                                    \
                          crypto_linkedsemi_init,                                   \
                          NULL,                                                     \
                          &crypto_linkedsemi_dev_data_##index,                      \
                          &crypto_linkedsemi_cfg_##index,                           \
                          POST_KERNEL,                                              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                       \
                          (void *)&crypto_enc_funcs);
DT_INST_FOREACH_STATUS_OKAY(CRYPTO_LINKEDSEMI_INIT)
