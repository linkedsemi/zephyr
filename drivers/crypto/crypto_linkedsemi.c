#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/byteorder.h>
#include <ls_hal_sm4.h>
#include <ls_hal_otbn.h>
#include <ls_msp_otbn.h>
#include "crypto_linkedsemi.h"
LOG_MODULE_REGISTER(crypto_linkedsem);
#include "crypto_linkedsemi_aes.h"
#include "crypto_linkedsemi_sha.h"

#define DT_DRV_COMPAT linkedsemi_crypto

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

static int crypto_linkedsemi_cipher_begin_session(const struct device *dev,
                                           struct cipher_ctx *ctx,
                                           enum cipher_algo algo,
                                           enum cipher_mode mode,
                                           enum cipher_op op_type)
{
    if (ctx->flags & ~(CRYPTO_LINKEDSEMI_CIPHER_CAPS)) {
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
    case CRYPTO_CIPHER_MODE_CTR:
    case CRYPTO_CIPHER_MODE_GCM:
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
        case CRYPTO_CIPHER_MODE_CTR:
            ctx->ops.ctr_crypt_hndlr = crypto_linkedsemi_ctr;
            break;
        case CRYPTO_CIPHER_MODE_GCM:
            ctx->ops.gcm_crypt_hndlr = crypto_linkedsemi_gcm_encrypt_auth;
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
        case CRYPTO_CIPHER_MODE_CTR:
            ctx->ops.ctr_crypt_hndlr = crypto_linkedsemi_ctr;
            break;
        case CRYPTO_CIPHER_MODE_GCM:
            ctx->ops.gcm_crypt_hndlr = crypto_linkedsemi_gcm_decrypt_auth;
            break;
        default:
            LOG_ERR("Unsupported");
            return -ENOTSUP;
        }
    }

    return 0;
}

static int crypto_linkedsemi_cipher_free_session(const struct device *dev,
                                          struct cipher_ctx *ctx)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(ctx);

    return 0;
}

static int crypto_linkedsemi_hash_begin_session(const struct device *dev,
                                                struct hash_ctx *ctx,
                                                enum hash_algo algo)
{
    struct crypto_linkedsemi_data *dev_data = dev->data;

    if (ctx->flags & ~(CRYPTO_LINKEDSEMI_HASH_CAPS)) {
        LOG_ERR("Unsupported flag");
        return -ENOTSUP;
    }

    switch(algo) {
    case CRYPTO_HASH_ALGO_SHA224:
    case CRYPTO_HASH_ALGO_SHA256:
        break;
    case CRYPTO_HASH_ALGO_SHA384:
    case CRYPTO_HASH_ALGO_SHA512:
        __fallthrough;
    default:
        LOG_ERR("Unsupported algo");
        return -ENOTSUP;
    }

    ctx->hash_hndlr = crypto_linkedsemi_sha;
    dev_data->hash_ctx = ctx;
    dev_data->hash_algo = algo;
    dev_data->sha_fifo_index = 0;
    dev_data->sha_pkt_in_buf_index = 0;
    dev_data->sha_total_len = 0;
    dev_data->sha_is_final = false;

    return 0;
}

static int crypto_linkedsemi_hash_free_session(const struct device *dev,
                                               struct hash_ctx *ctx)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(ctx);

    return 0;
}

static int crypto_linkedsemi_query_caps(const struct device *dev)
{
    ARG_UNUSED(dev);

    return CRYPTO_LINKEDSEMI_CIPHER_CAPS | CRYPTO_LINKEDSEMI_HASH_CAPS;
}

static int crypto_linkedsemi_init(const struct device *dev)
{
    const struct crypto_linkedsemi_config *cfg = dev->config;
    struct crypto_linkedsemi_data *dev_data = dev->data;

    k_mutex_init(&dev_data->cipher_mutex);
    k_sem_init(&dev_data->cipher_device_sync_sem, 0, K_SEM_MAX_LIMIT);
    k_mutex_init(&dev_data->hash_mutex);
    k_sem_init(&dev_data->hash_device_sync_sem, 0, K_SEM_MAX_LIMIT);
    cfg->irq_config_func(dev);

    return 0;
}

static struct crypto_driver_api crypto_enc_funcs = {
    .cipher_begin_session = crypto_linkedsemi_cipher_begin_session,
    .cipher_free_session = crypto_linkedsemi_cipher_free_session,
    .hash_begin_session = crypto_linkedsemi_hash_begin_session,
    .hash_free_session = crypto_linkedsemi_hash_free_session,
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
