#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/byteorder.h>
#include "crypto_linkedsemi.h"
LOG_MODULE_DECLARE(crypto_linkedsem);
#include "crypto_linkedsemi_aes.h"

void linkedsemi_crypto_isr(const struct device *dev)
{
    struct crypto_linkedsemi_data *dev_data = dev->data;
    const struct crypto_linkedsemi_config *dev_config = dev->config;

    union aes_reg_sr sr_un;
    sr_un.value = sys_read32(dev_config->reg_crypt + CRYPT_SR);
    if (sr_un.field.AESRIF) {
        union aes_reg_icfr aes_reg_icfr_un = {
            .field = {
                .AESIF = 1,
            },
        };
        sys_write32(aes_reg_icfr_un.value, dev_config->reg_crypt + CRYPT_ICFR);
        k_sem_give(&dev_data->cipher_device_sync_sem);
    } else if (sr_un.field.DESRIF) {
        __ASSERT(0, "TODO");
    }
}

int crypto_linkedsemi_single_block(const struct device *dev,
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
    union aes_reg_cr aes_reg_cr_un;
    int ret = 0;

    __ASSERT(pkt_in_len % AES_BLOCK_LEN_BYTE == 0, "padding before crypto");

    sys_memcpy_swap(u32_key, ctx_key_bit_stream, ctx_keylen);

    k_mutex_lock(&dev_data->cipher_mutex, K_FOREVER);

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

    aes_reg_cr_un = (union aes_reg_cr){
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

    sys_write32(aes_reg_cr_un.value, dev_config->reg_crypt + CRYPT_CR);

    k_sem_take(&dev_data->cipher_device_sync_sem, K_FOREVER);

    uint32_t out[] = {
        [0] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES3)),
        [1] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES2)),
        [2] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES1)),
        [3] = BSWAP_32(sys_read32(dev_config->reg_crypt + CRYPT_RES0)),
    };

    memcpy(pkt_out_buf, out, sizeof(out));

    k_mutex_unlock(&dev_data->cipher_mutex);

    return ret;
}

int crypto_linkedsemi_multiple_block(const struct device *dev,
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

int crypto_linkedsemi_ecb_encrypt(struct cipher_ctx *ctx,
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

int crypto_linkedsemi_ecb_decrypt(struct cipher_ctx *ctx,
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

int crypto_linkedsemi_cbc_encrypt(struct cipher_ctx *ctx,
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

int crypto_linkedsemi_cbc_decrypt(struct cipher_ctx *ctx,
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

int crypto_linkedsemi_ctr(struct cipher_ctx *ctx,
                                         struct cipher_pkt *pkt,
                                         uint8_t *ctr)
{
    const struct device *dev = ctx->device;
    uint8_t iv[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t out_tmp[AES_BLOCK_LEN_BYTE] = {0};
    uint32_t block_num = 0;
    int ret = 0;

    memcpy(iv, ctr, AES_BLOCK_LEN_BYTE - 4);
    // block_num = (iv[12] << 24) | (iv[13] << 16) | (iv[14] << 8) | (iv[15]);
    block_num = 0;
    for (uint32_t i = 0; i < pkt->in_len / AES_BLOCK_LEN_BYTE; i++) {
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            out_tmp,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            block_num++;
            iv[12] = (uint8_t)(block_num >> 24);
            iv[13] = (uint8_t)(block_num >> 16);
            iv[14] = (uint8_t)(block_num >> 8);
            iv[15] = (uint8_t)(block_num);
            mem_xor_128(pkt->out_buf + i * AES_BLOCK_LEN_BYTE,
                        pkt->in_buf + i * AES_BLOCK_LEN_BYTE, out_tmp);
            pkt->out_len += AES_BLOCK_LEN_BYTE;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            break;
        }
    }

    return ret;
}
