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
        union aes_reg_icfr aes_reg_icfr_un = { .field = { .AESIF = 1, }, };
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
    uint8_t c_iv[AES_BLOCK_LEN_BYTE] = {0};
    const uint8_t ivlen = AES_BLOCK_LEN_BYTE - (ctx->mode_params.ctr_info.ctr_len >> 3);
    uint32_t cnt = 0;
    int ret = 0;

    memcpy(iv, ctr, ivlen);
    for (uint32_t i = 0; i < pkt->in_len / AES_BLOCK_LEN_BYTE; i++) {
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            c_iv,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            cnt++;
            *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(cnt);
            mem_xor_128(pkt->out_buf + i * AES_BLOCK_LEN_BYTE,
                        pkt->in_buf + i * AES_BLOCK_LEN_BYTE, c_iv);
            pkt->out_len += AES_BLOCK_LEN_BYTE;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            break;
        }
    }

    return ret;
}

/* wolfssl */
static void gmult(uint64_t* x, uint64_t* y)
{
    uint64_t z[2] = {0,0};
    uint64_t v[2];
    int i, j;
    uint64_t v1;
    v[0] = x[0];
    v[1] = x[1];

    for (i = 0; i < 2; i++)
    {
        uint64_t y_tmp = y[i];
        for (j = 0; j < 64; j++)
        {
            uint64_t mask = 0 - (y_tmp >> 63);
            z[0] ^= v[0] & mask;
            z[1] ^= v[1] & mask;
            v1 = (0 - (v[1] & 1)) & 0xe100000000000000ull;
            v[1] >>= 1;
            v[1] |= v[0] << 63;
            v[0] >>= 1;
            v[0] ^= v1;
            y_tmp <<= 1;
        }
    }
    x[0] = z[0];
    x[1] = z[1];
}

/* wolfssl */
void ghash(uint8_t gcm_h[16], const uint8_t* a, uint32_t a_size, const uint8_t* c,
    uint32_t c_size, uint8_t* s, uint32_t s_size)
{
    uint64_t x[2] = {0,0};
    uint32_t blocks, partial;
    uint64_t big_h[2];

    memcpy(big_h, gcm_h, AES_BLOCK_LEN_BYTE);
    big_h[0] = BSWAP_64(big_h[0]);
    big_h[1] = BSWAP_64(big_h[1]);

    /* Hash in A, the Additional Authentication Data */
    if (a_size != 0 && a != NULL) {
        uint64_t big_a[2];
        blocks = a_size / AES_BLOCK_LEN_BYTE;
        partial = a_size % AES_BLOCK_LEN_BYTE;
        while (blocks--) {
            memcpy(big_a, a, AES_BLOCK_LEN_BYTE);
            big_a[0] = BSWAP_64(big_a[0]);
            big_a[1] = BSWAP_64(big_a[1]);
            x[0] ^= big_a[0];
            x[1] ^= big_a[1];
            gmult(x, big_h);
            a += AES_BLOCK_LEN_BYTE;
        }
        if (partial != 0) {
            memset(big_a, 0, AES_BLOCK_LEN_BYTE);
            memcpy(big_a, a, partial);
            big_a[0] = BSWAP_64(big_a[0]);
            big_a[1] = BSWAP_64(big_a[1]);
            x[0] ^= big_a[0];
            x[1] ^= big_a[1];
            gmult(x, big_h);
        }
    }

    /* Hash in C, the Ciphertext */
    if (c_size != 0 && c != NULL) {
        uint64_t big_c[2];
        blocks = c_size / AES_BLOCK_LEN_BYTE;
        partial = c_size % AES_BLOCK_LEN_BYTE;

        while (blocks--) {
            memcpy(big_c, c, AES_BLOCK_LEN_BYTE);
            big_c[0] = BSWAP_64(big_c[0]);
            big_c[1] = BSWAP_64(big_c[1]);
            x[0] ^= big_c[0];
            x[1] ^= big_c[1];
            gmult(x, big_h);
            c += AES_BLOCK_LEN_BYTE;
        }
        if (partial != 0) {
            memset(big_c, 0, AES_BLOCK_LEN_BYTE);
            memcpy(big_c, c, partial);
            big_c[0] = BSWAP_64(big_c[0]);
            big_c[1] = BSWAP_64(big_c[1]);
            x[0] ^= big_c[0];
            x[1] ^= big_c[1];
            gmult(x, big_h);
        }
    }

    /* Hash in the lengths in bits of A and C */
    {
        uint64_t len[2];
        len[0] = a_size; len[1] = c_size;

        /* Lengths are in bytes. Convert to bits. */
        len[0] *= 8;
        len[1] *= 8;

        x[0] ^= len[0];
        x[1] ^= len[1];
        gmult(x, big_h);
    }
    x[0] = BSWAP_64(x[0]);
    x[1] = BSWAP_64(x[1]);
    memcpy(s, x, s_size);
}

int crypto_linkedsemi_gcm_encrypt_auth(struct cipher_ctx *ctx,
                 struct cipher_aead_pkt *apkt,
                 uint8_t *nonce)
{
    const struct device *dev = ctx->device;
    struct cipher_pkt *pkt = apkt->pkt;

    uint8_t gcm_h[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t e_j0[AES_BLOCK_LEN_BYTE] = {0};

    uint8_t iv[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t c_iv[AES_BLOCK_LEN_BYTE] = {0};
    const uint8_t ivlen = ctx->mode_params.gcm_info.nonce_len;
    const bool is_block_len_align = ((pkt->in_len % AES_BLOCK_LEN_BYTE) == 0);
    uint8_t c_last_block_padding_zero[AES_BLOCK_LEN_BYTE] = {0};
    uint32_t cnt = 1;
    int ret = 0;

    memcpy(iv, nonce, ivlen);
/* GCTR */
    for (uint32_t i = 0; i < pkt->in_len / AES_BLOCK_LEN_BYTE; i++) {
        cnt++;
        *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(cnt);
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            c_iv,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            mem_xor_128(pkt->out_buf + i * AES_BLOCK_LEN_BYTE,
                        pkt->in_buf + i * AES_BLOCK_LEN_BYTE, c_iv);
            pkt->out_len += AES_BLOCK_LEN_BYTE;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
    if (!is_block_len_align) {
        cnt++;
        *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(cnt);
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            c_iv,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            const uint32_t last_block_len = pkt->in_len % AES_BLOCK_LEN_BYTE;
            mem_xor_n(pkt->out_buf + pkt->out_len,
                        pkt->in_buf + pkt->out_len, c_iv, last_block_len);
            pkt->out_len += last_block_len;

            memcpy(c_last_block_padding_zero, pkt->in_buf + pkt->out_len, last_block_len);
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
/* end GCTR */

/* GMAC */
/* GMAC: gcm_h: arr[128] = {0} ---encrypt---> gcm_h[128] */
    ret = crypto_linkedsemi_single_block(dev,
                                        ctx->key.bit_stream,
                                        ctx->keylen,
                                        gcm_h, /* in */
                                        gcm_h, /* out */
                                        AES_BLOCK_LEN_BYTE,
                                        true,
                                        CRYPTO_CIPHER_MODE_ECB,
                                        NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end gcm_h */

/* GMAC: encrypt j0 */
    *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(1);
    ret = crypto_linkedsemi_single_block(dev,
                                        ctx->key.bit_stream,
                                        ctx->keylen,
                                        iv, /* in */
                                        e_j0, /* out */
                                        AES_BLOCK_LEN_BYTE,
                                        true,
                                        CRYPTO_CIPHER_MODE_ECB,
                                        NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end encrypt j0 */
    ghash(gcm_h, apkt->ad, apkt->ad_len, pkt->out_buf, pkt->out_len, pkt->out_buf + pkt->out_len, ctx->mode_params.gcm_info.tag_len);
    mem_xor_n(pkt->out_buf + pkt->out_len, pkt->out_buf + pkt->out_len, e_j0, apkt->ad_len);
/* end GMAC */

    pkt->out_len += ctx->mode_params.gcm_info.tag_len;

    return 0;
}

int crypto_linkedsemi_gcm_decrypt_auth(struct cipher_ctx *ctx,
                 struct cipher_aead_pkt *apkt,
                 uint8_t *nonce)
{
    const struct device *dev = ctx->device;
    struct cipher_pkt *pkt = apkt->pkt;

    uint8_t gcm_h[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t e_j0[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t t_prime[AES_BLOCK_LEN_BYTE] = {0};

    uint8_t iv[AES_BLOCK_LEN_BYTE] = {0};
    uint8_t c_iv[AES_BLOCK_LEN_BYTE] = {0};
    const uint8_t ivlen = ctx->mode_params.gcm_info.nonce_len;
    const bool is_block_len_align = ((pkt->in_len % AES_BLOCK_LEN_BYTE) == 0);
    uint8_t c_last_block_padding_zero[AES_BLOCK_LEN_BYTE] = {0};
    uint32_t cnt = 1;
    int ret = 0;

    memcpy(iv, nonce, ivlen);
/* GCTR */
    for (uint32_t i = 0; i < pkt->in_len / AES_BLOCK_LEN_BYTE; i++) {
        cnt++;
        *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(cnt);
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            c_iv,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            mem_xor_128(pkt->out_buf + i * AES_BLOCK_LEN_BYTE,
                        pkt->in_buf + i * AES_BLOCK_LEN_BYTE, c_iv);
            pkt->out_len += AES_BLOCK_LEN_BYTE;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
    if (!is_block_len_align) {
        cnt++;
        *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(cnt);
        ret = crypto_linkedsemi_single_block(dev,
                                            ctx->key.bit_stream,
                                            ctx->keylen,
                                            iv,
                                            c_iv,
                                            AES_BLOCK_LEN_BYTE,
                                            true,
                                            CRYPTO_CIPHER_MODE_ECB,
                                            NULL);
        if (ret == 0) {
            const uint32_t last_block_len = pkt->in_len % AES_BLOCK_LEN_BYTE;
            mem_xor_n(pkt->out_buf + pkt->out_len,
                        pkt->in_buf + pkt->out_len, c_iv, last_block_len);
            pkt->out_len += last_block_len;

            memcpy(c_last_block_padding_zero, pkt->in_buf + pkt->out_len, last_block_len);
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
/* end GCTR */

/* GMAC */
/* GMAC: gcm_h: arr[128] = {0} ---encrypt---> gcm_h[128] */
    ret = crypto_linkedsemi_single_block(dev,
                                        ctx->key.bit_stream,
                                        ctx->keylen,
                                        gcm_h, /* in */
                                        gcm_h, /* out */
                                        AES_BLOCK_LEN_BYTE,
                                        true,
                                        CRYPTO_CIPHER_MODE_ECB,
                                        NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end gcm_h */

/* GMAC: encrypt j0 */
    *(uint32_t *)(&(iv[ivlen])) = BSWAP_32(1);
    ret = crypto_linkedsemi_single_block(dev,
                                        ctx->key.bit_stream,
                                        ctx->keylen,
                                        iv, /* in */
                                        e_j0, /* out */
                                        AES_BLOCK_LEN_BYTE,
                                        true,
                                        CRYPTO_CIPHER_MODE_ECB,
                                        NULL);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end encrypt j0 */
    ghash(gcm_h, apkt->ad, apkt->ad_len, pkt->in_buf, pkt->in_len, t_prime, sizeof(t_prime));
    mem_xor_n(t_prime, t_prime, e_j0, sizeof(t_prime));

    if (memcmp(t_prime, apkt->tag, ctx->mode_params.gcm_info.tag_len) != 0) {
        LOG_ERR("tag error");
        return -EFAULT;
    }
/* end GMAC */

    return 0;
}
