/*
* SPDX-License-Identifier: Apache-2.0
*
* Copyright (c) 2024 LINKEDSEMI Technology Inc.
*/

#define DT_DRV_COMPAT linkedsemi_sm4

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "crypto_linkedsemi_sm4.h"
#include "field_manipulate.h"

LOG_MODULE_REGISTER(sm4_linkedsemi, LOG_LEVEL_DBG);

void sm4_linkedsemi_isr(const struct device *dev)
{
    const struct sm4_linkedsemi_config *cfg = dev->config;
	struct sm4_linkedsemi_data *dev_data = dev->data;
	reg_sm4_t *reg = cfg->reg;

	uint32_t status = reg->INTR_STT;
	if (status & SM4_INTR_END_MASK) {
		reg->INTR_CLR = SM4_INTR_END_MASK;
		k_sem_give(&dev_data->cal_end_sem);
	}
	if (status & SM4_INTR_DATA_MASK) {
		for (uint8_t i = 0; i < 4; i++) {
			reg->CALC_WRD = dev_data->buffer[i];
		}
		reg->INTR_CLR = SM4_INTR_DATA_MASK;
		k_sem_give(&dev_data->wait_data_sem);
	}
	if (status & SM4_KEY_END_MASK) {
		reg->INTR_CLR = SM4_KEY_END_MASK;
		k_sem_give(&dev_data->key_ex_sem);
	}
}

static int sm4_linkedsemi_single_block(const struct device *dev, uint8_t *data_in,
				       uint8_t *data_out, bool is_decrypt)
{
	struct sm4_linkedsemi_data *dev_data = dev->data;
	const struct sm4_linkedsemi_config *cfg = dev->config;
	reg_sm4_t *reg = cfg->reg;
	k_mutex_lock(&dev_data->sm4_engine_mutex, K_FOREVER);

	memcpy((uint8_t *)dev_data->buffer, data_in, SM4_BLOCK_LENGTH);
	reg->SM4_CTRL = FIELD_BUILD(SM4_CALC_DEC, is_decrypt);
	reg->SM4_START = SM4_CALC_START_MASK;

	k_sem_take(&dev_data->wait_data_sem, K_FOREVER);

	k_sem_take(&dev_data->cal_end_sem, K_FOREVER);
	uint32_t out[] = {
		[0] = reg->CALC_RSLT0,
		[1] = reg->CALC_RSLT1,
		[2] = reg->CALC_RSLT2,
		[3] = reg->CALC_RSLT3,
	};
	memcpy(data_out, out, SM4_BLOCK_LENGTH);

	k_mutex_unlock(&dev_data->sm4_engine_mutex);
	return 0;
}

static void sm4_linkedsemi_key_expansion(const struct device *dev, const uint8_t *key)
{
    struct sm4_linkedsemi_data *dev_data = dev->data;
    const struct sm4_linkedsemi_config *cfg = dev->config;
	reg_sm4_t *reg = cfg->reg;
    uint32_t u32_key[4];
	
	k_mutex_lock(&dev_data->sm4_engine_mutex, K_FOREVER);

    sys_memcpy_swap(u32_key, key, SM4_KEY_LENGTH);
	reg->SM4_KEY0 = u32_key[0];
	reg->SM4_KEY1 = u32_key[1];
	reg->SM4_KEY2 = u32_key[2];
	reg->SM4_KEY3 = u32_key[3];

	reg->INTR_CLR = SM4_KEY_END_MASK;
	reg->SM4_START = SM4_KEY_START_MASK;
    k_sem_take(&dev_data->key_ex_sem, K_FOREVER);

    k_mutex_unlock(&dev_data->sm4_engine_mutex);
}

static int sm4_linkedsemi_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	const struct device *dev = ctx->device;
	int ret;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	for (uint8_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		ret = sm4_linkedsemi_single_block(dev, 
										pkt->in_buf + i * SM4_BLOCK_LENGTH,
						  				pkt->out_buf + i * SM4_BLOCK_LENGTH, 
						  				false);
		if (ret == 0) {
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: sm4 error", __func__);
			break;
		}
	}

    return ret;
}

static int sm4_linkedsemi_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	const struct device *dev = ctx->device;
	int ret;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	for (uint8_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		ret = sm4_linkedsemi_single_block(dev, 
										pkt->in_buf + i * SM4_BLOCK_LENGTH,
						  				pkt->out_buf + i * SM4_BLOCK_LENGTH, 
						  				true);
		if (ret == 0) {
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: sm4 error", __func__);
			break;
		}
	}

    return ret;
}

static int sm4_linkedsemi_cbc_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	const struct device *dev = ctx->device;
	uint8_t in_data[SM4_BLOCK_LENGTH];
	uint8_t c_iv[SM4_BLOCK_LENGTH];
	int ret;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	memcpy(c_iv, iv, SM4_BLOCK_LENGTH);
	for (uint8_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		mem_xor_128(in_data, pkt->in_buf + i * SM4_BLOCK_LENGTH, c_iv);
		ret = sm4_linkedsemi_single_block(dev, in_data, pkt->out_buf + i * SM4_BLOCK_LENGTH, false);
		if (ret == 0) {
			memcpy(c_iv, pkt->out_buf + i * SM4_BLOCK_LENGTH, SM4_BLOCK_LENGTH);
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: sm4 error", __func__);
			break;
		}
	}

    return ret;
}

static int sm4_linkedsemi_cbc_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	const struct device *dev = ctx->device;
	uint8_t out_data[SM4_BLOCK_LENGTH];
	uint8_t *c_iv = iv;
	int ret;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	for (uint8_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		ret = sm4_linkedsemi_single_block(dev, pkt->in_buf + i * SM4_BLOCK_LENGTH, out_data, true);
		if (ret == 0) {
			mem_xor_128(pkt->out_buf + i * SM4_BLOCK_LENGTH, out_data, c_iv);
			c_iv = pkt->in_buf + i * SM4_BLOCK_LENGTH;
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: sm4 error", __func__);
			break;
		}
	}

    return ret;
}

static inline void inc_ctr(uint8_t* ctr, uint32_t ctr_size)
{
    for (uint32_t i = ctr_size - 1; i >= 0; i--) {
        if (++ctr[i])
            break;
    }
}

static int sm4_linkedsemi_ctr_crypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *ctr)
{
	const struct device *dev = ctx->device;
	uint8_t iv[SM4_BLOCK_LENGTH] = {0};
	uint8_t out[SM4_BLOCK_LENGTH] = {0};
    const uint8_t cntlen = ctx->mode_params.ctr_info.ctr_len >> 3;
    const uint8_t ivlen = SM4_BLOCK_LENGTH - cntlen;
    const uint32_t unalign_block_len = pkt->in_len % SM4_BLOCK_LENGTH;
	int ret;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

    memcpy(iv, ctr, ivlen);
	for (uint8_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		ret = sm4_linkedsemi_single_block(dev, iv, out, false);
		if (ret == 0) {
            inc_ctr(iv + ivlen, cntlen);
			mem_xor_128(pkt->out_buf + i * SM4_BLOCK_LENGTH,
				    pkt->in_buf + i * SM4_BLOCK_LENGTH, out);
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: sm4 error", __func__);
			break;
		}
	}

	if (unalign_block_len != 0) {
		inc_ctr(iv + ivlen, cntlen);
		ret = sm4_linkedsemi_single_block(dev, iv, out, false);
		if (ret == 0) {
			mem_xor_n(pkt->out_buf + pkt->out_len, pkt->in_buf + pkt->out_len, out,
				  unalign_block_len);
			pkt->out_len += unalign_block_len;
		} else {
			LOG_ERR("%s: crypto error", __func__);
			return -EINVAL;
		}
	}

    return ret;
}

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

static void ghash(uint8_t gcm_h[16], const uint8_t* a, uint32_t a_size, const uint8_t* c,
    uint32_t c_size, uint8_t* s, uint32_t s_size)
{
    uint64_t x[2] = {0,0};
    uint32_t blocks, partial;
    uint64_t big_h[2];

    memcpy(big_h, gcm_h, SM4_BLOCK_LENGTH);
    big_h[0] = BSWAP_64(big_h[0]);
    big_h[1] = BSWAP_64(big_h[1]);

    /* Hash in A, the Additional Authentication Data */
    if (a_size != 0 && a != NULL) {
        uint64_t big_a[2];
        blocks = a_size / SM4_BLOCK_LENGTH;
        partial = a_size % SM4_BLOCK_LENGTH;
        while (blocks--) {
            memcpy(big_a, a, SM4_BLOCK_LENGTH);
            big_a[0] = BSWAP_64(big_a[0]);
            big_a[1] = BSWAP_64(big_a[1]);
            x[0] ^= big_a[0];
            x[1] ^= big_a[1];
            gmult(x, big_h);
            a += SM4_BLOCK_LENGTH;
        }
        if (partial != 0) {
            memset(big_a, 0, SM4_BLOCK_LENGTH);
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
        blocks = c_size / SM4_BLOCK_LENGTH;
        partial = c_size % SM4_BLOCK_LENGTH;

        while (blocks--) {
            memcpy(big_c, c, SM4_BLOCK_LENGTH);
            big_c[0] = BSWAP_64(big_c[0]);
            big_c[1] = BSWAP_64(big_c[1]);
            x[0] ^= big_c[0];
            x[1] ^= big_c[1];
            gmult(x, big_h);
            c += SM4_BLOCK_LENGTH;
        }
        if (partial != 0) {
            memset(big_c, 0, SM4_BLOCK_LENGTH);
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

static int sm4_linkedsemi_gcm_encrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				       uint8_t *nonce)
{
	const struct device *dev = ctx->device;
	struct cipher_pkt *pkt = apkt->pkt;

	uint8_t gcm_h[SM4_BLOCK_LENGTH] = {0};
	uint8_t c_j0[SM4_BLOCK_LENGTH] = {0};
	uint8_t iv[SM4_BLOCK_LENGTH] = {0};
	uint8_t c_iv[SM4_BLOCK_LENGTH] = {0};
	
	const uint8_t ivlen = ctx->mode_params.gcm_info.nonce_len;
	const uint8_t cntlen = SM4_BLOCK_LENGTH - ctx->mode_params.gcm_info.nonce_len;
	const uint32_t unalign_block_len = pkt->in_len % SM4_BLOCK_LENGTH;
	int ret = 0;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	memcpy(iv, nonce, ivlen);
    inc_ctr(iv + ivlen, cntlen);

	/* GCTR */
	for (uint32_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
		inc_ctr(iv + ivlen, cntlen);
		ret = sm4_linkedsemi_single_block(dev, iv, c_iv, false);
		if (ret == 0) {
			mem_xor_128(pkt->out_buf + i * SM4_BLOCK_LENGTH,
				    pkt->in_buf + i * SM4_BLOCK_LENGTH, c_iv);
			pkt->out_len += SM4_BLOCK_LENGTH;
		} else {
			LOG_ERR("%s: crypto error", __func__);
			return -EINVAL;
		}
	}

	if (unalign_block_len != 0) {
		inc_ctr(iv + ivlen, cntlen);
		ret = sm4_linkedsemi_single_block(dev, iv, c_iv, false);
		if (ret == 0) {
			mem_xor_n(pkt->out_buf + pkt->out_len, pkt->in_buf + pkt->out_len, c_iv,
				  unalign_block_len);
			pkt->out_len += unalign_block_len;
		} else {
			LOG_ERR("%s: crypto error", __func__);
			return -EINVAL;
		}
	}
	/* end GCTR */

	/* GMAC */
	/* GMAC: gcm_h: arr[128] = {0} ---encrypt---> gcm_h[128] */
	ret = sm4_linkedsemi_single_block(dev, gcm_h, gcm_h, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}
	/* GMAC: end gcm_h */

	/* GMAC: encrypt j0 */
	UNALIGNED_PUT(BSWAP_32(1), (uint32_t *)(iv + 12));
	ret = sm4_linkedsemi_single_block(dev, iv, c_j0, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}
	/* GMAC: end encrypt j0 */
	ghash(gcm_h, apkt->ad, apkt->ad_len, pkt->out_buf, pkt->out_len,
	      pkt->out_buf + pkt->out_len, ctx->mode_params.gcm_info.tag_len);
	mem_xor_n(apkt->tag, pkt->out_buf + pkt->out_len, c_j0, ctx->mode_params.gcm_info.tag_len);
	/* end GMAC */

	pkt->out_len = pkt->in_len;
	return 0;
}

static int sm4_linkedsemi_gcm_decrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				      uint8_t *nonce)
{
    const struct device *dev = ctx->device;
    struct cipher_pkt *pkt = apkt->pkt;

    uint8_t gcm_h[SM4_BLOCK_LENGTH] = {0};
    uint8_t c_j0[SM4_BLOCK_LENGTH] = {0};
    uint8_t t_prime[SM4_BLOCK_LENGTH] = {0};

    uint8_t iv[SM4_BLOCK_LENGTH] = {0};
    uint8_t c_iv[SM4_BLOCK_LENGTH] = {0};
    const uint8_t ivlen = ctx->mode_params.gcm_info.nonce_len;
    const uint8_t cntlen = SM4_BLOCK_LENGTH - ctx->mode_params.gcm_info.nonce_len;
    const uint32_t unalign_block_len = pkt->in_len % SM4_BLOCK_LENGTH;
    int ret = 0;

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

    memcpy(iv, nonce, ivlen);
    inc_ctr(iv + ivlen, cntlen);
/* GCTR */
    for (uint32_t i = 0; i < pkt->in_len / SM4_BLOCK_LENGTH; i++) {
        inc_ctr(iv + ivlen, cntlen);
		ret = sm4_linkedsemi_single_block(dev, iv, c_iv, false);
        if (ret == 0) {
            mem_xor_128(pkt->out_buf + i * SM4_BLOCK_LENGTH,
                        pkt->in_buf + i * SM4_BLOCK_LENGTH, c_iv);
            pkt->out_len += SM4_BLOCK_LENGTH;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
    if (unalign_block_len != 0) {
        inc_ctr(iv + ivlen, cntlen);
		ret = sm4_linkedsemi_single_block(dev, iv, c_iv, false);
        if (ret == 0) {
            mem_xor_n(pkt->out_buf + pkt->out_len,
                        pkt->in_buf + pkt->out_len, c_iv, unalign_block_len);
            pkt->out_len += unalign_block_len;
        } else {
            LOG_ERR("%s: crypto error", __func__);
            return -EINVAL;
        }
    }
/* end GCTR */

/* GMAC */
/* GMAC: gcm_h: arr[128] = {0} ---encrypt---> gcm_h[128] */
	ret = sm4_linkedsemi_single_block(dev, gcm_h, gcm_h, false);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end gcm_h */

/* GMAC: encrypt j0 */
    UNALIGNED_PUT(BSWAP_32(1), (uint32_t *)(iv + 12));
	ret = sm4_linkedsemi_single_block(dev, iv, c_j0, false);
    if (ret != 0) {
        LOG_ERR("%s: crypto error", __func__);
        return -EINVAL;
    }
/* GMAC: end encrypt j0 */
    ghash(gcm_h, apkt->ad, apkt->ad_len, pkt->in_buf, pkt->in_len, t_prime, sizeof(t_prime));
    mem_xor_n(t_prime, t_prime, c_j0, sizeof(t_prime));

    if (memcmp(t_prime, apkt->tag, ctx->mode_params.gcm_info.tag_len) != 0) {
        LOG_ERR("tag error");
        return -EFAULT;
    }
/* end GMAC */

    return 0;
}

static int ccm_cbc_mac(struct cipher_ctx *ctx, uint8_t *T, const uint8_t *data, uint32_t dlen,
		       uint32_t flag)
{
	const struct device *dev = ctx->device;
	uint32_t i;
	int ret = 0;

	if (flag > 0) {
		T[0] ^= (uint8_t)(dlen >> 8);
		T[1] ^= (uint8_t)(dlen);
		dlen += 2;
		i = 2;
	} else {
		i = 0;
	}

	while (i < dlen) {
		T[i++ % (SM4_BLOCK_LENGTH)] ^= *data++;
		if (((i % (SM4_BLOCK_LENGTH)) == 0) || dlen == i) {
			ret = sm4_linkedsemi_single_block(dev, T, T, false);
			if (ret != 0) {
				LOG_ERR("%s: crypto error", __func__);
				ret = -EINVAL;
			}
		}
	}

	return ret;
}

static int ccm_ctr_mode(struct cipher_ctx *ctx, uint8_t *out, uint32_t outlen, const uint8_t *in,
			uint32_t inlen, uint8_t *ctr)
{
	const struct device *dev = ctx->device;
	uint8_t buffer[SM4_BLOCK_LENGTH];
	uint8_t nonce[SM4_BLOCK_LENGTH];
	uint16_t block_num;
	uint32_t i;
	int ret = 0;

	if (out == NULL || in == NULL || ctr == NULL || inlen == 0 || outlen == 0 ||
	    outlen != inlen) {
		return -EINVAL;
	}

	memcpy(nonce, ctr, sizeof(nonce));
	block_num = (uint16_t)((nonce[14] << 8) | (nonce[15]));
	for (i = 0; i < inlen; i++) {
		if ((i % (SM4_BLOCK_LENGTH)) == 0) {
			block_num++;
			nonce[14] = (uint8_t)(block_num >> 8);
			nonce[15] = (uint8_t)(block_num);
			ret = sm4_linkedsemi_single_block(dev, nonce, buffer, false);
			if (ret != 0) {
				LOG_ERR("%s: crypto error", __func__);
				return -EINVAL;
			}
		}
		*out++ = buffer[i % (SM4_BLOCK_LENGTH)] ^ *in++;
	}

	ctr[14] = nonce[14];
	ctr[15] = nonce[15];
	return 0;
}

static int sm4_linkedsemi_ccm_encrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				       uint8_t *nonce)
{
	const struct device *dev = ctx->device;
	struct cipher_pkt *pkt = apkt->pkt;
	uint8_t b[SM4_BLOCK_LENGTH] = {0};
	uint8_t tag[SM4_BLOCK_LENGTH] = {0};
	const uint8_t nonce_len = ctx->mode_params.ccm_info.nonce_len;
	const uint8_t tag_len = ctx->mode_params.ccm_info.tag_len;
	const uint8_t q = 15 - nonce_len;
	uint32_t i;
	int ret = 0;

	if ((pkt->out_buf == NULL) || ((pkt->in_len > 0) && (pkt->in_buf == NULL)) ||
	    ((apkt->ad_len > 0) && (apkt->ad == NULL)) || (apkt->ad_len >= SM4_CCM_AAD_MAX_BYTES) ||
	    (pkt->in_len >= SM4_CCM_PAYLOAD_MAX_BYTES) || (pkt->out_buf_max < pkt->in_len) ||
	    (tag_len < 4) || (tag_len > 16) || (tag_len & 1)) {
		return -EINVAL;
	}
	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	b[0] = ((apkt->ad_len > 0) ? 0x40 : 0) | (((tag_len - 2) / 2 << 3)) | (q - 1);
	memcpy(&b[1], nonce, nonce_len);
	b[14] = (uint8_t)(pkt->in_len >> 8);
	b[15] = (uint8_t)(pkt->in_len);

	ret = sm4_linkedsemi_single_block(dev, b, tag, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}
	if (apkt->ad_len > 0) {
		ccm_cbc_mac(ctx, tag, apkt->ad, apkt->ad_len, 1);
	}
	if (pkt->in_len > 0) {
		ccm_cbc_mac(ctx, tag, pkt->in_buf, pkt->in_len, 0);
	}

	b[0] = q - 1;
	b[14] = b[15] = 0;
	ccm_ctr_mode(ctx, pkt->out_buf, pkt->in_len, pkt->in_buf, pkt->in_len, b);
	pkt->out_len = pkt->in_len;

	b[14] = b[15] = 0;
	ret = sm4_linkedsemi_single_block(dev, b, b, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}

	for (i = 0; i < tag_len; i++) {
		apkt->tag[i] = tag[i] ^ b[i];
	}

	return 0;
}

static int sm4_linkedsemi_ccm_decrypt(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt,
				       uint8_t *nonce)
{
	const struct device *dev = ctx->device;
	struct cipher_pkt *pkt = apkt->pkt;
	uint8_t b[SM4_BLOCK_LENGTH] = {0};
	uint8_t tag[SM4_BLOCK_LENGTH] = {0};
	const uint8_t tag_len = ctx->mode_params.ccm_info.tag_len;
	const uint8_t nonce_len = ctx->mode_params.ccm_info.nonce_len;
	const uint8_t q = 15 - nonce_len;
	uint32_t i;
	int ret = 0;

	if ((pkt->out_buf == NULL) || ((pkt->in_len > 0) && (pkt->in_buf == NULL)) ||
	    ((apkt->ad_len > 0) && (apkt->ad == NULL)) || (apkt->ad_len >= SM4_CCM_AAD_MAX_BYTES) ||
	    (pkt->in_len >= SM4_CCM_PAYLOAD_MAX_BYTES) || (pkt->out_buf_max < (pkt->in_len))) {
		return -EINVAL;
	}

	sm4_linkedsemi_key_expansion(dev, ctx->key.bit_stream);

	b[0] = q - 1;
	memcpy(&b[1], nonce, nonce_len);
	ccm_ctr_mode(ctx, pkt->out_buf, pkt->in_len, pkt->in_buf, pkt->in_len, b);

	b[14] = b[15] = 0;
	ret = sm4_linkedsemi_single_block(dev, b, b, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}
	for (i = 0; i < tag_len; i++) {
		tag[i] = apkt->tag[i] ^ b[i];
	}

	memset(b, 0, SM4_BLOCK_LENGTH);
	b[0] = ((apkt->ad_len > 0) ? 0x40 : 0) | (((tag_len - 2) / 2 << 3)) | (q - 1);
	memcpy(&b[1], nonce, nonce_len);
	b[14] = (uint8_t)(pkt->in_len >> 8);
	b[15] = (uint8_t)(pkt->in_len);
	
	ret = sm4_linkedsemi_single_block(dev, b, b, false);
	if (ret != 0) {
		LOG_ERR("%s: crypto error", __func__);
		return -EINVAL;
	}
	if (apkt->ad_len > 0) {
		ccm_cbc_mac(ctx, b, apkt->ad, apkt->ad_len, 1);
	}
	if (pkt->in_len > 0) {
		ccm_cbc_mac(ctx, b, pkt->out_buf, pkt->in_len, 0);
	}

	if (memcmp(b, tag, tag_len) == 0) {
		ret = 0;
	} else {
		// memset(pkt->out_buf, 0, pkt->in_len);
		ret = -1;
	}

	pkt->out_len = pkt->in_len;
	return ret;
}

static int sm4_linkedsemi_init(const struct device *dev)
{
	const struct sm4_linkedsemi_config *const cfg = dev->config;
    struct sm4_linkedsemi_data *dev_data = dev->data;
	reg_sm4_t *reg = cfg->reg;

	if (cfg->cctl_cfg.cctl_dev) {
		const struct device *clk_dev = cfg->cctl_cfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_DBG("%s device not ready", clk_dev->name);
			return -ENODEV;
		}
		clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->cctl_cfg);
	}
	
    cfg->irq_config_func(dev);
	reg->INTR_CLR = SM4_INTR_END_MASK | SM4_INTR_DATA_MASK | SM4_KEY_END_MASK;
	reg->INTR_MSK = SM4_INTR_END_MASK | SM4_INTR_DATA_MASK | SM4_KEY_END_MASK;

    k_mutex_init(&dev_data->sm4_engine_mutex);
	k_sem_init(&dev_data->wait_data_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&dev_data->key_ex_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&dev_data->cal_end_sem, 0, K_SEM_MAX_LIMIT);
	return 0;
}

static int sm4_linkedsemi_query_caps(const struct device *dev)
{
    ARG_UNUSED(dev);

    return SM4_LINKEDSEMI_CIPHER_CAPS;
}

static int sm4_linkedsemi_cipher_begin_session(const struct device *dev, struct cipher_ctx *ctx,
					       enum cipher_algo algo, enum cipher_mode mode,
					       enum cipher_op op_type)
{
	if (ctx->flags & ~(SM4_LINKEDSEMI_CIPHER_CAPS)) {
		LOG_ERR("Unsupported flag");
		return -ENOTSUP;
	}

	if (algo != CRYPTO_CIPHER_ALGO_SM4) {
		LOG_ERR("Unsupported algo");
		return -ENOTSUP;
	}

	if (ctx->keylen != SM4_KEY_LENGTH) {
		LOG_ERR("Unsupported");
		return -ENOTSUP;
	}

	if (op_type == CRYPTO_CIPHER_OP_ENCRYPT) {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = sm4_linkedsemi_ecb_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = sm4_linkedsemi_cbc_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CTR:
			ctx->ops.ctr_crypt_hndlr = sm4_linkedsemi_ctr_crypt;
			break;
		case CRYPTO_CIPHER_MODE_GCM:
			ctx->ops.gcm_crypt_hndlr = sm4_linkedsemi_gcm_encrypt;
			break;
		case CRYPTO_CIPHER_MODE_CCM:
			ctx->ops.ccm_crypt_hndlr = sm4_linkedsemi_ccm_encrypt;
			break;
		default:
			LOG_ERR("Unsupported");
			return -ENOTSUP;
		}
	} else {
		switch (mode) {
		case CRYPTO_CIPHER_MODE_ECB:
			ctx->ops.block_crypt_hndlr = sm4_linkedsemi_ecb_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CBC:
			ctx->ops.cbc_crypt_hndlr = sm4_linkedsemi_cbc_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CTR:
			ctx->ops.ctr_crypt_hndlr = sm4_linkedsemi_ctr_crypt;
			break;
		case CRYPTO_CIPHER_MODE_GCM:
			ctx->ops.gcm_crypt_hndlr = sm4_linkedsemi_gcm_decrypt;
			break;
		case CRYPTO_CIPHER_MODE_CCM:
			ctx->ops.ccm_crypt_hndlr = sm4_linkedsemi_ccm_decrypt;
			break;
	    default:
		    LOG_ERR("Unsupported");
		    return -ENOTSUP;
	    }
    }

    return 0;
}

static int sm4_linkedsemi_cipher_free_session(const struct device *dev, struct cipher_ctx *ctx)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(ctx);

    return 0;
}

static struct crypto_driver_api sm4_driver_api = {
    .query_hw_caps = sm4_linkedsemi_query_caps,
    .cipher_begin_session = sm4_linkedsemi_cipher_begin_session,
    .cipher_free_session = sm4_linkedsemi_cipher_free_session,
    .cipher_async_callback_set = NULL,
	.hash_begin_session = NULL,
	.hash_free_session = NULL,
	.hash_async_callback_set = NULL,
};

#define LS_SM4_INIT(idx)\
	static void sm4_linkedsemi_irq_config_func_##idx(const struct device *dev)\
	{ \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), sm4_linkedsemi_isr,\
			    DEVICE_DT_INST_GET(idx), 0);\
		irq_enable(DT_INST_IRQN(idx));\
	}\
	static struct sm4_linkedsemi_data sm4_linkedsemi_data_##idx;\
	static const struct sm4_linkedsemi_config sm4_linkedsemi_config_##idx = {\
		.reg = (void *)DT_INST_REG_ADDR(idx),\
        .irq_config_func = sm4_linkedsemi_irq_config_func_##idx,\
	};\
DEVICE_DT_INST_DEFINE(\
    idx,\
    sm4_linkedsemi_init,\
    NULL,\
    &sm4_linkedsemi_data_##idx,\
    &sm4_linkedsemi_config_##idx,\
    POST_KERNEL,\
    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
    (void *)&sm4_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_SM4_INIT)