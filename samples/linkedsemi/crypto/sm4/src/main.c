/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample to illustrate the usage of crypto APIs. The sample plaintext
 * and ciphertexts used for crosschecking are from TinyCrypt.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>

#define LOG_LEVEL CONFIG_SM4_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#if DT_HAS_COMPAT_STATUS_OKAY(linkedsemi_sm4)
#define SM4_DEV_COMPAT linkedsemi_sm4
#else
#error "You need to enable one crypto device"
#endif

static const uint8_t key[0x10] = {
	0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88,
	0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,
};

static uint8_t plaintext[0x40] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F,
};

static const uint8_t ecb_ciphertext[0x40] = {
	0xEF, 0x9F, 0x47, 0xA4, 0xCB, 0xF2, 0x69, 0x1C, 
	0xF7, 0x70, 0xF6, 0xA8, 0xB0, 0xC2, 0x15, 0xBB, 
	0x79, 0xDE, 0xC9, 0x04, 0xED, 0x99, 0xED, 0xB8, 
	0xC8, 0xE2, 0x6B, 0xCA, 0x7C, 0x2A, 0x35, 0x3A, 
	0xE3, 0x3F, 0xA4, 0x38, 0xEC, 0x62, 0xE8, 0xD1, 
	0x1E, 0xCA, 0x34, 0x21, 0x6A, 0x9D, 0x03, 0xB3, 
	0xD1, 0xFF, 0xB6, 0xED, 0xF0, 0xF4, 0x2E, 0xF1, 
	0xEF, 0xD1, 0x20, 0x45, 0xDC, 0x04, 0x6A, 0xA8,
};

static const uint8_t cbc_ciphertext[0x40] = {
	0x72, 0xEB, 0xA3, 0x03, 0x99, 0x47, 0xE1, 0x70, 
	0x92, 0xE9, 0x22, 0xD7, 0xCD, 0xA3, 0x8E, 0xA0, 
	0xE3, 0x75, 0xBD, 0x64, 0xF6, 0xAB, 0x38, 0x18, 
	0x8F, 0xBE, 0xD5, 0xB9, 0xF3, 0x82, 0xA7, 0x60, 
	0xAA, 0x0C, 0x92, 0x6A, 0x8A, 0x4B, 0x51, 0xCD, 
	0xE7, 0xE0, 0x2C, 0x23, 0x64, 0xF6, 0xB6, 0x56, 
	0xCD, 0xAB, 0x53, 0x29, 0xB8, 0xF7, 0x58, 0xD3, 
	0xD3, 0x47, 0xD4, 0x67, 0xE6, 0x17, 0x99, 0xF1,
};

static const uint8_t ctr_ciphertext[0x40] = {
	0x22, 0xC1, 0x82, 0x9E, 0x86, 0x5A, 0xA8, 0x42, 
	0xAE, 0x4D, 0x24, 0x45, 0x9F, 0xAD, 0x1D, 0x3D, 
	0xEF, 0xA8, 0x38, 0x4C, 0xA6, 0xE5, 0xE3, 0x46, 
	0x16, 0xF7, 0xC4, 0x29, 0xD9, 0xD2, 0x1D, 0x4A, 
	0xEC, 0xE2, 0xCA, 0x88, 0xB1, 0xB2, 0xD7, 0x5C, 
	0x0E, 0x65, 0x39, 0x9A, 0xFA, 0x93, 0x43, 0x3E, 
	0xF7, 0x77, 0x9F, 0xCC, 0x5C, 0x07, 0x0B, 0xD4, 
	0x67, 0xE4, 0xC1, 0x34, 0x38, 0x20, 0x8B, 0xBC,
};

static const uint8_t ccm_ciphertext[0x40] = {
	0x66, 0x56, 0x73, 0x00, 0xC6, 0xEB, 0xCE, 0xE0, 
	0x9D, 0x05, 0xEB, 0xC6, 0xE6, 0xF6, 0xD2, 0x5F, 
	0xCC, 0xAF, 0x8C, 0xFD, 0x31, 0x8F, 0x7D, 0x0D, 
	0x64, 0xFE, 0xD9, 0xA4, 0xF6, 0xCA, 0x3A, 0x0B, 
	0xD0, 0xB4, 0x3F, 0xA0, 0x9D, 0xB8, 0x58, 0xFC, 
	0x76, 0xD3, 0xED, 0xD9, 0x79, 0x75, 0x30, 0x2D, 
	0xB0, 0xF7, 0x27, 0x1E, 0x53, 0x6C, 0xA0, 0x1F, 
	0x5D, 0xA3, 0x4B, 0xBE, 0x47, 0xA7, 0x30, 0xD1,
};

static const uint8_t ccm_tag[0x10] = {
	0xAF, 0xD8, 0x10, 0x54, 0xF1, 0xF1, 0xFB, 0x71, 
	0x26, 0x3B, 0x0B, 0xEB, 0xED, 0x14, 0x16, 0xEA,
};

static const uint8_t gcm_ciphertext[0x40] = {
	0xCC, 0xC2, 0xEA, 0xA8, 0x91, 0x92, 0xF7, 0x7C, 
	0x2E, 0x45, 0x19, 0xBA, 0xDA, 0xB3, 0x63, 0x1E, 
	0xD7, 0x57, 0xBF, 0xEC, 0x7C, 0x27, 0x2B, 0xF4, 
	0x47, 0xC4, 0xE1, 0x14, 0x18, 0x00, 0xAB, 0x9C, 
	0xD0, 0x4C, 0xBD, 0x08, 0xA0, 0x1B, 0x2C, 0x4E, 
	0x25, 0xB2, 0xBA, 0xDE, 0x2B, 0x74, 0xA4, 0xD2, 
	0x61, 0xAA, 0x1A, 0xAA, 0x33, 0xE3, 0x4E, 0xC8, 
	0x64, 0xB5, 0xD5, 0x75, 0x2C, 0xEB, 0x23, 0x57, 
};

static const uint8_t gcm_tag[0x10] = {
	0x4F, 0x92, 0x63, 0x2B, 0xC3, 0x4A, 0x5F, 0x7C, 
	0xB7, 0x6C, 0x17, 0x1B, 0x00, 0x16, 0x8C, 0x7C,
};

uint32_t cap_flags;

static void print_buffer_comparison(const uint8_t *wanted_result,
				    uint8_t *result, size_t length)
{
	int i, j;

	printk("Was waiting for: \n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", wanted_result[i]);

		if (j == 0x10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\nBut got:\n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", result[i]);

		if (j == 0x10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\n");
}

int validate_hw_compatibility(const struct device *dev)
{
	uint32_t flags = 0U;

	flags = crypto_query_hwcaps(dev);
	if ((flags & CAP_RAW_KEY) == 0U) {
		LOG_INF("Please provision the key separately "
			"as the module doesnt support a raw key");
		return -1;
	}

	if ((flags & CAP_SYNC_OPS) == 0U) {
		LOG_ERR("The app assumes sync semantics. "
		  "Please rewrite the app accordingly before proceeding");
		return -1;
	}

	if ((flags & CAP_SEPARATE_IO_BUFS) == 0U) {
		LOG_ERR("The app assumes distinct IO buffers. "
		"Please rewrite the app accordingly before proceeding");
		return -1;
	}

	cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	return 0;

}

void ecb_mode(const struct device *dev)
{
	uint8_t encrypted[0x40] = {0};
	uint8_t decrypted[0x40] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4, CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_block_op(&ini, &encrypt)) {
		LOG_ERR("ECB mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ecb_ciphertext, sizeof(ecb_ciphertext))) {
		LOG_ERR("ECB mode ENCRYPT - Mismatch between expected and "
			"returned cipher text");
		print_buffer_comparison(ecb_ciphertext, encrypt.out_buf, sizeof(ecb_ciphertext));
		goto out;
	}

	LOG_INF("ECB mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4, CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	if (cipher_block_op(&ini, &decrypt)) {
		LOG_ERR("ECB mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, sizeof(plaintext))) {
		LOG_ERR("ECB mode DECRYPT - Mismatch between plaintext and "
			"decrypted cipher text");
		print_buffer_comparison(plaintext, decrypt.out_buf, sizeof(plaintext));
		goto out;
	}

	LOG_INF("ECB mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void cbc_mode(const struct device *dev)
{
	uint8_t encrypted[0x40] = {0};
	uint8_t decrypted[0x40] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	static uint8_t iv[0x10] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_cbc_op(&ini, &encrypt, iv)) {
		LOG_ERR("CBC mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, cbc_ciphertext, sizeof(cbc_ciphertext))) {
		LOG_ERR("CBC mode ENCRYPT - Mismatch between expected and "
			    "returned cipher text");
		print_buffer_comparison(cbc_ciphertext, encrypt.out_buf,
					sizeof(cbc_ciphertext));
		goto out;
	}

	LOG_INF("CBC mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	/* TinyCrypt keeps IV at the start of encrypted buffer */
	if (cipher_cbc_op(&ini, &decrypt, iv)) {
		LOG_ERR("CBC mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, sizeof(plaintext))) {
		LOG_ERR("CBC mode DECRYPT - Mismatch between plaintext and "
			    "decrypted cipher text");
		print_buffer_comparison(plaintext, decrypt.out_buf,
					sizeof(plaintext));
		goto out;
	}

	LOG_INF("CBC mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void ctr_mode(const struct device *dev)
{
	uint8_t encrypted[0x40] = {0};
	uint8_t decrypted[0x40] = {0};
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.flags = cap_flags,
		/*  ivlen + ctrlen = keylen , so ctrlen is 128 - 96 = 32 bits */
		.mode_params.ctr_info.ctr_len = 32,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(encrypted),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};
	uint8_t iv[12] = {
		0x00, 0x01, 0x02, 0x03, 
		0x04, 0x05, 0x06, 0x07, 
		0x08, 0x09, 0x0a, 0x0b,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &encrypt, iv)) {
		LOG_ERR("CTR mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ctr_ciphertext, sizeof(ctr_ciphertext))) {
		LOG_ERR("CTR mode ENCRYPT - Mismatch between expected "
			    "and returned cipher text");
		print_buffer_comparison(ctr_ciphertext, encrypt.out_buf,
					sizeof(ctr_ciphertext));
		goto out;
	}

	LOG_INF("CTR mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &decrypt, iv)) {
		LOG_ERR("CTR mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, sizeof(plaintext))) {
		LOG_ERR("CTR mode DECRYPT - Mismatch between plaintext "
			    "and decrypted cipher text");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, sizeof(plaintext));
		goto out;
	}

	LOG_INF("CTR mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void ccm_mode(const struct device *dev)
{
	uint8_t ccm_nonce[12] = {
		0x00, 0x01, 0x02, 0x03, 
		0x04, 0x05, 0x06, 0x07, 
		0x08, 0x09, 0x0a, 0x0b,
	};
	uint8_t ccm_aad[8] = {
		0x00, 0x01, 0x02, 0x03, 
		0x04, 0x05, 0x06, 0x07,
	};

	uint8_t encrypted[0x40];
	uint8_t decrypted[0x40];
	uint8_t tag[0x10];
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.mode_params.ccm_info = {
			.nonce_len = sizeof(ccm_nonce),
			.tag_len = sizeof(tag),
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf = encrypted,
		.out_buf_max = sizeof(encrypted),
	};
	struct cipher_aead_pkt ccm_op = {
		.ad = ccm_aad,
		.ad_len = sizeof(ccm_aad),
		.pkt = &encrypt,
		.tag = tag,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(plaintext),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	ccm_op.pkt = &encrypt;
	if (cipher_ccm_op(&ini, &ccm_op, ccm_nonce)) {
		LOG_ERR("CCM mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, ccm_ciphertext, sizeof(ccm_ciphertext))) {
		LOG_ERR("CCM mode ENCRYPT - Mismatch between expected "
			    "and returned cipher text");
		print_buffer_comparison(ccm_ciphertext,
					encrypt.out_buf, sizeof(ccm_ciphertext));
		goto out;
	}
	
	if (memcmp(ccm_op.tag, ccm_tag, sizeof(ccm_tag))) {
		LOG_ERR("CCM mode ENCRYPT - Mismatch between expected "
			"and returned TAG text");
		print_buffer_comparison(ccm_tag, ccm_op.tag, sizeof(ccm_tag));
		goto out;
	}

	LOG_INF("CCM mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	ccm_op.pkt = &decrypt;
	if (cipher_ccm_op(&ini, &ccm_op, ccm_nonce)) {
		LOG_ERR("CCM mode DECRYPT - Failed");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, sizeof(plaintext));
		goto out;
	}

	LOG_INF("CCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void gcm_mode(const struct device *dev)
{
	uint8_t gcm_nonce[12] = {
		0x00, 0x01, 0x02, 0x03, 
		0x04, 0x05, 0x06, 0x07, 
		0x08, 0x09, 0x0a, 0x0b,
	};
	uint8_t gcm_aad[8] = {
		0x00, 0x01, 0x02, 0x03, 
		0x04, 0x05, 0x06, 0x07,
	};
	uint8_t encrypted[0x40] = {0};
	uint8_t decrypted[0x40] = {0};
	uint8_t tag[0x10];
	struct cipher_ctx ini = {
		.keylen = sizeof(key),
		.key.bit_stream = key,
		.mode_params.gcm_info = {
			.nonce_len = sizeof(gcm_nonce),
			.tag_len = sizeof(tag)
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = sizeof(plaintext),
		.out_buf_max = sizeof(encrypted),
		.out_buf = encrypted,
	};
	struct cipher_aead_pkt gcm_op = {
		.ad = gcm_aad,
		.ad_len = sizeof(gcm_aad),
		.pkt = &encrypt,
		.tag = tag,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypted,
		.in_len = sizeof(plaintext),
		.out_buf = decrypted,
		.out_buf_max = sizeof(decrypted),
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	gcm_op.pkt = &encrypt;
	if (cipher_gcm_op(&ini, &gcm_op, gcm_nonce)) {
		LOG_ERR("GCM mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	if (memcmp(encrypt.out_buf, gcm_ciphertext, sizeof(gcm_ciphertext))) {
		LOG_ERR("GCM mode ENCRYPT - Mismatch between expected "
			"and returned cipher text");
		print_buffer_comparison(gcm_ciphertext, encrypt.out_buf, sizeof(gcm_ciphertext));
		goto out;
	}

	if (memcmp(gcm_op.tag, gcm_tag, sizeof(gcm_tag))) {
		LOG_ERR("GCM mode ENCRYPT - Mismatch between expected "
			"and returned TAG text");
		print_buffer_comparison(gcm_tag, gcm_op.tag, sizeof(gcm_tag));
		goto out;
	}

	LOG_INF("GCM mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_SM4,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	gcm_op.pkt = &decrypt;
	if (cipher_gcm_op(&ini, &gcm_op, gcm_nonce)) {
		LOG_ERR("GCM mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("GCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

struct mode_test {
	const char *mode;
	void (*mode_func)(const struct device *dev);
};

int main(void)
{
	LOG_INF("SM4 Sample");

	const struct device *const dev = DEVICE_DT_GET_ONE(SM4_DEV_COMPAT);
	if (!device_is_ready(dev)) {
		LOG_ERR("Crypto device is not ready\n");
		return 0;
	}

	const struct mode_test modes[] = {
		{.mode = "ECB Mode", .mode_func = ecb_mode},
		{.mode = "CBC Mode", .mode_func = cbc_mode},
		{.mode = "CTR Mode", .mode_func = ctr_mode},
		{.mode = "GCM Mode", .mode_func = gcm_mode},
		{.mode = "CCM Mode", .mode_func = ccm_mode},
		{},
	};

	if (validate_hw_compatibility(dev)) {
		LOG_ERR("Incompatible h/w");
		return 0;
	}

	for (int i = 0; modes[i].mode; i++) {
		LOG_INF("***** SM4 %s *****", modes[i].mode);
		modes[i].mode_func(dev);
	}
	
	return 0;
}
