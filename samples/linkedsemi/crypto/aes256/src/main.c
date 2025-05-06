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
#include <stdlib.h>
#include <zephyr/crypto/crypto.h>

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define CRYPTO_DRV_NAME				 CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#define CRYPTO_DEV_COMPAT			   linkedsemi_aes256


#define AES_INPUT_SIZE_MAX			  10240
#define AES_OUTPUT_SIZE_MAX			 AES_INPUT_SIZE_MAX

#define AES_AEAD_SIZE_MAX			   (128)
#define AES_AEAD_SIZE_MIN			   (0)

#define AES_CCM_TAG_SIZE_MAX			(16)
#define AES_CCM_TAG_SIZE_MIN			(4)
#define AES_CCM_TAG_SIZE_STEP		   (2)

#define AES_GCM_TAG_SIZE				(16)

#define CNT_MAX_TEST					(2)

#define AES_BLOCK_SIZE_BIT			  (128)
#define AES_KEY_SIZE_SPECIAL_MAX_BIT	(256)
#define AES_BLOCK_SIZE_BYTE			 (128 >> 3)
#define AES_KEY_SIZE_SPECIAL_MAX_BYTE   (256 >> 3)
#define AES_KEY_SIZE_MIN_BYTE		   (128 >> 3)

#define AES_CCM_NONCE_SIZE_BYTE		 13
#define AES_GCM_NONCE_SIZE_BYTE		 12

static uint8_t aes_key[AES_KEY_SIZE_SPECIAL_MAX_BYTE] = {0};
static uint8_t aes_iv[16] = {0};
static uint8_t aes_aead[AES_AEAD_SIZE_MAX] = {0};

static uint8_t aes_plaintext_buf[AES_INPUT_SIZE_MAX];
static uint8_t aes_plaintext_decrypted_buf[AES_INPUT_SIZE_MAX];
static uint8_t aes_ciphertext_buf_sw[AES_INPUT_SIZE_MAX];
static uint8_t aes_ciphertext_buf_hw[AES_INPUT_SIZE_MAX];

static uint8_t aes_ccm_tag_sw[AES_CCM_TAG_SIZE_MAX] = {0};
static uint8_t aes_ccm_tag_hw[AES_CCM_TAG_SIZE_MAX] = {0};

static uint8_t aes_gcm_tag_sw[AES_GCM_TAG_SIZE] = {0};
static uint8_t aes_gcm_tag_hw[AES_GCM_TAG_SIZE] = {0};


uint32_t cap_flags;

static void print_buffer_comparison(const uint8_t *wanted_result,
					uint8_t *result, size_t length)
{
	int i, j;

	printk("Was waiting for: \n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", wanted_result[i]);

		if (j == 10) {
			printk("\n");
			j = 0;
		}
	}

	printk("\nBut got:\n");

	for (i = 0, j = 1; i < length; i++, j++) {
		printk("0x%02x ", result[i]);

		if (j == 10) {
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

void ecb_mode(const struct device *dev,
				const uint8_t *key,
				const uint8_t key_len,
				const uint8_t *plaintext,
				const uint32_t plaintext_len,
				uint8_t *ciphertext,
				uint32_t *ciphertext_len)
{
	__ASSERT_NO_MSG(plaintext_len % (128/8) == 0);
	uint8_t *decrypted = aes_plaintext_decrypted_buf;
	__ASSERT_NO_MSG(decrypted);
	struct cipher_ctx ini = {
		.keylen = key_len,
		.key.bit_stream = key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = plaintext_len,
		.out_buf_max = plaintext_len,
		.out_buf = ciphertext,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = plaintext_len,
		.out_buf = decrypted,
		.out_buf_max = plaintext_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	if (cipher_block_op(&ini, &encrypt)) {
		LOG_ERR("ECB mode ENCRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	if (cipher_block_op(&ini, &decrypt)) {
		LOG_ERR("ECB mode DECRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, plaintext_len)) {
		LOG_ERR("ECB mode DECRYPT - Mismatch between plaintext and "
				"decrypted cipher text");
		print_buffer_comparison(plaintext, decrypt.out_buf,
					plaintext_len);
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("ECB mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void cbc_mode(const struct device *dev,
				const uint8_t *key,
				const uint8_t key_len,
				const uint8_t *iv,
				const uint8_t iv_len,
				const uint8_t *plaintext,
				const uint32_t plaintext_len,
				uint8_t *ciphertext,
				uint32_t *ciphertext_len)
{
	uint8_t *decrypted = aes_plaintext_decrypted_buf;
	__ASSERT_NO_MSG(decrypted);
	struct cipher_ctx ini = {
		.keylen = key_len,
		.key.bit_stream = key,
		.flags = cap_flags | CAP_NO_IV_PREFIX,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = plaintext_len,
		.out_buf_max = plaintext_len,
		.out_buf = ciphertext,
	};
	struct cipher_pkt decrypt = {
		.in_buf = encrypt.out_buf,
		.in_len = plaintext_len,
		.out_buf = decrypted,
		.out_buf_max = plaintext_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	if (cipher_cbc_op(&ini, &encrypt, iv)) {
		LOG_ERR("CBC mode ENCRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	LOG_INF("CBC mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CBC,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	/* TinyCrypt keeps IV at the start of ciphertext buffer */
	if (cipher_cbc_op(&ini, &decrypt, iv)) {
		LOG_ERR("CBC mode DECRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, plaintext_len)) {
		LOG_ERR("CBC mode DECRYPT - Mismatch between plaintext and "
				"decrypted cipher text");
		print_buffer_comparison(plaintext, decrypt.out_buf,
					plaintext_len);
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("CBC mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void ctr_mode(const struct device *dev,
				const uint8_t *key,
				const uint8_t key_len,
				const uint8_t *iv,
				const uint8_t iv_len,
				const uint8_t *plaintext,
				const uint32_t plaintext_len,
				uint8_t *ciphertext,
				uint32_t *ciphertext_len)
{
	uint8_t *decrypted = aes_plaintext_decrypted_buf;
	__ASSERT_NO_MSG(decrypted);
	struct cipher_ctx ini = {
		.keylen = key_len,
		.key.bit_stream = key,
		.flags = cap_flags,
		/*  ivlen + ctrlen = keylen , so ctrlen is 128 - 96 = 32 bits */
		.mode_params.ctr_info.ctr_len = 32,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = plaintext_len,
		.out_buf_max = plaintext_len,
		.out_buf = ciphertext,
	};
	struct cipher_pkt decrypt = {
		.in_buf = ciphertext,
		.in_len = plaintext_len,
		.out_buf = decrypted,
		.out_buf_max = plaintext_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &encrypt, iv)) {
		LOG_ERR("CTR mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	LOG_INF("CTR mode ENCRYPT - Match");
	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CTR,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		return;
	}

	if (cipher_ctr_op(&ini, &decrypt, iv)) {
		LOG_ERR("CTR mode DECRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, plaintext_len)) {
		LOG_ERR("CTR mode DECRYPT - Mismatch between plaintext "
				"and decrypted cipher text");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, plaintext_len);
		goto out;
	}

	LOG_INF("CTR mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void ccm_mode(const struct device *dev,
				const uint8_t *key,
				const uint8_t key_len,
				const uint8_t *nonce,
				const uint8_t nonce_len,
				const uint8_t *aead,
				const uint8_t aead_len,
				const uint8_t *tag,
				const uint32_t tag_len,
				const uint8_t *plaintext,
				const uint32_t plaintext_len,
				uint8_t *ciphertext,
				uint32_t *ciphertext_len)
{
	uint8_t *decrypted = aes_plaintext_decrypted_buf;
	__ASSERT_NO_MSG(decrypted);
	struct cipher_ctx ini = {
		.keylen = key_len,
		.key.bit_stream = key,
		.mode_params.ccm_info = {
			.nonce_len = 13,
			.tag_len = tag_len,
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = plaintext_len,
		.out_buf_max = plaintext_len,
		.out_buf = ciphertext,
	};
	struct cipher_aead_pkt ccm_op = {
		.ad = aead,
		.ad_len = aead_len,
		.pkt = &encrypt,
		.tag = tag,
	};
	struct cipher_pkt decrypt = {
		.in_buf = ciphertext,
		.in_len = plaintext_len,
		.out_buf = decrypted,
		.out_buf_max = plaintext_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	ccm_op.pkt = &encrypt;
	if (cipher_ccm_op(&ini, &ccm_op, nonce)) {
		LOG_ERR("CCM mode ENCRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_CCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	ccm_op.pkt = &decrypt;
	if (cipher_ccm_op(&ini, &ccm_op, nonce)) {
		LOG_ERR("CCM mode DECRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, plaintext_len)) {
		LOG_ERR("CCM mode DECRYPT - Mismatch between plaintext "
			"and decrypted cipher text");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, plaintext_len);
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("CCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

void gcm_mode(const struct device *dev,
				const uint8_t *key,
				const uint8_t key_len,
				const uint8_t *nonce,
				const uint8_t nonce_len,
				const uint8_t *aead,
				const uint8_t aead_len,
				const uint8_t *tag,
				const uint32_t tag_len,
				const uint8_t *plaintext,
				const uint32_t plaintext_len,
				uint8_t *ciphertext,
				uint32_t *ciphertext_len)
{
	uint8_t *decrypted = aes_plaintext_decrypted_buf;
	__ASSERT_NO_MSG(decrypted);
	struct cipher_ctx ini = {
		.keylen = key_len,
		.key.bit_stream = key,
		.mode_params.gcm_info = {
			.nonce_len = nonce_len,
			.tag_len = tag_len,
		},
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = plaintext,
		.in_len = plaintext_len,
		.out_buf_max = plaintext_len,
		.out_buf = ciphertext,
	};
	struct cipher_aead_pkt gcm_op = {
		.ad = aead,
		.ad_len = aead_len,
		.pkt = &encrypt,
		/* TinyCrypt always puts the tag at the end of the ciphered
		 * text, but other library such as mbedtls might be more
		 * flexible and can take a different buffer for it.  So to
		 * make sure test passes on all backends: enforcing the tag
		 * buffer to be after the ciphered text.
		 */
		.tag = tag,
	};
	struct cipher_pkt decrypt = {
		.in_buf = ciphertext,
		.in_len = plaintext_len,
		.out_buf = decrypted,
		.out_buf_max = plaintext_len,
	};

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return;
	}

	gcm_op.pkt = &encrypt;
	if (cipher_gcm_op(&ini, &gcm_op, nonce)) {
		LOG_ERR("GCM mode ENCRYPT - Failed");
		goto out;
	}

	LOG_INF("Output length (encryption): %d", encrypt.out_len);

	cipher_free_session(dev, &ini);

	if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_GCM,
				 CRYPTO_CIPHER_OP_DECRYPT)) {
		__ASSERT_NO_MSG(0);
		return;
	}

	gcm_op.pkt = &decrypt;
	if (cipher_gcm_op(&ini, &gcm_op, nonce)) {
		LOG_ERR("GCM mode DECRYPT - Failed");
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("Output length (decryption): %d", decrypt.out_len);

	if (memcmp(decrypt.out_buf, plaintext, plaintext_len)) {
		LOG_ERR("GCM mode DECRYPT - Mismatch between plaintext "
			"and decrypted cipher text");
		print_buffer_comparison(plaintext,
					decrypt.out_buf, plaintext_len);
		__ASSERT_NO_MSG(0);
		goto out;
	}

	LOG_INF("GCM mode DECRYPT - Match");
out:
	cipher_free_session(dev, &ini);
}

inline static void gen_rand_data(uint8_t *buf, uint32_t len)
{
	while (len--) {
		*buf++ = rand();
	}
}

inline static void print_hex(const uint8_t *ptr, uint32_t len)
{
	while (len-- != 0) {
		printk("%2.2x", *ptr++);
	}
}

void test_ecb(const struct device *sw_dev, const struct device *hw_dev)
{
	uint32_t out_len_sw;
	uint32_t out_len_hw;

	printk("\n%s\n", __func__);

	printk("size: %d\n", AES_OUTPUT_SIZE_MAX);
	for(uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
		printk("test_cnt: %d\n", test_cnt);
		for(uint32_t test_plaintext_len = AES_BLOCK_SIZE_BYTE; test_plaintext_len <= AES_INPUT_SIZE_MAX; test_plaintext_len += AES_BLOCK_SIZE_BYTE) {
			printk("test_plaintext_len: %d\n", test_plaintext_len);
			for(uint32_t test_key_len = AES_KEY_SIZE_MIN_BYTE; test_key_len <= AES_KEY_SIZE_SPECIAL_MAX_BYTE; test_key_len += 8) {
				printk("test_key_len(bit): %d\n", test_key_len << 3);
				memset(aes_ciphertext_buf_sw, 0, test_plaintext_len);
				memset(aes_ciphertext_buf_hw, 0, test_plaintext_len);

				gen_rand_data(aes_plaintext_buf, test_plaintext_len);
				gen_rand_data(aes_key, test_key_len);

				printk("input: ");
				print_hex(aes_plaintext_buf, test_plaintext_len);
				printk("\n");
				printk("key: ");
				print_hex(aes_key, test_key_len);
				printk("\n");

				printk("\n -------- sw -------- \n");
				ecb_mode(sw_dev,
						aes_key,
						test_key_len,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_sw,
						&out_len_sw);
				printk(" -------- sw -------- \n");

				printk("\n -------- hw -------- \n");
				ecb_mode(hw_dev,
						aes_key,
						test_key_len,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_hw,
						&out_len_hw);
				printk(" -------- hw -------- \n");

				printk("sw out result: ");
				print_hex(aes_ciphertext_buf_sw, test_plaintext_len);
				printk("\n");

				printk("hw out result: ");
				print_hex(aes_ciphertext_buf_hw, test_plaintext_len);
				printk("\n");

				printk("out compare result: ");
				if (memcmp(aes_ciphertext_buf_sw, aes_ciphertext_buf_hw, test_plaintext_len) == 0) {
					printk("compare pass\n");
				} else {
					printk("compare fail\n");
					__ASSERT_NO_MSG(0);
				}
			}
		}
	}
}

void test_cbc(const struct device *sw_dev, const struct device *hw_dev)
{
	uint32_t out_len_sw;
	uint32_t out_len_hw;

	printk("\n%s\n", __func__);

	printk("size: %d\n", AES_OUTPUT_SIZE_MAX);
	for(uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
		printk("test_cnt: %d\n", test_cnt);
		for(uint32_t test_plaintext_len = AES_BLOCK_SIZE_BYTE; test_plaintext_len <= AES_INPUT_SIZE_MAX; test_plaintext_len+=AES_BLOCK_SIZE_BYTE) {
			printk("test_plaintext_len: %d\n", test_plaintext_len);
			for(uint32_t test_key_len = AES_KEY_SIZE_MIN_BYTE; test_key_len <= AES_KEY_SIZE_SPECIAL_MAX_BYTE; test_key_len+=8) {
				printk("test_key_len(bit): %d\n", test_key_len << 3);
				memset(aes_ciphertext_buf_sw, 0, test_plaintext_len);
				memset(aes_ciphertext_buf_hw, 0, test_plaintext_len);

				gen_rand_data(aes_plaintext_buf, test_plaintext_len);
				gen_rand_data(aes_key, test_key_len);
				gen_rand_data(aes_iv, AES_BLOCK_SIZE_BYTE);

				printk("input: ");
				print_hex(aes_plaintext_buf, test_plaintext_len);
				printk("\n");
				printk("key: ");
				print_hex(aes_key, test_key_len);
				printk("\n");
				printk("iv: ");
				print_hex(aes_iv, AES_BLOCK_SIZE_BYTE);
				printk("\n");


				printk("\n -------- sw -------- \n");
				cbc_mode(sw_dev,
						aes_key,
						test_key_len,
						aes_iv,
						16,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_sw,
						&out_len_sw);
				printk(" -------- sw -------- \n");

				printk("\n -------- hw -------- \n");
				cbc_mode(hw_dev,
						aes_key,
						test_key_len,
						aes_iv,
						16,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_hw,
						&out_len_hw);
				printk(" -------- hw -------- \n");

				printk("sw out result: ");
				print_hex(aes_ciphertext_buf_sw, test_plaintext_len);
				printk("\n");

				printk("hw out result: ");
				print_hex(aes_ciphertext_buf_hw, test_plaintext_len);
				printk("\n");

				printk("out compare result: ");
				if (memcmp(aes_ciphertext_buf_sw, aes_ciphertext_buf_hw, test_plaintext_len) == 0) {
					printk("compare pass\n");
				} else {
					printk("compare fail\n");
					__ASSERT_NO_MSG(0);
				}
			}
		}
	}
}

void test_ctr(const struct device *sw_dev, const struct device *hw_dev)
{
	uint32_t out_len_sw;
	uint32_t out_len_hw;

	printk("\n%s\n", __func__);

	printk("size: %d\n", AES_OUTPUT_SIZE_MAX);
	for(uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
		printk("test_cnt: %d\n", test_cnt);
		for(uint32_t test_plaintext_len = AES_BLOCK_SIZE_BYTE; test_plaintext_len <= AES_INPUT_SIZE_MAX; test_plaintext_len+=AES_BLOCK_SIZE_BYTE) {
			printk("test_plaintext_len: %d\n", test_plaintext_len);
			for(uint32_t test_key_len = AES_KEY_SIZE_MIN_BYTE; test_key_len <= AES_KEY_SIZE_SPECIAL_MAX_BYTE; test_key_len+=8) {
				printk("test_key_len(bit): %d\n", test_key_len << 3);
				memset(aes_ciphertext_buf_sw, 0, test_plaintext_len);
				memset(aes_ciphertext_buf_hw, 0, test_plaintext_len);

				gen_rand_data(aes_plaintext_buf, test_plaintext_len);
				gen_rand_data(aes_key, test_key_len);
				gen_rand_data(aes_iv, AES_BLOCK_SIZE_BYTE);

				printk("input: ");
				print_hex(aes_plaintext_buf, test_plaintext_len);
				printk("\n");
				printk("key: ");
				print_hex(aes_key, test_key_len);
				printk("\n");
				printk("iv: ");
				print_hex(aes_iv, AES_BLOCK_SIZE_BYTE);
				printk("\n");


				printk("\n -------- sw -------- \n");
				ctr_mode(sw_dev,
						aes_key,
						test_key_len,
						aes_iv,
						16,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_sw,
						&out_len_sw);
				printk(" -------- sw -------- \n");

				printk("\n -------- hw -------- \n");
				ctr_mode(hw_dev,
						aes_key,
						test_key_len,
						aes_iv,
						16,
						aes_plaintext_buf,
						test_plaintext_len,
						aes_ciphertext_buf_hw,
						&out_len_hw);
				printk(" -------- hw -------- \n");

				printk("sw out result: ");
				print_hex(aes_ciphertext_buf_sw, test_plaintext_len);
				printk("\n");

				printk("hw out result: ");
				print_hex(aes_ciphertext_buf_hw, test_plaintext_len);
				printk("\n");

				printk("out compare result: ");
				if (memcmp(aes_ciphertext_buf_sw, aes_ciphertext_buf_hw, test_plaintext_len) == 0) {
					printk("compare pass\n");
				} else {
					printk("compare fail\n");
					__ASSERT_NO_MSG(0);
				}
			}
		}
	}
}

void test_ccm(const struct device *sw_dev, const struct device *hw_dev)
{
	uint32_t out_len_sw;
	uint32_t out_len_hw;

	printk("\n%s\n", __func__);

	printk("size: %d\n", AES_OUTPUT_SIZE_MAX);
	for(uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
		printk("test_cnt: %d\n", test_cnt);
		for(uint32_t test_plaintext_len = AES_BLOCK_SIZE_BYTE; test_plaintext_len <= AES_INPUT_SIZE_MAX; test_plaintext_len+=AES_BLOCK_SIZE_BYTE) {
			printk("test_plaintext_len: %d\n", test_plaintext_len);
			for(uint32_t test_key_len = AES_KEY_SIZE_MIN_BYTE; test_key_len <= AES_KEY_SIZE_SPECIAL_MAX_BYTE; test_key_len+=8) {
				for(uint32_t test_aead_len = AES_AEAD_SIZE_MIN; test_aead_len <= AES_AEAD_SIZE_MAX; test_aead_len++) {
					for(uint32_t test_tag_len = AES_CCM_TAG_SIZE_MIN; test_tag_len <= AES_CCM_TAG_SIZE_MAX; test_tag_len+=AES_CCM_TAG_SIZE_STEP) {
						printk("test_key_len(bit): %d\n", test_key_len << 3);
						memset(aes_ciphertext_buf_sw, 0, test_plaintext_len);
						memset(aes_ciphertext_buf_hw, 0, test_plaintext_len);

						gen_rand_data(aes_plaintext_buf, test_plaintext_len);
						gen_rand_data(aes_key, test_key_len);
						gen_rand_data(aes_iv, AES_CCM_NONCE_SIZE_BYTE);
						gen_rand_data(aes_aead, test_aead_len);

						printk("input: ");
						print_hex(aes_plaintext_buf, test_plaintext_len);
						printk("\n");
						printk("key: ");
						print_hex(aes_key, test_key_len);
						printk("\n");
						printk("iv: ");
						print_hex(aes_iv, AES_CCM_NONCE_SIZE_BYTE);
						printk("\n");
						printk("ccm_aead: ");
						print_hex(aes_aead, test_aead_len);
						printk("\n");

						printk("key len: %d: \n", test_key_len);
						printk("aead len: %d: \n", test_aead_len);
						printk("tag len: %d: \n", test_tag_len);

						printk("\n -------- sw -------- \n");
						ccm_mode(sw_dev,
								aes_key,
								test_key_len,
								aes_iv,
								AES_CCM_NONCE_SIZE_BYTE,
								aes_aead,
								test_aead_len,
								aes_ccm_tag_sw,
								test_tag_len,
								aes_plaintext_buf,
								test_plaintext_len,
								aes_ciphertext_buf_sw,
								&out_len_sw);
						printk(" -------- sw -------- \n");

						printk("\n -------- hw -------- \n");
						ccm_mode(hw_dev,
								aes_key,
								test_key_len,
								aes_iv,
								AES_CCM_NONCE_SIZE_BYTE,
								aes_aead,
								test_aead_len,
								aes_ccm_tag_hw,
								test_tag_len,
								aes_plaintext_buf,
								test_plaintext_len,
								aes_ciphertext_buf_hw,
								&out_len_hw);
						printk(" -------- hw -------- \n");

						printk("sw out result: ");
						print_hex(aes_ciphertext_buf_sw, test_plaintext_len);
						printk("\n");

						printk("hw out result: ");
						print_hex(aes_ciphertext_buf_hw, test_plaintext_len);
						printk("\n");

						printk("out compare result: ");
						if (memcmp(aes_ciphertext_buf_sw, aes_ciphertext_buf_hw, test_plaintext_len) == 0) {
							printk("compare pass\n");
						} else {
							printk("compare fail\n");
							__ASSERT_NO_MSG(0);
						}

						printk("sw ccm tag result: ");
						print_hex(aes_ccm_tag_sw, test_tag_len);
						printk("\n");

						printk("hw ccm tag result: ");
						print_hex(aes_ccm_tag_hw, test_tag_len);
						printk("\n");

						printk("ccm tag compare result: ");
						if (memcmp(aes_ccm_tag_sw, aes_ccm_tag_hw, test_tag_len) == 0) {
							printk("compare pass\n");
						} else {
							printk("compare fail\n");
							__ASSERT_NO_MSG(0);
						}
					}
				}
			}
		}
	}
}


void test_gcm(const struct device *sw_dev, const struct device *hw_dev)
{
	uint32_t out_len_sw;
	uint32_t out_len_hw;
	uint32_t test_tag_len = AES_GCM_TAG_SIZE;

	printk("\n%s\n", __func__);

	printk("size: %d\n", AES_OUTPUT_SIZE_MAX);
	for(uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
		printk("test_cnt: %d\n", test_cnt);
		for(uint32_t test_plaintext_len = AES_BLOCK_SIZE_BYTE; test_plaintext_len <= AES_INPUT_SIZE_MAX; test_plaintext_len+=AES_BLOCK_SIZE_BYTE) {
			printk("test_plaintext_len: %d\n", test_plaintext_len);
			for(uint32_t test_key_len = AES_KEY_SIZE_MIN_BYTE; test_key_len <= AES_KEY_SIZE_SPECIAL_MAX_BYTE; test_key_len+=8) {
				for(uint32_t test_aead_len = AES_AEAD_SIZE_MIN; test_aead_len <= AES_AEAD_SIZE_MAX; test_aead_len++) {
						printk("test_key_len(bit): %d\n", test_key_len << 3);
						memset(aes_ciphertext_buf_sw, 0, test_plaintext_len);
						memset(aes_ciphertext_buf_hw, 0, test_plaintext_len);

						gen_rand_data(aes_plaintext_buf, test_plaintext_len);
						gen_rand_data(aes_key, test_key_len);
						gen_rand_data(aes_iv, AES_GCM_NONCE_SIZE_BYTE);
						gen_rand_data(aes_aead, test_aead_len);

						printk("input: ");
						print_hex(aes_plaintext_buf, test_plaintext_len);
						printk("\n");
						printk("key: ");
						print_hex(aes_key, test_key_len);
						printk("\n");
						printk("iv: ");
						print_hex(aes_iv, AES_GCM_NONCE_SIZE_BYTE);
						printk("\n");
						printk("gcm_aead: ");
						print_hex(aes_aead, test_aead_len);
						printk("\n");

						printk("key len: %d: \n", test_key_len);
						printk("aead len: %d: \n", test_aead_len);
						printk("tag len: %d: \n", test_tag_len);

						printk("\n -------- sw -------- \n");
						gcm_mode(sw_dev,
								aes_key,
								test_key_len,
								aes_iv,
								AES_GCM_NONCE_SIZE_BYTE,
								aes_aead,
								test_aead_len,
								aes_gcm_tag_sw,
								test_tag_len,
								aes_plaintext_buf,
								test_plaintext_len,
								aes_ciphertext_buf_sw,
								&out_len_sw);
						printk(" -------- sw -------- \n");

						printk("\n -------- hw -------- \n");
						gcm_mode(hw_dev,
								aes_key,
								test_key_len,
								aes_iv,
								AES_GCM_NONCE_SIZE_BYTE,
								aes_aead,
								test_aead_len,
								aes_gcm_tag_hw,
								test_tag_len,
								aes_plaintext_buf,
								test_plaintext_len,
								aes_ciphertext_buf_hw,
								&out_len_hw);
						printk(" -------- hw -------- \n");

						printk("sw out result: ");
						print_hex(aes_ciphertext_buf_sw, test_plaintext_len);
						printk("\n");

						printk("hw out result: ");
						print_hex(aes_ciphertext_buf_hw, test_plaintext_len);
						printk("\n");

						printk("out compare result: ");
						if (memcmp(aes_ciphertext_buf_sw, aes_ciphertext_buf_hw, test_plaintext_len) == 0) {
							printk("compare pass\n");
						} else {
							printk("compare fail\n");
							__ASSERT_NO_MSG(0);
						}

						printk("sw gcm tag result: ");
						print_hex(aes_gcm_tag_sw, test_tag_len);
						printk("\n");

						printk("hw gcm tag result: ");
						print_hex(aes_gcm_tag_hw, test_tag_len);
						printk("\n");

						printk("gcm tag compare result: ");
						if (memcmp(aes_gcm_tag_sw, aes_gcm_tag_hw, test_tag_len) == 0) {
							printk("compare pass\n");
						} else {
							printk("compare fail\n");
							__ASSERT_NO_MSG(0);
						}
				}
			}
		}
	}
}

int main(void)
{
	const struct device *sw_dev = device_get_binding(CRYPTO_DRV_NAME);
	if (!sw_dev) {
		LOG_ERR("%s pseudo device not found", CRYPTO_DRV_NAME);
		return 0;
	}
	const struct device *const hw_dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
	if (!device_is_ready(hw_dev)) {
		LOG_ERR("Crypto device is not ready\n");
		return 0;
	}

	if (validate_hw_compatibility(hw_dev)) {
		LOG_ERR("Incompatible h/w");
		return 0;
	}

	LOG_INF("Cipher Sample");

	test_ecb(sw_dev, hw_dev);
	test_cbc(sw_dev, hw_dev);
	test_ctr(sw_dev, hw_dev);
	test_ccm(sw_dev, hw_dev);
	test_gcm(sw_dev, hw_dev);

	return 0;
}
