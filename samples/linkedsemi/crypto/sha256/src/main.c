/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define LOG_LEVEL CONFIG_SHA256_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#if DT_HAS_COMPAT_STATUS_OKAY(linkedsemi_sha256)
#define SHA256_DEV_COMPAT linkedsemi_sha256
#else
#error "You need to enable one crypto device"
#endif

static uint8_t result[0x20];

static uint8_t msg[]={
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
	0x31, 0x32, 0x33, 
};

static void print_array(uint8_t *ptr, uint32_t size)
{
	int i, j;

	for (i = 0, j = 1; i < size; i++, j++) {
		printk("0x%02x ", ptr[i]);

		if (j == 8) {
			j = 0;
			printk("\n");
		}
	}
}

static void sha224_test(const struct device *device)
{
	struct hash_ctx ctx;
	struct hash_pkt pkt = {
		.in_buf = msg,
		.in_len = sizeof(msg),
		.out_buf = result,
	};

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	if (hash_begin_session(device, &ctx, CRYPTO_HASH_ALGO_SHA224)) {
		LOG_ERR("Failed to init sha256 session\n");
		return;
	}
	
	hash_compute(&ctx, &pkt);

	LOG_INF("SHA224 calculate finish, hash result: ");
	print_array(result, 0x1C);
}

static void sha256_test(const struct device *device)
{
	struct hash_ctx ctx;
	struct hash_pkt pkt = {
		.in_buf = msg,
		.in_len = sizeof(msg),
		.out_buf = result,
	};

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	if (hash_begin_session(device, &ctx, CRYPTO_HASH_ALGO_SHA256)) {
		LOG_ERR("Failed to init sha256 session\n");
		return;
	}
	
	hash_compute(&ctx, &pkt);

	LOG_INF("SHA256 calculate finish, hash result: ");
	print_array(result, 0x20);
}

static void sm3_test(const struct device *device)
{
	struct hash_ctx ctx;
	struct hash_pkt pkt = {
		.in_buf = msg,
		.in_len = sizeof(msg),
		.out_buf = result,
	};

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	if (hash_begin_session(device, &ctx, CRYPTO_HASH_ALGO_SM3)) {
		LOG_ERR("Failed to init sha256 session\n");
		return;
	}
	
	hash_compute(&ctx, &pkt);

	LOG_INF("SM3 calculate finish, hash result: ");
	print_array(result, 0x20);
}

int main(void)
{
	LOG_INF("SHA256 Sample");

	const struct device *const dev = DEVICE_DT_GET_ONE(SHA256_DEV_COMPAT);
	if (!device_is_ready(dev)) {
		LOG_ERR("SHA256 device is not ready\n");
		return 0;
	}

	sha224_test(dev);

	sha256_test(dev);

	sm3_test(dev);

	return 0;
}
