/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define LOG_LEVEL CONFIG_SHA512_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#if DT_HAS_COMPAT_STATUS_OKAY(linkedsemi_sha512)
#define SHA512_DEV_COMPAT linkedsemi_sha512
#else
#error "You need to enable one crypto device"
#endif

static uint8_t result[0x40];

static uint32_t msg[]={
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
	0x31313131, 0x31313131, 0x31313131, 0x31313131, 
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

static void sha384_test(const struct device *device, size_t len)
{
	struct hash_ctx ctx;
	struct hash_pkt pkt = {
		.in_buf = (uint8_t *)msg,
		.in_len = len,
		.out_buf = result,
	};

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	if (hash_begin_session(device, &ctx, CRYPTO_HASH_ALGO_SHA384)) {
		LOG_ERR("Failed to init sha384 session\n");
		return;
	}

	if (hash_update(&ctx, &pkt)) {
		LOG_ERR("SHA384 UPDATE ERROR");
		goto done;
	}
	LOG_INF("SHA384 UPDATE %d bytes 0x31 Finish...", len);

	if (hash_compute(&ctx, &pkt)) {
		LOG_ERR("SHA384 CALC ERROR");
		goto done;
	}

	LOG_INF("SHA384 CALC %d bytes 0x31 Finish...", len * 2);
	print_array(result, 0x30);

done:
	hash_free_session(device, &ctx);
}

static void sha512_test(const struct device *device, size_t len)
{
	struct hash_ctx ctx;
	struct hash_pkt pkt = {
		.in_buf = (uint8_t *)msg,
		.in_len = len,
		.out_buf = result,
	};

	ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;
	if (hash_begin_session(device, &ctx, CRYPTO_HASH_ALGO_SHA512)) {
		LOG_ERR("Failed to init sha512 session\n");
		return;
	}

	if (hash_update(&ctx, &pkt)) {
		LOG_ERR("SHA512 UPDATE ERROR");
		goto done;
	}
	LOG_INF("SHA512 UPDATE %d bytes 0x31 Finish...", len);

	if (hash_compute(&ctx, &pkt)) {
		LOG_ERR("SHA512 CALC ERROR");
		goto done;
	}

	LOG_INF("SHA512 CALC %d bytes 0x31 Finish...", len * 2);
	print_array(result, 0x40);

done:
	hash_free_session(device, &ctx);
}

int main(void)
{
	LOG_INF("SHA512 Sample");

	const struct device *const dev = DEVICE_DT_GET_ONE(SHA512_DEV_COMPAT);
	if (!device_is_ready(dev)) {
		LOG_ERR("SHA512 device is not ready\n");
		return 0;
	}

	sha512_test(dev, 32);
	sha512_test(dev, 64);
	sha512_test(dev, 128);
	
	sha384_test(dev, 32);
	sha384_test(dev, 64);
	sha384_test(dev, 128);

	return 0;
}
