/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include <zephyr/ztest.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <string.h>

#define CRYPTO_DRV_NAME   CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#define CRYPTO_DEV_COMPAT linkedsemi_sha256

#define SHA_INPUT_SIZE_MAX  10240
#define SHA_INPUT_SIZE_MIN  0
#define SHA_OUTPUT_SIZE_MAX 64

#define CNT_MAX_TEST (2)

static uint8_t sha_plaintext_buf[SHA_INPUT_SIZE_MAX];
static uint8_t sha_hash_buf_sw[SHA_OUTPUT_SIZE_MAX];
static uint8_t sha_hash_buf_hw[SHA_OUTPUT_SIZE_MAX];

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

void sha256_mode(const struct device *dev,
                 const uint8_t *in_buf,
                 const int in_len,
                 const uint8_t *out_buf,
                 int *out_len)
{
    int ret;
    struct hash_ctx ctx;

    ctx.flags = CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

    ret = hash_begin_session(dev, &ctx, CRYPTO_HASH_ALGO_SHA256);
    __ASSERT(ret == 0, "Failed to init sha256 session");

    do {
        // int idx = 0;
        int remain = in_len;
        const int step = 1024;
        const int unalign_block_len = in_len % step;
        if (unalign_block_len > 0) {
            for (int i = 0; i < in_len / step; i++) {
                struct hash_pkt pkt = {
                    .in_buf = in_buf + i * step,
                    .in_len = step,
                    .out_buf = out_buf,
                };
                ret = hash_update(&ctx, &pkt);
                remain -= step;
            }
            if (unalign_block_len != 0) {
                struct hash_pkt pkt = {
                    .in_buf = in_buf + in_len - unalign_block_len,
                    .in_len = unalign_block_len,
                    .out_buf = out_buf,
                };
                ret = hash_compute(&ctx, &pkt);
                remain -= unalign_block_len;
            }
        } else {
            for (int i = 0; i < (in_len / step) - 1; i++) {
                struct hash_pkt pkt = {
                    .in_buf = in_buf + i * step,
                    .in_len = step,
                    .out_buf = out_buf,
                };
                ret = hash_update(&ctx, &pkt);
                remain -= step;
            }
            struct hash_pkt pkt = {
                .in_buf = in_buf + in_len - remain,
                .in_len = remain,
                .out_buf = out_buf,
            };
            ret = hash_compute(&ctx, &pkt);
        }
        printk("test result: ");
        for (uint8_t i = 0; i < 32; i++) {
            printk("%2.2x", out_buf[i]);
        };
        printk("\n");
        __ASSERT(ret == 0, "Failed to compute hash for test");
        // ret = memcmp(pkt.out_buf, sha256_results[9 - 1], 32);
        // __ASSERT(ret == 0, "Failed to compute hash for test");
        // printk("test match\n");
    } while (0);

    hash_free_session(dev, &ctx);
}

void test_sha256(const struct device *sw_dev, const struct device *hw_dev)
{
    uint32_t out_len_sw;
    uint32_t out_len_hw;

    printk("\n%s\n", __func__);

    printk("size: %d\n", SHA_OUTPUT_SIZE_MAX);
    for (uint32_t test_cnt = 0; test_cnt <= CNT_MAX_TEST; test_cnt++) {
        printk("test_cnt: %d\n", test_cnt);
        for (uint32_t test_plaintext_len = SHA_INPUT_SIZE_MIN; test_plaintext_len <= SHA_INPUT_SIZE_MAX; test_plaintext_len++) {
            printk("test_plaintext_len: %d\n", test_plaintext_len);
            memset(sha_hash_buf_sw, 0, test_plaintext_len);
            memset(sha_hash_buf_hw, 0, test_plaintext_len);

            if (test_plaintext_len > 0) {
                gen_rand_data(sha_plaintext_buf, test_plaintext_len);
            }

            printk("input: ");
            print_hex(sha_plaintext_buf, test_plaintext_len);
            printk("\n");

            printk("\n -------- sw -------- \n");
            sha256_mode(sw_dev,
                        sha_plaintext_buf,
                        test_plaintext_len,
                        sha_hash_buf_sw,
                        &out_len_sw);
            printk(" -------- sw -------- \n");

            printk("\n -------- hw -------- \n");
            sha256_mode(hw_dev,
                        sha_plaintext_buf,
                        test_plaintext_len,
                        sha_hash_buf_hw,
                        &out_len_hw);
            printk(" -------- hw -------- \n");

            printk("sw out result: ");
            print_hex(sha_hash_buf_sw, test_plaintext_len);
            printk("\n");

            printk("hw out result: ");
            print_hex(sha_hash_buf_hw, test_plaintext_len);
            printk("\n");

            printk("out compare result: ");
            if (memcmp(sha_hash_buf_sw, sha_hash_buf_hw, 32) == 0) {
                printk("compare pass\n");
            } else {
                printk("compare fail\n");
                __ASSERT_NO_MSG(0);
            }
        }
    }
}

int main(void)
{
    const struct device *sw_dev = device_get_binding(CRYPTO_DRV_NAME);
    if (!sw_dev) {
        printk("%s pseudo device not found", CRYPTO_DRV_NAME);
        return 0;
    }
    const struct device *const hw_dev = DEVICE_DT_GET_ONE(CRYPTO_DEV_COMPAT);
    if (!device_is_ready(hw_dev)) {
        printk("Crypto device is not ready\n");
        return 0;
    }

    // if (validate_hw_compatibility(hw_dev)) {
    //     printk("Incompatible h/w");
    //     return 0;
    // }

    printk("hash Sample");

    test_sha256(sw_dev, hw_dev);
    // test_gcm(sw_dev, hw_dev);

    return 0;
}
