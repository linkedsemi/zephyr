

/*
 * SHA256 TEST:
 * The software interface of wolfssl and the hardware interface adapted by linkedsemi on mbedtls are used for comparison
 * The test data ranges from 0bytes to 10,240 bytes
 */

#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>
#include <wolfssl/wolfcrypt/sha256.h>
#include <wolfssl/wolfcrypt/sm3.h>
#include "mbedtls/sha256.h"

typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;

#define TEST_TOTAL_SIZE KB(10)
//分片大小
#define TEST_STEP_SIZE KB(1)
__attribute__((aligned(32))) uint8_t sha_plaintext_buf[TEST_TOTAL_SIZE];
uint8_t sw_hash256_result[WC_SHA256_DIGEST_SIZE];
uint8_t hw_hash256_result[WC_SHA256_DIGEST_SIZE];
uint8_t sw_hash224_result[WC_SHA224_DIGEST_SIZE];
uint8_t hw_hash224_result[WC_SHA224_DIGEST_SIZE];
uint8_t sw_sm3_result[WC_SM3_DIGEST_SIZE];
uint8_t hw_sm3_result[WC_SM3_DIGEST_SIZE];

inline static void print_hex(const uint8_t *ptr, uint32_t len)
{
    while (len-- != 0) {
        printk("%2.2x", *ptr++);
    }
}

int sw_sha256_test(wc_Sha256* sha256, const byte* plaintext, word32 plaintext_len)
{
    int ret = 0;
    const byte* in_buf = NULL;
    word32 in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;
    
    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if((ret = wc_Sha256Update(sha256, in_buf, in_len)) != 0){
            __ASSERT_NO_MSG(0);
        }
    }

    if((ret = wc_Sha256Final(sha256, sw_hash256_result)) !=0){
        __ASSERT_NO_MSG(0);
    }

    wc_Sha256Free(sha256);

    return ret;
}

int hw_sha256_test(mbedtls_sha256_context *sha, const unsigned char *plaintext, size_t plaintext_len)
{
    int ret = 0;
    bool is224 = false;
    const unsigned char * in_buf = NULL;
    size_t in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;

    if ((ret = mbedtls_sha256_starts_dma(sha, is224)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if ((ret = mbedtls_sha256_update_dma(sha, in_buf, in_len)) != 0) {
        __ASSERT_NO_MSG(0);
        }
    }

    if ((ret = mbedtls_sha256_finish_dma(sha, hw_hash256_result)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    mbedtls_sha256_free(sha);

    return 0;
}

int sha256_test(void)
{
    wc_Sha256 sw_sha;
    mbedtls_sha256_context hw_sha;

    for(uint32_t test_plaintext_len = 0; test_plaintext_len <= TEST_TOTAL_SIZE; test_plaintext_len++)
    {
        printk("test_plaintext_len: %d\n", test_plaintext_len);
        memset(sw_hash256_result, 0, WC_SHA256_DIGEST_SIZE);
        memset(hw_hash256_result, 0, WC_SHA256_DIGEST_SIZE);

        if(test_plaintext_len > 0)
        {
            memset(sha_plaintext_buf, 2, test_plaintext_len);
        }

        // printk("input: ");
        // print_hex(sha_plaintext_buf, test_plaintext_len);
        // printk("\n");

        wc_InitSha256_ex(&sw_sha, NULL, NULL);

        sw_sha256_test(&sw_sha, (byte*)sha_plaintext_buf, (word32)test_plaintext_len);

        mbedtls_sha256_init_dma(&hw_sha);

        hw_sha256_test(&hw_sha, (char*)sha_plaintext_buf, (size_t)test_plaintext_len);

        printk("sw out result: ");
        print_hex(sw_hash256_result, WC_SHA256_DIGEST_SIZE);
        printk("\n");

        printk("hw out result: ");
        print_hex(hw_hash256_result, WC_SHA256_DIGEST_SIZE);
        printk("\n");

        printk("out compare result: ");
        if (memcmp(sw_hash256_result, hw_hash256_result, WC_SHA256_DIGEST_SIZE) == 0) {
            printk("compare pass\n");
        } else {
            printk("compare fail\n");
            __ASSERT_NO_MSG(0);
        }
    }
    return 0;
}

int sw_sha224_test(wc_Sha224* sha224, const byte* plaintext, word32 plaintext_len)
{
    int ret = 0;
    const byte* in_buf = NULL;
    word32 in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;
    
    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if((ret = wc_Sha224Update(sha224, in_buf, in_len)) != 0){
            __ASSERT_NO_MSG(0);
        }
    }

    if((ret = wc_Sha224Final(sha224, sw_hash224_result)) !=0){
        __ASSERT_NO_MSG(0);
    }

    wc_Sha224Free(sha224);

    return ret;
}

int hw_sha224_test(mbedtls_sha256_context *sha, const unsigned char *plaintext, size_t plaintext_len)
{
    int ret = 0;
    bool is224 = true;
    const unsigned char * in_buf = NULL;
    size_t in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;

    if ((ret = mbedtls_sha256_starts_dma(sha, is224)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if ((ret = mbedtls_sha256_update_dma(sha, in_buf, in_len)) != 0) {
        __ASSERT_NO_MSG(0);
        }
    }

    if ((ret = mbedtls_sha256_finish_dma(sha, hw_hash224_result)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    mbedtls_sha256_free(sha);

    return 0;
}

int sha224_test(void)
{
    wc_Sha224 sw_sha;
    mbedtls_sha256_context hw_sha;

    for(uint32_t test_plaintext_len = 0; test_plaintext_len <= TEST_TOTAL_SIZE; test_plaintext_len++)
    {
        printk("test_plaintext_len: %d\n", test_plaintext_len);
        memset(sw_hash224_result, 0, WC_SHA224_DIGEST_SIZE);
        memset(hw_hash224_result, 0, WC_SHA224_DIGEST_SIZE);

        if(test_plaintext_len > 0)
        {
            memset(sha_plaintext_buf, 2, test_plaintext_len);
        }

        wc_InitSha224_ex(&sw_sha, NULL, NULL);

        sw_sha224_test(&sw_sha, (byte*)sha_plaintext_buf, (word32)test_plaintext_len);

        mbedtls_sha256_init_dma(&hw_sha);

        hw_sha224_test(&hw_sha, (char*)sha_plaintext_buf, (size_t)test_plaintext_len);

        printk("sw out result: ");
        print_hex(sw_hash224_result, WC_SHA224_DIGEST_SIZE);
        printk("\n");

        printk("hw out result: ");
        print_hex(hw_hash224_result, WC_SHA224_DIGEST_SIZE);
        printk("\n");

        printk("out compare result: ");
        if (memcmp(sw_hash224_result, hw_hash224_result, WC_SHA224_DIGEST_SIZE) == 0) {
            printk("compare pass\n");
        } else {
            printk("compare fail\n");
            __ASSERT_NO_MSG(0);
        }
    }
    return 0;
}

int sw_sm3_test(wc_Sm3* sm3, const byte* plaintext, word32 plaintext_len)
{
    int ret = 0;
    const byte* in_buf = NULL;
    word32 in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;
    
    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if((ret = wc_Sm3Update(sm3, in_buf, in_len)) != 0){
            __ASSERT_NO_MSG(0);
        }
    }

    if((ret = wc_Sm3Final(sm3, sw_sm3_result)) !=0){
        __ASSERT_NO_MSG(0);
    }

    wc_Sm3Free(sm3);

    return ret;
}

int hw_sm3_test(mbedtls_sha256_context *sha, const unsigned char *plaintext, size_t plaintext_len)
{
    int ret = 0;
    const unsigned char * in_buf = NULL;
    size_t in_len;
    uint32_t trans_count = plaintext_len / TEST_STEP_SIZE;

    if ((ret = mbedtls_sm3_starts_dma(sha)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    for(int i = 0; i <= trans_count; i++) {
        if(trans_count == 0)
        {
            in_buf = plaintext;
            in_len = plaintext_len;
        }
        else if((i == trans_count) && (trans_count > 0)){
            in_buf = plaintext + TEST_STEP_SIZE * trans_count;
            in_len = plaintext_len - TEST_STEP_SIZE * trans_count;
        }
        else{
            in_buf = plaintext + TEST_STEP_SIZE * i;
            in_len = TEST_STEP_SIZE;
        }

        if ((ret = mbedtls_sm3_update_dma(sha, in_buf, in_len)) != 0) {
        __ASSERT_NO_MSG(0);
        }
    }

    if ((ret = mbedtls_sm3_finish_dma(sha, hw_sm3_result)) != 0) {
        __ASSERT_NO_MSG(0);
    }

    mbedtls_sm3_free(sha);

    return 0;
}

int sm3_test(void)
{
    wc_Sm3 sw_sha;
    mbedtls_sha256_context hw_sha;

    for(uint32_t test_plaintext_len = 0; test_plaintext_len <= TEST_TOTAL_SIZE; test_plaintext_len++)
    {
        printk("test_plaintext_len: %d\n", test_plaintext_len);
        memset(sw_sm3_result, 0, WC_SM3_DIGEST_SIZE);
        memset(hw_sm3_result, 0, WC_SM3_DIGEST_SIZE);

        if(test_plaintext_len > 0)
        {
            memset(sha_plaintext_buf, 2, test_plaintext_len);
        }

        wc_InitSm3(&sw_sha, NULL, NULL);

        sw_sm3_test(&sw_sha, (byte*)sha_plaintext_buf, (word32)test_plaintext_len);

        mbedtls_sm3_init_dma(&hw_sha);

        hw_sm3_test(&hw_sha, (char*)sha_plaintext_buf, (size_t)test_plaintext_len);

        printk("sw out result: ");
        print_hex(sw_sm3_result, WC_SM3_DIGEST_SIZE);
        printk("\n");

        printk("hw out result: ");
        print_hex(hw_sm3_result, WC_SM3_DIGEST_SIZE);
        printk("\n");

        printk("out compare result: ");
        if (memcmp(sw_sm3_result, hw_sm3_result, WC_SM3_DIGEST_SIZE) == 0) {
            printk("compare pass\n");
        } else {
            printk("compare fail\n");
            __ASSERT_NO_MSG(0);
        }
    }
    return 0;
}

int main(void)
{
    int ret;

    if ( (ret = sha224_test()) != 0)
        printk("SHA-224  test failed!\n");
    else
        printk("SHA-224  test passed!\n");

    if ( (ret = sha256_test()) != 0)
        printk("SHA-256  test failed!\n");
    else
        printk("SHA-256  test passed!\n");

    if ( (ret = sm3_test()) != 0)
        printk("SM3  test failed!\n");
    else
        printk("SM3  test passed!\n");

    return 0;
}