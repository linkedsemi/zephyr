#include <string.h>
#include <stdio.h>
#include "mbedtls/sha256.h"
#include "mbedtls/aes.h"

#define SHA224_DIGEST_SIZE 28
#define SHA256_DIGEST_SIZE 32

#define MTLS_AES_192
#define MTLS_AES_256
#define AES_BLOCK_SIZE 16

typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;

int test_sha224()
{
    int ret = 0;
    mbedtls_sha256_context sha[3];
    bool is224 = true;

    uint8_t      hash[SHA224_DIGEST_SIZE];

    testVector a, b, c;
    testVector test_sha[3];
    int times = sizeof(test_sha) / sizeof(struct testVector), i, j;

    a.input  = "";
    a.output = "\xd1\x4a\x02\x8c\x2a\x3a\x2b\xc9\x47\x61\x02\xbb\x28\x82\x34"
               "\xc4\x15\xa2\xb0\x1f\x82\x8e\xa6\x2a\xc5\xb3\xe4\x2f";
    a.inLen  = strlen(a.input);
    a.outLen = SHA224_DIGEST_SIZE;

    b.input  = "abc";
    b.output = "\x23\x09\x7d\x22\x34\x05\xd8\x22\x86\x42\xa4\x77\xbd\xa2\x55"
               "\xb3\x2a\xad\xbc\xe4\xbd\xa0\xb3\xf7\xe3\x6c\x9d\xa7";
    b.inLen  = strlen(b.input);
    b.outLen = SHA224_DIGEST_SIZE;

    c.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    c.output = "\x75\x38\x8b\x16\x51\x27\x76\xcc\x5d\xba\x5d\xa1\xfd\x89\x01"
               "\x50\xb0\xc6\x45\x5c\xb4\xf5\x8b\x19\x52\x52\x25\x25";
    c.inLen  = strlen(c.input);
    c.outLen = SHA224_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;
    test_sha[2] = c;

    for (i = 0; i < times; ++i) {
        
        mbedtls_sha256_init(&sha[i]);

        if ((ret = mbedtls_sha256_starts(&sha[i], is224)) != 0) {

            goto exit;
        }

        if ((ret = mbedtls_sha256_update(&sha[i], (char*)test_sha[i].input, (size_t)test_sha[i].inLen)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sha256_finish(&sha[i], hash)) != 0) {
            goto exit;
        }

        if (memcmp(hash, test_sha[i].output, SHA256_DIGEST_SIZE) != 0) {
            goto exit;
        }
    }

exit:
    for(j = 0; j < i; ++j)
    {
        mbedtls_sha256_free(&sha[j]);
    }
    return ret;
}

int test_sha256()
{
    int ret = 0;
    mbedtls_sha256_context sha[4];
    bool is224 = false;

    uint8_t      hash[SHA256_DIGEST_SIZE];

    testVector a, b, c, d;
    testVector test_sha[4];

    int times = sizeof(test_sha) / sizeof(struct testVector), i, j;

    a.input  = "";
    a.output = "\xe3\xb0\xc4\x42\x98\xfc\x1c\x14\x9a\xfb\xf4\xc8\x99\x6f\xb9"
               "\x24\x27\xae\x41\xe4\x64\x9b\x93\x4c\xa4\x95\x99\x1b\x78\x52"
               "\xb8\x55";
    a.inLen  = strlen(a.input);
    a.outLen = SHA256_DIGEST_SIZE;

    b.input  = "abc";
    b.output = "\xBA\x78\x16\xBF\x8F\x01\xCF\xEA\x41\x41\x40\xDE\x5D\xAE\x22"
               "\x23\xB0\x03\x61\xA3\x96\x17\x7A\x9C\xB4\x10\xFF\x61\xF2\x00"
               "\x15\xAD";
    b.inLen  = strlen(b.input);
    b.outLen = SHA256_DIGEST_SIZE;

    c.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    c.output = "\x24\x8D\x6A\x61\xD2\x06\x38\xB8\xE5\xC0\x26\x93\x0C\x3E\x60"
               "\x39\xA3\x3C\xE4\x59\x64\xFF\x21\x67\xF6\xEC\xED\xD4\x19\xDB"
               "\x06\xC1";
    c.inLen  = strlen(c.input);
    c.outLen = SHA256_DIGEST_SIZE;

    d.input  = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
               "aaaaaa"; /* this is BLOCKSIZE length */
    d.output = "\xFF\xE0\x54\xFE\x7A\xE0\xCB\x6D\xC6\x5C\x3A\xF9\xB6\x1D\x52"
               "\x09\xF4\x39\x85\x1D\xB4\x3D\x0B\xA5\x99\x73\x37\xDF\x15\x46"
               "\x68\xEB";
    d.inLen  = strlen(d.input);
    d.outLen = SHA256_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;
    test_sha[2] = c;
    test_sha[3] = d;

    for (i = 0; i < times; ++i) {
        
        mbedtls_sha256_init(&sha[i]);

        if ((ret = mbedtls_sha256_starts(&sha[i], is224)) != 0) {

            goto exit;
        }

        if ((ret = mbedtls_sha256_update(&sha[i], (char*)test_sha[i].input, (size_t)test_sha[i].inLen)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sha256_finish(&sha[i], hash)) != 0) {
            goto exit;
        }

        if (memcmp(hash, test_sha[i].output, SHA256_DIGEST_SIZE) != 0) {
            goto exit;
        }
    }

exit:
    for(j = 0; j < i; ++j)
    {
        mbedtls_sha256_free(&sha[j]);
    }
    return ret;
}

static int aes_ecb_test()
{
    mbedtls_aes_context aes;
    uint8_t cipher[AES_BLOCK_SIZE];
    uint8_t plain [AES_BLOCK_SIZE];
    int ret = 0;

    const uint32_t key_128[] = {
    0x63646566,
    0x38396162,
    0x34353637,
    0x30313233};
#ifdef MTLS_AES_192
    const uint32_t key_192[] = {
    0x34353637,
    0x30313233,
    0x63646566,
    0x38396162,
    0x34353637,
    0x30313233};
#endif
#ifdef MTLS_AES_256
    const uint32_t key_256[] = {
    0x63646566,
    0x38396162,
    0x34353637,
    0x30313233,
    0x63646566,
    0x38396162,
    0x34353637,
    0x30313233};
#endif
    const uint32_t iv[] = {
    0x63646566,
    0x39306162,
    0x35363738,
    0x31323334};

    const uint8_t msg[] = {
        0x6e, 0x6f, 0x77, 0x20, 0x69, 0x73, 0x20, 0x74,
        0x68, 0x65, 0x20, 0x74, 0x69, 0x6d, 0x65, 0x20
    };
    const uint8_t verify_ecb_128[AES_BLOCK_SIZE] = {
        0xd0, 0xc9, 0xd9, 0xc9, 0x40, 0xe8, 0x97, 0xb6,
        0xc8, 0x8c, 0x33, 0x3b, 0xb5, 0x8f, 0x85, 0xd1
    };
#ifdef MTLS_AES_192
    const uint8_t verify_ecb_192[AES_BLOCK_SIZE] = {
        0x06, 0x57, 0xee, 0x78, 0x3f, 0x96, 0x00, 0xb1,
        0xec, 0x76, 0x94, 0x30, 0x29, 0xbe, 0x15, 0xab
    };
#endif
#ifdef MTLS_AES_256
    const uint8_t verify_ecb_256[AES_BLOCK_SIZE] = {
        0xcd, 0xf2, 0x81, 0x3e, 0x73, 0x3e, 0xf7, 0x33,
        0x3d, 0x18, 0xfd, 0x41, 0x85, 0x37, 0x04, 0x82
    };
    const uint32_t niKey[] = {
    0x0914dff4,
    0x2d9810a3,
    0x3b6108d7,
    0x1f352c07,
    0x857d7781,
    0x2b73aef0,
    0x15ca71be,
    0x603deb10};
    const uint8_t niPlain[] = {
        0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
        0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a
    };
    const uint8_t niCipher[] = {
        0xf3,0xee,0xd1,0xbd,0xb5,0xd2,0xa0,0x3c,
        0x06,0x4b,0x5a,0x7e,0x3d,0xb1,0x81,0xf8
    };
#endif

    int i;
    struct {
        const uint32_t* key;
        int         keySz;
        const uint32_t* iv; /* null uses 0's */
        const uint8_t* plain;
        const uint8_t* verify;
    } testVec[] = {
        { key_128, 16, iv,   msg,     verify_ecb_128 },
#ifdef MTLS_AES_192
        { key_192, 24, iv,   msg,     verify_ecb_192 },
#endif
#ifdef MTLS_AES_256
        { key_256, 32, iv,   msg,     verify_ecb_256 },
        { niKey,   32, NULL, niPlain, niCipher }
#endif
    };
    #define AES_ECB_TEST_LEN (int)(sizeof(testVec) / sizeof(*testVec))

    for (i = 0; i < AES_ECB_TEST_LEN; i++) {

        mbedtls_aes_init(&aes);

        if((ret = mbedtls_aes_setkey_enc(&aes, (char *)testVec[i].key, testVec[i].keySz)) != 0 ) {
            goto exit;
        }

        memset(cipher, 0, AES_BLOCK_SIZE);
        if((ret = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, testVec[i].plain, cipher)) !=0 ) {
            goto exit;
        }
        if ((ret = memcmp(cipher, testVec[i].verify, AES_BLOCK_SIZE)) !=0 ) {
            goto exit;
        }

        memset(plain, 0, AES_BLOCK_SIZE);
        if((ret = mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, cipher, plain)) != 0) {
            goto exit;
        }
        if ((ret = memcmp(plain, testVec[i].plain, AES_BLOCK_SIZE)) !=0 ) {
            goto exit;
        }
    }

exit:
    return ret;
}

static int aes_cbc_test()
{
    mbedtls_aes_context aes;
    uint8_t cipher[AES_BLOCK_SIZE];
    uint8_t plain [AES_BLOCK_SIZE];
    int ret = 0;

    static const uint8_t msg[] = {
        0x6e,0x6f,0x77,0x20,0x69,0x73,0x20,0x74,
        0x68,0x65,0x20,0x74,0x69,0x6d,0x65,0x20
    };
    static const uint8_t verify[] =
    {
        0x95,0x94,0x92,0x57,0x5f,0x42,0x81,0x53,
        0x2c,0xcc,0x9d,0x46,0x77,0xa2,0x33,0xcb
    };
    static const uint32_t key[] = {
    0x63646566,
    0x38396162,
    0x34353637,
    0x30313233};
    static const uint32_t iv[] = {
    0x63646566,
    0x39306162,
    0x35363738,
    0x31323334};

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)key, AES_BLOCK_SIZE)) != 0) {
        goto exit;
    }

    memset(cipher, 0, sizeof(cipher));
    if((ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, AES_BLOCK_SIZE, (char *)iv, msg, cipher)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(cipher, verify, AES_BLOCK_SIZE)) != 0) {
       goto exit;
    }

    memset(plain, 0, sizeof(plain));
    if((ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, AES_BLOCK_SIZE, (char *)iv, cipher, plain)) != 0) {
        goto exit;
    }
    if (memcmp(plain, msg, AES_BLOCK_SIZE)) {
        goto exit;
    }
exit:
    return ret;
}

int aes_test()
{
    int ret = 0;
    if((ret = aes_ecb_test()) !=0)
    {
        printf("aes_ecb_test  test failed!\n");
    }else{
        printf("aes_ecb_test  test passed!\n");
    }

    if((ret = aes_cbc_test()) !=0)
    {
        printf("aes_cbc_test  test failed!\n");
    }else{
        printf("aes_cbc_test  test passed!\n");
    }

    return ret;
}

int main(void)
{
#if defined(CONFIG_MBEDTLS_SHA256_LINKEDSEMI)
    if(test_sha224() != 0)
    {
        printf("SHA-224  test failed!\n");
    }else{
        printf("SHA-224  test passed!\n");
    }

    if(test_sha256() != 0)
    {
        printf("SHA-256  test failed!\n");
    }else{
        printf("SHA-256  test passed!\n");
    }
#endif /* CONFIG_MBEDTLS_SHA256_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_CIPHER_AES_LINKEDSEMI)
    if(aes_test() != 0)
    {
        printf("AES  test failed!\n");
    }else{
        printf("AES  test passed!\n");
    }
#endif /* CONFIG_MBEDTLS_CIPHER_AES_LINKEDSEMI */

    return 0;
}

