#include <string.h>
#include <stdio.h>

#include "mbedtls/threading.h"

#if defined(CONFIG_MBEDTLS_SHA256_LINKEDSEMI) || defined(CONFIG_MBEDTLS_SHA512_LINKEDSEMI)
typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;
#endif /* CONFIG_MBEDTLS_SHA256_LINKEDSEMI || CONFIG_MBEDTLS_SHA512_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_SHA256_LINKEDSEMI)
#include "mbedtls/sha256.h"
#define SHA224_DIGEST_SIZE 28
#define SHA256_DIGEST_SIZE 32
#define SM3_DIGEST_SIZE 32

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

int test_sm3()
{
    mbedtls_sha256_context sm3[3];
    uint8_t   hash[SM3_DIGEST_SIZE];
    int ret = 0;

    testVector a, b, c;
    testVector test_sm3[3];
    int times = sizeof(test_sm3) / sizeof(struct testVector), i, j;

    a.input  = "";
    a.output = "\x1a\xb2\x1d\x83\x55\xcf\xa1\x7f\x8e\x61\x19\x48\x31\xe8\x1a"
               "\x8f\x22\xbe\xc8\xc7\x28\xfe\xfb\x74\x7e\xd0\x35\xeb\x50\x82"
               "\xaa\x2b";
    a.inLen  = strlen(a.input);
    a.outLen = SM3_DIGEST_SIZE;

    b.input  = "abc";
    b.output = "\x66\xc7\xf0\xf4\x62\xee\xed\xd9\xd1\xf2\xd4\x6b\xdc\x10\xe4"
               "\xe2\x41\x67\xc4\x87\x5c\xf2\xf7\xa2\x29\x7d\xa0\x2b\x8f\x4b"
               "\xa8\xe0";
    b.inLen  = strlen(b.input);
    b.outLen = SM3_DIGEST_SIZE;

    c.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    c.output = "\x63\x9b\x6c\xc5\xe6\x4d\x9e\x37\xa3\x90\xb1\x92\xdf\x4f\xa1"
               "\xea\x07\x20\xab\x74\x7f\xf6\x92\xb9\xf3\x8c\x4e\x66\xad\x7b"
               "\x8c\x05";
    c.inLen  = strlen(c.input);
    c.outLen = SM3_DIGEST_SIZE;

    test_sm3[0] = a;
    test_sm3[1] = b;
    test_sm3[2] = c;

    /* Test all the KATs. */
    for (i = 0; i < times; ++i) {

        mbedtls_sm3_init(&sm3[i]);

        if ((ret = mbedtls_sm3_starts(&sm3[i])) != 0) {

            goto exit;
        }

        if ((ret = mbedtls_sm3_update(&sm3[i], (char*)test_sm3[i].input, (size_t)test_sm3[i].inLen)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sm3_finish(&sm3[i], hash)) != 0) {
            goto exit;
        }

        if (memcmp(hash, test_sm3[i].output, SM3_DIGEST_SIZE) != 0) {
            goto exit;
        }
    }
exit:
    for(j = 0; j < i; ++j)
    {
        mbedtls_sm3_free(&sm3[j]);
    }

    return ret;
}
#endif /* CONFIG_MBEDTLS_SHA256_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_CIPHER_AES_LINKEDSEMI)
#include "mbedtls/aes.h"
#define MTLS_AES_192
#define MTLS_AES_256
#define AES_BLOCK_SIZE 16
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
#endif /* CONFIG_MBEDTLS_CIPHER_AES_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_SHA512_LINKEDSEMI)
#include "mbedtls/sha512.h"
#define SHA384_DIGEST_SIZE 48
#define SHA512_DIGEST_SIZE 64
int test_sha384()
{
    mbedtls_sha512_context sha[3];
    bool is384 = true;
    uint8_t      hash[SHA384_DIGEST_SIZE];
    uint8_t ret = 0;
    testVector a, b, c;
    testVector test_sha[3];
    int times = sizeof(test_sha) / sizeof(struct testVector), i, j;

    __attribute__((aligned(4))) uint8_t a_input[] = "";
    __attribute__((aligned(4))) uint8_t b_input[] = "abc";
    __attribute__((aligned(4))) uint8_t c_input[] = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";

    a.input = a_input;
    a.output = "\x38\xb0\x60\xa7\x51\xac\x96\x38\x4c\xd9\x32\x7e\xb1\xb1\xe3"
               "\x6a\x21\xfd\xb7\x11\x14\xbe\x07\x43\x4c\x0c\xc7\xbf\x63\xf6"
               "\xe1\xda\x27\x4e\xde\xbf\xe7\x6f\x65\xfb\xd5\x1a\xd2\xf1\x48"
               "\x98\xb9\x5b";
    a.inLen  = strlen(a.input);
    a.outLen = SHA384_DIGEST_SIZE;

    b.input = b_input;
    b.output = "\xcb\x00\x75\x3f\x45\xa3\x5e\x8b\xb5\xa0\x3d\x69\x9a\xc6\x50"
               "\x07\x27\x2c\x32\xab\x0e\xde\xd1\x63\x1a\x8b\x60\x5a\x43\xff"
               "\x5b\xed\x80\x86\x07\x2b\xa1\xe7\xcc\x23\x58\xba\xec\xa1\x34"
               "\xc8\x25\xa7";
    b.inLen  = strlen(b.input);
    b.outLen = SHA384_DIGEST_SIZE;

    c.input = c_input;
    c.output = "\x09\x33\x0c\x33\xf7\x11\x47\xe8\x3d\x19\x2f\xc7\x82\xcd\x1b"
               "\x47\x53\x11\x1b\x17\x3b\x3b\x05\xd2\x2f\xa0\x80\x86\xe3\xb0"
               "\xf7\x12\xfc\xc7\xc7\x1a\x55\x7e\x2d\xb9\x66\xc3\xe9\xfa\x91"
               "\x74\x60\x39";
    c.inLen  = strlen(c.input);
    c.outLen = SHA384_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;
    test_sha[2] = c;

    for (i = 0; i < times; ++i) {

        mbedtls_sha512_init(&sha[i]);

        if ((ret = mbedtls_sha512_starts(&sha[i], is384)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sha512_update(&sha[i], (char*)test_sha[i].input, (size_t)test_sha[i].inLen)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sha512_finish(&sha[i], hash)) != 0) {
            goto exit;
        }

        if (memcmp(hash, test_sha[i].output, SHA384_DIGEST_SIZE) != 0) {
            goto exit;
        }
    }
exit:
    for(j = 0; j < i; ++j)
    {
        mbedtls_sha512_free(&sha[j]);
    }
    return ret;
}

int test_sha512()
{
    mbedtls_sha512_context sha[3];
    bool is384 = false;
    uint8_t      hash[SHA512_DIGEST_SIZE];
    int ret = 0;
    testVector a, b, c;
    testVector test_sha[3];
    int times = sizeof(test_sha) / sizeof(struct testVector), i,j;
    __attribute__((aligned(4))) uint8_t a_input[] = "";
    __attribute__((aligned(4))) uint8_t b_input[] = "abc";
    __attribute__((aligned(4))) uint8_t c_input[] = "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";

    a.input  = a_input;
    a.output = "\xcf\x83\xe1\x35\x7e\xef\xb8\xbd\xf1\x54\x28\x50\xd6\x6d\x80"
               "\x07\xd6\x20\xe4\x05\x0b\x57\x15\xdc\x83\xf4\xa9\x21\xd3\x6c"
               "\xe9\xce\x47\xd0\xd1\x3c\x5d\x85\xf2\xb0\xff\x83\x18\xd2\x87"
               "\x7e\xec\x2f\x63\xb9\x31\xbd\x47\x41\x7a\x81\xa5\x38\x32\x7a"
               "\xf9\x27\xda\x3e";
    a.inLen  = strlen(a.input);
    a.outLen = SHA512_DIGEST_SIZE;

    b.input  = b_input;
    b.output = "\xdd\xaf\x35\xa1\x93\x61\x7a\xba\xcc\x41\x73\x49\xae\x20\x41"
               "\x31\x12\xe6\xfa\x4e\x89\xa9\x7e\xa2\x0a\x9e\xee\xe6\x4b\x55"
               "\xd3\x9a\x21\x92\x99\x2a\x27\x4f\xc1\xa8\x36\xba\x3c\x23\xa3"
               "\xfe\xeb\xbd\x45\x4d\x44\x23\x64\x3c\xe8\x0e\x2a\x9a\xc9\x4f"
               "\xa5\x4c\xa4\x9f";
    b.inLen  = strlen(b.input);
    b.outLen = SHA512_DIGEST_SIZE;

    c.input  = c_input;
    c.output = "\x8e\x95\x9b\x75\xda\xe3\x13\xda\x8c\xf4\xf7\x28\x14\xfc\x14"
               "\x3f\x8f\x77\x79\xc6\xeb\x9f\x7f\xa1\x72\x99\xae\xad\xb6\x88"
               "\x90\x18\x50\x1d\x28\x9e\x49\x00\xf7\xe4\x33\x1b\x99\xde\xc4"
               "\xb5\x43\x3a\xc7\xd3\x29\xee\xb6\xdd\x26\x54\x5e\x96\xe5\x5b"
               "\x87\x4b\xe9\x09";
    c.inLen  = strlen(c.input);
    c.outLen = SHA512_DIGEST_SIZE;

    test_sha[0] = a;
    test_sha[1] = b;
    test_sha[2] = c;

    for (i = 0; i < times; ++i) {

        mbedtls_sha512_init(&sha[i]);

        if ((ret = mbedtls_sha512_starts(&sha[i], is384)) != 0) {

            goto exit;
        }

        if ((ret = mbedtls_sha512_update(&sha[i], (char*)test_sha[i].input, (size_t)test_sha[i].inLen)) != 0) {
            goto exit;
        }

        if ((ret = mbedtls_sha512_finish(&sha[i], hash)) != 0) {
            goto exit;
        }

        if (memcmp(hash, test_sha[i].output, SHA512_DIGEST_SIZE) != 0) {
            goto exit;
        }
    }
exit:
    for(j = 0; j < i; ++j)
    {
        mbedtls_sha512_free(&sha[j]);
    }
    return ret;
}
#endif /* CONFIG_MBEDTLS_SHA512_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_SM4_LINKEDSEMI)
#include "mbedtls/sm4_alt.h"
#define SM4_BLOCK_SIZE 16
int test_sm4()
{
    /* draft-ribose-cfrg-sm4-10 A.2.1.1 */
    static const uint8_t k1[] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
        0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10
    };
    static const uint8_t p1[] = {
        0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB,
        0xCC, 0xCC, 0xCC, 0xCC, 0xDD, 0xDD, 0xDD, 0xDD,
        0xEE, 0xEE, 0xEE, 0xEE, 0xFF, 0xFF, 0xFF, 0xFF,
        0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB
    };
    static const uint8_t c1_ecb[] = {
        0x5E, 0xC8, 0x14, 0x3D, 0xE5, 0x09, 0xCF, 0xF7,
        0xB5, 0x17, 0x9F, 0x8F, 0x47, 0x4B, 0x86, 0x19,
        0x2F, 0x1D, 0x30, 0x5A, 0x7F, 0xB1, 0x7D, 0xF9,
        0x85, 0xF8, 0x1C, 0x84, 0x82, 0x19, 0x23, 0x04
    };

    mbedtls_sm4_context sm4;
    uint8_t enc[SM4_BLOCK_SIZE * 4];
    uint8_t dec[SM4_BLOCK_SIZE * 4];
    int ret;

    memset(enc, 0, SM4_BLOCK_SIZE * 4);
    memset(dec, 0, SM4_BLOCK_SIZE * 4);

    mbedtls_sm4_init(&sm4);

    /* Encrypt and decrypt with ECB. */
    if((ret = mbedtls_sm4_setkey(k1)) !=0 ) {
        goto exit;
    }

    if((ret = mbedtls_sm4_ecb_encrypt(&sm4, enc, p1, sizeof(p1))) !=0 ){
        goto exit;
    }

    if (memcmp(enc, c1_ecb, sizeof(c1_ecb)) != 0) {
       goto exit;
    }

    if((ret = mbedtls_sm4_ecb_decrypt(&sm4, dec, enc, sizeof(c1_ecb))) !=0 ) {
        goto exit;
    }

    if (memcmp(dec, p1, sizeof(p1)) != 0) {
        goto exit;
    }

exit:
    mbedtls_sm4_free(&sm4);

    return ret;
}

int sm4_ctr_test()
{
    /* draft-ribose-cfrg-sm4-10 A.2.5.1 */
    static const uint8_t k1[] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
        0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10
    };
    static const uint8_t i1[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };
    static const uint8_t p2[] = {
        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
        0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
        0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
        0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD, 0xDD,
        0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
        0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB
    };
    static const uint8_t c2_ctr[] = {
        0xAC, 0x32, 0x36, 0xCB, 0x97, 0x0C, 0xC2, 0x07,
        0x91, 0x36, 0x4C, 0x39, 0x5A, 0x13, 0x42, 0xD1,
        0xA3, 0xCB, 0xC1, 0x87, 0x8C, 0x6F, 0x30, 0xCD,
        0x07, 0x4C, 0xCE, 0x38, 0x5C, 0xDD, 0x70, 0xC7,
        0xF2, 0x34, 0xBC, 0x0E, 0x24, 0xC1, 0x19, 0x80,
        0xFD, 0x12, 0x86, 0x31, 0x0C, 0xE3, 0x7B, 0x92,
        0x6E, 0x02, 0xFC, 0xD0, 0xFA, 0xA0, 0xBA, 0xF3,
        0x8B, 0x29, 0x33, 0x85, 0x1D, 0x82, 0x45, 0x14
    };

    mbedtls_sm4_context sm4;
    uint8_t enc[SM4_BLOCK_SIZE * 4];
    uint8_t dec[SM4_BLOCK_SIZE * 4];
    int ret;

    mbedtls_sm4_init(&sm4);

    /* Encrypt and decrypt using encrypt with CTR. */
    if((ret = mbedtls_sm4_setkey(k1)) !=0 ) {
        goto exit;
    }

    if((ret = mbedtls_sm4_setiv(&sm4, i1)) !=0 ) {
        goto exit;
    }
            
    if((ret = mbedtls_sm4_ctr_crypto(&sm4, enc, p2, sizeof(p2)))!=0 ) {
        goto exit;
    }

    if (memcmp(enc, c2_ctr, sizeof(c2_ctr)) != 0)
        goto exit;

    mbedtls_sm4_init(&sm4);

    /* Encrypt and decrypt using encrypt with CTR. */
    if((ret = mbedtls_sm4_setkey(k1)) !=0 ) {
        goto exit;
    }

    if((ret = mbedtls_sm4_setiv(&sm4, i1)) !=0 ) {
        goto exit;
    }

    if((ret = mbedtls_sm4_ctr_crypto(&sm4, dec, enc, sizeof(c2_ctr))) !=0 ) {
        goto exit;
    }

    if (memcmp(dec, p2, sizeof(p2)) != 0)
        goto exit;

exit:
    mbedtls_sm4_free(&sm4);

    return 0;
}
#endif /* CONFIG_MBEDTLS_SM4_LINKEDSEMI */

int ecdsa_test(void);
int main(void)
{

    mbedtls_zephyr_threading_init();

#if defined(CONFIG_MBEDTLS_ECDSA_LINKEDSEMI)
    if(ecdsa_test() !=0)
    {
        printf("ECDSA  test failed!\n");
    }else
    {
        printf("ECDSA  test passed!\n");
    }

#endif

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

    if(test_sm3() != 0)
    {
        printf("SM3  test failed!\n");
    }else{
        printf("SM3  test passed!\n");
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

#if defined(CONFIG_MBEDTLS_SHA512_LINKEDSEMI)
    if(test_sha384() != 0)
    {
        printf("SHA-384  test failed!\n");
    }else{
        printf("SHA-384  test passed!\n");
    }

    if(test_sha512() != 0)
    {
        printf("SHA-512  test failed!\n");
    }else{
        printf("SHA-512  test passed!\n");
    }
#endif /* CONFIG_MBEDTLS_SHA512_LINKEDSEMI */

#if defined(CONFIG_MBEDTLS_SM4_LINKEDSEMI)
    if(test_sm4() != 0)
    {
        printf("sm4 test failed!\n");
    }else{
        printf("sm4  test passed!\n");
    }

    if(sm4_ctr_test() != 0)
    {
        printf("sm4_ctr_test failed!\n");
    }else{
        printf("sm4_ctr_test passed!\n");
    }

#endif /* CONFIG_MBEDTLS_SM4_LINKEDSEMI */

    return 0;
}

