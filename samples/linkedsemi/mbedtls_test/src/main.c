#include <string.h>
#include <stdio.h>
#include "mbedtls/sha256.h"

#define WC_SHA224_DIGEST_SIZE 28
#define SHA256_DIGEST_SIZE 32

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

    uint8_t      hash[WC_SHA224_DIGEST_SIZE];

    testVector a, b, c;
    testVector test_sha[3];
    int times = sizeof(test_sha) / sizeof(struct testVector), i, j;

    a.input  = "";
    a.output = "\xd1\x4a\x02\x8c\x2a\x3a\x2b\xc9\x47\x61\x02\xbb\x28\x82\x34"
               "\xc4\x15\xa2\xb0\x1f\x82\x8e\xa6\x2a\xc5\xb3\xe4\x2f";
    a.inLen  = strlen(a.input);
    a.outLen = WC_SHA224_DIGEST_SIZE;

    b.input  = "abc";
    b.output = "\x23\x09\x7d\x22\x34\x05\xd8\x22\x86\x42\xa4\x77\xbd\xa2\x55"
               "\xb3\x2a\xad\xbc\xe4\xbd\xa0\xb3\xf7\xe3\x6c\x9d\xa7";
    b.inLen  = strlen(b.input);
    b.outLen = WC_SHA224_DIGEST_SIZE;

    c.input  = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
    c.output = "\x75\x38\x8b\x16\x51\x27\x76\xcc\x5d\xba\x5d\xa1\xfd\x89\x01"
               "\x50\xb0\xc6\x45\x5c\xb4\xf5\x8b\x19\x52\x52\x25\x25";
    c.inLen  = strlen(c.input);
    c.outLen = WC_SHA224_DIGEST_SIZE;

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

int main(void)
{
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
    return 0;
}

