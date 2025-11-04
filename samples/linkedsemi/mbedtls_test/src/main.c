#include <string.h>
#include <stdio.h>

#include "mbedtls/threading.h"

typedef struct testVector {
    const char*  input;
    const char*  output;
    size_t inLen;
    size_t outLen;
} testVector;

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

        if ((ret = memcmp(hash, test_sha[i].output, SHA224_DIGEST_SIZE)) != 0) {
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

        if ((ret = memcmp(hash, test_sha[i].output, SHA256_DIGEST_SIZE)) != 0) {
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

        if ((ret = memcmp(hash, test_sm3[i].output, SM3_DIGEST_SIZE)) != 0) {
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

int test_sha224_dma()
{
    int ret = 0;
    mbedtls_sha256_context sha;
    bool is224 = true;

    uint8_t      hash[SHA224_DIGEST_SIZE];
    uint8_t      hash_output[SHA224_DIGEST_SIZE] = 
                {0xF0,0x62,0x1E,0x96,0x01,0xDE,0x98,0xB4,0x8F,0xFF,0xBC,0x7A,0x16,0xFD,
                 0xF2,0xDE,0xA1,0x48,0xF7,0x49,0x51,0xD5,0xC4,0xEF,0x73,0xA2,0xD6,0x0E};

    #define test_len 16257  //pass: 8127,8128,8129,9000,16256,16257,36852
    __attribute__((aligned(32))) uint8_t big_buffer[test_len];

    memset(big_buffer, 0x2, sizeof(big_buffer));

    mbedtls_sha256_init_dma(&sha);

    if ((ret = mbedtls_sha256_starts_dma(&sha, is224)) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sha256_update_dma(&sha, big_buffer, sizeof(big_buffer))) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sha256_finish_dma(&sha, hash)) != 0) {
        goto exit;
    }

    if ((ret = memcmp(hash, hash_output, SHA224_DIGEST_SIZE)) != 0) {
        goto exit;
    }

exit:

    mbedtls_sha256_free(&sha);

    return ret;
}

int test_sha256_dma()
{
    int ret = 0;
    mbedtls_sha256_context sha;
    bool is224 = false;

    uint8_t      hash[SHA256_DIGEST_SIZE];
    uint8_t      hash_output[SHA256_DIGEST_SIZE] = 
                 {0x79,0x14,0x3E,0x04,0xBF,0xB7,0x6E,0xCE,0xF7,0xD6,0xDF,0xDF,0x69,0xEB,0x3A,0xD2,
                  0xDE,0x49,0xA5,0xBC,0x41,0xD0,0x25,0xFC,0x0C,0x5D,0x52,0x30,0x2C,0x46,0xB6,0x7B};

    #define test_len 16257
    __attribute__((aligned(32))) uint8_t big_buffer[test_len];

    memset(big_buffer, 0x2, sizeof(big_buffer));

    mbedtls_sha256_init_dma(&sha);

    if ((ret = mbedtls_sha256_starts_dma(&sha, is224)) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sha256_update_dma(&sha, big_buffer, sizeof(big_buffer))) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sha256_finish_dma(&sha, hash)) != 0) {
        goto exit;
    }

    if ((ret = memcmp(hash, hash_output, SHA256_DIGEST_SIZE)) != 0) {
        goto exit;
    }

exit:

    mbedtls_sha256_free(&sha);

    return ret;
}

int test_sm3_dma()
{
    mbedtls_sha256_context sm3;
    int ret = 0;
    uint8_t   hash[SM3_DIGEST_SIZE];

    uint8_t   hash_output[SM3_DIGEST_SIZE] = 
                {0xF0,0x69,0x89,0xD9,0xEC,0x31,0xB9,0xD0,0x0C,0x87,0x0A,0x73,0xE7,0xB6,0x76,0x20,
                 0xEC,0xAC,0xE0,0x39,0x53,0x5A,0x4A,0xBC,0xA2,0x11,0x95,0x9C,0xF1,0xE9,0xCA,0xAA};

    #define test_len 16257
    __attribute__((aligned(32))) uint8_t big_buffer[test_len];

    memset(big_buffer, 0x2, sizeof(big_buffer));

    mbedtls_sm3_init_dma(&sm3);

    if ((ret = mbedtls_sm3_starts_dma(&sm3)) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sm3_update_dma(&sm3, big_buffer, sizeof(big_buffer))) != 0) {
        goto exit;
    }

    if ((ret = mbedtls_sm3_finish_dma(&sm3, hash)) != 0) {
        goto exit;
    }

    if ((ret = memcmp(hash, hash_output, SM3_DIGEST_SIZE)) != 0) {
        goto exit;
    }

exit:

    mbedtls_sm3_free(&sm3);

    return ret;
}

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

    const unsigned char key_128[] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};
#ifdef MTLS_AES_192
    const unsigned char key_192[] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
#endif
#ifdef MTLS_AES_256
    const unsigned char key_256[] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};
#endif

   const unsigned char niKey[] = {
    0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe, 
    0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
    0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7,
    0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4};

    __attribute__((aligned(4))) const uint8_t msg[] = {
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
        const unsigned char * key;
        int         keybits;
        const uint8_t* plain;
        const uint8_t* verify;
    } testVec[] = {
        { key_128, 128,  msg,     verify_ecb_128 },
#ifdef MTLS_AES_192
        { key_192, 192,  msg,     verify_ecb_192 },
#endif
#ifdef MTLS_AES_256
        { key_256, 256,  msg,     verify_ecb_256 },
        { niKey,   256,  niPlain, niCipher }
#endif
    };
    #define AES_ECB_TEST_LEN (int)(sizeof(testVec) / sizeof(*testVec))

    for (i = 0; i < AES_ECB_TEST_LEN; i++) {

        mbedtls_aes_init(&aes);

        if((ret = mbedtls_aes_setkey_enc(&aes, (char *)testVec[i].key, testVec[i].keybits)) != 0 ) {
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
        mbedtls_aes_init(&aes);

        if ((ret = mbedtls_aes_setkey_dec(&aes, (char *)testVec[i].key, testVec[i].keybits)) != 0 ) {
            goto exit;
        }
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

    const unsigned char key[] = {
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};

    unsigned char enc_iv[] = {
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x30, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};
    unsigned char dec_iv[] = {
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x30, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66};

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)key, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }

    memset(cipher, 0, sizeof(cipher));
    if((ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, AES_BLOCK_SIZE, (char *)enc_iv, msg, cipher)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(cipher, verify, AES_BLOCK_SIZE)) != 0) {
       goto exit;
    }

    memset(plain, 0, sizeof(plain));
    if ((ret = mbedtls_aes_setkey_dec(&aes, (char *)key, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }
    if((ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, AES_BLOCK_SIZE, (char *)dec_iv, cipher, plain)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(plain, msg, AES_BLOCK_SIZE)) !=0) {
        goto exit;
    }
exit:
    return ret;
}

static int aes_ctr_test()
{
    mbedtls_aes_context aes;
    uint8_t ctr_plaintext[] = {
        //Block #1
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
        //Block #2
        0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        //Block #3
        0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
        //Block #4
        0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

    uint8_t ctr_ciphertext_128[] = {
        0x87, 0x4d, 0x61, 0x91, 0xb6, 0x20, 0xe3, 0x26, 0x1b, 0xef, 0x68, 0x64, 0x99, 0x0d, 0xb6, 0xce,
        0x98, 0x06, 0xf6, 0x6b, 0x79, 0x70, 0xfd, 0xff, 0x86, 0x17, 0x18, 0x7b, 0xb9, 0xff, 0xfd, 0xff,
        0x5a, 0xe4, 0xdf, 0x3e, 0xdb, 0xd5, 0xd3, 0x5e, 0x5b, 0x4f, 0x09, 0x02, 0x0d, 0xb0, 0x3e, 0xab,
        0x1e, 0x03, 0x1d, 0xda, 0x2f, 0xbe, 0x03, 0xd1, 0x79, 0x21, 0x70, 0xa0, 0xf3, 0x00, 0x9c, 0xee};

    const unsigned char ctr_key_128[] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    uint8_t ciphertext_buff[64];
    uint8_t plaintext_buff[64];
    int ret = 0;
    size_t current_nc_off = 0;
    size_t *nc_off_ptr = &current_nc_off;
    unsigned char nonce_counter_enc[16] = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff};
    unsigned char nonce_counter_dec[16] = {0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff};
    unsigned char stream_block[16] = { 0 };

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)ctr_key_128, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }

    memset(ciphertext_buff, 0, sizeof(ciphertext_buff));
    if((ret = mbedtls_aes_crypt_ctr(&aes, sizeof(ctr_plaintext), nc_off_ptr, nonce_counter_enc, stream_block, ctr_plaintext, ciphertext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(ciphertext_buff, ctr_ciphertext_128,  sizeof(ctr_ciphertext_128))) != 0) {
       goto exit;
    }

    memset(plaintext_buff, 0, sizeof(plaintext_buff));
    if ((ret = mbedtls_aes_crypt_ctr(&aes, sizeof(ctr_ciphertext_128), nc_off_ptr, nonce_counter_dec, stream_block, ctr_ciphertext_128, plaintext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(plaintext_buff, ctr_plaintext, sizeof(ctr_plaintext))) != 0) {
        goto exit;
    }
exit:
    return ret;
}

static int aes_cfb128_test()
{
    mbedtls_aes_context aes;
    uint8_t cfb128_plaintext[] = {
        //Block #1
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
        //Block #2
        0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        //Block #3
        0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
        //Block #4
        0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

    uint8_t cfb128_ciphertext[] = {
        0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34, 0x49, 0xf8, 0xe8, 0x3c, 0xfb, 0x4a,
        0xc8, 0xa6, 0x45, 0x37, 0xa0, 0xb3, 0xa9, 0x3f, 0xcd, 0xe3, 0xcd, 0xad, 0x9f, 0x1c, 0xe5, 0x8b,
        0x26, 0x75, 0x1f, 0x67, 0xa3, 0xcb, 0xb1, 0x40, 0xb1, 0x80, 0x8c, 0xf1, 0x87, 0xa4, 0xf4, 0xdf,
        0xc0, 0x4b, 0x05, 0x35, 0x7c, 0x5d, 0x1c, 0x0e, 0xea, 0xc4, 0xc6, 0x6f, 0x9f, 0xf7, 0xf2, 0xe6};

    const unsigned char cfb128_key[] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    unsigned char cfb128_enc_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    unsigned char cfb128_dec_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    uint8_t ciphertext_buff[64];
    uint8_t plaintext_buff[64];
    int ret = 0;
    size_t iv_off = 0;

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)cfb128_key, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }

    memset(ciphertext_buff, 0, sizeof(ciphertext_buff));
    if((ret = mbedtls_aes_crypt_cfb128(&aes, MBEDTLS_AES_ENCRYPT, sizeof(cfb128_plaintext), &iv_off , cfb128_enc_iv, cfb128_plaintext, ciphertext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(ciphertext_buff, cfb128_ciphertext,  sizeof(cfb128_ciphertext))) != 0) {
       goto exit;
    }

    memset(plaintext_buff, 0, sizeof(plaintext_buff));
    if ((ret = mbedtls_aes_crypt_cfb128(&aes, MBEDTLS_AES_DECRYPT, sizeof(cfb128_ciphertext), &iv_off, cfb128_dec_iv, cfb128_ciphertext, plaintext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(plaintext_buff, cfb128_plaintext, sizeof(cfb128_plaintext))) != 0) {
        goto exit;
    }
exit:
    return ret;
}

static int aes_cfb8_test()
{
    mbedtls_aes_context aes;
    uint8_t cfb8_plaintext[] = {
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d};

    uint8_t cfb8_ciphertext[] = {
        0x3b, 0x79, 0x42, 0x4c, 0x9c, 0x0d, 0xd4, 0x36, 0xba, 0xce, 0x9e, 0x0e, 0xd4, 0x58, 0x6a, 0x4f, 0x32, 0xb9};

    const unsigned char cfb8_key[] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    unsigned char cfb8_enc_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    unsigned char cfb8_dec_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    uint8_t ciphertext_buff[64];
    uint8_t plaintext_buff[64];
    int ret = 0;

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)cfb8_key, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }

    memset(ciphertext_buff, 0, sizeof(ciphertext_buff));
    if((ret = mbedtls_aes_crypt_cfb8(&aes, MBEDTLS_AES_ENCRYPT, sizeof(cfb8_plaintext), cfb8_enc_iv, cfb8_plaintext, ciphertext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(ciphertext_buff, cfb8_ciphertext,  sizeof(cfb8_ciphertext))) != 0) {
       goto exit;
    }

    memset(plaintext_buff, 0, sizeof(plaintext_buff));
    if ((ret = mbedtls_aes_crypt_cfb8(&aes, MBEDTLS_AES_DECRYPT, sizeof(cfb8_ciphertext), cfb8_dec_iv, cfb8_ciphertext, plaintext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(plaintext_buff, cfb8_plaintext, sizeof(cfb8_plaintext))) != 0) {
        goto exit;
    }
exit:
    return ret;
}

static int aes_ofb_test()
{
    mbedtls_aes_context aes;
    uint8_t ofb_plaintext[] = {
        //Block #1
        0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
        //Block #2
        0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
        //Block #3
        0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
        //Block #4
        0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

    uint8_t ofb_ciphertext[] = {
        0x3b, 0x3f, 0xd9, 0x2e, 0xb7, 0x2d, 0xad, 0x20, 0x33, 0x34, 0x49, 0xf8, 0xe8, 0x3c, 0xfb, 0x4a,
        0x77, 0x89, 0x50, 0x8d, 0x16, 0x91, 0x8f, 0x03, 0xf5, 0x3c, 0x52, 0xda, 0xc5, 0x4e, 0xd8, 0x25,
        0x97, 0x40, 0x05, 0x1e, 0x9c, 0x5f, 0xec, 0xf6, 0x43, 0x44, 0xf7, 0xa8, 0x22, 0x60, 0xed, 0xcc,
        0x30, 0x4c, 0x65, 0x28, 0xf6, 0x59, 0xc7, 0x78, 0x66, 0xa5, 0x10, 0xd9, 0xc1, 0xd6, 0xae, 0x5e};

    const unsigned char ofb_key[] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    unsigned char ofb_enc_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    unsigned char ofb_dec_iv[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    size_t iv_off = 0;
    uint8_t ciphertext_buff[64];
    uint8_t plaintext_buff[64];
    int ret = 0;

    mbedtls_aes_init(&aes);

    if((ret = mbedtls_aes_setkey_enc(&aes, (char *)ofb_key, AES_BLOCK_SIZE*8)) != 0) {
        goto exit;
    }

    memset(ciphertext_buff, 0, sizeof(ciphertext_buff));
    if((ret = mbedtls_aes_crypt_ofb(&aes, sizeof(ofb_plaintext), &iv_off, ofb_enc_iv, ofb_plaintext, ciphertext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(ciphertext_buff, ofb_ciphertext,  sizeof(ofb_ciphertext))) != 0) {
       goto exit;
    }

    memset(plaintext_buff, 0, sizeof(plaintext_buff));
    if ((ret = mbedtls_aes_crypt_ofb(&aes, sizeof(ofb_ciphertext), &iv_off, ofb_dec_iv, ofb_ciphertext, plaintext_buff)) != 0) {
        goto exit;
    }
    if ((ret = memcmp(plaintext_buff, ofb_plaintext, sizeof(ofb_plaintext))) != 0) {
        goto exit;
    }
exit:
    return ret;
}

static int aes_xts_test()
{
    int ret = 0;

    static const unsigned char aes_test_xts_key[][32] =
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
          0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
          0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
          0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 },
        { 0xff, 0xfe, 0xfd, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8,
          0xf7, 0xf6, 0xf5, 0xf4, 0xf3, 0xf2, 0xf1, 0xf0,
          0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
          0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22 },
    };

    static const unsigned char aes_test_xts_data_unit[][16] =
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        { 0x33, 0x33, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        { 0x33, 0x33, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
    };

    static const unsigned char aes_test_xts_ct32[][32] =
    {
        { 0x91, 0x7c, 0xf6, 0x9e, 0xbd, 0x68, 0xb2, 0xec,
          0x9b, 0x9f, 0xe9, 0xa3, 0xea, 0xdd, 0xa6, 0x92,
          0xcd, 0x43, 0xd2, 0xf5, 0x95, 0x98, 0xed, 0x85,
          0x8c, 0x02, 0xc2, 0x65, 0x2f, 0xbf, 0x92, 0x2e },
        { 0xc4, 0x54, 0x18, 0x5e, 0x6a, 0x16, 0x93, 0x6e,
          0x39, 0x33, 0x40, 0x38, 0xac, 0xef, 0x83, 0x8b,
          0xfb, 0x18, 0x6f, 0xff, 0x74, 0x80, 0xad, 0xc4,
          0x28, 0x93, 0x82, 0xec, 0xd6, 0xd3, 0x94, 0xf0 },
        { 0xaf, 0x85, 0x33, 0x6b, 0x59, 0x7a, 0xfc, 0x1a,
          0x90, 0x0b, 0x2e, 0xb2, 0x1e, 0xc9, 0x49, 0xd2,
          0x92, 0xdf, 0x4c, 0x04, 0x7e, 0x0b, 0x21, 0x53,
          0x21, 0x86, 0xa5, 0x97, 0x1a, 0x22, 0x7a, 0x89 },
    };

    static const unsigned char aes_test_xts_pt32[][32] =
    {
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
        { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 },
        { 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
          0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44 },
    };

    static const int num_tests =
        sizeof(aes_test_xts_key) / sizeof(*aes_test_xts_key);
    mbedtls_aes_xts_context ctx_xts;

    mbedtls_aes_xts_init(&ctx_xts);

    for (uint8_t i = 0; i < num_tests << 1; i++) {
        mbedtls_aes_xts_init(&ctx_xts);

        const unsigned char *data_unit;
        uint8_t u = i >> 1;
        uint8_t mode = i & 1;
        unsigned char key[32];
        unsigned char buf[64];
        const unsigned char *aes_tests;

        memset(key, 0, sizeof(key));
        memcpy(key, aes_test_xts_key[u], 32);
        data_unit = aes_test_xts_data_unit[u];

        uint32_t len = sizeof(*aes_test_xts_ct32);

        if (mode == MBEDTLS_AES_DECRYPT) {
            ret = mbedtls_aes_xts_setkey_dec(&ctx_xts, key, 256);
            if (ret != 0) {
                goto exit;
            }
            memcpy(buf, aes_test_xts_ct32[u], len);
            aes_tests = aes_test_xts_pt32[u];
        } else {
            ret = mbedtls_aes_xts_setkey_enc(&ctx_xts, key, 256);
            if (ret != 0) {
                goto exit;
            }
            memcpy(buf, aes_test_xts_pt32[u], len);
            aes_tests = aes_test_xts_ct32[u];
        }

        ret = mbedtls_aes_crypt_xts(&ctx_xts, mode, len, data_unit,
                                    buf, buf);
        if (ret != 0) {
            goto exit;
        }

        if (memcmp(buf, aes_tests, len) != 0) {
            ret = 1;
            goto exit;
        }
    }

    mbedtls_aes_xts_free(&ctx_xts);

    ret = 0;

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

    if((ret = aes_cfb128_test()) !=0)
    {
        printf("aes_cfb128_test  test failed!\n");
    }else{
        printf("aes_cfb128_test  test passed!\n");
    }

    if((ret = aes_cfb8_test()) !=0)
    {
        printf("aes_cfb8_test  test failed!\n");
    }else{
        printf("aes_cfb8_test  test passed!\n");
    }

    if((ret = aes_ofb_test()) !=0)
    {
        printf("aes_ofb_test  test failed!\n");
    }else{
        printf("aes_ofb_test  test passed!\n");
    }

    if((ret = aes_ctr_test()) !=0)
    {
        printf("aes_ctr_test  test failed!\n");
    }else{
        printf("aes_ctr_test  test passed!\n");
    }

    if((ret = aes_xts_test()) !=0)
    {
        printf("aes_xts_test  test failed!\n");
    }else{
        printf("aes_xts_test  test passed!\n");
    }

    return ret;
}

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

        if ((ret = memcmp(hash, test_sha[i].output, SHA384_DIGEST_SIZE)) != 0) {
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

        if ((ret = memcmp(hash, test_sha[i].output, SHA512_DIGEST_SIZE)) != 0) {
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

#if defined(CONFIG_MBEDTLS_SM4_LINKEDSEMI_HARDWARE_ALT)
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

    if ((ret = memcmp(enc, c1_ecb, sizeof(c1_ecb))) != 0) {
       goto exit;
    }

    if((ret = mbedtls_sm4_ecb_decrypt(&sm4, dec, enc, sizeof(c1_ecb))) !=0 ) {
        goto exit;
    }

    if ((ret = memcmp(dec, p1, sizeof(p1))) != 0) {
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

    if ((ret = memcmp(enc, c2_ctr, sizeof(c2_ctr))) != 0)
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

    if ((ret = memcmp(dec, p2, sizeof(p2))) != 0)
        goto exit;

exit:
    mbedtls_sm4_free(&sm4);

    return 0;
}
#endif /* CONFIG_MBEDTLS_SM4_LINKEDSEMI_HARDWARE_ALT */

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    int ecdsa_test(void);
#endif

int main(void)
{

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
#if defined(CONFIG_MBEDTLS_ECDSA_SECP256R1_SECP384R1_SM2_LINKEDSEMI_OTBN_ALT)
    if(ecdsa_test() !=0)
    {
        printf("ECDSA  test failed!\n");
    }else
    {
        printf("ECDSA  test passed!\n");
    }
#endif
#endif

    if(test_sha224() != 0)
    {
        printf("SHA-224  test failed!\n");
#if defined(CONFIG_MBEDTLS_SHA256_SM3_LINKEDSEMI_OTBN_ALT)
        printf("otbn not sopported sha224\n");
#endif
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

    if(test_sha224_dma() != 0)
    {
        printf("SHA-224 dma mode test failed!\n");
    }else{
        printf("SHA-224 dma mode test passed!\n");
    }

    if(test_sha256_dma() != 0)
    {
        printf("SHA-256 dma mode test failed!\n");
    }else{
        printf("SHA-256 dma mode test passed!\n");
    }

    if(test_sm3_dma() != 0)
    {
        printf("SM3 dma test failed!\n");
    }else{
        printf("SM3 dma test passed!\n");
    }

    if(aes_test() != 0)
    {
        printf("AES  test failed!\n");
    }else{
        printf("AES  test passed!\n");
    }


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

    return 0;
}