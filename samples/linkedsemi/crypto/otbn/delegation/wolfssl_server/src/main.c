/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "stdio.h"
#include <stdint.h>

#if !defined(WOLFSSL_USER_SETTINGS) && !defined(WOLFSSL_NO_OPTIONS_H)
    #include <wolfssl/options.h>
#endif
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/version.h>
#include <wolfssl/wolfcrypt/types.h>
#include <wolfssl/wolfcrypt/wc_port.h>
#include <wolfssl/wolfcrypt/mem_track.h>
#ifdef HAVE_ECC
    #include <wolfssl/wolfcrypt/ecc.h>
#endif
#ifdef WOLFSSL_SM2
    #include <wolfssl/wolfcrypt/sm2.h>
#endif
#if !defined(WC_NO_RNG)
    #include <wolfssl/wolfcrypt/random.h>
#endif
#include <wolfssl/wolfcrypt/types.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfcrypt/wc_port.h>

LOG_MODULE_REGISTER(main);

#ifdef WOLFSSL_STATIC_MEMORY
    static WOLFSSL_HEAP_HINT* HEAP_HINT;
#else
    #define HEAP_HINT NULL
#endif /* WOLFSSL_STATIC_MEMORY */
#define ECC_SIG_SIZE        ECC_MAX_SIG_SIZE
#define ERROR_OUT(err, eLabel) do { ret = (err); goto eLabel; } while (0)
#if defined(WOLFSSL_ECDSA_DETERMINISTIC_K) || defined(WOLFSSL_ECDSA_DETERMINISTIC_K_VARIANT)
#define HAVE_ECC_DETERMINISTIC_K
#define ECC_DIGEST_SIZE     WC_SHA256_DIGEST_SIZE
#else
#define ECC_DIGEST_SIZE     MAX_ECC_BYTES
#endif


static int devId = INVALID_DEVID;

static int ecc_test_curve_size(WC_RNG* rng, int keySize, int testVerifyCount,
    int curve_id, const ecc_set_type* dp)
{
    word32  x = 0;
    int ret;
#if !defined(ECC_TIMING_RESISTANT) || (defined(ECC_TIMING_RESISTANT) && \
    !defined(WC_NO_RNG) && !defined(WOLFSSL_KCAPI_ECC)) && \
    defined(HAVE_ECC_SIGN)
    WC_DECLARE_VAR(sig, byte, ECC_SIG_SIZE, HEAP_HINT);
    WC_DECLARE_VAR(digest, byte, ECC_DIGEST_SIZE, HEAP_HINT);
    int     i;
#ifdef HAVE_ECC_VERIFY
    int     verify;
#endif /* HAVE_ECC_VERIFY */
#endif /* HAVE_ECC_SIGN */
#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    ecc_key *userA = (ecc_key *)XMALLOC(sizeof *userA, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    ecc_key *userB = (ecc_key *)XMALLOC(sizeof *userB, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    ecc_key *pubKey = (ecc_key *)XMALLOC(sizeof *pubKey, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
#else
    ecc_key userA[1];
    ecc_key userB[1];
    ecc_key pubKey[1];
#endif
#if !defined(ECC_TIMING_RESISTANT) || (defined(ECC_TIMING_RESISTANT) && \
    !defined(WC_NO_RNG) && !defined(WOLFSSL_KCAPI_ECC)) && \
    defined(HAVE_ECC_SIGN)
    WC_ALLOC_VAR(sig, byte, ECC_SIG_SIZE, HEAP_HINT);
    WC_ALLOC_VAR(digest, byte, ECC_DIGEST_SIZE, HEAP_HINT);
#endif
	int     curveSize;
    XMEMSET(userA, 0, sizeof *userA);
    XMEMSET(userB, 0, sizeof *userB);
    XMEMSET(pubKey, 0, sizeof *pubKey);

    ret = wc_ecc_init_ex(userA, HEAP_HINT, devId);
    if (ret != 0)
		goto done;
    ret = wc_ecc_init_ex(userB, HEAP_HINT, devId);
    if (ret != 0)
        goto done;
    ret = wc_ecc_init_ex(pubKey, HEAP_HINT, devId);
    if (ret != 0)
        goto done;

	ret = wc_ecc_make_key_ex(rng, keySize, userA, curve_id);
    if (ret == WC_NO_ERR_TRACE(ECC_CURVE_OID_E))
        goto done; /* catch case, where curve is not supported */
    if (ret != 0)
        goto done;

    if (wc_ecc_get_curve_idx(curve_id) != -1) {
        curveSize = wc_ecc_get_curve_size_from_id(userA->dp->id);
        if (curveSize != userA->dp->size)
            goto done;
    }

    ret = wc_ecc_check_key(userA);
    if (ret != 0)
        goto done;


#ifdef HAVE_ECC_SIGN
    /* some hardware doesn't support sign/verify of all zero digest */
#if !defined(WC_TEST_NO_ECC_SIGN_VERIFY_ZERO_DIGEST)
    /* test DSA sign hash with zeros */
    for (i = 0; i < (int)ECC_DIGEST_SIZE; i++) {
        digest[i] = 0;
    }

    x = ECC_SIG_SIZE;
    do {
    #if defined(WOLFSSL_ASYNC_CRYPT)
        ret = wc_AsyncWait(ret, &userA->asyncDev, WC_ASYNC_FLAG_CALL_AGAIN);
    #endif
        if (ret == 0)
            ret = wc_ecc_sign_hash(digest, ECC_DIGEST_SIZE, sig, &x, rng,
                                                                        userA);
    } while (ret == WC_NO_ERR_TRACE(WC_PENDING_E));
    if (ret != 0)
        goto done;

#ifdef HAVE_ECC_VERIFY
    for (i=0; i<testVerifyCount; i++) {
        verify = 0;
        do {
        #if defined(WOLFSSL_ASYNC_CRYPT)
            ret = wc_AsyncWait(ret, &userA->asyncDev, WC_ASYNC_FLAG_CALL_AGAIN);
        #endif
            if (ret == 0)
                ret = wc_ecc_verify_hash(sig, x, digest, ECC_DIGEST_SIZE,
                                                               &verify, userA);
        } while (ret == WC_NO_ERR_TRACE(WC_PENDING_E));
        if (ret != 0)
            goto done;
        if (verify != 1)
        {
            ret = -1;
            printf("ecdsa verify failed\n");
            goto done;
        }
            
    }
#endif /* HAVE_ECC_VERIFY */
#endif /* !WC_TEST_NO_ECC_SIGN_VERIFY_ZERO_DIGEST */

    /* test DSA sign hash with sequence (0,1,2,3,4,...) */
    for (i = 0; i < (int)ECC_DIGEST_SIZE; i++) {
        digest[i] = (byte)i;
    }

    x = ECC_SIG_SIZE;
    do {
    #if defined(WOLFSSL_ASYNC_CRYPT)
        ret = wc_AsyncWait(ret, &userA->asyncDev, WC_ASYNC_FLAG_CALL_AGAIN);
    #endif
        if (ret == 0)
            ret = wc_ecc_sign_hash(digest, ECC_DIGEST_SIZE, sig, &x, rng, userA);
    } while (ret == WC_NO_ERR_TRACE(WC_PENDING_E));
    if (ret != 0)
        goto done;

#ifdef HAVE_ECC_VERIFY
    for (i=0; i<testVerifyCount; i++) {
        verify = 0;
        do {
        #if defined(WOLFSSL_ASYNC_CRYPT)
            ret = wc_AsyncWait(ret, &userA->asyncDev, WC_ASYNC_FLAG_CALL_AGAIN);
        #endif
            if (ret == 0)
                ret = wc_ecc_verify_hash(sig, x, digest, ECC_DIGEST_SIZE, &verify, userA);
        } while (ret == WC_NO_ERR_TRACE(WC_PENDING_E));
        if (ret != 0)
            goto done;
        if (verify != 1)
            goto done;
    }
#endif /* HAVE_ECC_VERIFY */
#endif /* HAVE_ECC_SIGN */

done:

#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    if (userA != NULL) {
        wc_ecc_free(userA);
        XFREE(userA, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
    if (userB != NULL) {
        wc_ecc_free(userB);
        XFREE(userB, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
    if (pubKey != NULL) {
        wc_ecc_free(pubKey);
        XFREE(pubKey, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
#else
    wc_ecc_free(pubKey);
    wc_ecc_free(userB);
    wc_ecc_free(userA);
#endif

#ifdef HAVE_ECC_SIGN
    WC_FREE_VAR(sig, HEAP_HINT);
    WC_FREE_VAR(digest, HEAP_HINT);
#endif
    (void)keySize;
    (void)curve_id;
    (void)rng;

    if(ret)
    {
        printf("ecdsa test failed\n");
    }

	return ret;
}


#undef  ECC_TEST_VERIFY_COUNT
#define ECC_TEST_VERIFY_COUNT 2
static int ecc_test_curve(WC_RNG* rng, int keySize, int curve_id)
{
    int ret;
    WOLFSSL_MSG_EX("ecc_test_curve keySize = %d", keySize);

    ret = ecc_test_curve_size(rng, keySize, ECC_TEST_VERIFY_COUNT, curve_id,
        NULL);
    if (ret < 0) {
        if (ret == WC_NO_ERR_TRACE(ECC_CURVE_OID_E)) {
            /* ignore error for curves not found */
            /* some curve sizes are only available with:
                HAVE_ECC_SECPR2, HAVE_ECC_SECPR3, HAVE_ECC_BRAINPOOL
                and HAVE_ECC_KOBLITZ */
        }
        else {
            printf("ecc_test_curve_size %d failed!\n", keySize);
            return ret;
        }
    }

    return ret;
}



static int ecc_sm2_test_curve(WC_RNG* rng, int testVerifyCount)
{
    const ecc_set_type* dp = wc_ecc_get_curve_params(
        wc_ecc_get_curve_idx(ECC_SM2P256V1));
    int keySize = 32;
    int curve_id = ECC_SM2P256V1;
    word32  x = 0;
#ifdef HAVE_ECC_SIGN
    WC_DECLARE_VAR(sig, byte, ECC_SIG_SIZE, HEAP_HINT);
    WC_DECLARE_VAR(digest, byte, ECC_DIGEST_SIZE, HEAP_HINT);
    int     i;
    int     verify;
#endif /* HAVE_ECC_SIGN */
    int    ret = 0;
#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    ecc_key *userA = (ecc_key *)XMALLOC(sizeof *userA, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    ecc_key *userB = (ecc_key *)XMALLOC(sizeof *userB, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    ecc_key *pubKey = (ecc_key *)XMALLOC(sizeof *pubKey, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
#else
    ecc_key userA[1];
    ecc_key userB[1];
    ecc_key pubKey[1];
#endif
#ifndef WC_NO_RNG
    int     curveSize;
#endif

#ifdef HAVE_ECC_SIGN
    WC_ALLOC_VAR(sig, byte, ECC_SIG_SIZE, HEAP_HINT);
    WC_ALLOC_VAR(digest, byte, ECC_DIGEST_SIZE, HEAP_HINT);
#endif

#ifdef WC_DECLARE_VAR_IS_HEAP_ALLOC

#ifdef HAVE_ECC_SIGN
    if (sig == NULL || digest == NULL)
        goto done;
#endif
#endif /* WOLFSSL_SMALL_STACK && !WOLFSSL_NO_MALLOC */

    (void)testVerifyCount;
    (void)dp;
    (void)x;

#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    if ((userA == NULL) ||
        (userB == NULL) ||
        (pubKey == NULL))
        goto done;
#endif

    XMEMSET(userA, 0, sizeof *userA);
    XMEMSET(userB, 0, sizeof *userB);
    XMEMSET(pubKey, 0, sizeof *pubKey);

    ret = wc_ecc_init_ex(userA, HEAP_HINT, devId);
    if (ret != 0)
        goto done;
    ret = wc_ecc_init_ex(userB, HEAP_HINT, devId);
    if (ret != 0)
        goto done;
    ret = wc_ecc_init_ex(pubKey, HEAP_HINT, devId);
    if (ret != 0)
        goto done;

#ifndef WC_NO_RNG
    ret = wc_ecc_sm2_make_key(rng, userA, WC_ECC_FLAG_NONE);
    if (ret == WC_NO_ERR_TRACE(ECC_CURVE_OID_E))
        goto done; /* catch case, where curve is not supported */
    if (ret != 0)
        goto done;

    if (wc_ecc_get_curve_idx(curve_id) != -1) {
        curveSize = wc_ecc_get_curve_size_from_id(userA->dp->id);
        if (curveSize != userA->dp->size) {
            goto done;
        }
    }

    ret = wc_ecc_check_key(userA);
    if (ret != 0)
        goto done;

    ret = wc_ecc_sm2_make_key(rng, userB, WC_ECC_FLAG_NONE);
    if (ret != 0)
        goto done;

    /* only perform the below tests if the key size matches */
    if (dp == NULL && keySize > 0 && wc_ecc_size(userA) != keySize)
    if (ret != 0) {
        ret = ECC_CURVE_OID_E;
        goto done;
    }

#endif /* !WC_NO_RNG */

#if !defined(ECC_TIMING_RESISTANT) || (defined(ECC_TIMING_RESISTANT) && \
    !defined(WC_NO_RNG))
#ifdef HAVE_ECC_SIGN
    /* ECC w/out Shamir has issue with all 0 digest */
    /* WC_BIGINT doesn't have 0 len well on hardware */
    /* Cryptocell has issues with all 0 digest */
    /* test DSA sign hash with sequence (0,1,2,3,4,...) */
    for (i = 0; i < (int)ECC_DIGEST_SIZE; i++) {
        digest[i] = (byte)i;
    }

    x = ECC_SIG_SIZE;
    ret = wc_ecc_sm2_sign_hash(digest, ECC_DIGEST_SIZE, sig, &x, rng, userA);
    if (ret != 0)
        goto done;

    for (i = 0; i < testVerifyCount; i++) {
        verify = 0;
        ret = wc_ecc_sm2_verify_hash(sig, x, digest, ECC_DIGEST_SIZE, &verify,
                                                                         userA);
        if (ret != 0)
            goto done;
        if (verify != 1)
            goto done;
    }
#endif /* HAVE_ECC_SIGN */
#endif /* !ECC_TIMING_RESISTANT || (ECC_TIMING_RESISTANT && !WC_NO_RNG) */

done:

#if defined(WOLFSSL_SMALL_STACK) && !defined(WOLFSSL_NO_MALLOC)
    if (userA != NULL) {
        wc_ecc_free(userA);
        XFREE(userA, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
    if (userB != NULL) {
        wc_ecc_free(userB);
        XFREE(userB, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
    if (pubKey != NULL) {
        wc_ecc_free(pubKey);
        XFREE(pubKey, HEAP_HINT, DYNAMIC_TYPE_TMP_BUFFER);
    }
#else
    wc_ecc_free(pubKey);
    wc_ecc_free(userB);
    wc_ecc_free(userA);
#endif

#ifdef HAVE_ECC_SIGN
    WC_FREE_VAR(sig, HEAP_HINT);
    WC_FREE_VAR(digest, HEAP_HINT);
#endif

    (void)keySize;
    (void)curve_id;
    (void)rng;

    return ret;
}

static int test_sm2_verify_case(void)
{
    ecc_key key;
    int ret, res;

    /* test key values */
    const char qx[] = "637F1B135036C933DC3F7A8EBB1B7B2FD1DFBD268D4F894B5AD47DBDBECD558F";
    const char qy[] = "E88101D08048E36CCBF61CA38DDF7ABA542B4486E99E49F3A7470A857A096433";

    /* use canned hash value hash = H(ZA||M) */
    const byte hash[] = {
        0x3B,0xFA,0x5F,0xFB,0xC4,0x27,0x8C,0x9D,
        0x02,0x3A,0x19,0xCB,0x1E,0xAA,0xD2,0xF1,
        0x50,0x69,0x5B,0x20
    };

    const byte sig[] = {
        0x30,0x45,0x02,0x21,0x00,0xD2,0xFC,0xA3,
        0x88,0xE3,0xDF,0xA3,0x00,0x73,0x9B,0x3C,
        0x2A,0x0D,0xAD,0x44,0xA2,0xFC,0x62,0xD5,
        0x6B,0x84,0x54,0xD8,0x40,0x22,0x62,0x3D,
        0x5C,0xA6,0x61,0x9B,0xE7,0x02,0x20,0x1D,
        0xB5,0xB5,0xD9,0xD8,0xF1,0x20,0xDD,0x97,
        0x92,0xBF,0x7E,0x9B,0x3F,0xE6,0x3C,0x4B,
        0x03,0xD8,0x80,0xBD,0xB7,0x27,0x7E,0x6A,
        0x84,0x23,0xDE,0x61,0x7C,0x8D,0xDC
    };

    const byte badSig[] = {
        0x30,0x45,0x02,0x21,0x00,0xD2,0xFC,0xA3,
        0x88,0xE3,0xDF,0xA3,0x00,0x73,0x9B,0x3C,
        0x2A,0x0D,0xAD,0x44,0xA2,0xFC,0x62,0xD5,
        0x6B,0x84,0x54,0xD8,0x40,0x22,0x62,0x3D,
        0x5C,0xA6,0x61,0x9B,0xE7,0x02,0x20,0x1D,
        0xB5,0xB5,0xE9,0xD8,0xF1,0x20,0xDD,0x97,
        0x92,0xBF,0x7E,0x9B,0x3F,0xE6,0x3C,0x4B,
        0x03,0xD8,0x80,0xBD,0xB7,0x27,0x7E,0x6A,
        0x84,0x23,0xDE,0x61,0x7C,0x8D,0xDC
    };


    ret = wc_ecc_init_ex(&key, HEAP_HINT, devId);
    if (ret != 0)
        goto done;
    ret = wc_ecc_import_raw(&key, qx, qy, NULL, "SM2P256V1");
    if (ret != 0)
        goto done;

    ret = wc_ecc_sm2_verify_hash(sig, sizeof(sig), hash, sizeof(hash), &res,
            &key);
    if (ret != 0)
        goto done;

    if (res != 1)
        goto done;

    /* now test a case that should fail */
    ret = wc_ecc_sm2_verify_hash(badSig, sizeof(badSig), hash, sizeof(hash),
            &res, &key);
    if (ret != 0)
        goto done;

    if (res == 1)
        goto done;
done:
    wc_ecc_free(&key);
    return ret;
}

static int test_sm2_verify(void)
{
    int ret = 0;

#ifdef HAVE_ECC_VERIFY
    ret = test_sm2_verify_case();
    if (ret != 0)
        return ret;
#endif /* HAVE_ECC_VERIFY */

    return ret;
}

int wolfcrypt_ecc_test(void)
{
	int ret = 0;
    WC_RNG rng;
	printf(" wolfSSL version %s\n", LIBWOLFSSL_VERSION_STRING);
/* HAVE_ECC256 */
    ret = ecc_test_curve(&rng, 32, ECC_CURVE_DEF);
    if (ret != 0) {
        printf("keySize=32, test failed\n");
    }else
    {
        printf("keySize=32, test succeed\n");
    }
/* HAVE_ECC384 */
    ret = ecc_test_curve(&rng, 48, ECC_CURVE_DEF);
    if (ret != 0) {
        printf("keySize=48, test failed\n");
    }else
    {
        printf("keySize=48, test succeed\n");
    }

#if defined(WOLFSSL_SM2)
    ret = test_sm2_verify();
    if (ret != 0) {
        printf("SM2 Verify failed\n");
    }else
    {
        printf("SM2 Verify succeed\n");
    }
    ret = ecc_sm2_test_curve(&rng, ECC_TEST_VERIFY_COUNT);
    if (ret != 0) {
        printf("SM2 test failed\n");
    }else
    {
        printf("SM2 test succeed\n");
    }
#endif

    return ret;
} 

int main(void)
{
    // ls_otbn_delegation_server_chanels_init();
	int ret;
	if ((ret = wolfCrypt_Init()) != 0) {
		printf("wolfCrypt_Init failed %d\n", (int)ret);
	}

	return 0;
}