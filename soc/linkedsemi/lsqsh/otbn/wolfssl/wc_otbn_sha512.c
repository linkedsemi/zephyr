#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sha512.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include "otbn_hash.h"

static void sha512_output_raw(const uint8_t *state_bytes, byte *hash, int words)
{
    for (int i = 0; i < words; i++) {
        uint64_t w;
        memcpy(&w, state_bytes + i * 0x20, sizeof(w));
        *hash++ = (byte)(w >> 56);
        *hash++ = (byte)(w >> 48);
        *hash++ = (byte)(w >> 40);
        *hash++ = (byte)(w >> 32);
        *hash++ = (byte)(w >> 24);
        *hash++ = (byte)(w >> 16);
        *hash++ = (byte)(w >> 8);
        *hash++ = (byte)(w >> 0);
    }
}

int wc_InitSha512_ex(wc_Sha512* sha512, void* heap, int devId)
{
    int ret;

    if (sha512 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha512, 0, sizeof(wc_Sha512));
    ret = otbn_hash_init(&sha512->otbnCtx, OTBN_HASH_ALGO_SHA512);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha512->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha512Update(wc_Sha512* sha512, const byte* data, word32 len)
{
    int ret;

    if (sha512 == NULL || (data == NULL && len > 0)) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_update(&sha512->otbnCtx, data, len);
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

int wc_Sha512Final(wc_Sha512* sha512, byte* hash)
{
    int ret;

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha512->otbnCtx, hash);
    if (ret != 0) {
        return WC_HW_E;
    }

    return wc_InitSha512_ex(sha512, sha512->heap, INVALID_DEVID);
}

int wc_Sha512FinalRaw(wc_Sha512* sha512, byte* hash)
{
    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    sha512_output_raw(sha512->otbnCtx.state_bytes, hash, 8);
    return 0;
}

int wc_Sha512GetHash(wc_Sha512* sha512, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha512* tmpSha512;
#else
    wc_Sha512  tmpSha512[1];
#endif

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha512 = (wc_Sha512*)XMALLOC(sizeof(wc_Sha512), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha512 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha512Copy(sha512, tmpSha512);
    if (ret == 0) {
        ret = wc_Sha512Final(tmpSha512, hash);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha512, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif
    return ret;
}

int wc_Sha512Copy(wc_Sha512* src, wc_Sha512* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha512));
    return 0;
}

#if (defined(OPENSSL_EXTRA) || defined(HAVE_CURL)) && \
    !defined(WOLFSSL_KCAPI_HASH)
int wc_Sha512Transform(wc_Sha512* sha512, const unsigned char* data)
{
    int ret;

    if (sha512 == NULL || data == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef LITTLE_ENDIAN_ORDER
    {
        uint8_t swapped[WC_SHA512_BLOCK_SIZE];
        for (int i = 0; i < WC_SHA512_BLOCK_SIZE; i += 8) {
            swapped[i]     = data[i + 7];
            swapped[i + 1] = data[i + 6];
            swapped[i + 2] = data[i + 5];
            swapped[i + 3] = data[i + 4];
            swapped[i + 4] = data[i + 3];
            swapped[i + 5] = data[i + 2];
            swapped[i + 6] = data[i + 1];
            swapped[i + 7] = data[i];
        }
        ret = otbn_hash_transform(&sha512->otbnCtx, swapped);
    }
#else
    ret = otbn_hash_transform(&sha512->otbnCtx, data);
#endif
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

#if defined(WOLFSSL_SHA512) && !defined(WOLFSSL_NOSHA512_224) && \
    (!defined(HAVE_FIPS) || FIPS_VERSION_GE(5, 3)) && \
    !defined(HAVE_SELFTEST)
int wc_Sha512_224Transform(wc_Sha512* sha, const unsigned char* data)
{
    return wc_Sha512Transform(sha, data);
}
#endif /* SHA512/224 */

#if defined(WOLFSSL_SHA512) && !defined(WOLFSSL_NOSHA512_256) && \
    (!defined(HAVE_FIPS) || FIPS_VERSION_GE(5, 3)) && \
    !defined(HAVE_SELFTEST)
int wc_Sha512_256Transform(wc_Sha512* sha, const unsigned char* data)
{
    return wc_Sha512Transform(sha, data);
}
#endif /* SHA512/256 */
#endif /* OPENSSL_EXTRA || HAVE_CURL */

#ifdef WOLFSSL_SHA384
int wc_InitSha384_ex(wc_Sha384* sha384, void* heap, int devId)
{
    int ret;

    if (sha384 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha384, 0, sizeof(wc_Sha384));
    ret = otbn_hash_init(&sha384->otbnCtx, OTBN_HASH_ALGO_SHA384);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha384->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha384Update(wc_Sha384* sha384, const byte* data, word32 len)
{
    int ret;

    if (sha384 == NULL || (data == NULL && len > 0)) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_update(&sha384->otbnCtx, data, len);
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

int wc_Sha384Final(wc_Sha384* sha384, byte* hash)
{
    int ret;
    byte tmp[WC_SHA512_DIGEST_SIZE];

    if (sha384 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha384->otbnCtx, tmp);
    if (ret != 0) {
        return WC_HW_E;
    }
    XMEMCPY(hash, tmp, WC_SHA384_DIGEST_SIZE);

    return wc_InitSha384_ex(sha384, sha384->heap, INVALID_DEVID);
}

int wc_Sha384FinalRaw(wc_Sha384* sha384, byte* hash)
{
    if (sha384 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    sha512_output_raw(sha384->otbnCtx.state_bytes, hash, 6);
    return 0;
}

int wc_Sha384GetHash(wc_Sha384* sha384, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha384* tmpSha384;
#else
    wc_Sha384  tmpSha384[1];
#endif

    if (sha384 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha384 = (wc_Sha384*)XMALLOC(sizeof(wc_Sha384), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha384 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha384Copy(sha384, tmpSha384);
    if (ret == 0) {
        ret = wc_Sha384Final(tmpSha384, hash);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha384, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif
    return ret;
}

int wc_Sha384Copy(wc_Sha384* src, wc_Sha384* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha384));
    return 0;
}
#endif /* WOLFSSL_SHA384 */

#ifdef WOLFSSL_HASH_FLAGS
int wc_Sha512SetFlags(wc_Sha512* sha512, word32 flags)
{
    if (sha512) {
        sha512->flags = flags;
    }
    return 0;
}

int wc_Sha512GetFlags(wc_Sha512* sha512, word32* flags)
{
    if (sha512 && flags) {
        *flags = sha512->flags;
    }
    return 0;
}

#ifdef WOLFSSL_SHA384
int wc_Sha384SetFlags(wc_Sha384* sha384, word32 flags)
{
    if (sha384) {
        sha384->flags = flags;
    }
    return 0;
}

int wc_Sha384GetFlags(wc_Sha384* sha384, word32* flags)
{
    if (sha384 && flags) {
        *flags = sha384->flags;
    }
    return 0;
}
#endif /* WOLFSSL_SHA384 */
#endif /* WOLFSSL_HASH_FLAGS */

#if defined(WOLFSSL_SHA512) && !defined(WOLFSSL_NOSHA512_224) && \
    (!defined(HAVE_FIPS) || FIPS_VERSION_GE(5, 3)) && !defined(HAVE_SELFTEST)
int wc_InitSha512_224_ex(wc_Sha512* sha512, void* heap, int devId)
{
    int ret;

    if (sha512 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha512, 0, sizeof(wc_Sha512));
    ret = otbn_hash_init(&sha512->otbnCtx, OTBN_HASH_ALGO_SHA512);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha512->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha512_224FinalRaw(wc_Sha512* sha512, byte* hash)
{
    byte tmp[WC_SHA512_DIGEST_SIZE];

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    sha512_output_raw(sha512->otbnCtx.state_bytes, tmp, 8);
    XMEMCPY(hash, tmp, WC_SHA512_224_DIGEST_SIZE);
    return 0;
}

int wc_Sha512_224Final(wc_Sha512* sha512, byte* hash)
{
    int ret;
    byte tmp[WC_SHA512_DIGEST_SIZE];

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha512->otbnCtx, tmp);
    if (ret != 0) {
        return WC_HW_E;
    }
    XMEMCPY(hash, tmp, WC_SHA512_224_DIGEST_SIZE);

    return wc_InitSha512_224_ex(sha512, sha512->heap, INVALID_DEVID);
}

int wc_Sha512_224GetHash(wc_Sha512* sha512, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha512* tmpSha512;
#else
    wc_Sha512  tmpSha512[1];
#endif

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha512 = (wc_Sha512*)XMALLOC(sizeof(wc_Sha512), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha512 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha512Copy(sha512, tmpSha512);
    if (ret == 0) {
        ret = wc_Sha512_224Final(tmpSha512, hash);
        wc_Sha512Free(tmpSha512);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha512, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}

int wc_Sha512_224Copy(wc_Sha512* src, wc_Sha512* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha512));
    return 0;
}
#endif /* WOLFSSL_NOSHA512_224 */

#if defined(WOLFSSL_SHA512) && !defined(WOLFSSL_NOSHA512_256) && \
    (!defined(HAVE_FIPS) || FIPS_VERSION_GE(5, 3)) && !defined(HAVE_SELFTEST)
int wc_InitSha512_256_ex(wc_Sha512* sha512, void* heap, int devId)
{
    int ret;

    if (sha512 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha512, 0, sizeof(wc_Sha512));
    ret = otbn_hash_init(&sha512->otbnCtx, OTBN_HASH_ALGO_SHA512);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha512->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha512_256FinalRaw(wc_Sha512* sha512, byte* hash)
{
    byte tmp[WC_SHA512_DIGEST_SIZE];

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    sha512_output_raw(sha512->otbnCtx.state_bytes, tmp, 8);
    XMEMCPY(hash, tmp, WC_SHA512_256_DIGEST_SIZE);
    return 0;
}

int wc_Sha512_256Final(wc_Sha512* sha512, byte* hash)
{
    int ret;
    byte tmp[WC_SHA512_DIGEST_SIZE];

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha512->otbnCtx, tmp);
    if (ret != 0) {
        return WC_HW_E;
    }
    XMEMCPY(hash, tmp, WC_SHA512_256_DIGEST_SIZE);

    return wc_InitSha512_256_ex(sha512, sha512->heap, INVALID_DEVID);
}

int wc_Sha512_256GetHash(wc_Sha512* sha512, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha512* tmpSha512;
#else
    wc_Sha512  tmpSha512[1];
#endif

    if (sha512 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha512 = (wc_Sha512*)XMALLOC(sizeof(wc_Sha512), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha512 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha512Copy(sha512, tmpSha512);
    if (ret == 0) {
        ret = wc_Sha512_256Final(tmpSha512, hash);
        wc_Sha512Free(tmpSha512);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha512, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}

int wc_Sha512_256Copy(wc_Sha512* src, wc_Sha512* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha512));
    return 0;
}
#endif /* WOLFSSL_NOSHA512_256 */
