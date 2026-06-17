#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sha256.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include "otbn_hash.h"

int wc_InitSha256_ex(wc_Sha256* sha256, void* heap, int devId)
{
    int ret;

    if (sha256 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha256, 0, sizeof(wc_Sha256));
    ret = otbn_hash_init(&sha256->otbnCtx, OTBN_HASH_ALGO_SHA256);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha256->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha256Update(wc_Sha256* sha256, const byte* data, word32 len)
{
    int ret;

    if (sha256 == NULL || (data == NULL && len > 0)) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_update(&sha256->otbnCtx, data, len);
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

int wc_Sha256Final(wc_Sha256* sha256, byte* hash)
{
    int ret;

    if (sha256 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha256->otbnCtx, hash);
    if (ret != 0) {
        return WC_HW_E;
    }

    return wc_InitSha256_ex(sha256, sha256->heap, INVALID_DEVID);
}

int wc_Sha256FinalRaw(wc_Sha256* sha256, byte* hash)
{
    const uint32_t *s;
    word32 w;
    int i;

    if (sha256 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    /* Return the current intermediate state (H7..H0) in big-endian bytes. */
    s = (const uint32_t *)sha256->otbnCtx.state_bytes;
    for (i = 0; i < 8; i++) {
        w = s[7 - i];
        *hash++ = (byte)(w >> 24);
        *hash++ = (byte)(w >> 16);
        *hash++ = (byte)(w >> 8);
        *hash++ = (byte)(w >> 0);
    }
    return 0;
}

int wc_Sha256GetHash(wc_Sha256* sha256, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha256* tmpSha256;
#else
    wc_Sha256  tmpSha256[1];
#endif

    if (sha256 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha256 = (wc_Sha256*)XMALLOC(sizeof(wc_Sha256), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha256 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha256Copy(sha256, tmpSha256);
    if (ret == 0) {
        ret = wc_Sha256Final(tmpSha256, hash);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha256, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif
    return ret;
}

int wc_Sha256Copy(wc_Sha256* src, wc_Sha256* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha256));
    return 0;
}

#if !defined(WOLFSSL_KCAPI_HASH) && !defined(WOLFSSL_AFALG_HASH)
#if defined(OPENSSL_EXTRA) || defined(HAVE_CURL)
int wc_Sha256Transform(wc_Sha256* sha256, const unsigned char* data)
{
    int ret;

    if (sha256 == NULL || data == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef LITTLE_ENDIAN_ORDER
    {
        uint8_t swapped[WC_SHA256_BLOCK_SIZE];
        for (int i = 0; i < WC_SHA256_BLOCK_SIZE; i += 4) {
            swapped[i]     = data[i + 3];
            swapped[i + 1] = data[i + 2];
            swapped[i + 2] = data[i + 1];
            swapped[i + 3] = data[i];
        }
        ret = otbn_hash_transform(&sha256->otbnCtx, swapped);
    }
#else
    ret = otbn_hash_transform(&sha256->otbnCtx, data);
#endif
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}
#endif /* OPENSSL_EXTRA || HAVE_CURL */
#endif /* !WOLFSSL_KCAPI_HASH && !WOLFSSL_AFALG_HASH */

#ifdef WOLFSSL_SHA224
int wc_InitSha224_ex(wc_Sha224* sha224, void* heap, int devId)
{
    int ret;

    if (sha224 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sha224, 0, sizeof(wc_Sha224));
    ret = otbn_hash_init(&sha224->otbnCtx, OTBN_HASH_ALGO_SHA224);
    if (ret != 0) {
        return WC_HW_E;
    }

    sha224->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sha224Update(wc_Sha224* sha224, const byte* data, word32 len)
{
    int ret;

    if (sha224 == NULL || (data == NULL && len > 0)) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_update(&sha224->otbnCtx, data, len);
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

int wc_Sha224Final(wc_Sha224* sha224, byte* hash)
{
    int ret;
    byte tmp[WC_SHA256_DIGEST_SIZE];

    if (sha224 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sha224->otbnCtx, tmp);
    if (ret != 0) {
        return WC_HW_E;
    }
    XMEMCPY(hash, tmp, WC_SHA224_DIGEST_SIZE);

    return wc_InitSha224_ex(sha224, sha224->heap, INVALID_DEVID);
}

int wc_Sha224FinalRaw(wc_Sha224* sha224, byte* hash)
{
    return wc_Sha256FinalRaw((wc_Sha256*)sha224, hash);
}

int wc_Sha224GetHash(wc_Sha224* sha224, byte* hash)
{
    int ret;
#ifdef WOLFSSL_SMALL_STACK
    wc_Sha224* tmpSha224;
#else
    wc_Sha224  tmpSha224[1];
#endif

    if (sha224 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

#ifdef WOLFSSL_SMALL_STACK
    tmpSha224 = (wc_Sha224*)XMALLOC(sizeof(wc_Sha224), NULL,
        DYNAMIC_TYPE_TMP_BUFFER);
    if (tmpSha224 == NULL) {
        return MEMORY_E;
    }
#endif

    ret = wc_Sha224Copy(sha224, tmpSha224);
    if (ret == 0) {
        ret = wc_Sha224Final(tmpSha224, hash);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(tmpSha224, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif
    return ret;
}

int wc_Sha224Copy(wc_Sha224* src, wc_Sha224* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY(dst, src, sizeof(wc_Sha224));
    return 0;
}
#endif /* WOLFSSL_SHA224 */

#ifdef WOLFSSL_HASH_FLAGS
int wc_Sha256SetFlags(wc_Sha256* sha256, word32 flags)
{
    if (sha256) {
        sha256->flags = flags;
    }
    return 0;
}

int wc_Sha256GetFlags(wc_Sha256* sha256, word32* flags)
{
    if (sha256 && flags) {
        *flags = sha256->flags;
    }
    return 0;
}

#ifdef WOLFSSL_SHA224
int wc_Sha224SetFlags(wc_Sha224* sha224, word32 flags)
{
    if (sha224) {
        sha224->flags = flags;
    }
    return 0;
}

int wc_Sha224GetFlags(wc_Sha224* sha224, word32* flags)
{
    if (sha224 && flags) {
        *flags = sha224->flags;
    }
    return 0;
}
#endif /* WOLFSSL_SHA224 */
#endif /* WOLFSSL_HASH_FLAGS */
