#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sm3.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include "otbn_hash.h"

int wc_InitSm3(wc_Sm3* sm3, void* heap, int devId)
{
    int ret;

    if (sm3 == NULL) {
        return BAD_FUNC_ARG;
    }

    XMEMSET(sm3, 0, sizeof(wc_Sm3));
    ret = otbn_hash_init(&sm3->otbnCtx, OTBN_HASH_ALGO_SM3);
    if (ret != 0) {
        return WC_HW_E;
    }

    sm3->heap = heap;
    (void)devId;
    return 0;
}

int wc_Sm3Update(wc_Sm3* sm3, const byte* data, word32 len)
{
    int ret;

    if (sm3 == NULL || (data == NULL && len > 0)) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_update(&sm3->otbnCtx, data, len);
    if (ret != 0) {
        return WC_HW_E;
    }
    return 0;
}

int wc_Sm3Final(wc_Sm3* sm3, byte* hash)
{
    int ret;

    if (sm3 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = otbn_hash_final(&sm3->otbnCtx, hash);
    if (ret != 0) {
        return WC_HW_E;
    }

    return wc_InitSm3(sm3, sm3->heap, INVALID_DEVID);
}

int wc_Sm3FinalRaw(wc_Sm3* sm3, byte* hash)
{
    const uint32_t *s;
    word32 w;
    int i;

    if (sm3 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    /* Return the current intermediate state in big-endian bytes. */
    s = (const uint32_t *)sm3->otbnCtx.state_bytes;
    for (i = 0; i < 8; i++) {
        w = s[i];
        *hash++ = (byte)(w >> 24);
        *hash++ = (byte)(w >> 16);
        *hash++ = (byte)(w >> 8);
        *hash++ = (byte)(w >> 0);
    }
    return 0;
}

int wc_Sm3Copy(const wc_Sm3* src, wc_Sm3* dst)
{
    if (src == NULL || dst == NULL) {
        return BAD_FUNC_ARG;
    }
    XMEMCPY((void*)dst, (const void*)src, sizeof(wc_Sm3));
    return 0;
}

int wc_Sm3GetHash(wc_Sm3* sm3, byte* hash)
{
    int ret;
    wc_Sm3 tmpSm3;

    if (sm3 == NULL || hash == NULL) {
        return BAD_FUNC_ARG;
    }

    ret = wc_Sm3Copy(sm3, &tmpSm3);
    if (ret == 0) {
        ret = wc_Sm3Final(&tmpSm3, hash);
    }
    return ret;
}

void wc_Sm3Free(wc_Sm3* sm3)
{
    if (sm3 == NULL) {
        return;
    }
    XMEMSET(sm3, 0, sizeof(wc_Sm3));
}

#ifdef WOLFSSL_HASH_FLAGS
int wc_Sm3SetFlags(wc_Sm3* sm3, word32 flags)
{
    if (sm3) {
        sm3->flags = flags;
    }
    return 0;
}

int wc_Sm3GetFlags(const wc_Sm3* sm3, word32* flags)
{
    if (sm3 && flags) {
        *flags = sm3->flags;
    }
    return 0;
}
#endif /* WOLFSSL_HASH_FLAGS */
