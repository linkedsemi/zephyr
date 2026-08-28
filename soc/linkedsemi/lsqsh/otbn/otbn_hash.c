#include "otbn_hash.h"
#include <string.h>
#include <errno.h>
#include <stddef.h>
#include "ls_otbn_config.h"
#include "ls_hal_otbn.h"

/* Compile-time layout checks: all members used for DMEM I/O must be
 * 4-byte aligned so they can be passed to ls_otbn_dmem_write/read. */
_Static_assert(sizeof(otbn_hash_algo_t) == 4,
               "otbn_hash_algo_t must be 4 bytes");
_Static_assert(sizeof(otbn_hash_ctx_t) % 4 == 0,
               "otbn_hash_ctx_t size must be 4-byte aligned");
_Static_assert(offsetof(otbn_hash_ctx_t, algo) % 4 == 0, "algo offset");
_Static_assert(offsetof(otbn_hash_ctx_t, firmware_id) % 4 == 0,
               "firmware_id offset");
_Static_assert(offsetof(otbn_hash_ctx_t, total_len) % 4 == 0,
               "total_len offset");
_Static_assert(offsetof(otbn_hash_ctx_t, remain_len) % 4 == 0,
               "remain_len offset");
_Static_assert(offsetof(otbn_hash_ctx_t, dmem_msg_idx) % 4 == 0,
               "dmem_msg_idx offset");
_Static_assert(offsetof(otbn_hash_ctx_t, remain_data) % 4 == 0,
               "remain_data offset");
_Static_assert(offsetof(otbn_hash_ctx_t, state_bytes) % 4 == 0,
               "state_bytes offset");

/* Block sizes for hash algorithms */
#define SHA256_BLOCK_SIZE   64
#define SM3_BLOCK_SIZE      64
#define SHA512_BLOCK_SIZE   128

/* Firmware images (defined in modules/hal/linkedsemi/hal_driver/src/otbn/text_array/) */
extern const char sha256_text[540];
extern const char sha512_text[1588];
extern const char sm3_text[1000];
extern const char sm3_dmem[320];

/* ========================================================================
 * SHA-256 constants / layout
 * ======================================================================== */
#define SHA256_IMEM_SIZE            540
#define SHA256_STATE_OFFSET         0x0
#define SHA256_STATE_SIZE           0x20
#define SHA256_MASK_OFFSET          0x20
#define SHA256_MASK_SIZE            0x20
#define SHA256_K_OFFSET             0x40
#define SHA256_K_SIZE               0x100
#define SHA256_MSG_OFFSET           0x140
#define SHA256_MSG_SIZE             0x800
#define SHA256_MSG_BLOCKS           (SHA256_MSG_SIZE / SHA256_BLOCK_SIZE)
#define SHA256_IMEM_BLOCKNUM_OFFSET 0x0
#define SHA256_IMEM_BLOCKNUM_DATA   0x00000f13

static const uint32_t sha256_state_init[8] = {
    0x5be0cd19, 0x1f83d9ab, 0x9b05688c, 0x510e527f,
    0xa54ff53a, 0x3c6ef372, 0xbb67ae85, 0x6a09e667
};

static const uint32_t sha224_state_init[8] = {
    0xbefa4fa4, 0x64f98fa7, 0x68581511, 0xffc00b31,
    0xf70e5939, 0x3070dd17, 0x367cd507, 0xc1059ed8
};

static const uint32_t sha256_bswap32_mask[8] = {
    0x000000ff, 0x000000ff, 0x000000ff, 0x000000ff,
    0x000000ff, 0x000000ff, 0x000000ff, 0x000000ff
};

static const uint32_t sha256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

/* ========================================================================
 * SHA-384 / SHA-512 constants / layout
 * ======================================================================== */
#define SHA512_IMEM_SIZE            1600
#define SHA512_STATE_OFFSET         0x0
#define SHA512_STATE_SIZE           0x100
#define SHA512_MSG_OFFSET           0x3a0
#define SHA512_MSG_SIZE             0x818
#define SHA512_MSG_BLOCKS           (SHA512_MSG_SIZE / SHA512_BLOCK_SIZE)
#define SHA512_N_CHUNKS_OFFSET      0x380
#define SHA512_PTR_STATE_OFFSET     0xbbc
#define SHA512_PTR_MSG_OFFSET       0xbc0
#define SHA512_K_OFFSET             0x100
#define SHA512_K_SIZE               0x280

static const uint64_t sha384_state_init[64] = {
    0xcbbb9d5dc1059ed8, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x629a292a367cd507, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x9159015a3070dd17, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x152fecd8f70e5939, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x67332667ffc00b31, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x8eb44a8768581511, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0xdb0c2e0d64f98fa7, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x47b5481dbefa4fa4, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
};

static const uint64_t sha512_state_init[64] = {
    0x6a09e667f3bcc908, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0xbb67ae8584caa73b, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x3c6ef372fe94f82b, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0xa54ff53a5f1d36f1, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x510e527fade682d1, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x9b05688c2b3e6c1f, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x1f83d9abfb41bd6b, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
    0x5be0cd19137e2179, 0x0000000000000000, 0x0000000000000000, 0x0000000000000000,
};

static const uint64_t sha512_K[80] = {
    0x428a2f98d728ae22, 0x7137449123ef65cd, 0xb5c0fbcfec4d3b2f, 0xe9b5dba58189dbbc,
    0x3956c25bf348b538, 0x59f111f1b605d019, 0x923f82a4af194f9b, 0xab1c5ed5da6d8118,
    0xd807aa98a3030242, 0x12835b0145706fbe, 0x243185be4ee4b28c, 0x550c7dc3d5ffb4e2,
    0x72be5d74f27b896f, 0x80deb1fe3b1696b1, 0x9bdc06a725c71235, 0xc19bf174cf692694,
    0xe49b69c19ef14ad2, 0xefbe4786384f25e3, 0x0fc19dc68b8cd5b5, 0x240ca1cc77ac9c65,
    0x2de92c6f592b0275, 0x4a7484aa6ea6e483, 0x5cb0a9dcbd41fbd4, 0x76f988da831153b5,
    0x983e5152ee66dfab, 0xa831c66d2db43210, 0xb00327c898fb213f, 0xbf597fc7beef0ee4,
    0xc6e00bf33da88fc2, 0xd5a79147930aa725, 0x06ca6351e003826f, 0x142929670a0e6e70,
    0x27b70a8546d22ffc, 0x2e1b21385c26c926, 0x4d2c6dfc5ac42aed, 0x53380d139d95b3df,
    0x650a73548baf63de, 0x766a0abb3c77b2a8, 0x81c2c92e47edaee6, 0x92722c851482353b,
    0xa2bfe8a14cf10364, 0xa81a664bbc423001, 0xc24b8b70d0f89791, 0xc76c51a30654be30,
    0xd192e819d6ef5218, 0xd69906245565a910, 0xf40e35855771202a, 0x106aa07032bbd1b8,
    0x19a4c116b8d2d0c8, 0x1e376c085141ab53, 0x2748774cdf8eeb99, 0x34b0bcb5e19b48a8,
    0x391c0cb3c5c95a63, 0x4ed8aa4ae3418acb, 0x5b9cca4f7763e373, 0x682e6ff3d6b2b8a3,
    0x748f82ee5defb2fc, 0x78a5636f43172f60, 0x84c87814a1f0ab72, 0x8cc702081a6439ec,
    0x90befffa23631e28, 0xa4506cebde82bde9, 0xbef9a3f7b2c67915, 0xc67178f2e372532b,
    0xca273eceea26619c, 0xd186b8c721c0c207, 0xeada7dd6cde0eb1e, 0xf57d4f7fee6ed178,
    0x06f067aa72176fba, 0x0a637dc5a2c898a6, 0x113f9804bef90dae, 0x1b710b35131c471b,
    0x28db77f523047d84, 0x32caab7b40c72493, 0x3c9ebe0a15c9bebc, 0x431d67c49c100d4c,
    0x4cc5d4becb3e42b6, 0x597f299cfc657e2a, 0x5fcb6fab3ad6faec, 0x6c44198c4a475817
};

/* ========================================================================
 * SM3 constants / layout
 * ======================================================================== */
#define SM3_IMEM_SIZE               1000
#define SM3_STATE_OFFSET            0x120
#define SM3_STATE_SIZE              0x20
#define SM3_BLOCKNUM_OFFSET         0xBC0
#define SM3_MSG_OFFSET              0x380
#define SM3_MSG_SIZE                0x840
#define SM3_MAX_CHUNKS              33

static const uint32_t sm3_state_init[8] = {
    0x7380166f, 0x4914b2b9, 0x172442d7, 0xda8a0600,
    0xa96f30bc, 0x163138aa, 0xe38dee4d, 0xb0fb0e4e
};

/* ========================================================================
 * Helpers
 * ======================================================================== */
static inline uint32_t min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static int dmem_write_aligned(uint32_t offset, const void *src, uint32_t size)
{
    /* ls_otbn_dmem_write expects 4-byte aligned offset/size and a uint32_t* */
    return ls_otbn_dmem_write(offset, (const uint32_t *)src, size);
}

static int dmem_read_aligned(uint32_t offset, void *dst, uint32_t size)
{
    return ls_otbn_dmem_read(offset, (uint32_t *)dst, size);
}

static int write_block(uint32_t dmem_offset, const uint8_t *src, uint32_t size)
{
    return ls_otbn_dmem_write(dmem_offset, (const uint32_t *)src, size);
}

static int load_firmware(otbn_hash_ctx_t *ctx)
{
    int ret;

    /* Scrub the whole DMEM before every hash run. OTBN hash firmware may
     * read parts of the message buffer or scratch areas that are not filled
     * by the current operation; stale data left behind by a previous session
     * can have corrupted integrity codes and triggers DMEM_INTG_VIOLATION
     * (observed as a wait-idle timeout and ERR_BITS = 0x20000 on SHA-512
     * final). This must happen even when the firmware ID is the same, because
     * a different context may have left scratch data behind. */
    // ret = ls_otbn_dmem_set(0, 0, OTBN_DMEM_SIZE);
    // if (ret != 0) {
    //     return ret;
    // }

    /* IMEM content is tracked across sessions (ls_otbn_imem_firmware_*):
     * a session held by another module (RSA/ECC) may have replaced the
     * image, so the private-cache shortcut must consult the shared state. */
    if (ls_otbn_imem_firmware_get() == ctx->firmware_id) {
        return 0;
    }

    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
        ret = ls_otbn_imem_write(0, (const uint32_t *)sha256_text, SHA256_IMEM_SIZE);
        if (ret) return ret;
        ret = dmem_write_aligned(SHA256_MASK_OFFSET, sha256_bswap32_mask, SHA256_MASK_SIZE);
        if (ret) return ret;
        ret = dmem_write_aligned(SHA256_K_OFFSET, sha256_K, SHA256_K_SIZE);
        break;
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:{
        ret = ls_otbn_imem_write(0, (const uint32_t *)sha512_text, SHA512_IMEM_SIZE);
        if (ret) return ret;
        uint32_t ptr_state = SHA512_STATE_OFFSET;
        uint32_t ptr_msg = SHA512_MSG_OFFSET;
        ret = dmem_write_aligned(SHA512_K_OFFSET, sha512_K, SHA512_K_SIZE);
        if (ret) return ret;
        ret = ls_otbn_dmem_write(SHA512_PTR_STATE_OFFSET, &ptr_state, sizeof(uint32_t));
        if (ret) return ret;
        ret = ls_otbn_dmem_write(SHA512_PTR_MSG_OFFSET, &ptr_msg, sizeof(uint32_t));
        break;
        }
    case OTBN_HASH_ALGO_SM3:
        ret = ls_otbn_imem_write(0, (const uint32_t *)sm3_text, SM3_IMEM_SIZE);
        if (ret) return ret;
        ret = dmem_write_aligned(0, sm3_dmem, 320);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    if (ret == 0) {
        ls_otbn_imem_firmware_confirm(ctx->firmware_id);
    }
    return ret;
}

static int restore_state(otbn_hash_ctx_t *ctx)
{
    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
        return dmem_write_aligned(SHA256_STATE_OFFSET, ctx->state_bytes, SHA256_STATE_SIZE);
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:
        return dmem_write_aligned(SHA512_STATE_OFFSET, ctx->state_bytes, SHA512_STATE_SIZE);
    case OTBN_HASH_ALGO_SM3:
        return dmem_write_aligned(SM3_STATE_OFFSET, ctx->state_bytes, SM3_STATE_SIZE);
    default:
        return -EINVAL;
    }
}

static int save_state(otbn_hash_ctx_t *ctx)
{
    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
        return dmem_read_aligned(SHA256_STATE_OFFSET, ctx->state_bytes, SHA256_STATE_SIZE);
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:
        return dmem_read_aligned(SHA512_STATE_OFFSET, ctx->state_bytes, SHA512_STATE_SIZE);
    case OTBN_HASH_ALGO_SM3:
        return dmem_read_aligned(SM3_STATE_OFFSET, ctx->state_bytes, SM3_STATE_SIZE);
    default:
        return -EINVAL;
    }
}

static int execute_sha256(uint32_t block_count)
{
    uint32_t insruct = SHA256_IMEM_BLOCKNUM_DATA | ((block_count << 20) & 0xfff00000);
    int ret = ls_otbn_imem_write(SHA256_IMEM_BLOCKNUM_OFFSET, &insruct, sizeof(uint32_t));
    if (ret) return ret;
    return ls_otbn_cmd(OTBN_CMD_EXECUTE);
}

static int execute_sha512(uint32_t block_count)
{
    int ret = ls_otbn_dmem_write(SHA512_N_CHUNKS_OFFSET, &block_count, sizeof(uint32_t));
    if (ret) return ret;
    return ls_otbn_cmd(OTBN_CMD_EXECUTE);
}

static int execute_sm3(uint32_t chunk_count)
{
    int ret = ls_otbn_dmem_write(SM3_BLOCKNUM_OFFSET, &chunk_count, sizeof(uint32_t));
    if (ret) return ret;
    return ls_otbn_cmd(OTBN_CMD_EXECUTE);
}

static int sha256_process_block(otbn_hash_ctx_t *ctx, const uint8_t *block)
{
    int ret = write_block(ctx->dmem_msg_idx, block, SHA256_BLOCK_SIZE);
    if (ret) return ret;
    ctx->dmem_msg_idx += SHA256_BLOCK_SIZE;
    if (ctx->dmem_msg_idx >= SHA256_MSG_OFFSET + SHA256_MSG_SIZE) {
        ret = execute_sha256(SHA256_MSG_BLOCKS);
        if (ret) return ret;
        ctx->dmem_msg_idx = SHA256_MSG_OFFSET;
    }
    return 0;
}

static int sha512_write_block(uint32_t dmem_offset, const uint8_t *src)
{
    uint64_t block[SHA512_BLOCK_SIZE / sizeof(uint64_t)];
    uint8_t dword[8];
    const uint8_t *p = src;

    for (int i = 0; i < (int)(SHA512_BLOCK_SIZE / sizeof(uint64_t)); i++) {
        for (int j = 7; j >= 0; j--) {
            dword[j] = *p++;
        }
        memcpy(&block[i], dword, 8);
    }
    return ls_otbn_dmem_write(dmem_offset, (const uint32_t *)block, SHA512_BLOCK_SIZE);
}

static int sha512_process_block(otbn_hash_ctx_t *ctx, const uint8_t *block)
{
    int ret = sha512_write_block(ctx->dmem_msg_idx, block);
    if (ret) 
    {
        return ret;
    }
    ctx->dmem_msg_idx += SHA512_BLOCK_SIZE;
    if (ctx->dmem_msg_idx >= SHA512_MSG_OFFSET +
        SHA512_MSG_BLOCKS * SHA512_BLOCK_SIZE) {
        ret = execute_sha512(SHA512_MSG_BLOCKS);
        if (ret) return ret;
        ctx->dmem_msg_idx = SHA512_MSG_OFFSET;
    }
    return 0;
}

static int sm3_process_blocks(otbn_hash_ctx_t *ctx, const uint8_t *data,
                              uint32_t chunk_count)
{
    uint32_t written = 0;

    while (chunk_count > 0) {
        uint32_t batch = (chunk_count > SM3_MAX_CHUNKS) ? SM3_MAX_CHUNKS : chunk_count;
        int ret;

        ret = restore_state(ctx);
        if (ret) return ret;

        for (uint32_t i = 0; i < batch; i++) {
            ret = write_block(SM3_MSG_OFFSET + i * SM3_BLOCK_SIZE,
                              data + written * SM3_BLOCK_SIZE + i * SM3_BLOCK_SIZE,
                              SM3_BLOCK_SIZE);
            if (ret) return ret;
        }

        ret = execute_sm3(batch);
        if (ret) return ret;
        ret = save_state(ctx);
        if (ret) return ret;

        chunk_count -= batch;
        written += batch;
    }
    return 0;
}

/* ========================================================================
 * Algorithm-specific output helpers
 * ======================================================================== */
static void output_sha256(const uint8_t *state, uint8_t *digest)
{
    const uint32_t *s = (const uint32_t *)state;
    for (int i = 0; i < 8; i++) {
        uint32_t w = s[7 - i];
        *digest++ = (uint8_t)(w >> 24);
        *digest++ = (uint8_t)(w >> 16);
        *digest++ = (uint8_t)(w >> 8);
        *digest++ = (uint8_t)(w >> 0);
    }
}

static void output_sha224(const uint8_t *state, uint8_t *digest)
{
    const uint32_t *s = (const uint32_t *)state;
    /* SHA-224 digest is the first 7 words (H0..H6) of the 8-word state.
     * DMEM stores the state in reverse word order (H7..H0), so we skip
     * the first word and output the remaining 7 in reverse. */
    for (int i = 0; i < 7; i++) {
        uint32_t w = s[7 - i];
        *digest++ = (uint8_t)(w >> 24);
        *digest++ = (uint8_t)(w >> 16);
        *digest++ = (uint8_t)(w >> 8);
        *digest++ = (uint8_t)(w >> 0);
    }
}

static void output_sm3(const uint8_t *state, uint8_t *digest)
{
    const uint32_t *s = (const uint32_t *)state;
    /* SM3 state is stored H0..H7 in DMEM, so output in native order. */
    for (int i = 0; i < 8; i++) {
        uint32_t w = s[i];
        *digest++ = (uint8_t)(w >> 24);
        *digest++ = (uint8_t)(w >> 16);
        *digest++ = (uint8_t)(w >> 8);
        *digest++ = (uint8_t)(w >> 0);
    }
}

static void output_sha512(const uint8_t *state, uint8_t *digest, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++) {
        uint64_t w;
        memcpy(&w, state + i * 0x20, sizeof(w));
        *digest++ = (uint8_t)(w >> 56);
        *digest++ = (uint8_t)(w >> 48);
        *digest++ = (uint8_t)(w >> 40);
        *digest++ = (uint8_t)(w >> 32);
        *digest++ = (uint8_t)(w >> 24);
        *digest++ = (uint8_t)(w >> 16);
        *digest++ = (uint8_t)(w >> 8);
        *digest++ = (uint8_t)(w >> 0);
    }
}

/* ========================================================================
 * Public API
 * ======================================================================== */
int otbn_hash_init(otbn_hash_ctx_t *ctx, otbn_hash_algo_t algo)
{
    if (ctx == NULL) {
        return -EINVAL;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->algo = algo;

    switch (algo) {
    case OTBN_HASH_ALGO_SHA224:
        ctx->firmware_id = OTBN_FIRMWARE_SHA256;
        ctx->dmem_msg_idx = SHA256_MSG_OFFSET;
        memcpy(ctx->state_bytes, sha224_state_init, SHA256_STATE_SIZE);
        break;
    case OTBN_HASH_ALGO_SHA256:
        ctx->firmware_id = OTBN_FIRMWARE_SHA256;
        ctx->dmem_msg_idx = SHA256_MSG_OFFSET;
        memcpy(ctx->state_bytes, sha256_state_init, SHA256_STATE_SIZE);
        break;
    case OTBN_HASH_ALGO_SHA384:
        ctx->firmware_id = OTBN_FIRMWARE_SHA384;
        ctx->dmem_msg_idx = SHA512_MSG_OFFSET;
        memcpy(ctx->state_bytes, sha384_state_init, SHA512_STATE_SIZE);
        break;
    case OTBN_HASH_ALGO_SHA512:
        ctx->firmware_id = OTBN_FIRMWARE_SHA512;
        ctx->dmem_msg_idx = SHA512_MSG_OFFSET;
        memcpy(ctx->state_bytes, sha512_state_init, SHA512_STATE_SIZE);
        break;
    case OTBN_HASH_ALGO_SM3:
        ctx->firmware_id = OTBN_FIRMWARE_SM3;
        ctx->dmem_msg_idx = SM3_MSG_OFFSET;
        memcpy(ctx->state_bytes, sm3_state_init, SM3_STATE_SIZE);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

int otbn_hash_update(otbn_hash_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    if (ctx == NULL || (data == NULL && len > 0)) {
        return -EINVAL;
    }
    if (len == 0) {
        return 0;
    }

    ctx->total_len += len;

    /* Drain existing partial block first */
    if (ctx->remain_len > 0) {
        uint32_t need = 0;
        uint32_t block_size;

        switch (ctx->algo) {
        case OTBN_HASH_ALGO_SHA224:
        case OTBN_HASH_ALGO_SHA256:
        case OTBN_HASH_ALGO_SM3:
            block_size = SHA256_BLOCK_SIZE;
            break;
        case OTBN_HASH_ALGO_SHA384:
        case OTBN_HASH_ALGO_SHA512:
            block_size = SHA512_BLOCK_SIZE;
            break;
        default:
            return -EINVAL;
        }

        need = block_size - ctx->remain_len;
        uint32_t copy_len = min_u32(len, need);
        memcpy(ctx->remain_data + ctx->remain_len, data, copy_len);
        ctx->remain_len += copy_len;
        data += copy_len;
        len -= copy_len;

        if (ctx->remain_len < block_size) {
            return 0;
        }
    }

    /* Figure out whether there is any full block to process */
    uint32_t block_size;
    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
    case OTBN_HASH_ALGO_SM3:
        block_size = SHA256_BLOCK_SIZE;
        break;
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:
        block_size = SHA512_BLOCK_SIZE;
        break;
    default:
        return -EINVAL;
    }

    bool have_full_block = (ctx->remain_len >= block_size) || (len >= block_size);
    if (!have_full_block) {
        if (len > 0) {
            memcpy(ctx->remain_data + ctx->remain_len, data, len);
            ctx->remain_len += len;
        }
        return 0;
    }

    int ret = ls_otbn_session_acquire(ctx->firmware_id, 10);
    if (ret) return ret;

    ret = load_firmware(ctx);
    if (ret) goto out_release;

    ret = restore_state(ctx);
    if (ret) goto out_release;

    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256: {
        /* Process the pending full block from remain_data */
        if (ctx->remain_len >= SHA256_BLOCK_SIZE) {
            ret = sha256_process_block(ctx, ctx->remain_data);
            if (ret) goto out_release;
            ctx->remain_len = 0;
        }
        while (len >= SHA256_BLOCK_SIZE) {
            ret = sha256_process_block(ctx, data);
            if (ret) goto out_release;
            data += SHA256_BLOCK_SIZE;
            len -= SHA256_BLOCK_SIZE;
        }
        break;
    }
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512: {
        if (ctx->remain_len >= SHA512_BLOCK_SIZE) {
            ret = sha512_process_block(ctx, ctx->remain_data);
            if (ret) goto out_release;
            ctx->remain_len = 0;
        }
        while (len >= SHA512_BLOCK_SIZE) {
            ret = sha512_process_block(ctx, data);
            if (ret) goto out_release;
            data += SHA512_BLOCK_SIZE;
            len -= SHA512_BLOCK_SIZE;
        }
        break;
    }
    case OTBN_HASH_ALGO_SM3: {
        uint32_t chunks;
        if (ctx->remain_len >= SM3_BLOCK_SIZE) {
            /* remain_data has one full chunk */
            chunks = 1 + (len / SM3_BLOCK_SIZE);
            uint32_t process_len = chunks * SM3_BLOCK_SIZE;
            if (process_len <= sizeof(ctx->remain_data) + 128) {
                /* concatenate into a temporary buffer on stack */
                uint8_t buf[SM3_MAX_CHUNKS * SM3_BLOCK_SIZE];
                uint32_t total_input_len = SM3_BLOCK_SIZE + len;
                uint32_t process_input_len = process_len;
                if (process_input_len > total_input_len) {
                    process_input_len = total_input_len;
                    chunks = process_input_len / SM3_BLOCK_SIZE;
                }
                memcpy(buf, ctx->remain_data, SM3_BLOCK_SIZE);
                memcpy(buf + SM3_BLOCK_SIZE, data, process_input_len - SM3_BLOCK_SIZE);
                ret = sm3_process_blocks(ctx, buf, chunks);
                if (ret) goto out_release;
                data += (process_input_len - SM3_BLOCK_SIZE);
                len -= (process_input_len - SM3_BLOCK_SIZE);
                ctx->remain_len = 0;
            } else {
                /* Process the single remain chunk first */
                ret = sm3_process_blocks(ctx, ctx->remain_data, 1);
                if (ret) goto out_release;
                ctx->remain_len = 0;
            }
        }
        if (len >= SM3_BLOCK_SIZE) {
            chunks = len / SM3_BLOCK_SIZE;
            uint32_t process_len = chunks * SM3_BLOCK_SIZE;
            ret = sm3_process_blocks(ctx, data, chunks);
            if (ret) goto out_release;
            data += process_len;
            len -= process_len;
        }
        break;
    }
    default:
        ret = -EINVAL;
        goto out_release;
    }

    if (len > 0) {
        memcpy(ctx->remain_data + ctx->remain_len, data, len);
        ctx->remain_len += len;
    }

    /* Flush any buffered full blocks before saving state.  The DMEM message
     * buffer is shared OTBN state; if we leave blocks unexecuted here, a
     * different context could acquire the session next and overwrite them.
     */
    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
        if (ctx->dmem_msg_idx != SHA256_MSG_OFFSET) {
            ret = execute_sha256((ctx->dmem_msg_idx - SHA256_MSG_OFFSET) /
                                 SHA256_BLOCK_SIZE);
            if (ret) goto out_release;
            ctx->dmem_msg_idx = SHA256_MSG_OFFSET;
        }
        break;
    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:
        if (ctx->dmem_msg_idx != SHA512_MSG_OFFSET) {
            ret = execute_sha512((ctx->dmem_msg_idx - SHA512_MSG_OFFSET) /
                                 SHA512_BLOCK_SIZE);
            if (ret) goto out_release;
            ctx->dmem_msg_idx = SHA512_MSG_OFFSET;
        }
        break;
    case OTBN_HASH_ALGO_SM3:
        if (ctx->dmem_msg_idx != SM3_MSG_OFFSET) {
            ret = execute_sm3((ctx->dmem_msg_idx - SM3_MSG_OFFSET) /
                              SM3_BLOCK_SIZE);
            if (ret) goto out_release;
            ctx->dmem_msg_idx = SM3_MSG_OFFSET;
        }
        break;
    default:
        ret = -EINVAL;
        goto out_release;
    }

    /* Save updated intermediate state back to context */
    ret = save_state(ctx);

out_release:
    ls_otbn_session_release();
    return ret;
}

int otbn_hash_final(otbn_hash_ctx_t *ctx, uint8_t *digest)
{
    if (ctx == NULL || digest == NULL) {
        return -EINVAL;
    }

    int ret = ls_otbn_session_acquire(ctx->firmware_id, 10);
    if (ret) return ret;

    ret = load_firmware(ctx);
    if (ret) goto out_release;

    ret = restore_state(ctx);
    if (ret) goto out_release;

    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256: {
        uint64_t bit_len = ctx->total_len * 8;

        /* dmem_msg_idx is kept from update so that any buffered full blocks
         * are still in DMEM and will be processed together with padding. */

        ctx->remain_data[ctx->remain_len++] = 0x80;
        if (ctx->remain_len == SHA256_BLOCK_SIZE) {
            ret = sha256_process_block(ctx, ctx->remain_data);
            if (ret) goto out_release;
            ctx->remain_len = 0;
        }

        while (ctx->remain_len != (SHA256_BLOCK_SIZE - 8)) {
            ctx->remain_data[ctx->remain_len++] = 0x0;
            if (ctx->remain_len == SHA256_BLOCK_SIZE) {
                ret = sha256_process_block(ctx, ctx->remain_data);
                if (ret) goto out_release;
                ctx->remain_len = 0;
            }
        }

        for (int i = 0; i < 8; i++) {
            ctx->remain_data[SHA256_BLOCK_SIZE - 1 - i] = (uint8_t)(bit_len >> (8 * i));
        }
        ret = sha256_process_block(ctx, ctx->remain_data);
        if (ret) goto out_release;

        /* Trigger firmware to process buffered blocks plus padding */
        if (ctx->dmem_msg_idx != SHA256_MSG_OFFSET) {
            ret = execute_sha256((ctx->dmem_msg_idx - SHA256_MSG_OFFSET) / SHA256_BLOCK_SIZE);
            if (ret) goto out_release;
            ctx->dmem_msg_idx = SHA256_MSG_OFFSET;
        }

        ret = save_state(ctx);
        if (ret) goto out_release;
        if (ctx->algo == OTBN_HASH_ALGO_SHA224) {
            output_sha224(ctx->state_bytes, digest);
        } else {
            output_sha256(ctx->state_bytes, digest);
        }
        break;
    }

    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512: {
        uint64_t bit_len = ctx->total_len * 8;
        uint32_t digest_words = (ctx->algo == OTBN_HASH_ALGO_SHA384) ? 6 : 8;

        /* dmem_msg_idx is kept from update so that any buffered full blocks
         * are still in DMEM and will be processed together with padding. */

        ctx->remain_data[ctx->remain_len++] = 0x80;
        if (ctx->remain_len == SHA512_BLOCK_SIZE) {
            ret = sha512_process_block(ctx, ctx->remain_data);
            if (ret) goto out_release;
            ctx->remain_len = 0;
        }

        while (ctx->remain_len != (SHA512_BLOCK_SIZE - 16)) {
            ctx->remain_data[ctx->remain_len++] = 0x0;
            if (ctx->remain_len == SHA512_BLOCK_SIZE) {
                ret = sha512_process_block(ctx, ctx->remain_data);
                if (ret) goto out_release;
                ctx->remain_len = 0;
            }
        }

        memset(&ctx->remain_data[ctx->remain_len], 0, 8);
        ctx->remain_len += 8;

        for (int i = 0; i < 8; i++) {
            ctx->remain_data[SHA512_BLOCK_SIZE - 1 - i] = (uint8_t)(bit_len >> (8 * i));
        }
        ret = sha512_process_block(ctx, ctx->remain_data);
        if (ret) goto out_release;

        /* If the padding block was buffered but not executed, trigger it now */
        if (ctx->dmem_msg_idx != SHA512_MSG_OFFSET) {
            ret = execute_sha512((ctx->dmem_msg_idx - SHA512_MSG_OFFSET) / SHA512_BLOCK_SIZE);
            if (ret) goto out_release;
            ctx->dmem_msg_idx = SHA512_MSG_OFFSET;
        }

        ret = save_state(ctx);
        if (ret) goto out_release;
        output_sha512(ctx->state_bytes, digest, digest_words);
        break;
    }

    case OTBN_HASH_ALGO_SM3: {
        uint64_t bit_len = ctx->total_len * 8;

        ctx->remain_data[ctx->remain_len++] = 0x80;
        if (ctx->remain_len == SM3_BLOCK_SIZE) {
            ret = restore_state(ctx);
            if (ret) goto out_release;
            ret = sm3_process_blocks(ctx, ctx->remain_data, 1);
            if (ret) goto out_release;
            ctx->remain_len = 0;
        }

        while (ctx->remain_len != (SM3_BLOCK_SIZE - 8)) {
            ctx->remain_data[ctx->remain_len++] = 0x0;
            if (ctx->remain_len == SM3_BLOCK_SIZE) {
                ret = restore_state(ctx);
                if (ret) goto out_release;
                ret = sm3_process_blocks(ctx, ctx->remain_data, 1);
                if (ret) goto out_release;
                ctx->remain_len = 0;
            }
        }

        for (int i = 0; i < 8; i++) {
            ctx->remain_data[SM3_BLOCK_SIZE - 1 - i] = (uint8_t)(bit_len >> (8 * i));
        }
        ret = restore_state(ctx);
        if (ret) goto out_release;
        ret = sm3_process_blocks(ctx, ctx->remain_data, 1);
        if (ret) goto out_release;

        output_sm3(ctx->state_bytes, digest);
        break;
    }

    default:
        ret = -EINVAL;
        goto out_release;
    }

out_release:
    ls_otbn_session_release();
    memset(ctx, 0, sizeof(*ctx));
    return ret;
}

int otbn_hash_transform(otbn_hash_ctx_t *ctx, const uint8_t *data)
{
    int ret;

    if (ctx == NULL || data == NULL) {
        return -EINVAL;
    }

    ret = ls_otbn_session_acquire(ctx->firmware_id, 10);
    if (ret) {
        return ret;
    }

    ret = load_firmware(ctx);
    if (ret) {
        goto out_release;
    }

    ret = restore_state(ctx);
    if (ret) {
        goto out_release;
    }

    switch (ctx->algo) {
    case OTBN_HASH_ALGO_SHA224:
    case OTBN_HASH_ALGO_SHA256:
        ret = write_block(SHA256_MSG_OFFSET, data, SHA256_BLOCK_SIZE);
        if (ret) {
            goto out_release;
        }
        ret = execute_sha256(1);
        break;

    case OTBN_HASH_ALGO_SHA384:
    case OTBN_HASH_ALGO_SHA512:
        ret = sha512_write_block(SHA512_MSG_OFFSET, data);
        if (ret) {
            goto out_release;
        }
        ret = execute_sha512(1);
        break;

    case OTBN_HASH_ALGO_SM3:
        ret = write_block(SM3_MSG_OFFSET, data, SM3_BLOCK_SIZE);
        if (ret) {
            goto out_release;
        }
        ret = execute_sm3(1);
        break;

    default:
        ret = -EINVAL;
        break;
    }
    if (ret) {
        goto out_release;
    }

    ret = save_state(ctx);

out_release:
    ls_otbn_session_release();
    return ret;
}
