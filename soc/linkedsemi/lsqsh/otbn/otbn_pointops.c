#include "otbn_pointops.h"

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ls_otbn_pointops, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

#include "ls_hal_otbn.h" /* HAL_OTBN_Error_Bit_Get */

/* ========================================================================== */
/* Firmware images (defined in hal_driver/src/otbn/text_array/)               */
/* ========================================================================== */

extern const uint8_t g_ecc_p256_imem[];
extern const uint32_t g_ecc_p256_imem_size;
extern const uint8_t g_ecc_p256_dmem[];
extern const uint32_t g_ecc_p256_dmem_size;

extern const uint8_t g_ecc_p384_imem[];
extern const uint32_t g_ecc_p384_imem_size;
extern const uint8_t g_ecc_p384_dmem[];
extern const uint32_t g_ecc_p384_dmem_size;

extern const uint8_t g_ecc_p521_imem[];
extern const uint32_t g_ecc_p521_imem_size;
extern const uint8_t g_ecc_p521_dmem[];
extern const uint32_t g_ecc_p521_dmem_size;

extern const uint8_t g_ecc_sm2_imem[];
extern const uint32_t g_ecc_sm2_imem_size;
extern const uint8_t g_ecc_sm2_dmem[];
extern const uint32_t g_ecc_sm2_dmem_size;

/* ========================================================================== */
/* DMEM protocol constants (match the OTBN firmware main.s symbol layout)     */
/* ========================================================================== */

/* P-256 pointops firmware */
#define P256_MODE_SCALAR_MULT    (1)
#define P256_MODE_POINT_ADD      (2)
#define P256_MODE_BASE_MULT      (5)
#define P256_MODE_OFFSET         (224)
#define P256_SCALAR_D_OFFSET     (256)
#define P256_X_OFFSET            (384)
#define P256_Y_OFFSET            (416)
#define P256_QX_OFFSET           (448)
#define P256_QY_OFFSET           (480)
#define P256_FIELD_BYTES         (32)
#define P256_DMEM_TOTAL          (1120)

/* P-384 pointops firmware */
#define P384_MODE_SCALAR_MULT    (1)
#define P384_MODE_POINT_ADD      (2)
#define P384_MODE_BASE_MULT      (5)
#define P384_MODE_OFFSET         (320)
#define P384_SCALAR_D_OFFSET     (352)
#define P384_X_OFFSET            (480)
#define P384_Y_OFFSET            (544)
#define P384_QX_OFFSET           (608)
#define P384_QY_OFFSET           (672)
#define P384_RX_OFFSET           (736)
#define P384_RY_OFFSET           (800)
#define P384_FIELD_BYTES         (48)
#define P384_DMEM_TOTAL          (1696)

/* SM2 pointops firmware — same DMEM layout as P-256, different curve params */
#define SM2_MODE_OFFSET          (224)
#define SM2_SCALAR_D_OFFSET      (256)
#define SM2_X_OFFSET             (384)
#define SM2_Y_OFFSET             (416)
#define SM2_QX_OFFSET            (448)
#define SM2_QY_OFFSET            (480)
#define SM2_FIELD_BYTES          (32)
#define SM2_DMEM_TOTAL           (1088)

/* P-521 pointops firmware: atomic field/point operations only. Scalar
 * multiplication is built in software (Montgomery ladder) on top of these. */
#define P521_MODE_FIELD_ADD      (0x100)
#define P521_MODE_FIELD_SUB      (0x101)
#define P521_MODE_FIELD_MUL      (0x102)
#define P521_MODE_POINT_DBL      (0x200)
#define P521_MODE_POINT_ADD      (0x201)
#define P521_MODE_OFFSET         (0)
#define P521_P_OFFSET            (0x20)
#define P521_Q_OFFSET            (0x140)
#define P521_R_OFFSET            (0x260)
#define P521_FIELD_BYTES         (66)
#define P521_COORD_BYTES         (96) /* 3 WDRs, 4-byte aligned for DMEM access */
/* OTBN DMEM hardware capacity (0xC00 bytes); ls_otbn_dmem_set() rejects
 * writes beyond it, and the firmware's scratch area must fit inside. */
#define P521_DMEM_TOTAL          (3072)
/* Zero the full DMEM on every session so the initial image plus firmware
 * scratch space all start clean. */
#define OTBN_DMEM_CAPACITY       (3072)

/* Session acquire timeout. Point operations complete in milliseconds;
 * 10 s matches the mbedtls ECDSA integration. */
#define POINTOPS_SESSION_TIMEOUT_S (10)

/* ========================================================================== */
/* Shared helpers                                                             */
/* ========================================================================== */

/* Per-curve firmware + DMEM layout for the single-shot P-256/P-384 ops.
 * Sizes are const variables in the HAL text_array files, hence pointers
 * (their addresses are constant expressions, the values are not). */
struct pointops_curve_layout {
    otbn_firmware_t fw_id;
    const uint8_t *imem;
    const uint32_t *imem_size;
    const uint8_t *dmem;
    const uint32_t *dmem_size;
    uint32_t dmem_total;
    uint32_t mode_off;
    uint32_t d_off;      /* scalar */
    uint32_t px_off;     /* input P.x */
    uint32_t py_off;     /* input P.y */
    uint32_t qx_off;     /* input Q.x (point add) */
    uint32_t qy_off;     /* input Q.y (point add) */
    uint32_t rx_off;     /* result x */
    uint32_t ry_off;     /* result y */
    uint8_t field_bytes;
};

static const struct pointops_curve_layout p256_layout = {
    .fw_id = OTBN_FIRMWARE_ECC_P256_POINTOPS,
    .imem = g_ecc_p256_imem,
    .imem_size = &g_ecc_p256_imem_size,
    .dmem = g_ecc_p256_dmem,
    .dmem_size = &g_ecc_p256_dmem_size,
    .dmem_total = P256_DMEM_TOTAL,
    .mode_off = P256_MODE_OFFSET,
    .d_off = P256_SCALAR_D_OFFSET,
    .px_off = P256_X_OFFSET,
    .py_off = P256_Y_OFFSET,
    .qx_off = P256_QX_OFFSET,
    .qy_off = P256_QY_OFFSET,
    .rx_off = P256_X_OFFSET,
    .ry_off = P256_Y_OFFSET,
    .field_bytes = P256_FIELD_BYTES,
};

static const struct pointops_curve_layout p384_layout = {
    .fw_id = OTBN_FIRMWARE_ECC_P384_POINTOPS,
    .imem = g_ecc_p384_imem,
    .imem_size = &g_ecc_p384_imem_size,
    .dmem = g_ecc_p384_dmem,
    .dmem_size = &g_ecc_p384_dmem_size,
    .dmem_total = P384_DMEM_TOTAL,
    .mode_off = P384_MODE_OFFSET,
    .d_off = P384_SCALAR_D_OFFSET,
    .px_off = P384_X_OFFSET,
    .py_off = P384_Y_OFFSET,
    .qx_off = P384_QX_OFFSET,
    .qy_off = P384_QY_OFFSET,
    .rx_off = P384_RX_OFFSET,
    .ry_off = P384_RY_OFFSET,
    .field_bytes = P384_FIELD_BYTES,
};

static const struct pointops_curve_layout sm2_layout = {
    .fw_id = OTBN_FIRMWARE_ECC_SM2_POINTOPS,
    .imem = g_ecc_sm2_imem,
    .imem_size = &g_ecc_sm2_imem_size,
    .dmem = g_ecc_sm2_dmem,
    .dmem_size = &g_ecc_sm2_dmem_size,
    .dmem_total = SM2_DMEM_TOTAL,
    .mode_off = SM2_MODE_OFFSET,
    .d_off = SM2_SCALAR_D_OFFSET,
    .px_off = SM2_X_OFFSET,
    .py_off = SM2_Y_OFFSET,
    .qx_off = SM2_QX_OFFSET,
    .qy_off = SM2_QY_OFFSET,
    .rx_off = SM2_X_OFFSET,
    .ry_off = SM2_Y_OFFSET,
    .field_bytes = SM2_FIELD_BYTES,
};

static const struct pointops_curve_layout *pointops_get_layout(
    enum ls_otbn_pointops_curve curve)
{
    switch (curve) {
    case LS_OTBN_POINTOPS_CURVE_P256:
        return &p256_layout;
    case LS_OTBN_POINTOPS_CURVE_P384:
        return &p384_layout;
    case LS_OTBN_POINTOPS_CURVE_SM2:
        return &sm2_layout;
    default:
        return NULL;
    }
}

/* Program the pointops firmware into OTBN if it is not already resident.
 * The shared IMEM tracking state makes this cheap across sessions. */
static int pointops_load_firmware(const struct pointops_curve_layout *c)
{
    int err;

    if (ls_otbn_imem_firmware_get() == c->fw_id) {
        return 0;   /* already resident */
    }

    err = ls_otbn_imem_write(0, (const uint32_t *)c->imem, *c->imem_size);
    if (err != 0) {
        return err;
    }

    ls_otbn_imem_firmware_confirm(c->fw_id);
    return 0;
}

/* Restore the initial DMEM image. The firmware assumes a pristine DMEM
 * before every run (the HAL drivers do the same per call), so scratch
 * data from a previous operation must not leak into the next one. */
static int pointops_reset_dmem(const struct pointops_curve_layout *c)
{
    int err;

    err = ls_otbn_dmem_set(0, 0, OTBN_DMEM_CAPACITY);
    if (err != 0) {
        return err;
    }
    return ls_otbn_dmem_write(0, (const uint32_t *)c->dmem, *c->dmem_size);
}

/* Write the mode word and execute the firmware, then report hardware errors. */
static int pointops_run_mode(uint32_t mode_off, uint32_t mode)
{
    int err = ls_otbn_dmem_write(mode_off, &mode, sizeof(mode));
    if (err != 0) {
        return err;
    }

    err = ls_otbn_cmd(OTBN_CMD_EXECUTE);
    if (err != 0) {
        return err;
    }

    uint32_t err_bit = HAL_OTBN_Error_Bit_Get();
    if (err_bit != 0) {
        LOG_ERR("OTBN pointops firmware error, error bit: 0x%x", err_bit);
        return -EIO;
    }
    return 0;
}

/* Inputs/outputs of a single-shot P-256/P-384 operation. */
struct pointops_affine_args {
    const uint8_t *d;   /* scalar (scalar/base mult) */
    const uint8_t *px;  /* input P.x */
    const uint8_t *py;  /* input P.y */
    const uint8_t *qx;  /* input Q.x (point add) */
    const uint8_t *qy;  /* input Q.y (point add) */
    uint8_t *rx;        /* result x */
    uint8_t *ry;        /* result y */
};

/* Run one single-shot operation inside an exclusive session. */
static int pointops_session_run(otbn_firmware_t fw_id,
                                int (*op)(const struct pointops_curve_layout *c,
                                          const struct pointops_affine_args *a),
                                const struct pointops_affine_args *arg)
{
    const struct pointops_curve_layout *c = pointops_get_layout(
        (enum ls_otbn_pointops_curve)fw_id);
    int err;

    if (c == NULL) {
        return -EINVAL;
    }

    err = ls_otbn_session_acquire(fw_id, POINTOPS_SESSION_TIMEOUT_S);
    if (err != 0) {
        return err;
    }

    err = pointops_load_firmware(c);
    if (err == 0) {
        err = pointops_reset_dmem(c);
    }
    if (err == 0) {
        err = op(c, arg);
    }

    int rel = ls_otbn_session_release();
    if (rel != 0) {
        LOG_ERR("%s: session release failed (%d)", __func__, rel);
        if (err == 0) {
            err = rel;
        }
    }

    return err;
}

/* ========================================================================== */
/* P-256 / P-384 single-shot operations                                       */
/* ========================================================================== */

static int curve_scalar_mult(const struct pointops_curve_layout *c,
                             const struct pointops_affine_args *a)
{
    int err;

    err = ls_otbn_dmem_write(c->d_off, (const uint32_t *)a->d, c->field_bytes);
    if (err != 0) {
        return err;
    }
    err = ls_otbn_dmem_write(c->px_off, (const uint32_t *)a->px, c->field_bytes);
    if (err != 0) {
        return err;
    }
    err = ls_otbn_dmem_write(c->py_off, (const uint32_t *)a->py, c->field_bytes);
    if (err != 0) {
        return err;
    }

    err = pointops_run_mode(c->mode_off, P256_MODE_SCALAR_MULT);
    if (err != 0) {
        return err;
    }

    err = ls_otbn_dmem_read(c->rx_off, (uint32_t *)a->rx, c->field_bytes);
    if (err != 0) {
        return err;
    }
    return ls_otbn_dmem_read(c->ry_off, (uint32_t *)a->ry, c->field_bytes);
}

static int curve_base_mult(const struct pointops_curve_layout *c,
                           const struct pointops_affine_args *a)
{
    int err;

    err = ls_otbn_dmem_write(c->d_off, (const uint32_t *)a->d, c->field_bytes);
    if (err != 0) {
        return err;
    }

    err = pointops_run_mode(c->mode_off, P256_MODE_BASE_MULT);
    if (err != 0) {
        return err;
    }

    err = ls_otbn_dmem_read(c->rx_off, (uint32_t *)a->rx, c->field_bytes);
    if (err != 0) {
        return err;
    }
    return ls_otbn_dmem_read(c->ry_off, (uint32_t *)a->ry, c->field_bytes);
}

static int curve_point_add(const struct pointops_curve_layout *c,
                           const struct pointops_affine_args *a)
{
    int err;

    err = ls_otbn_dmem_write(c->px_off, (const uint32_t *)a->px, c->field_bytes);
    if (err != 0) {
        return err;
    }
    err = ls_otbn_dmem_write(c->py_off, (const uint32_t *)a->py, c->field_bytes);
    if (err != 0) {
        return err;
    }
    err = ls_otbn_dmem_write(c->qx_off, (const uint32_t *)a->qx, c->field_bytes);
    if (err != 0) {
        return err;
    }
    err = ls_otbn_dmem_write(c->qy_off, (const uint32_t *)a->qy, c->field_bytes);
    if (err != 0) {
        return err;
    }

    err = pointops_run_mode(c->mode_off, P256_MODE_POINT_ADD);
    if (err != 0) {
        return err;
    }

    err = ls_otbn_dmem_read(c->rx_off, (uint32_t *)a->rx, c->field_bytes);
    if (err != 0) {
        return err;
    }
    return ls_otbn_dmem_read(c->ry_off, (uint32_t *)a->ry, c->field_bytes);
}

/* ========================================================================== */
/* P-521: atomic ops + software Montgomery ladder                             */
/* ========================================================================== */

/* P-521 modulus: p = 2^521 - 1, stored as 66 bytes little-endian. */
static const uint8_t p521_p[P521_FIELD_BYTES] = {
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0x01
};

/* Base point G (66 bytes LE) */
static const uint8_t p521_gx[P521_FIELD_BYTES] = {
    0x66,0xbd,0xe5,0xc2,0x31,0x7e,0x7e,0xf9, 0x9b,0x42,0x6a,0x85,0xc1,0xb3,0x48,0x33,
    0xde,0xa8,0xff,0xa2,0x27,0xc1,0x1d,0xfe, 0x28,0x59,0xe7,0xef,0x77,0x5e,0x4b,0xa1,
    0xba,0x3d,0x4d,0x6b,0x60,0xaf,0x28,0xf8, 0x21,0xb5,0x3f,0x05,0x39,0x81,0x64,0x9c,
    0x42,0xb4,0x95,0x23,0x66,0xcb,0x3e,0x9e, 0xcd,0xe9,0x04,0x04,0xb7,0x06,0x8e,0x85,
    0xc6,0x00
};

static const uint8_t p521_gy[P521_FIELD_BYTES] = {
    0x50,0x66,0xd1,0x9f,0x76,0x94,0xbe,0x88, 0x40,0xc2,0x72,0xa2,0x86,0x70,0x3c,0x35,
    0x61,0x07,0xad,0x3f,0x01,0xb9,0x50,0xc5, 0x40,0x26,0xf4,0x5e,0x99,0x72,0xee,0x97,
    0x2c,0x66,0x3e,0x27,0x17,0xbd,0xaf,0x17, 0x68,0x44,0x9b,0x57,0x49,0x44,0xf5,0x98,
    0xd9,0x1b,0x7d,0x2c,0xb4,0x5f,0x8a,0x5c, 0x04,0xc0,0x3b,0x9a,0x78,0x6a,0x29,0x39,
    0x18,0x01
};

static const uint8_t p521_one[P521_FIELD_BYTES] = {
    0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00
};

/* result = (a * b) mod p, where a,b < p (66 bytes LE) */
static void p521_mul_mod(uint8_t *result, const uint8_t *a, const uint8_t *b)
{
    uint8_t prod[132];
    memset(prod, 0, sizeof(prod));

    for (int i = 0; i < P521_FIELD_BYTES; i++) {
        uint16_t carry = 0;
        for (int j = 0; j < P521_FIELD_BYTES; j++) {
            uint16_t p_val = (uint16_t)a[i] * b[j] + prod[i+j] + carry;
            prod[i+j] = (uint8_t)p_val;
            carry = p_val >> 8;
        }
        prod[i+66] = (uint8_t)carry;
    }

    /* Reduce mod p = 2^521 - 1: 2^521 == 1 (mod p), so the upper 521 bits
     * of the 132-byte product fold onto the lower 521 bits. The folding
     * addition can itself carry past bit 520, hence the second round
     * folds that residue again; after two rounds the result is < 2^521. */
    for (int round = 0; round < 2; round++) {
        uint8_t high[66];
        memset(high, 0, 66);
        for (int i = 0; i < 65; i++) {
            high[i] = (prod[65 + i] >> 1) | (prod[66 + i] << 7);
        }
        high[65] = prod[130] >> 1;

        prod[65] &= 0x01;          /* keep only bit 520 */
        memset(prod + 66, 0, 66);  /* clear bytes 66..131 */

        uint16_t carry = 0;
        for (int i = 0; i < 66; i++) {
            uint16_t sum = (uint16_t)prod[i] + high[i] + carry;
            prod[i] = (uint8_t)sum;
            carry = sum >> 8;
        }
        if (carry) {
            prod[65] &= 0x01;        /* keep only bit 520 */
            uint16_t c = 1;
            for (int i = 0; i < 66 && c; i++) {
                uint16_t s = (uint16_t)prod[i] + c;
                prod[i] = (uint8_t)s;
                c = s >> 8;
            }
        }
    }
    memcpy(result, prod, 66);
}

/* result = a^(-1) mod p using Fermat's little theorem: a^(p-2) mod p */
static void p521_inv_mod(uint8_t *result, const uint8_t *a)
{
    uint8_t exp[66];
    memcpy(exp, p521_p, 66);
    uint16_t borrow = 2;
    for (int i = 0; i < 66; i++) {
        uint16_t diff = (uint16_t)exp[i] - borrow;
        exp[i] = (uint8_t)diff;
        borrow = (diff >> 8) & 1;
        if (!borrow) break;
    }

    uint8_t base[66];
    memcpy(base, a, 66);
    memset(result, 0, 66);
    result[0] = 1;

    for (int i = 520; i >= 0; i--) {
        uint8_t tmp[66];
        p521_mul_mod(tmp, result, result);
        memcpy(result, tmp, 66);

        int byte_idx = i / 8;
        int bit_idx = i % 8;
        if (exp[byte_idx] & (1 << bit_idx)) {
            p521_mul_mod(tmp, result, base);
            memcpy(result, tmp, 66);
        }
    }
}

/**
 * Convert Jacobian (X, Y, Z) to affine (x, y):
 *   z_inv = Z^(-1) mod p
 *   x = X * z_inv^2 mod p
 *   y = Y * z_inv^3 mod p
 */
static void p521_jacobian_to_affine(uint8_t *ax, uint8_t *ay,
                                    const uint8_t *jx, const uint8_t *jy,
                                    const uint8_t *jz)
{
    uint8_t z_inv[66], z_inv2[66], z_inv3[66];

    int z_zero = 1;
    for (int i = 0; i < 66; i++) {
        if (jz[i]) { z_zero = 0; break; }
    }
    if (z_zero) {
        memset(ax, 0, 66);
        memset(ay, 0, 66);
        return;
    }

    p521_inv_mod(z_inv, jz);
    p521_mul_mod(z_inv2, z_inv, z_inv);
    p521_mul_mod(z_inv3, z_inv2, z_inv);
    p521_mul_mod(ax, jx, z_inv2);
    p521_mul_mod(ay, jy, z_inv3);
}

/* Write the P-521 atomic mode word (DMEM offset 0) and execute. */
static int p521_run_mode(uint32_t mode)
{
    int err = ls_otbn_dmem_write(P521_MODE_OFFSET, &mode, sizeof(mode));
    if (err != 0) {
        return err;
    }

    err = ls_otbn_cmd(OTBN_CMD_EXECUTE);
    if (err != 0) {
        return err;
    }

    uint32_t err_bit = HAL_OTBN_Error_Bit_Get();
    if (err_bit != 0) {
        LOG_ERR("OTBN P-521 firmware error, error bit: 0x%x", err_bit);
        return -EIO;
    }
    return 0;
}

/* Copy 66 bytes from src to a zero-padded 96-byte buffer, then write to DMEM. */
static int p521_write_coord(uint32_t offset, const uint8_t *src)
{
    uint32_t buf[P521_COORD_BYTES / 4];
    memset(buf, 0, sizeof(buf));
    memcpy(buf, src, P521_FIELD_BYTES);
    return ls_otbn_dmem_write(offset, buf, P521_COORD_BYTES);
}

/* Read 96 bytes from DMEM, then copy only the first 66 bytes to dst. */
static int p521_read_coord(uint32_t offset, uint8_t *dst)
{
    uint32_t buf[P521_COORD_BYTES / 4];
    int err = ls_otbn_dmem_read(offset, buf, P521_COORD_BYTES);
    if (err != 0) {
        return err;
    }
    memcpy(dst, buf, P521_FIELD_BYTES);
    return 0;
}

/* Check if a coordinate (66 bytes) is all zeros */
static int p521_is_zero(const uint8_t *a)
{
    for (int i = 0; i < 66; i++) {
        if (a[i]) return 0;
    }
    return 1;
}

/* Check if two coordinates (66 bytes each) are equal */
static int p521_coord_eq(const uint8_t *a, const uint8_t *b)
{
    for (int i = 0; i < 66; i++) {
        if (a[i] != b[i]) return 0;
    }
    return 1;
}

/* Point doubling via OTBN */
static int p521_point_dbl(const uint8_t *px, const uint8_t *py, const uint8_t *pz,
                          uint8_t *rx, uint8_t *ry, uint8_t *rz)
{
    /* Identity point: Z=0 -> double is identity */
    if (p521_is_zero(pz)) {
        memset(rx, 0, 66);
        memset(ry, 0, 66);
        ry[0] = 1;
        memset(rz, 0, 66);
        return 0;
    }

    int err = p521_write_coord(P521_P_OFFSET, px);
    if (err != 0) return err;
    err = p521_write_coord(P521_P_OFFSET + 96, py);
    if (err != 0) return err;
    err = p521_write_coord(P521_P_OFFSET + 192, pz);
    if (err != 0) return err;
    err = p521_run_mode(P521_MODE_POINT_DBL);
    if (err != 0) return err;
    err = p521_read_coord(P521_R_OFFSET, rx);
    if (err != 0) return err;
    err = p521_read_coord(P521_R_OFFSET + 96, ry);
    if (err != 0) return err;
    return p521_read_coord(P521_R_OFFSET + 192, rz);
}

/* Point addition via OTBN */
static int p521_point_add(const uint8_t *px, const uint8_t *py, const uint8_t *pz,
                          const uint8_t *qx, const uint8_t *qy, const uint8_t *qz,
                          uint8_t *rx, uint8_t *ry, uint8_t *rz)
{
    int p_is_zero = p521_is_zero(pz);
    int q_is_zero = p521_is_zero(qz);

    /* P + O = P */
    if (p_is_zero) {
        memcpy(rx, qx, 66);
        memcpy(ry, qy, 66);
        memcpy(rz, qz, 66);
        return 0;
    }

    /* O + Q = Q */
    if (q_is_zero) {
        memcpy(rx, px, 66);
        memcpy(ry, py, 66);
        memcpy(rz, pz, 66);
        return 0;
    }

    /* P == Q -> use point doubling */
    if (p521_coord_eq(px, qx) && p521_coord_eq(py, qy) && p521_coord_eq(pz, qz)) {
        return p521_point_dbl(px, py, pz, rx, ry, rz);
    }

    int err = p521_write_coord(P521_P_OFFSET, px);
    if (err != 0) return err;
    err = p521_write_coord(P521_P_OFFSET + 96, py);
    if (err != 0) return err;
    err = p521_write_coord(P521_P_OFFSET + 192, pz);
    if (err != 0) return err;
    err = p521_write_coord(P521_Q_OFFSET, qx);
    if (err != 0) return err;
    err = p521_write_coord(P521_Q_OFFSET + 96, qy);
    if (err != 0) return err;
    err = p521_write_coord(P521_Q_OFFSET + 192, qz);
    if (err != 0) return err;
    err = p521_run_mode(P521_MODE_POINT_ADD);
    if (err != 0) return err;
    err = p521_read_coord(P521_R_OFFSET, rx);
    if (err != 0) return err;
    err = p521_read_coord(P521_R_OFFSET + 96, ry);
    if (err != 0) return err;
    return p521_read_coord(P521_R_OFFSET + 192, rz);
}

/* Montgomery ladder: 521 iterations, MSB to LSB */
static int p521_scalar_mult(const uint8_t *k,
                            const uint8_t *px, const uint8_t *py,
                            uint8_t *rx, uint8_t *ry)
{
    /* Jacobian point buffers */
    uint8_t r0_x[66], r0_y[66], r0_z[66];
    uint8_t r1_x[66], r1_y[66], r1_z[66];

    /* R0 = identity (0, 1, 0) */
    memset(r0_x, 0, 66);
    memset(r0_y, 0, 66);
    r0_y[0] = 1;
    memset(r0_z, 0, 66);

    /* R1 = P (px, py, 1) */
    memcpy(r1_x, px, 66);
    memcpy(r1_y, py, 66);
    memset(r1_z, 0, 66);
    r1_z[0] = 1;

    /* Pointers for swap */
    uint8_t *p0x = r0_x, *p0y = r0_y, *p0z = r0_z;
    uint8_t *p1x = r1_x, *p1y = r1_y, *p1z = r1_z;

    /* Temporary result buffers */
    uint8_t t_x[66], t_y[66], t_z[66];

    for (int i = 520; i >= 0; i--) {
        int byte_idx = i >> 3;  /* i / 8 */
        int bit_idx = i & 7;   /* i % 8 */
        int bit = (k[byte_idx] >> bit_idx) & 1;

        if (bit == 0) {
            /* Swap R0 and R1 pointers */
            uint8_t *tx = p0x; p0x = p1x; p1x = tx;
            uint8_t *ty = p0y; p0y = p1y; p1y = ty;
            uint8_t *tz = p0z; p0z = p1z; p1z = tz;
        }

        /* R0 = R0 + R1 */
        int err = p521_point_add(p0x, p0y, p0z, p1x, p1y, p1z, t_x, t_y, t_z);
        if (err != 0) return err;
        memcpy(p0x, t_x, 66);
        memcpy(p0y, t_y, 66);
        memcpy(p0z, t_z, 66);

        /* R1 = 2 * R1 */
        err = p521_point_dbl(p1x, p1y, p1z, t_x, t_y, t_z);
        if (err != 0) return err;
        memcpy(p1x, t_x, 66);
        memcpy(p1y, t_y, 66);
        memcpy(p1z, t_z, 66);

        if (bit == 0) {
            /* Swap back */
            uint8_t *tx = p0x; p0x = p1x; p1x = tx;
            uint8_t *ty = p0y; p0y = p1y; p1y = ty;
            uint8_t *tz = p0z; p0z = p1z; p1z = tz;
        }
    }

    /* Convert R0 to affine */
    p521_jacobian_to_affine(rx, ry, p0x, p0y, p0z);
    return 0;
}

static int p521_load_firmware(void)
{
    int err;

    if (ls_otbn_imem_firmware_get() == OTBN_FIRMWARE_ECC_P521_POINTOPS) {
        return 0;
    }

    err = ls_otbn_imem_write(0, (const uint32_t *)g_ecc_p521_imem,
                             g_ecc_p521_imem_size);
    if (err != 0) {
        return err;
    }

    ls_otbn_imem_firmware_confirm(OTBN_FIRMWARE_ECC_P521_POINTOPS);
    return 0;
}

/* P-521 also expects a pristine DMEM per run (see pointops_reset_dmem). */
static int p521_reset_dmem(void)
{
    int err;

    err = ls_otbn_dmem_set(0, 0, P521_DMEM_TOTAL);
    if (err != 0) {
        return err;
    }
    return ls_otbn_dmem_write(0, (const uint32_t *)g_ecc_p521_dmem,
                              g_ecc_p521_dmem_size);
}

/* Run a P-521 operation inside an exclusive session. */
static int p521_session_run(int (*op)(void *arg), void *arg)
{
    int err = ls_otbn_session_acquire(OTBN_FIRMWARE_ECC_P521_POINTOPS,
                                      POINTOPS_SESSION_TIMEOUT_S);
    if (err != 0) {
        return err;
    }

    err = p521_load_firmware();
    if (err == 0) {
        err = p521_reset_dmem();
    }
    if (err == 0) {
        err = op(arg);
    }

    int rel = ls_otbn_session_release();
    if (rel != 0) {
        LOG_ERR("%s: session release failed (%d)", __func__, rel);
        if (err == 0) {
            err = rel;
        }
    }

    return err;
}

struct p521_scalar_mult_args {
    const uint8_t *k;
    const uint8_t *px;
    const uint8_t *py;
    uint8_t *rx;
    uint8_t *ry;
};

static int p521_scalar_mult_op(void *arg)
{
    struct p521_scalar_mult_args *a = arg;
    return p521_scalar_mult(a->k, a->px, a->py, a->rx, a->ry);
}

struct p521_point_add_args {
    const uint8_t *px;
    const uint8_t *py;
    const uint8_t *qx;
    const uint8_t *qy;
    uint8_t *rx;
    uint8_t *ry;
};

static int p521_point_add_op(void *arg)
{
    struct p521_point_add_args *a = arg;
    uint8_t jx[66], jy[66], jz[66];

    /* P + Q (both affine Z=1) */
    int err = p521_point_add(a->px, a->py, p521_one, a->qx, a->qy, p521_one,
                             jx, jy, jz);
    if (err != 0) {
        return err;
    }
    p521_jacobian_to_affine(a->rx, a->ry, jx, jy, jz);
    return 0;
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

int ls_otbn_pointops_field_bytes(enum ls_otbn_pointops_curve curve, size_t *bytes)
{
    if (bytes == NULL) {
        return -EINVAL;
    }

    switch (curve) {
    case LS_OTBN_POINTOPS_CURVE_P256:
        *bytes = P256_FIELD_BYTES;
        return 0;
    case LS_OTBN_POINTOPS_CURVE_P384:
        *bytes = P384_FIELD_BYTES;
        return 0;
    case LS_OTBN_POINTOPS_CURVE_P521:
        *bytes = P521_FIELD_BYTES;
        return 0;
    case LS_OTBN_POINTOPS_CURVE_SM2:
        *bytes = SM2_FIELD_BYTES;
        return 0;
    default:
        return -EINVAL;
    }
}

int ls_otbn_pointops_scalar_mult(enum ls_otbn_pointops_curve curve,
                                 const uint8_t *scalar,
                                 const uint8_t *px, const uint8_t *py,
                                 uint8_t *rx, uint8_t *ry)
{
    const struct pointops_curve_layout *c;

    if (scalar == NULL || px == NULL || py == NULL || rx == NULL || ry == NULL) {
        return -EINVAL;
    }

    if (curve == LS_OTBN_POINTOPS_CURVE_P521) {
        struct p521_scalar_mult_args args = {
            .k = scalar, .px = px, .py = py, .rx = rx, .ry = ry,
        };
        return p521_session_run(p521_scalar_mult_op, &args);
    }

    c = pointops_get_layout(curve);
    if (c == NULL) {
        return -EINVAL;
    }

    struct pointops_affine_args args = {
        .d = scalar, .px = px, .py = py, .rx = rx, .ry = ry,
    };
    return pointops_session_run(c->fw_id, curve_scalar_mult, &args);
}

int ls_otbn_pointops_base_mult(enum ls_otbn_pointops_curve curve,
                               const uint8_t *scalar,
                               uint8_t *rx, uint8_t *ry)
{
    const struct pointops_curve_layout *c;

    if (scalar == NULL || rx == NULL || ry == NULL) {
        return -EINVAL;
    }

    if (curve == LS_OTBN_POINTOPS_CURVE_P521) {
        struct p521_scalar_mult_args args = {
            .k = scalar, .px = p521_gx, .py = p521_gy, .rx = rx, .ry = ry,
        };
        return p521_session_run(p521_scalar_mult_op, &args);
    }

    c = pointops_get_layout(curve);
    if (c == NULL) {
        return -EINVAL;
    }

    struct pointops_affine_args args = {
        .d = scalar, .rx = rx, .ry = ry,
    };
    return pointops_session_run(c->fw_id, curve_base_mult, &args);
}

int ls_otbn_pointops_point_add(enum ls_otbn_pointops_curve curve,
                               const uint8_t *px, const uint8_t *py,
                               const uint8_t *qx, const uint8_t *qy,
                               uint8_t *rx, uint8_t *ry)
{
    const struct pointops_curve_layout *c;

    if (px == NULL || py == NULL || qx == NULL || qy == NULL ||
        rx == NULL || ry == NULL) {
        return -EINVAL;
    }

    if (curve == LS_OTBN_POINTOPS_CURVE_P521) {
        struct p521_point_add_args args = {
            .px = px, .py = py, .qx = qx, .qy = qy, .rx = rx, .ry = ry,
        };
        return p521_session_run(p521_point_add_op, &args);
    }

    c = pointops_get_layout(curve);
    if (c == NULL) {
        return -EINVAL;
    }

    struct pointops_affine_args args = {
        .px = px, .py = py, .qx = qx, .qy = qy, .rx = rx, .ry = ry,
    };
    return pointops_session_run(c->fw_id, curve_point_add, &args);
}
