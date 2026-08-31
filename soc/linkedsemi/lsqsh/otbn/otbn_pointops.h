#ifndef _LS_OTBN_POINTOPS_H_
#define _LS_OTBN_POINTOPS_H_

#include <stddef.h>
#include "ls_otbn_config.h"

/**
 * @file otbn_pointops.h
 * @brief LinkedSemi OTBN ECC point operations (P-256/P-384/P-521) Zephyr API.
 *
 * OS-integrated wrapper around the HAL pointops drivers
 * (ls_hal_otbn_p{256,384,521}_pointops). Every operation runs inside an
 * exclusive OTBN session (@ref ls_otbn_session_acquire), so the API is safe
 * to call from multiple threads. The HAL drivers program IMEM/DMEM directly
 * and poll for completion; the session layer guarantees exclusivity and
 * module initialization.
 *
 * Scalars and coordinates are plain byte arrays in little-endian order,
 * matching the OTBN DMEM protocol:
 *   - P-256: 32 bytes per field element
 *   - P-384: 48 bytes per field element
 *   - P-521: 66 bytes per field element
 *
 * All functions return 0 on success or a negative errno:
 *   -EBUSY (OTBN busy/timeout acquiring the session), -ETIMEDOUT (firmware
 *   execution timeout), -EINVAL (bad curve or NULL argument), -EIO
 *   (firmware reported an error).
 */

/** @brief Curves supported by the point operations API. */
enum ls_otbn_pointops_curve {
    LS_OTBN_POINTOPS_CURVE_P256 = OTBN_FIRMWARE_ECC_P256_POINTOPS,
    LS_OTBN_POINTOPS_CURVE_P384 = OTBN_FIRMWARE_ECC_P384_POINTOPS,
    LS_OTBN_POINTOPS_CURVE_P521 = OTBN_FIRMWARE_ECC_P521_POINTOPS,
};

/**
 * @brief Return the field element size in bytes for a curve.
 *
 * @param curve Curve identifier.
 * @param bytes Output: field size (32/48/66).
 *
 * @return 0 on success, -EINVAL for an unknown curve or NULL output.
 */
int ls_otbn_pointops_field_bytes(enum ls_otbn_pointops_curve curve,
                                 size_t *bytes);

/**
 * @brief Compute R = scalar * P.
 *
 * @param curve  Curve identifier.
 * @param scalar Scalar, little-endian, field bytes long.
 * @param px     P.x, little-endian, field bytes long.
 * @param py     P.y, little-endian, field bytes long.
 * @param rx     Output R.x, field bytes.
 * @param ry     Output R.y, field bytes.
 *
 * @return 0 on success, negative errno on failure.
 */
int ls_otbn_pointops_scalar_mult(enum ls_otbn_pointops_curve curve,
                                 const uint8_t *scalar,
                                 const uint8_t *px, const uint8_t *py,
                                 uint8_t *rx, uint8_t *ry);

/**
 * @brief Compute R = scalar * G (the curve base point).
 *
 * @param curve  Curve identifier.
 * @param scalar Scalar, little-endian, field bytes long.
 * @param rx     Output R.x, field bytes.
 * @param ry     Output R.y, field bytes.
 *
 * @return 0 on success, negative errno on failure.
 */
int ls_otbn_pointops_base_mult(enum ls_otbn_pointops_curve curve,
                               const uint8_t *scalar,
                               uint8_t *rx, uint8_t *ry);

/**
 * @brief Compute R = P + Q.
 *
 * @param curve Curve identifier.
 * @param px    P.x, little-endian, field bytes long.
 * @param py    P.y, little-endian, field bytes long.
 * @param qx    Q.x, little-endian, field bytes long.
 * @param qy    Q.y, little-endian, field bytes long.
 * @param rx    Output R.x, field bytes.
 * @param ry    Output R.y, field bytes.
 *
 * @return 0 on success, negative errno on failure.
 */
int ls_otbn_pointops_point_add(enum ls_otbn_pointops_curve curve,
                               const uint8_t *px, const uint8_t *py,
                               const uint8_t *qx, const uint8_t *qy,
                               uint8_t *rx, uint8_t *ry);

#endif /* _LS_OTBN_POINTOPS_H_ */
