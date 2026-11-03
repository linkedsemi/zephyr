/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * mbedtls-facing half of the OTBN error domain.
 *
 * ls_otbn_err.h is deliberately free of mbedtls headers: the delegation bus
 * and the OTBN-only entry points must build in an image that does not link
 * mbedtls. Only the families that actually speak mbedtls include this file,
 * for the two translations between the driver domain and MBEDTLS_ERR_*.
 */

#ifndef LS_OTBN_ERR_MBEDTLS_H
#define LS_OTBN_ERR_MBEDTLS_H

#include <stdbool.h>

#include "ls_otbn_err.h"

#include "mbedtls/bignum.h"
#include "mbedtls/error.h"
#include "mbedtls/ecp.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The mbedtls-domain projection of LS_OTBN_ERR_BUSY: the cause is the same,
 * only the domain differs. mbedtls has no "busy" code, so this vendor
 * extension lives inside the mbedtls range. This is the one definition; the
 * driver side keeps LS_OTBN_ERR_BUSY, and ls_otbn_err_to_mbedtls() maps
 * between them.
 */
#define MBEDTLS_ERR_LS_OTBN_BUSY      (-0x135)

/**
 * \brief   Mark a failure raised by the mbedtls crypto layer.
 *
 *          The serving core applies this before the status goes on the wire,
 *          so the wire carries driver codes only and both sides speak one
 *          domain. The exact backend value is not lost: the caller keeps it
 *          alongside (the status out-parameter of the custom entry points, or
 *          reply arg[0] on the wire), and the mbedtls-parity client restores
 *          it from there.
 *
 *          Only apply it to a value that is known to come from mbedtls: the
 *          mbedtls low range overlaps errno, so a raw errno such as -EINVAL
 *          would be misread. Driver codes pass through unchanged.
 */
static inline int ls_otbn_err_from_mbedtls(int rc)
{
	if (rc == 0 || ls_otbn_err_is_custom(rc)) {
		return rc;
	}

	return LS_OTBN_ERR_MBEDTLS;
}

/**
 * \brief   Translate a driver code into the closest mbedtls code.
 *
 *          mbedtls codes pass through unchanged. The mapping is lossy by
 *          design: several driver failures share one mbedtls value, because
 *          mbedtls has nothing finer to say about them. It is shared by the
 *          mbedtls families; codes that must stay family-specific (for
 *          example MBEDTLS_ERR_RSA_*) are handled inside that family.
 */
static inline int ls_otbn_err_to_mbedtls(int rc)
{
	if (!ls_otbn_err_is_custom(rc)) {
		return rc;
	}

	switch (rc) {
	case LS_OTBN_ERR_INVAL:
	case LS_OTBN_ERR_RANGE:
	case LS_OTBN_ERR_CURVE:
	case LS_OTBN_ERR_MSGSIZE:
		return MBEDTLS_ERR_ECP_BAD_INPUT_DATA;
	case LS_OTBN_ERR_NOSUPP:
		return MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE;
	case LS_OTBN_ERR_BUSY:
	case LS_OTBN_ERR_ACCESS:
		return MBEDTLS_ERR_LS_OTBN_BUSY;
	case LS_OTBN_ERR_NOSYS:
		return MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED;
	case LS_OTBN_ERR_RNG:
		return MBEDTLS_ERR_ECP_RANDOM_FAILED;
	case LS_OTBN_ERR_NOENT:
	case LS_OTBN_ERR_PROTO:
	case LS_OTBN_ERR_IO:
	case LS_OTBN_ERR_TIMEOUT:
	default:
		return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* LS_OTBN_ERR_MBEDTLS_H */
