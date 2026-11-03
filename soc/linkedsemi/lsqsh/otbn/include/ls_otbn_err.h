/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * OTBN error codes.
 *
 * Why a private range: the OTBN proxy used to carry two error domains in one
 * int -- Zephyr errno (-1..-134) and mbedtls' MBEDTLS_ERR_* (-0x0002..-0x7F80).
 * Those overlap exactly where it hurts: MBEDTLS_ERR_MPI_* is -0x0002..-0x000E,
 * i.e. the same numbers as -EPERM..-EFAULT, and MBEDTLS_ERR_PLATFORM_* is
 * -0x0070..-0x0072, inside the errno range as well.  A caller could not tell
 * "the peer's mbedtls said INVALID_KEY" from "this core's driver said ENOENT".
 *
 * LS_OTBN_ERR_* sits below -0x10000, which neither domain reaches. The wire
 * only carries driver codes; the mbedtls-facing translators in
 * ls_otbn_err_mbedtls.h convert at the library boundary, so the two domains
 * never meet in one value.
 *
 * Which entry points return what:
 *   - the serving core marks a failure from its crypto layer with
 *     ls_otbn_err_from_mbedtls() before answering, so the status on the wire
 *     is always a driver code and both cores speak one domain. The backend's
 *     exact value travels alongside in the reply's arg[0].
 *   - mbedtls-parity entry points (mbedtls_ls_otbn_ecdsa_sign(),
 *     mbedtls_ls_otbn_rsa_modexp*(), ...) return MBEDTLS_ERR_* only. For
 *     LS_OTBN_ERR_MBEDTLS they restore the exact backend value from arg[0];
 *     for the other driver codes they translate with ls_otbn_err_to_mbedtls().
 *   - OTBN-only entry points (the key-slot registry, the slot signer) return
 *     LS_OTBN_ERR_* and never leak a backend code: a crypto-layer failure is
 *     LS_OTBN_ERR_MBEDTLS, and the _ex variants report the backend's own value
 *     through their status out-parameter. They cannot be folded into mbedtls:
 *     "slot not registered", "cross-core protocol fault" and "mailbox timeout"
 *     have no mbedtls equivalent, and mapping them all onto
 *     PLATFORM_HW_ACCEL_FAILED would make the three indistinguishable exactly
 *     where it matters.
 */

#ifndef LS_OTBN_ERR_H
#define LS_OTBN_ERR_H

#include <stdbool.h>
#include <errno.h>

/*
 * Deliberately free of mbedtls headers: the delegation bus and the OTBN-only
 * entry points must build in an image that does not link mbedtls. The
 * mbedtls-facing converters live in ls_otbn_err_mbedtls.h.
 */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Driver codes. Every value is LS_OTBN_ERR_BASE - n; the base itself is never
 * a return value, it is the boundary between the two domains.
 */
#define LS_OTBN_ERR_BASE              (-0x10000)

#define LS_OTBN_ERR_INVAL             (LS_OTBN_ERR_BASE - 1)  /* NULL / bad argument */
#define LS_OTBN_ERR_NOSUPP            (LS_OTBN_ERR_BASE - 2)  /* curve or op unsupported */
#define LS_OTBN_ERR_RANGE             (LS_OTBN_ERR_BASE - 3)  /* key slot id out of range */
#define LS_OTBN_ERR_NOENT             (LS_OTBN_ERR_BASE - 4)  /* key slot not registered */
#define LS_OTBN_ERR_CURVE             (LS_OTBN_ERR_BASE - 5)  /* key belongs to another curve */
#define LS_OTBN_ERR_PROTO             (LS_OTBN_ERR_BASE - 6)  /* peer descriptor inconsistent */
#define LS_OTBN_ERR_MSGSIZE           (LS_OTBN_ERR_BASE - 7)  /* peer rejected the length */
#define LS_OTBN_ERR_BUSY              (LS_OTBN_ERR_BASE - 8)  /* peer request queue full */
#define LS_OTBN_ERR_ACCESS            (LS_OTBN_ERR_BASE - 9)  /* another call in flight */
#define LS_OTBN_ERR_IO                (LS_OTBN_ERR_BASE - 10) /* request not sent */
#define LS_OTBN_ERR_TIMEOUT           (LS_OTBN_ERR_BASE - 11) /* no reply in time */
#define LS_OTBN_ERR_NOSYS             (LS_OTBN_ERR_BASE - 12) /* peer does not serve this op */
#define LS_OTBN_ERR_RNG               (LS_OTBN_ERR_BASE - 13) /* nonce source failed */

/*
 * Marker for a failure raised by the mbedtls library (the crypto backend of
 * this family). The library's exact code is NOT folded into the return value:
 * it is reported through the caller's status out-parameter (and carried in
 * the reply's arg[0] across the mailbox). The driver domain therefore stays
 * closed, while the detail remains available for logs and for the
 * mbedtls-parity entry points that translate it back.
 */
#define LS_OTBN_ERR_MBEDTLS            (LS_OTBN_ERR_BASE - 14)

/** True when \p rc belongs to the driver domain. */
static inline bool ls_otbn_err_is_custom(int rc)
{
	return rc <= LS_OTBN_ERR_BASE;
}

/**
 * \brief   Translate a driver errno into a driver code.
 *
 *          Used where a helper still returns plain errno and the value is about
 *          to go on the wire.  A driver code already passes through unchanged,
 *          so the call is idempotent.
 *
 *          Do NOT apply it to a result that may be an mbedtls code: mbedtls'
 *          low range (-0x0002..-0x000E) has the same values as -EPERM..-EFAULT,
 *          and mapping those again would turn a genuine MBEDTLS_ERR_MPI_* into
 *          a driver code.  Only code that knows the domain may translate.
 */
static inline int ls_otbn_err_from_errno(int err)
{
	if (err == 0 || ls_otbn_err_is_custom(err)) {
		return err;
	}

	switch (-err) {
	case EINVAL:
		return LS_OTBN_ERR_INVAL;
	case ENOTSUP:
		return LS_OTBN_ERR_NOSUPP;
	case ERANGE:
		return LS_OTBN_ERR_RANGE;
	case ENOENT:
		return LS_OTBN_ERR_NOENT;
	case EMSGSIZE:
		return LS_OTBN_ERR_MSGSIZE;
	case EBUSY:
		return LS_OTBN_ERR_BUSY;
	case EACCES:
		return LS_OTBN_ERR_ACCESS;
	case EIO:
		return LS_OTBN_ERR_IO;
	case ETIMEDOUT:
		return LS_OTBN_ERR_TIMEOUT;
	case ENOSYS:
		return LS_OTBN_ERR_NOSYS;
	case EBADMSG:
		return LS_OTBN_ERR_CURVE;
	case EPROTO:
		return LS_OTBN_ERR_PROTO;
	default:
		return LS_OTBN_ERR_INVAL;
	}
}

#ifdef __cplusplus
}
#endif

#endif /* LS_OTBN_ERR_H */
