/*
 * Public interface of the OTBN ECDSA delegation proxy (mbedtls_otbn_ecdsa_mbox.c).
 *
 * The functions below are the ones an application calls; the channel they
 * travel over is brought up separately, once, by
 * ls_otbn_delegation_client_init() -- see ls_otbn_delegation.h for that, and
 * for the rules every delegated call follows (one call at a time, buffers must
 * stay alive until the call returns, every failure is a return value).
 *
 * They exist whenever CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_MBOX is enabled on
 * the delegation client and forward the operation to the peer core; the peer
 * answers them with its own OTBN when it runs the delegation server with the
 * same family switch. A build may also hand
 * these entry points to the mbedtls library itself, so that the plain
 * mbedtls_ecdsa_* names resolve here too
 * (CONFIG_MBEDTLS_ECDSA_LINKEDSEMI_OTBN_ALT_VIA_DELEGATION).
 */

#ifndef MBEDTLS_OTBN_ECDSA_MBOX_H
#define MBEDTLS_OTBN_ECDSA_MBOX_H

#include <stddef.h>

#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief   Generate an ECDSA key pair on the peer core.
 *
 *          Same contract as mbedtls_ecdsa_genkey(): the key pair is written
 *          into \p ctx. If \p f_rng is non-NULL, this core generates the
 *          key-generation seed and sends it to the peer; \p p_rng is passed
 *          to it. If \p f_rng is NULL, the peer supplies its own entropy.
 *
 * \param ctx       Context to fill with the generated key pair.
 * \param gid       Curve: SECP256R1, SECP384R1 or SM2. Any other curve fails
 *                  with MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE before anything is
 *                  sent.
 * \param f_rng     Optional seed source; NULL lets the peer choose.
 * \param p_rng     Context for \p f_rng.
 *
 * \return          0 on success. Failures keep a mbedtls value:
 *                  MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE for a curve this proxy
 *                  does not serve; the local group/MPI or the caller's f_rng
 *                  error unchanged; the peer's exact MBEDTLS_ERR_* when it
 *                  rejected the operation. Transport failures become
 *                  MBEDTLS_ERR_LS_OTBN_BUSY (another call in flight, or the
 *                  peer's queue full), MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED
 *                  (send failed, timeout, protocol error) or
 *                  MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED (the peer does not
 *                  serve this family).
 */
int mbedtls_ls_otbn_ecdsa_genkey(mbedtls_ecdsa_context *ctx, mbedtls_ecp_group_id gid,
                                 int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);

/**
 * \brief   Compute an ECDSA signature on the peer core.
 *
 *          Same contract as mbedtls_ecdsa_sign(): \p buf / \p blen is the hash,
 *          truncated to the curve size when longer, and the signature is
 *          written to \p r and \p s. If \p f_rng is non-NULL, this core
 *          generates the per-signature nonce and sends it to the peer; the
 *          callback must supply a CSPRNG and unique value per signature, and
 *          \p p_rng is passed to it. If \p f_rng is NULL, the peer supplies
 *          the nonce. \p grp must be loaded with a supported curve and \p d
 *          must be a valid private key for it; an unsupported curve fails with
 *          MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE rather than being forwarded.
 *
 * \return          0 on success. As in _genkey(), plus
 *                  MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL when \p d does not fit the
 *                  curve size and MBEDTLS_ERR_ECP_INVALID_KEY when the peer
 *                  rejects a key outside [1, n-1].
 */
int mbedtls_ls_otbn_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
                               const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
                               int (*f_rng)(void *, unsigned char *, size_t), void *p_rng);

/**
 * \brief   Verify an ECDSA signature on the peer core.
 *
 *          Same contract as mbedtls_ecdsa_verify(): \p buf / \p blen is the
 *          hash, truncated to the curve size when longer, \p Q the public key
 *          (must be a real affine point, Z != 0) and \p r / \p s the signature.
 *          Unsupported curves fail with MBEDTLS_ERR_ECP_FEATURE_UNAVAILABLE,
 *          as in _sign().
 *
 * \return          0 if the signature is valid,
 *                  MBEDTLS_ERR_ECP_VERIFY_FAILED if it is not. Otherwise the
 *                  same values as _sign() (unsupported curve, a malformed key
 *                  or signature, and the transport failures).
 */
int mbedtls_ls_otbn_ecdsa_verify(mbedtls_ecp_group *grp,
                                 const unsigned char *buf, size_t blen,
                                 const mbedtls_ecp_point *Q,
                                 const mbedtls_mpi *r,
                                 const mbedtls_mpi *s);

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_OTBN_ECDSA_MBOX_H */
