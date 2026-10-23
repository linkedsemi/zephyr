/*
 * Slot-based ECDSA signing on the OTBN delegation bus.
 *
 * The private key never travels over the mailbox. The serving core -- the one
 * that owns the OTBN, normally CPU1 -- keeps a table of key slots; the
 * requesting core (normally CPU2) sends only a slot id and the hash, and
 * receives r/s. The serving application registers each key buffer once with
 * ls_otbn_ecdsa_key_slot_register().
 *
 * The slot table borrows the caller's key buffer: it stores the pointer, it
 * does not copy the key. The buffer must therefore stay alive and unchanged
 * until the slot is unregistered, and it must be reachable from the serving
 * core. The module cannot detect a dangling or reused buffer; the caller owns
 * that lifetime. Unregistering clears the slot descriptor only -- zeroing the
 * key material remains the caller's responsibility.
 *
 * Both images must come from the same build tree with
 * CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_SLOT_MBOX enabled: the client defines
 * the signing entry point, the server defines the registry and the handler.
 *
 * Role gating: mbedtls_ls_otbn_ecdsa_sign_slot() is declared only in the
 * delegation client image, and ls_otbn_ecdsa_key_slot_register()/_unregister()
 * only in the delegation server image. Calling an entry point on the wrong
 * side is therefore a compile error, not a link error.
 *
 * The usual delegation rules apply (one call at a time, buffers must stay
 * alive until the call returns, every failure is a return value); see
 * ls_otbn_delegation.h and mbedtls_otbn_ecdsa_mbox.h.
 */

#ifndef MBEDTLS_OTBN_ECDSA_SLOT_MBOX_H
#define MBEDTLS_OTBN_ECDSA_SLOT_MBOX_H

#include <stddef.h>
#include <stdint.h>

#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"

/*
 * This header is only usable when the slot-mbox family is built. Turn a
 * missing switch (or a build without a delegation role) into a compile-time
 * error instead of an obscure link error.
 */
#if !defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_SLOT_MBOX)
#error "mbedtls_otbn_ecdsa_slot_mbox.h requires CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_SLOT_MBOX"
#endif

#if !defined(CONFIG_LS_OTBN_DELEGATION_CLIENT) && \
	!defined(CONFIG_LS_OTBN_DELEGATION_SERVER)
#error "the slot-mbox path needs a delegation role (client or server)"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Client-side entry point: only the delegation client image defines it. */
#if defined(CONFIG_LS_OTBN_DELEGATION_CLIENT)

/**
 * \brief   Compute an ECDSA signature on the peer core using a key slot.
 *
 *          Same contract as mbedtls_ls_otbn_ecdsa_sign(), except that the
 *          private key is selected by \p slot_id on the peer core instead of
 *          being passed in. \p buf / \p blen is the hash, truncated to the
 *          curve size when longer, and the signature is written to \p r and
 *          \p s. If \p f_rng is non-NULL, this core generates the
 *          per-signature nonce and sends it to the peer; the callback must
 *          supply a CSPRNG and unique value per signature, and \p p_rng is
 *          passed to it. If \p f_rng is NULL, the peer supplies the nonce.
 *          \p grp must be loaded with a supported curve (SECP256R1, SECP384R1
 *          or SM2); any other curve fails with -EIO before anything is sent.
 *
 * \param grp       Group of the key registered in \p slot_id.
 * \param r         Output signature component r.
 * \param s         Output signature component s.
 * \param slot_id   Slot index in the serving core's key table.
 * \param buf       Hash to sign.
 * \param blen      Length of \p buf in bytes.
 * \param f_rng     Optional nonce source; NULL lets the peer choose.
 * \param p_rng     Context for \p f_rng.
 *
 * \return          0 on success, the peer's code on failure: -ENOENT if the
 *                  slot is not registered, -EINVAL if the slot's curve does
 *                  not match \p grp, -EOPNOTSUPP if the peer does not support
 *                  the curve; -EIO if the request could not be sent,
 *                  -ETIMEDOUT if no matching reply arrived in time.
 */
int mbedtls_ls_otbn_ecdsa_sign_slot(mbedtls_ecp_group *grp,
                                    mbedtls_mpi *r, mbedtls_mpi *s,
                                    uint32_t slot_id,
                                    const unsigned char *buf, size_t blen,
                                    int (*f_rng)(void *, unsigned char *, size_t),
                                    void *p_rng);

#endif /* CONFIG_LS_OTBN_DELEGATION_CLIENT */

/* Serving-core registry: only the delegation server image defines it. */
#if defined(CONFIG_LS_OTBN_DELEGATION_SERVER)

/**
 * \brief   Register a private key in a slot on the serving core.
 *
 *          The key buffer is borrowed, not copied: \p d must stay alive and
 *          unchanged until ls_otbn_ecdsa_key_slot_unregister() is called for
 *          the same slot. The buffer holds the private scalar as a fixed-size
 *          little-endian integer -- the same layout
 *          mbedtls_mpi_write_binary_le() produces -- of 32 bytes for
 *          SECP256R1/SM2 and 48 bytes for SECP384R1. Registering an occupied
 *          slot replaces its descriptor; the previous key buffer is not
 *          touched.
 *
 *          Only available on the serving core (the delegation server build).
 *
 * \param slot_id   Slot index, in [0, CONFIG_LS_OTBN_ECDSA_SLOT_MBOX_COUNT - 1].
 * \param gid       Curve of \p d.
 * \param d         Borrowed private key buffer, little-endian.
 *
 * \return          0 on success, -EINVAL if \p slot_id is out of range or
 *                  \p d is NULL, -EOPNOTSUPP if \p gid is not one of the
 *                  supported curves, otherwise the mbedtls ECP error from
 *                  key validation (for example MBEDTLS_ERR_ECP_INVALID_KEY).
 */
int ls_otbn_ecdsa_key_slot_register(uint32_t slot_id,
                                    mbedtls_ecp_group_id gid,
                                    const uint8_t *d);

/**
 * \brief   Remove a key slot from the serving core.
 *
 *          Clears the descriptor only. The caller's key buffer is not freed
 *          or zeroed -- that remains the caller's responsibility. Signing
 *          requests for the slot fail with -ENOENT afterwards.
 *
 *          Only available on the serving core (the delegation server build).
 *
 * \param slot_id   Slot index.
 *
 * \return          0 on success, -EINVAL if \p slot_id is out of range,
 *                  -ENOENT if the slot is not registered.
 */
int ls_otbn_ecdsa_key_slot_unregister(uint32_t slot_id);

#endif /* CONFIG_LS_OTBN_DELEGATION_SERVER */

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_OTBN_ECDSA_SLOT_MBOX_H */
