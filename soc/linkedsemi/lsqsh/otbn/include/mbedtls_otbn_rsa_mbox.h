/*
 * Public interface of the OTBN RSA delegation proxy
 * (mbedtls_otbn_rsa_mbox.c).
 *
 * The same split as the ECDSA family applies: the client functions below exist
 * whenever CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT is enabled and
 * forward the operation to the peer core over the delegation bus; the peer
 * needs its own OTBN RSA support (CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT) to
 * answer them.
 *
 * Buffers are big-endian and num_bytes wide, matching mbedtls_rsa_public_otbn()
 * / mbedtls_rsa_private_otbn(), so a caller can switch between the local OTBN
 * path and the delegated one without re-encoding anything. Only the modulus
 * sizes the OTBN RSA firmware implements are accepted: 2048, 3072, 4096 bits.
 */

#ifndef MBEDTLS_OTBN_RSA_MBOX_H
#define MBEDTLS_OTBN_RSA_MBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief   Modular exponentiation with a public exponent, on the peer core.
 *
 *          Computes out = in^exp mod n. Typical use is RSA public-key
 *          encryption or signature verification.
 *
 * \param in            Big-endian input, n_size_bits/8 bytes.
 * \param exp           Big-endian public exponent, n_size_bits/8 bytes.
 * \param n             Big-endian modulus, n_size_bits/8 bytes.
 * \param n_size_bits   Modulus size: 2048, 3072 or 4096.
 * \param out           Big-endian result, n_size_bits/8 bytes.
 *
 * \return              0 on success, the peer's error code, or -EINVAL for an
 *                      unsupported modulus size, -EIO / -ETIMEDOUT when the
 *                      peer could not be reached.
 */
int mbedtls_ls_otbn_rsa_modexp_encrypt(const uint8_t *in, const uint8_t *exp,
				       const uint8_t *n, uint32_t n_size_bits,
				       uint8_t *out);

/**
 * \brief   Modular exponentiation with a private exponent, on the peer core.
 *
 *          Computes out = in^d mod n. Typical use is RSA decryption or
 *          signature generation. No blinding is applied: the exponent is used
 *          as given, exactly like the local mbedtls_rsa_private_otbn().
 *
 * \param in            Big-endian input, n_size_bits/8 bytes.
 * \param d             Big-endian private exponent, n_size_bits/8 bytes.
 * \param n             Big-endian modulus, n_size_bits/8 bytes.
 * \param n_size_bits   Modulus size: 2048, 3072 or 4096.
 * \param out           Big-endian result, n_size_bits/8 bytes.
 *
 * \return              See mbedtls_ls_otbn_rsa_modexp_encrypt().
 */
int mbedtls_ls_otbn_rsa_modexp_decrypt(const uint8_t *in, const uint8_t *d,
				       const uint8_t *n, uint32_t n_size_bits,
				       uint8_t *out);

/* ------------------------------------------------- server side backend ----
 * Byte-level modexp on the local OTBN, provided by
 * modules/crypto/mbedtls/library/rsa_alt.c. Declared here because the
 * delegation server is its only context-free caller. Present only when the
 * local OTBN RSA switch is on.
 */
int mbedtls_ls_otbn_rsa_modexp(const uint8_t *input, uint8_t *output,
			       const uint8_t *exp, const uint8_t *n,
			       size_t num_bytes, bool is_public);

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_OTBN_RSA_MBOX_H */
