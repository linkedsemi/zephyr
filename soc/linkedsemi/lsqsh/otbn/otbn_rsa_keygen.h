#ifndef OTBN_RSA_KEYGEN_H
#define OTBN_RSA_KEYGEN_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ls_otbn_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file otbn_rsa_keygen.h
 * @brief OTBN-accelerated RSA key generation.
 *
 * The OTBN keygen firmware provides two primitives, both driven from here:
 *   - CHECK_PRIME: Miller-Rabin on a host-supplied candidate prime.
 *   - KEY_FROM_PQ: derive n = p*q and d = e^-1 mod lcm(p-1, q-1) from p and q.
 * Candidate generation and the small-prime trial division run on the CPU,
 * because the firmware reads one candidate at a time and has no cheap way to
 * reject the many composites that never deserve a Miller-Rabin round.
 *
 * The RNG is injected by the caller, so the same code path serves both fresh
 * keys (pass a DRBG) and reproducible ones (pass a deterministic stream).
 */

/** @brief Modulus size. Selects the firmware mode word, so the enum order must
 *  stay in sync with the MODE_RSA_KEYGEN_* constants in ls_otbn_rsa.h. */
typedef enum {
    OTBN_RSA_KEYGEN_2048 = 0,
    OTBN_RSA_KEYGEN_3072,
    OTBN_RSA_KEYGEN_4096,
} otbn_rsa_keygen_size_t;

/** @brief Public exponent. The firmware derives d with a dedicated F4 routine
 *  (e^-1 mod lcm(p-1, q-1)), so only 65537 is supported. */
#define OTBN_RSA_KEYGEN_EXPONENT   65537u

/** @brief Per-size byte counts, for sizing caller buffers. */
#define OTBN_RSA_KEYGEN_MODULUS_BYTES_2048   256
#define OTBN_RSA_KEYGEN_MODULUS_BYTES_3072   384
#define OTBN_RSA_KEYGEN_MODULUS_BYTES_4096   512
#define OTBN_RSA_KEYGEN_PRIME_BYTES_2048     128
#define OTBN_RSA_KEYGEN_PRIME_BYTES_3072     192
#define OTBN_RSA_KEYGEN_PRIME_BYTES_4096     256

/** @brief Largest buffers, so callers can size a union/stack slot. */
#define OTBN_RSA_KEYGEN_MODULUS_BYTES_MAX    OTBN_RSA_KEYGEN_MODULUS_BYTES_4096
#define OTBN_RSA_KEYGEN_PRIME_BYTES_MAX      OTBN_RSA_KEYGEN_PRIME_BYTES_4096

/**
 * @brief Candidate-prime source.
 *
 * Fills @p buf with @p len random bytes. A DRBG yields fresh keys; a
 * deterministic stream makes generation reproducible.
 *
 * @return 0 on success, negative errno on failure.
 */
typedef int (*otbn_rsa_rng_cb)(void *ctx, uint8_t *buf, size_t len);

/** @brief Modulus size in bytes, or 0 if @p size is unsupported. */
uint32_t otbn_rsa_modulus_bytes(otbn_rsa_keygen_size_t size);

/** @brief Prime size in bytes (half the modulus), or 0 if unsupported. */
uint32_t otbn_rsa_prime_bytes(otbn_rsa_keygen_size_t size);

/** @brief Key generation request. All output buffers are little-endian, must be
 *  4-byte aligned and sized with otbn_rsa_prime_bytes()/otbn_rsa_modulus_bytes().
 *  On success p_le/q_le hold the two primes and n_le/d_le the derived key. */
typedef struct {
    otbn_rsa_keygen_size_t size;
    otbn_rsa_rng_cb        rng;
    void                  *rng_ctx;
    uint8_t               *p_le;
    uint8_t               *q_le;
    uint8_t               *n_le;
    uint8_t               *d_le;
} otbn_rsa_keygen_req_t;

/**
 * @brief Generate an RSA key pair.
 *
 * Holds the OTBN session for the whole search, so the call blocks until both
 * primes are found or the attempt budget is exhausted. The caller is
 * responsible for validating the result (e.g. n = p*q, d*e = 1).
 *
 * @param req Request, see @ref otbn_rsa_keygen_req_t.
 *
 * @return 0 on success, -EINVAL for bad arguments or unsupported size,
 *         -EBUSY if the OTBN session is held by another thread, -ETIMEDOUT on
 *         OTBN timeout, -EIO on OTBN execution error, -ENODATA if no prime was
 *         found within the attempt budget, or the RNG's own error code.
 */
int otbn_rsa_keygen(const otbn_rsa_keygen_req_t *req);

/**
 * @brief Build one candidate prime from the RNG callback.
 *
 * Exported for callers that drive their own prime search. The candidate is odd,
 * has its two most significant bits set, and is congruent to 3 (mod 4).
 *
 * @warning The 3 (mod 4) residue is not cosmetic: the OTBN CHECK_PRIME
 *          interface only accepts candidates in that class, so a caller that
 *          must reproduce another implementation's candidate stream bit for bit
 *          cannot use this helper. See the precondition documented on the
 *          MODE_RSA_KEYGEN_CHECK_PRIME_* constants in ls_otbn_rsa.h.
 *
 * @param size         Modulus size.
 * @param rng          Candidate source, see @ref otbn_rsa_rng_cb.
 * @param rng_ctx      Opaque context passed to @p rng.
 * @param candidate_le Output buffer, otbn_rsa_prime_bytes(@p size) bytes.
 *
 * @return 0 on success, negative errno otherwise.
 */
int otbn_rsa_gen_candidate(otbn_rsa_keygen_size_t size, otbn_rsa_rng_cb rng,
                           void *rng_ctx, uint8_t *candidate_le);

/**
 * @brief Test a candidate for primality with the OTBN CHECK_PRIME firmware.
 *
 * @param size           Modulus size.
 * @param candidate_le   Candidate, otbn_rsa_prime_bytes(@p size) bytes, little
 *                       endian and 4-byte aligned. Must be congruent to
 *                       3 (mod 4); any other residue is reported composite.
 * @param probably_prime Set to true when the firmware accepts the candidate.
 *
 * @return 0 on success, -EINVAL for bad arguments, -EACCES if the current
 *         thread does not hold an OTBN session, -ETIMEDOUT/-EIO on OTBN error.
 */
int otbn_rsa_check_prime(otbn_rsa_keygen_size_t size,
                         const uint8_t *candidate_le, bool *probably_prime);

/**
 * @brief Derive n and d from two primes, using the OTBN KEY_FROM_PQ firmware.
 *
 * Reads no randomness, so the result is a pure function of p, q and e.
 *
 * @param size     Modulus size.
 * @param p_le     First prime, otbn_rsa_prime_bytes(@p size) bytes.
 * @param q_le     Second prime, same size, little endian and 4-byte aligned.
 * @param n_le     Output modulus, otbn_rsa_modulus_bytes(@p size) bytes.
 * @param d_le     Output private exponent, same size as @p n_le.
 *
 * @return 0 on success, -EINVAL for bad arguments, -EACCES if the current
 *         thread does not hold an OTBN session, -ETIMEDOUT/-EIO on OTBN error.
 */
int otbn_rsa_key_from_pq(otbn_rsa_keygen_size_t size, const uint8_t *p_le,
                         const uint8_t *q_le, uint8_t *n_le, uint8_t *d_le);

#ifdef __cplusplus
}
#endif

#endif /* OTBN_RSA_KEYGEN_H */
