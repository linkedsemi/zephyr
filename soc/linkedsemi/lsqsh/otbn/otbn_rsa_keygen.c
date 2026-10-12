/*
 * Copyright (c) 2026 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "otbn_rsa_keygen.h"

#include <string.h>
#include <errno.h>
#include <stddef.h>

#include "ls_otbn_config.h"
#include "ls_hal_otbn.h"
#include "ls_otbn_rsa.h"
#include "ls_msp_otbn.h"

/* Session acquire timeout in seconds. Failure to get the session is reported as
 * -EBUSY: the caller can retry, and the OTBN is a shared accelerator. */
#define OTBN_RSA_KEYGEN_SESSION_TIMEOUT_S   10

/* Attempt budget per prime. The search has no guaranteed bound, so a broken or
 * biased RNG must not hang the caller forever; 20000 candidates is two orders
 * of magnitude above the ~130 survivors a 4096-bit prime needs. */
#define OTBN_RSA_KEYGEN_MAX_TRIES           20000

/* Small primes up to ~10000 for CPU-side trial division. Skipping 2 is fine
 * because candidates are forced odd. Rejecting a candidate here costs a few
 * hundred cycles, while sending it to OTBN costs a full Miller-Rabin round, so
 * this table carries most of the composite filtering. */
static const uint16_t otbn_rsa_small_primes[] = {
        3,    5,    7,   11,   13,   17,   19,   23,   29,   31,
       37,   41,   43,   47,   53,   59,   61,   67,   71,   73,
       79,   83,   89,   97,  101,  103,  107,  109,  113,  127,
      131,  137,  139,  149,  151,  157,  163,  167,  173,  179,
      181,  191,  193,  197,  199,  211,  223,  227,  229,  233,
      239,  241,  251,  257,  263,  269,  271,  277,  281,  283,
      293,  307,  311,  313,  317,  331,  337,  347,  349,  353,
      359,  367,  373,  379,  383,  389,  397,  401,  409,  419,
      421,  431,  433,  439,  443,  449,  457,  461,  463,  467,
      479,  487,  491,  499,  503,  509,  521,  523,  541,  547,
      557,  563,  569,  571,  577,  587,  593,  599,  601,  607,
      613,  617,  619,  631,  641,  643,  647,  653,  659,  661,
      673,  677,  683,  691,  701,  709,  719,  727,  733,  739,
      743,  751,  757,  761,  769,  773,  787,  797,  809,  811,
      821,  823,  827,  829,  839,  853,  857,  859,  863,  877,
      881,  883,  887,  907,  911,  919,  929,  937,  941,  947,
      953,  967,  971,  977,  983,  991,  997, 1009, 1013, 1019,
     1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087,
     1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153,
     1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229,
     1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297,
     1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373, 1381,
     1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453,
     1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 1523,
     1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597,
     1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 1663,
     1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741,
     1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823,
     1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901,
     1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993,
     1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063,
     2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113, 2129, 2131,
     2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221,
     2237, 2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287, 2293,
     2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 2371,
     2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423, 2437,
     2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 2539,
     2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 2621,
     2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689,
     2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741, 2749,
     2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833,
     2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903, 2909,
     2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001,
     3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079, 3083,
     3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169, 3181, 3187,
     3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 3259,
     3271, 3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343,
     3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407, 3413, 3433,
     3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 3517,
     3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571, 3581,
     3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643, 3659,
     3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727, 3733,
     3739, 3761, 3767, 3769, 3779, 3793, 3797, 3803, 3821, 3823,
     3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907, 3911,
     3917, 3919, 3923, 3929, 3931, 3943, 3947, 3967, 3989, 4001,
     4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051, 4057, 4073,
     4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139, 4153,
     4157, 4159, 4177, 4201, 4211, 4217, 4219, 4229, 4231, 4241,
     4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297, 4327,
     4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409, 4421,
     4423, 4441, 4447, 4451, 4457, 4463, 4481, 4483, 4493, 4507,
     4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583, 4591,
     4597, 4603, 4621, 4637, 4639, 4643, 4649, 4651, 4657, 4663,
     4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733, 4751, 4759,
     4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 4861,
     4871, 4877, 4889, 4903, 4909, 4919, 4931, 4933, 4937, 4943,
     4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 5009,
     5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087, 5099,
     5101, 5107, 5113, 5119, 5147, 5153, 5167, 5171, 5179, 5189,
     5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279, 5281,
     5297, 5303, 5309, 5323, 5333, 5347, 5351, 5381, 5387, 5393,
     5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441, 5443, 5449,
     5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521, 5527,
     5531, 5557, 5563, 5569, 5573, 5581, 5591, 5623, 5639, 5641,
     5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693, 5701,
     5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791, 5801,
     5807, 5813, 5821, 5827, 5839, 5843, 5849, 5851, 5857, 5861,
     5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939, 5953,
     5981, 5987, 6007, 6011, 6029, 6037, 6043, 6047, 6053, 6067,
     6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131, 6133, 6143,
     6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221, 6229,
     6247, 6257, 6263, 6269, 6271, 6277, 6287, 6299, 6301, 6311,
     6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 6373,
     6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473, 6481,
     6491, 6521, 6529, 6547, 6551, 6553, 6563, 6569, 6571, 6577,
     6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673, 6679,
     6689, 6691, 6701, 6703, 6709, 6719, 6733, 6737, 6761, 6763,
     6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829, 6833, 6841,
     6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 6947,
     6949, 6959, 6961, 6967, 6971, 6977, 6983, 6991, 6997, 7001,
     7013, 7019, 7027, 7039, 7043, 7057, 7069, 7079, 7103, 7109,
     7121, 7127, 7129, 7151, 7159, 7177, 7187, 7193, 7207, 7211,
     7213, 7219, 7229, 7237, 7243, 7247, 7253, 7283, 7297, 7307,
     7309, 7321, 7331, 7333, 7349, 7351, 7369, 7393, 7411, 7417,
     7433, 7451, 7457, 7459, 7477, 7481, 7487, 7489, 7499, 7507,
     7517, 7523, 7529, 7537, 7541, 7547, 7549, 7559, 7561, 7573,
     7577, 7583, 7589, 7591, 7603, 7607, 7621, 7639, 7643, 7649,
     7669, 7673, 7681, 7687, 7691, 7699, 7703, 7717, 7723, 7727,
     7741, 7753, 7757, 7759, 7789, 7793, 7817, 7823, 7829, 7841,
     7853, 7867, 7873, 7877, 7879, 7883, 7901, 7907, 7919, 7927,
     7933, 7937, 7949, 7951, 7963, 7993, 8009, 8011, 8017, 8039,
     8053, 8059, 8069, 8081, 8087, 8089, 8093, 8101, 8111, 8117,
     8123, 8147, 8161, 8167, 8171, 8179, 8191, 8209, 8219, 8221,
     8231, 8233, 8237, 8243, 8263, 8269, 8273, 8287, 8291, 8293,
     8297, 8311, 8317, 8329, 8353, 8363, 8369, 8377, 8387, 8389,
     8419, 8423, 8429, 8431, 8443, 8447, 8461, 8467, 8501, 8513,
     8521, 8527, 8537, 8539, 8543, 8563, 8573, 8581, 8597, 8599,
     8609, 8623, 8627, 8629, 8641, 8647, 8663, 8669, 8677, 8681,
     8689, 8693, 8699, 8707, 8713, 8719, 8731, 8737, 8741, 8747,
     8753, 8761, 8779, 8783, 8803, 8807, 8819, 8821, 8831, 8837,
     8839, 8849, 8861, 8863, 8867, 8887, 8893, 8923, 8929, 8933,
     8941, 8951, 8963, 8969, 8971, 8999, 9001, 9007, 9011, 9013,
     9029, 9041, 9043, 9049, 9059, 9067, 9091, 9103, 9109, 9127,
     9133, 9137, 9151, 9157, 9161, 9173, 9181, 9187, 9199, 9203,
     9209, 9221, 9227, 9239, 9241, 9257, 9277, 9281, 9283, 9293,
     9311, 9319, 9323, 9337, 9341, 9343, 9349, 9371, 9377, 9391,
     9397, 9403, 9413, 9419, 9421, 9431, 9433, 9437, 9439, 9461,
     9463, 9467, 9473, 9479, 9491, 9497, 9511, 9521, 9533, 9539,
     9547, 9551, 9587, 9601, 9613, 9619, 9623, 9629, 9631, 9643,
     9649, 9661, 9677, 9679, 9689, 9697, 9719, 9721, 9733, 9739,
     9743, 9749, 9767, 9769, 9781, 9787, 9791, 9803, 9811, 9817,
     9829, 9833, 9839, 9851, 9857, 9859, 9871, 9883, 9887, 9901,
     9907, 9923, 9929, 9931, 9941, 9949, 9967, 9973,
};

#define OTBN_RSA_NUM_SMALL_PRIMES \
    (sizeof(otbn_rsa_small_primes) / sizeof(otbn_rsa_small_primes[0]))

/* Candidate verdicts returned by the CHECK_PRIME wrapper. */
#define OTBN_RSA_COMPOSITE   1
#define OTBN_RSA_PRIME       0

/* CHECK_PRIME reports its verdict at RSA_KEYGEN_OFFSET_STATUS; the firmware
 * writes all ones when the candidate survives all Miller-Rabin rounds. */
#define OTBN_RSA_KEYGEN_STATUS_PASS   0xFFFFFFFFU

static uint32_t otbn_rsa_bits(otbn_rsa_keygen_size_t size)
{
    switch (size) {
    case OTBN_RSA_KEYGEN_2048:
        return 2048;
    case OTBN_RSA_KEYGEN_3072:
        return 3072;
    case OTBN_RSA_KEYGEN_4096:
        return 4096;
    default:
        return 0;
    }
}

uint32_t otbn_rsa_modulus_bytes(otbn_rsa_keygen_size_t size)
{
    return otbn_rsa_bits(size) / 8;
}

uint32_t otbn_rsa_prime_bytes(otbn_rsa_keygen_size_t size)
{
    return otbn_rsa_bits(size) / 16;
}

static uint32_t otbn_rsa_check_prime_mode(otbn_rsa_keygen_size_t size)
{
    switch (size) {
    case OTBN_RSA_KEYGEN_2048:
        return MODE_RSA_KEYGEN_CHECK_PRIME_2048;
    case OTBN_RSA_KEYGEN_3072:
        return MODE_RSA_KEYGEN_CHECK_PRIME_3072;
    case OTBN_RSA_KEYGEN_4096:
        return MODE_RSA_KEYGEN_CHECK_PRIME_4096;
    default:
        return 0;
    }
}

static uint32_t otbn_rsa_key_from_pq_mode(otbn_rsa_keygen_size_t size)
{
    switch (size) {
    case OTBN_RSA_KEYGEN_2048:
        return MODE_RSA_KEYGEN_KEY_FROM_PQ_2048;
    case OTBN_RSA_KEYGEN_3072:
        return MODE_RSA_KEYGEN_KEY_FROM_PQ_3072;
    case OTBN_RSA_KEYGEN_4096:
        return MODE_RSA_KEYGEN_KEY_FROM_PQ_4096;
    default:
        return 0;
    }
}

/* Buffers handed to the OTBN memory helpers must be 4-byte aligned, since the
 * helpers program the DMEM/IMEM bus from 32-bit words. */
static bool otbn_rsa_is_aligned(const void *buffer)
{
    return (((uintptr_t)buffer & 0x3U) == 0U);
}

static void otbn_rsa_zeroize(void *buffer, size_t len)
{
    volatile uint8_t *p = (volatile uint8_t *)buffer;

    while (len-- > 0) {
        *p++ = 0;
    }
}

/* Load the keygen image and publish it through the shared IMEM state, so other
 * modules (hash/ECC/modexp) reload their own image before their next use.
 * IMEM is hardware memory and survives session acquire/release, so the write is
 * skipped when the image is already resident. */
static int otbn_rsa_ensure_keygen_firmware(void)
{
    int ret;

    if (ls_otbn_imem_firmware_get() == OTBN_FIRMWARE_RSA_KEYGEN) {
        return 0;
    }

    ret = ls_otbn_imem_write(0, (const uint32_t *)rsa_keygen_imem,
                             RSA_KEYGEN_IMEM_SIZE);
    if (ret != 0) {
        return ret;
    }

    ls_otbn_imem_firmware_confirm(OTBN_FIRMWARE_RSA_KEYGEN);

    return 0;
}

/* Hand the command to OTBN and surface execution errors. ls_otbn_cmd() only
 * logs the error bit, so it has to be read here or a failed execution would be
 * taken for a result. */
static int otbn_rsa_execute(void)
{
    int ret = ls_otbn_cmd(OTBN_CMD_EXECUTE);

    if (ret != 0) {
        return ret;
    }
    if (HAL_OTBN_Error_Bit_Get()) {
        return -EIO;
    }

    return 0;
}

/* Trial-divide a candidate against the small primes. Returns true when a
 * factor was found, i.e. the candidate is composite. */
static bool otbn_rsa_has_small_factor(const uint8_t *candidate_le,
                                      size_t prime_bytes)
{
    for (size_t idx = 0; idx < OTBN_RSA_NUM_SMALL_PRIMES; idx++) {
        uint32_t divisor = otbn_rsa_small_primes[idx];
        uint32_t rem = 0;

        /* The candidate is little-endian: fold it in from the most
         * significant byte so the intermediate value stays below 2^32. */
        for (int i = (int)(prime_bytes - 1); i >= 0; i--) {
            rem = ((rem << 8) | candidate_le[i]) % divisor;
        }

        if (rem == 0) {
            return true;
        }
    }

    return false;
}

int otbn_rsa_gen_candidate(otbn_rsa_keygen_size_t size, otbn_rsa_rng_cb rng,
                           void *rng_ctx, uint8_t *candidate_le)
{
    uint32_t prime_bytes = otbn_rsa_prime_bytes(size);
    int ret;

    if (rng == NULL || candidate_le == NULL || prime_bytes == 0) {
        return -EINVAL;
    }
    if (!otbn_rsa_is_aligned(candidate_le)) {
        return -EINVAL;
    }

    ret = rng(rng_ctx, candidate_le, prime_bytes);
    if (ret != 0) {
        return ret;
    }

    /* Force odd and congruent to 3 (mod 4) so the value handed to CHECK_PRIME
     * is the same value the firmware tests. Also set the two most significant
     * bits so that p*q has the requested bit length. */
    candidate_le[0] |= 0x03U;
    candidate_le[prime_bytes - 1] |= 0xC0U;

    return 0;
}

/* CHECK_PRIME with the session already held by the caller. Returns
 * OTBN_RSA_PRIME, OTBN_RSA_COMPOSITE, or a negative errno. */
static int otbn_rsa_check_prime_locked(otbn_rsa_keygen_size_t size,
                                       const uint8_t *candidate_le)
{
    uint32_t mode = otbn_rsa_check_prime_mode(size);
    uint32_t prime_bytes = otbn_rsa_prime_bytes(size);
    uint32_t status = 0;
    int ret;

    ret = ls_otbn_dmem_write(RSA_KEYGEN_OFFSET_MODE, &mode, sizeof(mode));
    if (ret != 0) {
        return ret;
    }

    ret = ls_otbn_dmem_write(RSA_KEYGEN_OFFSET_P, (const uint32_t *)candidate_le,
                             prime_bytes);
    if (ret != 0) {
        return ret;
    }

    ret = otbn_rsa_execute();
    if (ret != 0) {
        return ret;
    }

    ret = ls_otbn_dmem_read(RSA_KEYGEN_OFFSET_STATUS, &status, sizeof(status));
    if (ret != 0) {
        return ret;
    }

    return (status == OTBN_RSA_KEYGEN_STATUS_PASS) ? OTBN_RSA_PRIME
                                                   : OTBN_RSA_COMPOSITE;
}

int otbn_rsa_check_prime(otbn_rsa_keygen_size_t size,
                         const uint8_t *candidate_le, bool *probably_prime)
{
    int ret;

    if (candidate_le == NULL || probably_prime == NULL) {
        return -EINVAL;
    }
    if (otbn_rsa_prime_bytes(size) == 0 || !otbn_rsa_is_aligned(candidate_le)) {
        return -EINVAL;
    }

    ret = ls_otbn_session_acquire(OTBN_FIRMWARE_RSA_KEYGEN,
                                  OTBN_RSA_KEYGEN_SESSION_TIMEOUT_S);
    if (ret != 0) {
        return -EBUSY;
    }

    ret = otbn_rsa_ensure_keygen_firmware();
    if (ret == 0) {
        ret = ls_otbn_dmem_set(0, 0, RSA_KEYGEN_DMEM_SIZE);
    }
    if (ret == 0) {
        ret = otbn_rsa_check_prime_locked(size, candidate_le);
    }

    ls_otbn_session_release();

    if (ret < 0) {
        return ret;
    }

    *probably_prime = (ret == OTBN_RSA_PRIME);

    return 0;
}

/* KEY_FROM_PQ with the session already held by the caller. */
static int otbn_rsa_key_from_pq_locked(otbn_rsa_keygen_size_t size,
                                       const uint8_t *p_le, const uint8_t *q_le,
                                       uint8_t *n_le, uint8_t *d_le)
{
    uint32_t mode = otbn_rsa_key_from_pq_mode(size);
    uint32_t prime_bytes = otbn_rsa_prime_bytes(size);
    uint32_t modulus_bytes = otbn_rsa_modulus_bytes(size);
    int ret;

    ret = ls_otbn_dmem_write(RSA_KEYGEN_OFFSET_MODE, &mode, sizeof(mode));
    if (ret != 0) {
        return ret;
    }

    ret = ls_otbn_dmem_write(RSA_KEYGEN_OFFSET_P, (const uint32_t *)p_le,
                             prime_bytes);
    if (ret != 0) {
        return ret;
    }

    ret = ls_otbn_dmem_write(RSA_KEYGEN_OFFSET_Q, (const uint32_t *)q_le,
                             prime_bytes);
    if (ret != 0) {
        return ret;
    }

    ret = otbn_rsa_execute();
    if (ret != 0) {
        return ret;
    }

    ret = ls_otbn_dmem_read(RSA_KEYGEN_OFFSET_N, (uint32_t *)n_le,
                            modulus_bytes);
    if (ret != 0) {
        return ret;
    }

    return ls_otbn_dmem_read(RSA_KEYGEN_OFFSET_D, (uint32_t *)d_le,
                             modulus_bytes);
}

int otbn_rsa_key_from_pq(otbn_rsa_keygen_size_t size, const uint8_t *p_le,
                         const uint8_t *q_le, uint8_t *n_le, uint8_t *d_le)
{
    int ret;

    if (p_le == NULL || q_le == NULL || n_le == NULL || d_le == NULL) {
        return -EINVAL;
    }
    if (otbn_rsa_prime_bytes(size) == 0 ||
        !otbn_rsa_is_aligned(p_le) || !otbn_rsa_is_aligned(q_le) ||
        !otbn_rsa_is_aligned(n_le) || !otbn_rsa_is_aligned(d_le)) {
        return -EINVAL;
    }

    ret = ls_otbn_session_acquire(OTBN_FIRMWARE_RSA_KEYGEN,
                                  OTBN_RSA_KEYGEN_SESSION_TIMEOUT_S);
    if (ret != 0) {
        return -EBUSY;
    }

    ret = otbn_rsa_ensure_keygen_firmware();
    if (ret == 0) {
        ret = ls_otbn_dmem_set(0, 0, RSA_KEYGEN_DMEM_SIZE);
    }
    if (ret == 0) {
        ret = otbn_rsa_key_from_pq_locked(size, p_le, q_le, n_le, d_le);
    }

    ls_otbn_session_release();

    return ret;
}

/* Search for one prime, writing it to out_le. The caller holds the session. */
static int otbn_rsa_find_prime(const otbn_rsa_keygen_req_t *req,
                               uint32_t prime_bytes, uint8_t *out_le)
{
    uint32_t tries = 0;
    int ret;

    do {
        ret = otbn_rsa_gen_candidate(req->size, req->rng, req->rng_ctx, out_le);
        if (ret != 0) {
            return ret;
        }

        if (otbn_rsa_has_small_factor(out_le, prime_bytes)) {
            continue;
        }

        ret = otbn_rsa_check_prime_locked(req->size, out_le);
        if (ret < 0) {
            return ret;
        }
        if (ret == OTBN_RSA_PRIME) {
            return 0;
        }
    } while (++tries < OTBN_RSA_KEYGEN_MAX_TRIES);

    return -ENODATA;
}

int otbn_rsa_keygen(const otbn_rsa_keygen_req_t *req)
{
    uint32_t prime_bytes;
    int ret;

    if (req == NULL || req->rng == NULL) {
        return -EINVAL;
    }

    prime_bytes = otbn_rsa_prime_bytes(req->size);
    if (prime_bytes == 0) {
        return -EINVAL;
    }
    if (req->p_le == NULL || req->q_le == NULL ||
        req->n_le == NULL || req->d_le == NULL) {
        return -EINVAL;
    }
    if (!otbn_rsa_is_aligned(req->p_le) || !otbn_rsa_is_aligned(req->q_le) ||
        !otbn_rsa_is_aligned(req->n_le) || !otbn_rsa_is_aligned(req->d_le)) {
        return -EINVAL;
    }
    if (otbn_rsa_check_prime_mode(req->size) == 0 ||
        otbn_rsa_key_from_pq_mode(req->size) == 0) {
        return -EINVAL;
    }

    ret = ls_otbn_session_acquire(OTBN_FIRMWARE_RSA_KEYGEN,
                                  OTBN_RSA_KEYGEN_SESSION_TIMEOUT_S);
    if (ret != 0) {
        return -EBUSY;
    }

    ret = otbn_rsa_ensure_keygen_firmware();
    if (ret == 0) {
        ret = ls_otbn_dmem_set(0, 0, RSA_KEYGEN_DMEM_SIZE);
    }

    if (ret == 0) {
        ret = otbn_rsa_find_prime(req, prime_bytes, req->p_le);
    }
    if (ret == 0) {
        ret = otbn_rsa_find_prime(req, prime_bytes, req->q_le);
    }
    if (ret == 0) {
        ret = otbn_rsa_key_from_pq_locked(req->size, req->p_le, req->q_le,
                                         req->n_le, req->d_le);
    }

    if (ret != 0) {
        /* Do not leave a half-built key behind for the caller to use. */
        otbn_rsa_zeroize(req->p_le, prime_bytes);
        otbn_rsa_zeroize(req->q_le, prime_bytes);
        otbn_rsa_zeroize(req->n_le, otbn_rsa_modulus_bytes(req->size));
        otbn_rsa_zeroize(req->d_le, otbn_rsa_modulus_bytes(req->size));
    }

    ls_otbn_session_release();

    return ret;
}
