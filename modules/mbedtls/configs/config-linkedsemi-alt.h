#ifndef _MBEDTLS_LS_CFG_ALT_
#define _MBEDTLS_LS_CFG_ALT_

#if defined(CONFIG_MBEDTLS_CIPHER_AES_LINKEDSEMI)
#define MBEDTLS_AES_ALT
#endif

/* The ALT macros mean "mbedtls_ecdsa_* is not the software implementation".
 * Both hardware entries of choice MBEDTLS_ECDSA_LINKEDSEMI_OTBN_ALT_MODE set
 * them: the local OTBN entry, and the delegation entry (where
 * otbn/mbedtls/mbedtls_otbn_crypto_mbox.c supplies mbedtls_ecdsa_* instead of
 * ecdsa_alt.c). The entry that is OFF - software - leaves them undefined, and
 * that is also the only setting under which MBEDTLS_ECP_RESTARTABLE is legal.
 */
#if defined(CONFIG_MBEDTLS_ECDSA_SECP256R1_SECP384R1_SM2_LINKEDSEMI_OTBN_ALT) || \
    defined(CONFIG_MBEDTLS_ECDSA_LINKEDSEMI_OTBN_ALT_VIA_DELEGATION)
#define MBEDTLS_ECDSA_SIGN_ALT
#define MBEDTLS_ECDSA_VERIFY_ALT
#define MBEDTLS_ECDSA_GENKEY_ALT
#endif
/* Defining these ALT macros makes MBEDTLS_ECP_RESTARTABLE illegal (mbedtls
 * check_config.h). See 810_driver_test/KNOWN_ISSUES.md #120.
 */

#if defined(CONFIG_MBEDTLS_SHA224_SHA256_LINKEDSEMI_HARDWARE_ALT)||defined(CONFIG_MBEDTLS_SHA224_SHA256_LINKEDSEMI_OTBN_ALT)
#define MBEDTLS_SHA256_ALT
#endif

#if defined(CONFIG_MBEDTLS_SHA384_SHA512_LINKEDSEMI_HARDWARE_ALT)||defined(CONFIG_MBEDTLS_SHA384_SHA512_LINKEDSEMI_OTBN_ALT)
#define MBEDTLS_SHA512_ALT
#endif

#if defined(CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT)
#define MBEDTLS_RSA_OTBN_HOOK
#define MBEDTLS_RSA_OTBN_KEYGEN_HOOK
#endif

#if defined(CONFIG_MBEDTLS_THREADING_LINKEDSEMI_ALT)
#define MBEDTLS_THREADING_ALT
#define MBEDTLS_THREADING_C
#endif

#endif /*_MBEDTLS_LS_CFG_ALT_*/