#ifndef _MBEDTLS_LS_CFG_TLS_
#define _MBEDTLS_LS_CFG_TLS_

#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY
#define MBEDTLS_MEMORY_BUFFER_ALLOC_C
#define MBEDTLS_MEMORY_ALIGN_MULTIPLE (sizeof(void *))
#define MBEDTLS_PLATFORM_EXIT_ALT
#define MBEDTLS_NO_PLATFORM_ENTROPY

#include "config-linkedsemi-alt.h"

#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_GENPRIME
#define MBEDTLS_OID_C
#define MBEDTLS_MD_C

#define MBEDTLS_ECP_C
#define MBEDTLS_ECDSA_C
// #define MBEDTLS_ECDSA_DETERMINISTIC
#if defined(MBEDTLS_ECDSA_C)
#define MBEDTLS_ASN1_PARSE_C
#endif

#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED

#if defined(MBEDTLS_ECDSA_C) || defined(MBEDTLS_RSA_C) || defined(MBEDTLS_PK_WRITE_C)
#define MBEDTLS_ASN1_WRITE_C
#endif

#if defined(MBEDTLS_ECDSA_C)
#define MBEDTLS_ASN1_PARSE_C
#endif

#if defined(MBEDTLS_DHM_C) || \
    defined(MBEDTLS_ECP_C) || \
    defined(MBEDTLS_RSA_C) || \
    defined(MBEDTLS_GENPRIME)
#define MBEDTLS_BIGNUM_C
#endif

#define MBEDTLS_SHA224_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA256_SMALLER
#define MBEDTLS_SHA384_C
#define MBEDTLS_SHA512_C
#define MBEDTLS_CIPHER_MODE_CTR
#define MBEDTLS_CIPHER_MODE_OFB
#define MBEDTLS_CIPHER_MODE_CFB
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_CIPHER_MODE_XTS
#define MBEDTLS_AES_FEWER_TABLES
#define MBEDTLS_AES_ROM_TABLES
#define MBEDTLS_AES_C
#define MBEDTLS_CCM_C
#define MBEDTLS_GCM_C

/*
 * MBEDTLS_ECP_RESTARTABLE cannot coexist with the Linkedsemi ECC alternative.
 * Restartable ECP needs an interruptible implementation; both the OTBN hardware
 * path and the delegation proxy are blocking. mbedtls rejects the combination
 * in check_config.h -- fail here first, so the conflicting switch is named.
 * Placed at the end of MBEDTLS_CONFIG_FILE, so a -D from the build or a define
 * earlier in this chain is already visible. MBEDTLS_USER_CONFIG_FILE is
 * processed AFTER this file and is caught only by check_config.h.
 * See 810_driver_test/KNOWN_ISSUES.md #120.
 */
#if defined(MBEDTLS_ECP_RESTARTABLE)
#if defined(MBEDTLS_ECDSA_SIGN_ALT) || defined(MBEDTLS_ECDSA_VERIFY_ALT) || defined(MBEDTLS_ECDSA_GENKEY_ALT)
#error "Linkedsemi ECC ALT cannot coexist with MBEDTLS_ECP_RESTARTABLE. Remove MBEDTLS_ECP_RESTARTABLE, or set CONFIG_MBEDTLS_ECDSA_SECP256R1_SECP384R1_SM2_LINKEDSEMI_OTBN_ALT=n and CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT=n. See 810_driver_test/KNOWN_ISSUES.md #120."
#endif
#endif

#endif /*_MBEDTLS_LS_CFG_TLS_*/