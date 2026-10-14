zephyr_include_directories(.)
# Public OTBN API headers: the ones consumed from outside this directory
# (HAL, the mbedtls/wolfssl ports, samples, test projects). Internal ones stay
# next to their sources.
zephyr_include_directories(include)
# mbedtls
if(CONFIG_MBEDTLS)
zephyr_include_directories(${ZEPHYR_BASE}/../modules/crypto/mbedtls/include/)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN
                            mbedtls/mbedtls_otbn_msp.c
                            )
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION
                            mbedtls/mbedtls_otbn_ecdsa_mbox.c
                            mbedtls/mbedtls_otbn_rsa_mbox.c
                            )
endif()

# The delegation bus is crypto-library agnostic -- it must be built whenever
# either library enables a delegation side (LS_OTBN_DELEGATION is selected by
# both the mbedtls and the wolfssl switches), including a wolfssl-only image
# where CONFIG_MBEDTLS is off.
zephyr_library_sources_ifdef(CONFIG_LS_OTBN_DELEGATION otbn_delegation.c)

# wolfssl
if(CONFIG_WOLFSSL)
zephyr_include_directories(wolfssl/.)
zephyr_library_sources_ifdef(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION wolfssl/wolfssl_otbn_crypto_mbox.c)
if(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_SHA224_SHA256_ALT)
  zephyr_library_sources(wolfssl/wc_otbn_sha256.c)
endif()
if(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_SHA384_SHA512_ALT)
  zephyr_library_sources(wolfssl/wc_otbn_sha512.c)
endif()
if(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_SM3_ALT)
  zephyr_library_sources(wolfssl/wc_otbn_sm3.c)
endif()
zephyr_library_sources_ifdef(CONFIG_WOLFSSL
                            wolfssl/random_internal.c
                            )
endif()

# Common OTBN source files
zephyr_library_sources_ifdef(CONFIG_LINKEDSEMI_OTBN otbn_common.c otbn_hash.c otbn_ecc.c otbn_pointops.c)
zephyr_library_sources_ifdef(CONFIG_LS_OTBN_RSA_KEYGEN otbn_rsa_keygen.c)
