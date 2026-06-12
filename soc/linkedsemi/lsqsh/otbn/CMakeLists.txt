zephyr_include_directories(.)
# mbedtls
if(CONFIG_MBEDTLS)
zephyr_include_directories(mbedtls/.)
zephyr_include_directories(${ZEPHYR_BASE}/../modules/crypto/mbedtls/include/)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN
                            mbedtls/mbedtls_otbn_msp.c
                            )
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION mbedtls/mbedtls_otbn_crypto_mbox.c)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_SHA384_SHA512_LINKEDSEMI_OTBN_ALT
                            mbedtls/mbedtls_otbn_sha512.c
                            mbedtls/mbedtls_otbn_sha384.c
                            )
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_SHA256_SM3_LINKEDSEMI_OTBN_ALT
                            mbedtls/mbedtls_otbn_sha256.c
                            mbedtls/mbedtls_otbn_sm3.c
                            )
endif()

# wolfssl
if(CONFIG_WOLFSSL)
zephyr_include_directories(wolfssl/.)
zephyr_library_sources_ifdef(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION wolfssl/wolfssl_otbn_crypto_mbox.c)
zephyr_library_sources_ifdef(CONFIG_WOLFSSL
                            wolfssl/random_internal.c
                            )
endif()

# Common OTBN source files
zephyr_library_sources_ifdef(CONFIG_LINKEDSEMI_OTBN otbn_common.c)
