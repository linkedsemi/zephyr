


#obtn mbox
zephyr_include_directories(.)
# mbedtls
zephyr_include_directories(${ZEPHYR_BASE}/../modules/crypto/mbedtls/include/)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN 
                            mbedtls_otbn_msp.c 
                            otbn_common.c
                            )
zephyr_library_sources_ifdef(CONFIG_MEBDTLS_LINKEDSEMI_OTBN_DELEGATION mbedtls_otbn_crypto_mbox.c)
if(CONFIG_MBEDTLS)
# zephyr_library_sources(
#                 mbedtls_otbn_sha256.c 
#                 mbedtls_otbn_sha384.c 
#                 mbedtls_otbn_sha512.c 
#                 mbedtls_otbn_sm3.c
# )
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_SHA384_SHA512_LINKEDSEMI_OTBN_ALT 
                            mbedtls_otbn_sha512.c 
                            mbedtls_otbn_sha384.c 
                            )
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_SHA256_SM3_LINKEDSEMI_OTBN_ALT 
                            mbedtls_otbn_sha256.c 
                            mbedtls_otbn_sm3.c
                            )
endif()

# wolfssl
zephyr_library_sources_ifdef(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION wolfssl_otbn_crypto_mbox.c)
zephyr_library_sources_ifdef(CONFIG_WOLFSSL 
                            wolfssl_otbn_msp.c
                            otbn_common.c
                            )
