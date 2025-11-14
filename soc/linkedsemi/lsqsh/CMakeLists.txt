zephyr_include_directories(.)
zephyr_sources(soc.c)
zephyr_sources(psram.c)
zephyr_sources(dma.c)
zephyr_sources(soc_reset.c)

zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH malloc_newlib.c)


#obtn mbox
zephyr_include_directories(otbn)
# mbedtls
zephyr_include_directories(${ZEPHYR_BASE}/../modules/crypto/mbedtls/include/)
zephyr_library_sources_ifdef(CONFIG_MBEDTLS_LINKEDSEMI_OTBN otbn/mbedtls_otbn_msp.c)
zephyr_library_sources_ifdef(CONFIG_MEBDTLS_LINKEDSEMI_OTBN_DELEGATION otbn/mbedtls_otbn_crypto_mbox.c)
# wolfssl
zephyr_library_sources_ifdef(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION otbn/wolfssl_otbn_crypto_mbox.c)
zephyr_library_sources_ifdef(CONFIG_WOLFSSL otbn/wolfssl_otbn_msp.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/linkedsemi/common/linker.ld CACHE INTERNAL "")

function(add_prebuilt_library lib_name prebuilt_path)
  add_library(${lib_name} STATIC IMPORTED GLOBAL)
  set_target_properties(${lib_name} PROPERTIES
    IMPORTED_LOCATION ${BLOBS_DIR}/${prebuilt_path}
  )
  zephyr_link_libraries(${lib_name})
endfunction()