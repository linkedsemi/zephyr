zephyr_include_directories(.)
zephyr_sources(soc.c)
zephyr_sources(psram.c)
zephyr_sources(dma.c)
zephyr_sources(soc_reset.c)

zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH malloc_newlib.c)
zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_TLSF_LSQSH malloc_newlib_tlsf.c tlsf/tlsf.c)


add_subdirectory(otbn)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/linkedsemi/common/linker.ld CACHE INTERNAL "")

function(add_prebuilt_library lib_name prebuilt_path)
  add_library(${lib_name} STATIC IMPORTED GLOBAL)
  set_target_properties(${lib_name} PROPERTIES
    IMPORTED_LOCATION ${BLOBS_DIR}/${prebuilt_path}
  )
  zephyr_link_libraries(${lib_name})
endfunction()