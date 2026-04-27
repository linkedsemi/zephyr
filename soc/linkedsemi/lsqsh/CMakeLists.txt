zephyr_include_directories(.)
zephyr_sources(soc.c)
zephyr_sources(psram.c)
zephyr_sources(dma.c)
zephyr_sources(soc_reset_reason.c)
zephyr_sources(soc_reboot.c)
zephyr_library_sources_ifdef(CONFIG_IOPMP_LINKEDSEMI iopmp.c)

zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH malloc_newlib.c)
zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_TLSF_LSQSH malloc_newlib_tlsf.c tlsf/tlsf.c)
zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_TLSF_MULIT_LSQSH malloc_tlsf_mulit_heap.c tlsf/tlsf.c)
zephyr_library_sources_ifdef(CONFIG_HEAP_DEBUG_LSQSH heap_debug.c)
zephyr_library_include_directories_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH ${ZEPHYR_BASE}/lib/heap)
zephyr_library_sources_ifdef(CONFIG_CXX_OPERATOR_NEW_DELETE cxx_crt_init.cpp)

add_subdirectory(otbn)
add_subdirectory(coredump)
add_subdirectory_ifdef(CONFIG_SHELL shell)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/linkedsemi/common/linker.ld CACHE INTERNAL "")

function(add_prebuilt_library lib_name prebuilt_path)
  add_library(${lib_name} STATIC IMPORTED GLOBAL)
  set_target_properties(${lib_name} PROPERTIES
    IMPORTED_LOCATION ${BLOBS_DIR}/${prebuilt_path}
  )
  zephyr_link_libraries(${lib_name})
endfunction()
