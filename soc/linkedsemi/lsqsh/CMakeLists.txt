zephyr_compile_definitions(_GNU_SOURCE)
zephyr_include_directories(.)
zephyr_sources(soc.c)
zephyr_sources(psram.c)
zephyr_sources(dma.c)
zephyr_sources(soc_reset_reason.c)
zephyr_sources(soc_reboot.c)
zephyr_sources_ifdef(CONFIG_NETWORKING_MODULE plugin_section.c)

zephyr_library_sources_ifdef(CONFIG_IOPMP_LINKEDSEMI iopmp.c)
zephyr_library_sources_ifdef(CONFIG_SMP smp/lsqsh_e906_smp.c)

add_subdirectory(libc)
add_subdirectory(otbn)
add_subdirectory(coredump)
add_subdirectory(shell)
add_subdirectory(uart_agent)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/linkedsemi/common/linker.ld CACHE INTERNAL "")

function(add_prebuilt_library lib_name prebuilt_path)
  add_library(${lib_name} STATIC IMPORTED GLOBAL)
  set_target_properties(${lib_name} PROPERTIES
    IMPORTED_LOCATION ${BLOBS_DIR}/${prebuilt_path}
  )
  zephyr_link_libraries(${lib_name})
endfunction()
