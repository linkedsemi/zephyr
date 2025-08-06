zephyr_include_directories(.)
zephyr_sources(soc.c)
zephyr_sources(psram.c)
zephyr_sources(dma.c)
zephyr_sources(fpga.c)
zephyr_sources(soc_reset.c)
zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH malloc_newlib.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/soc/linkedsemi/common/linker.ld CACHE INTERNAL "")
