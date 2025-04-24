zephyr_sources(soc.c)
zephyr_include_directories(.)

zephyr_sources(../e906_common/soc_irq.S)
zephyr_sources(../e906_common/irq_driver.c)
zephyr_sources(../e906_common/soc_isr_stacking_e906.c)
zephyr_sources(psram.c)
zephyr_library_sources_ifdef(CONFIG_COMMON_LIBC_MALLOC_LSQSH malloc.c)
zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC_MALLOC_LSQSH malloc_newlib.c)
zephyr_include_directories(../e906_common)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
