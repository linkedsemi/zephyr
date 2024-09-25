zephyr_sources(soc.c)
zephyr_sources(soc_irq.S)
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
