zephyr_sources(soc.c)
zephyr_include_directories(.)

zephyr_sources(../e906_common/soc_irq.S)
zephyr_sources(../e906_common/irq_driver.c)
zephyr_sources(../e906_common/soc_isr_stacking_e906.c)
zephyr_include_directories(../e906_common)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
