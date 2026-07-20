zephyr_include_directories(.)
zephyr_sources(soc_isr_stacking_e906.c)
zephyr_sources(irq_driver.c)
zephyr_sources_ifdef(CONFIG_SMP atomic_cas_smp.c)
zephyr_sources_ifdef(CONFIG_THREAD_ERROR_ISOLATION fatal.c)

zephyr_sources_ifdef(CONFIG_SMP secondary_cpu_reset.S)
