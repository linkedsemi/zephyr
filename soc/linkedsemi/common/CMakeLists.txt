add_subdirectory(espi_lpc)
add_subdirectory_ifdef(CONFIG_RISCV e906)
zephyr_include_directories(.)
zephyr_sources(soc_common.c)
zephyr_linker_sources(
    RAMFUNC_SECTION
    xip_banned.ld
)