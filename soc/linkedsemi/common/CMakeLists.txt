add_subdirectory(espi_lpc)
add_subdirectory_ifdef(CONFIG_RISCV e906)
zephyr_include_directories(.)