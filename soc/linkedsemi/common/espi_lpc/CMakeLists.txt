zephyr_include_directories(.)
#zephyr_sources_ifdef(CONFIG_ESPI_LPC espi_lpc_common.c)
zephyr_library_sources(espi_lpc_common.c)
