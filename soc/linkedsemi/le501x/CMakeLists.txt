zephyr_sources(soc.c)

zephyr_include_directories(.)

zephyr_library_sources_ifdef(CONFIG_PM power.c)
zephyr_library_sources_ifdef(CONFIG_BT_LINKEDSEMI hci_vendor.c)

zephyr_linker_sources(RAM_SECTIONS controller_em.ld)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
