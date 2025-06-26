
zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PECI_CORE    peci-core.c peci-dev.c)

zephyr_library_link_libraries(zephyr-osal)