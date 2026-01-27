# MP5023 PMBus Sensor Driver

zephyr_library()
zephyr_library_sources(mp5023.c)
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
zephyr_library_link_libraries(drivers__sensor__ls_pmbus)
