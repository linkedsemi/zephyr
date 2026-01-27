# Copyright (c) 2025 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

add_library(SYSFS INTERFACE)

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SYSFS sysfs.c)
zephyr_library_link_libraries(SYSFS)
