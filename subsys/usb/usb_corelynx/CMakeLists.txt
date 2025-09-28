# Copyright (c) 2024, Alibaba
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

zephyr_library_sources_ifdef(
	CONFIG_USB_DEVICE_STACK_NEXT
	corelynx_usbd_init.c
)
