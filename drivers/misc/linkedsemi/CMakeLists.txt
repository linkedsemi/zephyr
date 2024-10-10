# Copyright (c) 2023 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SPID_LINKEDSEMI spid_linkedsemi.c)
