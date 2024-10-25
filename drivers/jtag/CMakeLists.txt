# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_JTAG_LS         jtag_ls.c)
zephyr_library_sources_ifdef(CONFIG_JTAG_SHELL		jtag_shell.c)