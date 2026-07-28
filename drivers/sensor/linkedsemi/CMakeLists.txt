# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_CAP_LS cap/cap_ls.c)
zephyr_sources_ifdef(CONFIG_CASEOPEN_LS caseopen/caseopen_ls.c)