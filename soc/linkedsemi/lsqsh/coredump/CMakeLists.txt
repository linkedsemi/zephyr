# Copyright (c) 2024 LinkedSemi Semiconductor
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  ${ZEPHYR_BASE}/subsys/debug/coredump
  ${ZEPHYR_BASE}/soc/linkedsemi/lsqsh/coredump
)

zephyr_library_sources_ifdef(
  CONFIG_DEBUG_COREDUMP_BACKEND_EMMC
  coredump_backend_emmc.c
  emmc_baremetal.c
)

# Include directory for header files (emmc_coredump_sdhci.h)
zephyr_include_directories(${ZEPHYR_BASE}/soc/linkedsemi/lsqsh/coredump)