# Copyright (c) 2024 LinkedSemi Semiconductor
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/${ARCH}/include
  ${ZEPHYR_BASE}/subsys/debug/coredump
  ${ZEPHYR_BASE}/soc/linkedsemi/lsqsh/coredump
  ${ZEPHYR_BASE}/../modules/fs/fatfs/include
  ${ZEPHYR_BASE}/modules/fatfs
)

zephyr_library_compile_definitions_ifdef(
  CONFIG_FS_FATFS_MULTI_PARTITION
  ZEPHYR_CONFIG_OVERRIDE=zephyr_fatfs_config.h
)

zephyr_library_sources_ifdef(
  CONFIG_DEBUG_COREDUMP_BACKEND_EMMC
  coredump_backend_emmc.c
  emmc_baremetal.c
)

# FATFS multi-partition support (depends on CONFIG_FS_FATFS_MULTI_PARTITION)
zephyr_library_sources_ifdef(
  CONFIG_FS_FATFS_MULTI_PARTITION
  emmc_partition.c
)

# Include directory for header files (emmc_coredump_sdhci.h, emmc_partition.h)
zephyr_include_directories(${ZEPHYR_BASE}/soc/linkedsemi/lsqsh/coredump)