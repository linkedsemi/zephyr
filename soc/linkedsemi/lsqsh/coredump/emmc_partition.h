/*
 * Copyright (c) 2024 LinkedSemi Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file emmc_partition.h
 * @brief eMMC FATFS multi-partition interface
 *
 * This module provides eMMC multi-partition support for:
 * - FAT32 user data partition (Partition 1)
 * - Coredump storage partition (Partition 2)
 *
 * Partition Layout:
 *   +------------------+------------------------+------------------------+------------------------+
 *   |     Sector 0     |     Sector 1-2047      |    Sector 2048+        |   Sector N+            |
 *   |       MBR        |     Reserved (1MB)     |   Partition 1 (FAT32)  |   Partition 2 (Coredump)|
 *   +------------------+------------------------+------------------------+------------------------+
 *
 * Configuration:
 *   Partition info is read from device tree:
 *     &mmc_disk1 {
 *         emmc-partitions {
 *             mbr-area-size = <...>;           // e.g. 0x100000 (1MB)
 *             user-partition-size = <...>;     // in sectors
 *             coredump-partition-size = <...>; // in sectors
 *         };
 *     };
 *
 * Note:
 *   - disk-name in device tree MUST match the volume alias in FF_VOLUME_STRS
 *     (e.g., CONFIG_ZEPHYR_FATFS_SD2_ALIAS_UNIX = "SD2")
 *   - VolToPart array is always required for FATFS multi-partition support.
 *     Default mapping: pdrv 4 -> physical drive 4, partition 1.
 *     If mounting other disks, user MUST redefine VolToPart externally
 *     (weak symbol) to override the default.
 *
 * Usage:
 *   1. Call init_emmc_backend() to initialize eMMC and create MBR partition table
 *   2. (Optional) Format FAT32 partition using emmc_format_partition()
 *   3. Mount FATFS using EMMC_FATFS_MOUNT_POINT as mount point
 *   4. Access coredump region via emmc_get_coredump_info()
 */

#ifndef EMC_PARTITION_H
#define EMC_PARTITION_H

#include <stdint.h>

/**
 * @name Configuration
 * @{
 */

/**
 * eMMC disk device name.
 * When CONFIG_ZEPHYR_FATFS_UNIX_MNTPOINT=y: from Kconfig volume alias
 * Otherwise: fixed "SD2"
 */
#ifdef CONFIG_ZEPHYR_FATFS_UNIX_MNTPOINT
#define EMMC_DISK_NAME  CONFIG_ZEPHYR_FATFS_SD2_ALIAS_UNIX
#else
#define EMMC_DISK_NAME  "SD2"
#endif

/**
 * FATFS mount point for Partition 1 (FAT32 user data).
 *
 * With CONFIG_ZEPHYR_FATFS_UNIX_MNTPOINT=y:
 *   Mount point is /EMMC_DISK_NAME (Unix style)
 * Without:
 *   Mount point is /EMMC_DISK_NAME: (FATFS legacy style)
 */
#ifdef CONFIG_ZEPHYR_FATFS_UNIX_MNTPOINT
#define EMMC_FATFS_MOUNT_POINT  "/" EMMC_DISK_NAME
#else
#define EMMC_FATFS_MOUNT_POINT  "/" EMMC_DISK_NAME ":"
#endif
/** @} */

/**
 * @name Functions
 * @{
 */

/**
 * @brief Initialize eMMC backend and create partition table.
 *
 * This function performs:
 *   1. Initialize eMMC card via disk access layer (DISK_IOCTL_CTRL_INIT)
 *   2. Create MBR partition table using device tree configuration
 *
 * @note This function should be called once at system startup,
 *       before any FATFS or coredump operations.
 *
 * @return 0 on success, negative errno on error
 * @retval -ENODEV if eMMC device not found
 * @retval -EIO if disk init or partition write fails
 */
int init_emmc_backend(void);

/**
 * @brief Get coredump partition information.
 *
 * Returns the starting sector and size of the coredump partition
 * as defined in device tree.
 *
 * @note Partition 2 start = USER_PART_START + USER_PART_SIZE
 *
 * @param[out] start_sector Starting sector number of coredump partition
 * @param[out] sector_count Number of sectors in coredump partition
 *
 * @return 0 on success
 * @retval -ENODEV if emmc-partitions node not found in device tree
 */
int emmc_get_coredump_info(uint32_t *start_sector, uint32_t *sector_count);

/**
 * @brief Create MBR partition table.
 *
 * Writes a Master Boot Record (MBR) to sector 0 of the eMMC,
 * creating two partitions:
 *
 *   Partition 1 (FAT32):
 *     - Start: USER_PART_START (from device tree, typically 2048)
 *     - Size: USER_PART_SIZE sectors
 *     - Type: 0x0C (FAT32 LBA)
 *
 *   Partition 2 (Coredump):
 *     - Start: USER_PART_START + USER_PART_SIZE
 *     - Size: COREDUMP_SECTORS (from device tree)
 *     - Type: 0x0C (FAT32 LBA)
 *
 * @warning This function writes to sector 0 of the eMMC.
 *          Ensure no valid data exists in the first sector.
 *
 * @note After calling this function, the disk should be
 *       re-initialized to recognize the new partition layout.
 *
 * @return 0 on success
 * @retval -EIO if disk write fails
 */
int emmc_create_partition(void);

/**
 * @brief Format FAT32 partition.
 *
 * Creates a FAT32 filesystem on the specified partition area.
 * This includes:
 *   - DBR (DOS Boot Record) sector
 *   - FSInfo sector
 *   - FAT1 and FAT2 tables
 *   - Root directory area
 *
 * @note This function only writes filesystem structures,
 *       it does not create partition boundaries (use emmc_create_partition).
 *
 * @param start_sector Starting sector of the partition to format
 * @param sector_count Number of sectors in the partition
 *
 * @return 0 on success
 * @retval -EIO if any write operation fails
 */
int emmc_format_partition(uint32_t start_sector, uint32_t sector_count);

/** @} */

#endif /* EMC_PARTITION_H */