/*
 * Copyright (c) 2024 LinkedSemi Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#include "emmc_partition.h"
#include <ff.h>

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/devicetree.h>
#include <string.h>

LOG_MODULE_REGISTER(emmc_fatfs_multi_part, CONFIG_LOG_DEFAULT_LEVEL);

/* eMMC partition node in device tree
 * The emmc-partitions node must have label "emmc_parts".
 * Its parent is the disk node (e.g., mmc_disk1).
 */
#define EMMC_PARTS_NODE  DT_NODELABEL(emmc_parts)
#define EMMC_DISK_NODE   DT_PARENT(EMMC_PARTS_NODE)
#define MBR_AREA_SIZE    DT_PROP(EMMC_PARTS_NODE, mbr_area_size)
#define USER_PART_START  MBR_AREA_SIZE
#define USER_PART_SIZE   DT_PROP(EMMC_PARTS_NODE, user_partition_size)
#define COREDUMP_SECTORS  DT_PROP(EMMC_PARTS_NODE, coredump_partition_size)

/*===========================================================
 * CONFIG_FS_FATFS_MULTI_PARTITION - FATFS multi-partition
 *===========================================================*/
#if defined(CONFIG_FS_FATFS_MULTI_PARTITION)

#include <ff.h>

/* Default VolToPart: each pdrv points to itself by default
 * pdrv 4 → drive 4, partition 1 (eMMC user partition)
 * User can override this array to add custom disk mappings
 */
__attribute__((weak))
PARTITION VolToPart[FF_VOLUMES] = {
	{0, 0}, {1, 0}, {2, 0}, {3, 0},
	{4, 1}, {5, 0}, {6, 0}, {7, 0}
};

/* Create partition table (MBR) using DT values */
int emmc_create_partition(void)
{
	uint8_t buf[512];
	uint8_t *pte;
	uint32_t part1_start, part1_sectors, part2_start;

	part1_start = USER_PART_START;
	part1_sectors = USER_PART_SIZE;
	part2_start = part1_start + part1_sectors;

	LOG_INF("Partition from DT: part1 start=%u size=%u, part2 start=%u size=%u",
		part1_start, part1_sectors, part2_start, COREDUMP_SECTORS);

	memset(buf, 0, 512);

	/* Partition 1 */
	pte = buf + 0x1BE;
	pte[0] = 0x80; pte[4] = 0x0C;
	pte[8] = (uint8_t)(part1_start & 0xFF);
	pte[9] = (uint8_t)((part1_start >> 8) & 0xFF);
	pte[10] = (uint8_t)((part1_start >> 16) & 0xFF);
	pte[11] = (uint8_t)((part1_start >> 24) & 0xFF);
	pte[12] = (uint8_t)(part1_sectors & 0xFF);
	pte[13] = (uint8_t)((part1_sectors >> 8) & 0xFF);
	pte[14] = (uint8_t)((part1_sectors >> 16) & 0xFF);
	pte[15] = (uint8_t)((part1_sectors >> 24) & 0xFF);

	/* Partition 2 */
	pte = buf + 0x1CE;
	pte[4] = 0x0C;
	pte[8] = (uint8_t)(part2_start & 0xFF);
	pte[9] = (uint8_t)((part2_start >> 8) & 0xFF);
	pte[10] = (uint8_t)((part2_start >> 16) & 0xFF);
	pte[11] = (uint8_t)((part2_start >> 24) & 0xFF);
	pte[12] = (uint8_t)(COREDUMP_SECTORS & 0xFF);
	pte[13] = (uint8_t)((COREDUMP_SECTORS >> 8) & 0xFF);
	pte[14] = (uint8_t)((COREDUMP_SECTORS >> 16) & 0xFF);
	pte[15] = (uint8_t)((COREDUMP_SECTORS >> 24) & 0xFF);

	buf[510] = 0x55; buf[511] = 0xAA;

	return disk_access_write(EMMC_DISK_NAME, buf, 0, 1) != 0 ? -EIO : 0;
}

/* Format FAT32 partition */
int emmc_format_partition(uint32_t start_sector, uint32_t sector_count)
{
	uint8_t buf[512];
	uint32_t spc = (sector_count < 16 * 1024 * 1024) ? 8 : 16;
	uint32_t reserved = 32, num_fats = 2;
	uint32_t fat_size = (sector_count * 4 / 512) / spc;
	fat_size = ((fat_size < 128) ? 128 : fat_size + 31) & ~31;
	uint32_t first_fat = start_sector + reserved;
	uint32_t root_dir = start_sector + reserved + num_fats * fat_size;

	LOG_INF("FAT32: start=%u total=%u secPerClus=%u secPerFat=%u",
		start_sector, sector_count, spc, fat_size);

	/* Clear DBR sectors */
	memset(buf, 0, 512);
	for (int i = 0; i < 3; i++) {
		if (disk_access_write(EMMC_DISK_NAME, buf, start_sector + i, 1) != 0) {
			return -EIO;
		}
	}

	/* DBR */
	memset(buf, 0, 512);
	buf[0] = 0xEB; buf[1] = 0x58; buf[2] = 0x90;
	memcpy(&buf[3], "MSDOS5.0", 8);
	buf[11] = 0x00; buf[12] = 0x02;
	buf[13] = (uint8_t)spc;
	buf[14] = reserved & 0xFF; buf[15] = (reserved >> 8) & 0xFF;
	buf[16] = (uint8_t)num_fats;
	buf[21] = 0xF8;
	buf[36] = (uint8_t)(fat_size & 0xFF);
	buf[37] = (uint8_t)((fat_size >> 8) & 0xFF);
	buf[38] = (uint8_t)((fat_size >> 16) & 0xFF);
	buf[39] = (uint8_t)((fat_size >> 24) & 0xFF);
	buf[44] = 0x02; buf[48] = 0x01; buf[50] = 0x06;
	buf[64] = 0x80; buf[66] = 0x29;
	memcpy(&buf[71], "NO NAME    ", 11);
	memcpy(&buf[82], "FAT32   ", 8);
	buf[510] = 0x55; buf[511] = 0xAA;

	if (disk_access_write(EMMC_DISK_NAME, buf, start_sector, 1) != 0) return -EIO;
	if (disk_access_write(EMMC_DISK_NAME, buf, start_sector + 6, 1) != 0) return -EIO;

	/* FAT1/FAT2 */
	memset(buf, 0, 512);
	buf[0] = 0xF8; buf[3] = 0xFF; buf[7] = 0x0F;
	if (disk_access_write(EMMC_DISK_NAME, buf, first_fat, 1) != 0) return -EIO;
	if (disk_access_write(EMMC_DISK_NAME, buf, first_fat + fat_size, 1) != 0) return -EIO;

	/* FSInfo */
	memset(buf, 0, 512);
	buf[0] = 0x52; buf[1] = 0x52; buf[2] = 0x41; buf[3] = 0x41;
	buf[4] = 0x72; buf[5] = 0x72; buf[6] = 0x41; buf[7] = 0x41;
	buf[488] = 0x55; buf[489] = 0xAA;
	if (disk_access_write(EMMC_DISK_NAME, buf, start_sector + 1, 1) != 0) return -EIO;

	/* Clear root dir */
	memset(buf, 0, 512);
	for (uint32_t i = 0; i < spc; i++) {
		if (disk_access_write(EMMC_DISK_NAME, buf, root_dir + i, 1) != 0) return -EIO;
	}

	return 0;
}

#else
int emmc_create_partition(void) { return -ENOTSUP; }
int emmc_format_partition(uint32_t s, uint32_t c) { return -ENOTSUP; }
#endif /* CONFIG_FS_FATFS_MULTI_PARTITION */


/*===========================================================
 * CONFIG_DEBUG_COREDUMP_BACKEND_EMMC - eMMC coredump backend
 *===========================================================*/
#if defined(CONFIG_DEBUG_COREDUMP_BACKEND_EMMC)

/* Get partition info from device tree - used by coredump backend */
int emmc_get_coredump_info(uint32_t *start_sector, uint32_t *sector_count)
{
	if (!DT_NODE_EXISTS(EMMC_PARTS_NODE)) {
		LOG_ERR("emmc-partitions node not found in device tree");
		return -ENODEV;
	}

	*start_sector = USER_PART_START + USER_PART_SIZE;
	*sector_count = COREDUMP_SECTORS;

	LOG_INF("eMMC partition info from DT: coredump_start=%u, coredump_size=%u sectors",
		*start_sector, *sector_count);

	return 0;	
}
extern int coredump_emmc_backend_init(void);
/* Initialize eMMC coredump backend */
int init_coredump_emmc_backend(void)
{
	int ret;

	LOG_INF("Initializing eMMC coredump backend...");

	/* Verify device tree disk-name matches Kconfig volume alias */
	if (strcmp(DT_PROP(EMMC_DISK_NODE, disk_name), EMMC_DISK_NAME) != 0) {
		LOG_ERR("disk-name mismatch: DT='%s' vs Kconfig='%s'",
			DT_PROP(EMMC_DISK_NODE, disk_name), EMMC_DISK_NAME);
		return -EINVAL;
	}

	ret = disk_access_ioctl(EMMC_DISK_NAME, DISK_IOCTL_CTRL_INIT, NULL);
	if (ret != 0) {
		LOG_ERR("DISK_IOCTL_CTRL_INIT failed: %d", ret);
		return ret;
	}
	LOG_INF("eMMC disk initialized successfully");

#if defined(CONFIG_FS_FATFS_MULTI_PARTITION)
	/* Verify VolToPart[4] maps to drive 4, partition 1
	 * FF_VOLUME_STRS[4] = CONFIG_ZEPHYR_FATFS_SD2_ALIAS_UNIX
	 * So pdrv 4 must point to physical drive 4, partition 1
	 */
	if (VolToPart[4].pd != 4 || VolToPart[4].pt != 1) {
		LOG_ERR("VolToPart[4] must be {4, 1}, got {%d, %d}",
			VolToPart[4].pd, VolToPart[4].pt);
		return -EINVAL;
	}

	/* Create partition table using device tree info */
	LOG_INF("Creating partition table...");
	ret = emmc_create_partition();
	if (ret != 0) {
		LOG_ERR("emmc_create_partition failed: %d", ret);
		return ret;
	}

	LOG_INF("Partition table created successfully");
#endif


	coredump_emmc_backend_init();

	LOG_INF("eMMC coredump backend initialized");
	return 0;
}

#else
int emmc_get_coredump_info(uint32_t *s, uint32_t *c) { return -ENOTSUP; }
int init_emmc_backend(void) { return -ENOTSUP; }
#endif /* CONFIG_DEBUG_COREDUMP_BACKEND_EMMC */
