/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_DIRENT_H_
#define ZEPHYR_INCLUDE_POSIX_DIRENT_H_

#include <limits.h>

#include <zephyr/posix/posix_types.h>

#ifdef CONFIG_POSIX_FILE_SYSTEM
#include <zephyr/fs/fs.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void DIR;

/* From linux/fs_types.h
   these are defined by POSIX and also present in glibc's dirent.h */
#define DT_UNKNOWN	0
#define DT_FIFO		1
#define DT_CHR		2
#define DT_DIR		4
#define DT_BLK		6
#define DT_REG		8
#define DT_LNK		10
#define DT_SOCK		12
#define DT_WHT		14

struct dirent {
	unsigned int d_ino;
	unsigned char d_type;
	char d_name[PATH_MAX + 1];
};

/* Directory related operations */
DIR *opendir(const char *dirname);
int closedir(DIR *dirp);
struct dirent *readdir(DIR *dirp);
int readdir_r(DIR *ZRESTRICT dirp, struct dirent *ZRESTRICT entry,
	      struct dirent **ZRESTRICT result);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_POSIX_FILE_SYSTEM */

#endif	/* ZEPHYR_INCLUDE_POSIX_DIRENT_H_ */
