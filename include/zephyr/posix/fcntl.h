/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_POSIX_FCNTL_H_
#define ZEPHYR_POSIX_FCNTL_H_

#ifdef CONFIG_PICOLIBC
#define O_CREAT	 0x0040
#define O_TRUNC	 0x0200
#define O_APPEND 0x0400
#else
#define O_CREAT	 0x0200
#define O_TRUNC	 0x0400
#define O_APPEND 0x0008
#endif

#define O_ACCMODE (O_RDONLY | O_WRONLY | O_RDWR)

#define O_RDONLY 00
#define O_WRONLY 01
#define O_RDWR	 02

#define O_EXCL	   0x0800
#define O_NONBLOCK 0x4000

#define F_DUPFD 0
#define F_GETFL 3
#define F_SETFL 4

#ifdef __cplusplus
extern "C" {
#endif

int open(const char *name, int flags, ...);
int fcntl(int fildes, int cmd, ...);

/*
 * Close every posix file fd whose devfs char-device object equals
 * @p target_filep. Used by char-device drivers (e.g. i2c-dev) to reclaim fds
 * left open after a command aborted via longjmp/exit() and skipped its own
 * close(). Only a pointer comparison is performed, so it is safe for any fd
 * state. Returns the number of fds closed.
 */
int zvfs_close_fds_with_filep(const void *target_filep);

#ifdef __cplusplus
}
#endif

#include "zephyr/posix/compat/fcntl_compat.h"

#endif /* ZEPHYR_POSIX_FCNTL_H_ */
