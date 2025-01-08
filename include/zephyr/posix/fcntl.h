/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sys/types.h>
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
#define O_NOCTTY   0x0100

#define F_DUPFD 0
#define F_GETFL 3
#define F_SETFL 4

/* open/fcntl - O_SYNC is only implemented on blocks devices and on files
   located on an ext2 file system */

#ifndef O_CREAT
#define O_CREAT		00000100	/* not fcntl */
#endif
#ifndef O_EXCL
#define O_EXCL		00000200	/* not fcntl */
#endif
#ifndef O_NOCTTY
#define O_NOCTTY	00000400	/* not fcntl */
#endif
#ifndef O_TRUNC
#define O_TRUNC		00001000	/* not fcntl */
#endif
#ifndef O_APPEND
#define O_APPEND	00002000
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK	00004000
#endif
#ifndef O_SYNC
#define O_SYNC		00010000
#endif
#ifndef FASYNC
#define FASYNC		00020000	/* fcntl, for BSD compatibility */
#endif
#ifndef O_DIRECT
#define O_DIRECT	00040000	/* direct disk access hint */
#endif
#ifndef O_LARGEFILE
#define O_LARGEFILE	00100000
#endif
#ifndef O_DIRECTORY
#define O_DIRECTORY	00200000	/* must be a directory */
#endif
#ifndef O_NOFOLLOW
#define O_NOFOLLOW	00400000	/* don't follow links */
#endif
#ifndef O_NOATIME
#define O_NOATIME	01000000
#endif
#ifndef O_CLOEXEC
#define O_CLOEXEC	02000000	/* set close_on_exec */
#endif
#ifndef O_NDELAY
#define O_NDELAY	O_NONBLOCK
#endif

#define F_DUPFD		0	/* dup */
#define F_GETFD		1	/* get close_on_exec */
#define F_SETFD		2	/* set/clear close_on_exec */
#define F_GETFL		3	/* get file->f_flags */
#define F_SETFL		4	/* set file->f_flags */
#ifndef F_GETLK
#define F_GETLK		5
#define F_SETLK		6
#define F_SETLKW	7
#endif
#ifndef F_SETOWN
#define F_SETOWN	8	/* for sockets. */
#define F_GETOWN	9	/* for sockets. */
#endif
#ifndef F_SETSIG
#define F_SETSIG	10	/* for sockets. */
#define F_GETSIG	11	/* for sockets. */
#endif

#ifndef CONFIG_64BIT
#ifndef F_GETLK64
#define F_GETLK64	12	/*  using 'struct flock64' */
#define F_SETLK64	13
#define F_SETLKW64	14
#endif
#endif

#ifndef F_SETOWN_EX
#define F_SETOWN_EX	15
#define F_GETOWN_EX	16
#endif

#define F_OWNER_TID	0
#define F_OWNER_PID	1
#define F_OWNER_PGRP	2

/* for F_[GET|SET]FL */
#define FD_CLOEXEC	1	/* actually anything with low bit set goes */

/* for posix fcntl() and lockf() */
#ifndef F_RDLCK
#define F_RDLCK		0
#define F_WRLCK		1
#define F_UNLCK		2
#endif

#define AT_FDCWD	-100
# define F_DUPFD_CLOEXEC 1030
struct flock
  {
    short int l_type;   /* Type of lock: F_RDLCK, F_WRLCK, or F_UNLCK.  */
    short int l_whence; /* Where `l_start' is relative to (like `lseek').  */
#if __WORDSIZE == 64 || !defined __USE_FILE_OFFSET64
    __off_t l_start;    /* Offset where the lock begins.  */
    __off_t l_len;      /* Size of the locked area; zero means until EOF.  */
#else
    __off64_t l_start;  /* Offset where the lock begins.  */
    __off64_t l_len;    /* Size of the locked area; zero means until EOF.  */
#endif
    __pid_t l_pid;      /* Process holding the lock.  */
  };

#ifdef __cplusplus
extern "C" {
#endif

int open(const char *name, int flags, ...);
int fcntl(int fildes, int cmd, ...);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_POSIX_FCNTL_H_ */
