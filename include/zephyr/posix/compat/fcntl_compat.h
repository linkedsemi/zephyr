/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_POSIX_FCNTL_COMPAT_H_
#define ZEPHYR_INCLUDE_POSIX_FCNTL_COMPAT_H_

#ifndef O_CREAT
#define O_CREAT 00000100
#endif

#ifndef O_EXCL
#define O_EXCL 00000200
#endif

#ifndef O_TRUNC
#define O_TRUNC 00001000
#endif

#ifndef O_APPEND
#define O_APPEND 00002000
#endif

#ifndef O_NONBLOCK
#define O_NONBLOCK 00004000
#endif

#ifndef O_SYNC
#define O_SYNC 00010000
#endif

#ifndef FASYNC
#define FASYNC 00020000
#endif

#ifndef O_DIRECT
#define O_DIRECT 00040000
#endif

#ifndef O_LARGEFILE
#define O_LARGEFILE 00100000
#endif

#ifndef O_DIRECTORY
#define O_DIRECTORY 00200000
#endif

#ifndef O_NOFOLLOW
#define O_NOFOLLOW 00400000
#endif

#ifndef O_NOATIME
#define O_NOATIME 01000000
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 02000000
#endif

#ifndef O_NDELAY
#define O_NDELAY O_NONBLOCK
#endif

#ifndef F_DUPFD
#define F_DUPFD 0
#endif

#ifndef F_GETFD
#define F_GETFD 1
#endif

#ifndef F_SETFD
#define F_SETFD 2
#endif

#ifndef F_GETFL
#define F_GETFL 3
#endif

#ifndef F_SETFL
#define F_SETFL 4
#endif

#ifndef F_GETLK
#define F_GETLK 5
#endif

#ifndef F_SETLK
#define F_SETLK 6
#endif

#ifndef F_SETLKW
#define F_SETLKW 7
#endif

#ifndef F_SETOWN
#define F_SETOWN 8
#endif

#ifndef F_GETOWN
#define F_GETOWN 9
#endif

#ifndef F_SETSIG
#define F_SETSIG 10
#endif

#ifndef F_GETSIG
#define F_GETSIG 11
#endif

#ifndef F_SETOWN_EX
#define F_SETOWN_EX 15
#endif

#ifndef F_GETOWN_EX
#define F_GETOWN_EX 16
#endif

#ifndef F_OWNER_TID
#define F_OWNER_TID 0
#endif

#ifndef F_OWNER_PID
#define F_OWNER_PID 1
#endif

#ifndef F_OWNER_PGRP
#define F_OWNER_PGRP 2
#endif

#ifndef FD_CLOEXEC
#define FD_CLOEXEC 1
#endif

#ifndef F_RDLCK
#define F_RDLCK 0
#endif

#ifndef F_WRLCK
#define F_WRLCK 1
#endif

#ifndef F_UNLCK
#define F_UNLCK 2
#endif

#ifndef AT_FDCWD
#define AT_FDCWD -100
#endif

#ifndef F_DUPFD_CLOEXEC
#define F_DUPFD_CLOEXEC 1030
#endif

#ifndef O_NOCTTY
#define O_NOCTTY 00000400 /* not fcntl */
#endif

#ifndef CONFIG_64BIT
#ifndef F_GETLK64
#define F_GETLK64 12 /*  using 'struct flock64' */
#define F_SETLK64 13
#define F_SETLKW64 14
#endif
#endif

#endif /* ZEPHYR_INCLUDE_POSIX_FCNTL_COMPAT_H_ */
