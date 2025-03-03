/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_POSIX_UNISTD_H_
#define ZEPHYR_INCLUDE_POSIX_UNISTD_H_

#include <zephyr/posix/posix_types.h>
#include <zephyr/net/net_ip.h>

#ifdef CONFIG_POSIX_API
#include <zephyr/fs/fs.h>
#endif
#ifdef CONFIG_NETWORKING
/* For zsock_gethostname() */
#include <zephyr/net/socket.h>
#include <zephyr/net/hostname.h>
#endif
#include <zephyr/posix/sys/confstr.h>
#include <zephyr/posix/sys/stat.h>
#include <zephyr/posix/sys/sysconf.h>

#include "posix_features.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_POSIX_API
/* File related operations */
int close(int file);
ssize_t write(int file, const void *buffer, size_t count);
ssize_t read(int file, void *buffer, size_t count);
off_t lseek(int file, off_t offset, int whence);
int fsync(int fd);
int ftruncate(int fd, off_t length);

#ifdef CONFIG_POSIX_SYNCHRONIZED_IO
int fdatasync(int fd);
#endif /* CONFIG_POSIX_SYNCHRONIZED_IO */

/* File System related operations */
int rename(const char *old, const char *newp);
int unlink(const char *path);
int stat(const char *path, struct stat *buf);
int mkdir(const char *path, mode_t mode);
int rmdir(const char *path);

FUNC_NORETURN void _exit(int status);

#ifdef CONFIG_NETWORKING
static inline int gethostname(char *buf, size_t len)
{
	return zsock_gethostname(buf, len);
}
#endif /* CONFIG_NETWORKING */

#endif /* CONFIG_POSIX_API */

#ifdef CONFIG_POSIX_C_LIB_EXT
int getopt(int argc, char *const argv[], const char *optstring);
extern char *optarg;
extern int opterr, optind, optopt;
#endif

int getentropy(void *buffer, size_t length);
pid_t getpid(void);
unsigned sleep(unsigned int seconds);
int usleep(useconds_t useconds);
#if _POSIX_C_SOURCE >= 2
size_t confstr(int name, char *buf, size_t len);
#endif

#ifdef CONFIG_POSIX_SYSCONF_IMPL_MACRO
#define sysconf(x) (long)CONCAT(__z_posix_sysconf, x)
#else
long sysconf(int opt);
#endif /* CONFIG_POSIX_SYSCONF_IMPL_FULL */

typedef unsigned long rlim_t;

struct rlimit {
	rlim_t	rlim_cur;
	rlim_t	rlim_max;
};
#define RLIMIT_STACK	3		/* max stack size */

#define RLIM_INFINITY	(~0UL)
int getrlimit (int __resource, struct rlimit *__rlp);
int setrlimit (int __resource, const struct rlimit *__rlp);

int     pipe (int __fildes[2]);
ssize_t readv(int fd, const struct iovec *iov, int iovcnt);
ssize_t writev(int fd, const struct iovec *iov, int iovcnt);
int	mkstemp (char *);
pid_t   fork (void);
int     dup (int __fildes);
int     dup2 (int __fildes, int __fildes2);
int	access (const char *__path, int __amode);
int     execve (const char *__path, char * const __argv[], char * const __envp[]);
int	 dirfd(void *);
time_t timegm(struct tm *tm);

#define STDIN_FILENO    0       /* standard input file descriptor */
#define STDOUT_FILENO   1       /* standard output file descriptor */
#define STDERR_FILENO   2       /* standard error file descriptor */

#define	F_OK	0
#define	R_OK	4
#define	W_OK	2
#define	X_OK	1

#define WIFCONTINUED(status) ((status)==0xffff)
#ifdef __cplusplus
}
#endif

#endif	/* ZEPHYR_INCLUDE_POSIX_UNISTD_H_ */
