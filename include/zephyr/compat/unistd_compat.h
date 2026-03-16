/*
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef ZEPHYR_INCLUDE_COMPAT_UNISTD_COMPAT_H_
 #define ZEPHYR_INCLUDE_COMPAT_UNISTD_COMPAT_H_

 #include <zephyr/posix/posix_types.h>

 typedef unsigned long rlim_t;

 struct rlimit {
	 rlim_t	rlim_cur;
	 rlim_t	rlim_max;
 };

 #ifndef RLIMIT_STACK
 #define RLIMIT_STACK	3
 #endif

 #ifndef RLIM_INFINITY
 #define RLIM_INFINITY	(~0UL)
 #endif

 #ifndef STDIN_FILENO
 #define STDIN_FILENO	0
 #endif

 #ifndef STDOUT_FILENO
 #define STDOUT_FILENO	1
 #endif

 #ifndef STDERR_FILENO
 #define STDERR_FILENO	2
 #endif

 #ifndef F_OK
 #define F_OK		0
 #endif

 #ifndef R_OK
 #define R_OK		4
 #endif

 #ifndef W_OK
 #define W_OK		2
 #endif

 #ifndef X_OK
 #define X_OK		1
 #endif

 #ifndef WIFCONTINUED
 #define WIFCONTINUED(status)	((status)==0xffff)
 #endif

 #ifdef __cplusplus
 extern "C" {
 #endif

 int getrlimit(int __resource, struct rlimit *__rlp);
 int setrlimit(int __resource, const struct rlimit *__rlp);
 int pipe(int __fildes[2]);
 ssize_t readv(int fd, const struct iovec *iov, int iovcnt);
 ssize_t writev(int fd, const struct iovec *iov, int iovcnt);
 int mkstemp(char *);
 pid_t fork(void);
 int dup(int __fildes);
 int dup2(int __fildes, int __fildes2);
 /* Note: access() is defined in basu_zephyr_compat.h as basu_access_fallback */
 int execve(const char *__path, char * const __argv[], char * const __envp[]);
 int dirfd(void *);
 time_t timegm(struct tm *tm);

 #ifdef __cplusplus
 }
 #endif

 #endif /* ZEPHYR_INCLUDE_COMPAT_UNISTD_COMPAT_H_ */
