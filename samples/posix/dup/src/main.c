/*
 * Copyright (c) 2026 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Verify the Zephyr dup() implementation.  The important property under test
 * is that a duplicated descriptor shares the same underlying object, while
 * close() releases that object only when the last descriptor is gone.  An
 * eventfd is used as the object so the sample does not need a mounted
 * filesystem.
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/eventfd.h>

static int failures;

#define CHECK(cond, name)                                                    \
	do {                                                                 \
		if (cond) {                                                    \
			printf("PASS: %s\n", name);                           \
		} else {                                                       \
			printf("FAIL: %s (errno=%d)\n", name, errno);         \
			failures++;                                            \
		}                                                              \
	} while (0)

int main(void)
{
	int fd1;
	int fd2;
	int fd3;
	eventfd_t val;

	printf("Zephyr dup() sample\n");

	fd1 = eventfd(0, EFD_NONBLOCK);
	CHECK(fd1 >= 0, "eventfd create");

	fd2 = dup(fd1);
	CHECK(fd2 >= 0, "dup returns valid fd");
	CHECK(fd2 != fd1, "dup returns distinct fd");

	/* Duplicate must refer to the same eventfd object. */
	val = 42;
	CHECK(write(fd1, &val, sizeof(val)) == sizeof(val),
	      "write via original fd");

	val = 0;
	CHECK(read(fd2, &val, sizeof(val)) == sizeof(val),
	      "read via duplicated fd");
	CHECK(val == 42, "duplicated fd observes same object");

	/* Closing one descriptor must not release the shared object. */
	CHECK(close(fd1) == 0, "close original fd (reverse close)");

	val = 7;
	CHECK(write(fd2, &val, sizeof(val)) == sizeof(val),
	      "write via duplicated fd after closing original");

	val = 0;
	CHECK(read(fd2, &val, sizeof(val)) == sizeof(val),
	      "read via duplicated fd after closing original");
	CHECK(val == 7, "duplicated fd still owns object");

	CHECK(close(fd2) == 0, "close duplicated fd");

	/* Invalid descriptor must fail cleanly. */
	errno = 0;
	CHECK(dup(-1) == -1 && errno == EBADF, "dup(-1) fails with EBADF");

	/* Closing the duplicated fd first must not release the shared object:
	 * the original descriptor has to stay functional. */
	fd1 = eventfd(0, EFD_NONBLOCK);
	CHECK(fd1 >= 0, "eventfd create (reverse close)");
	fd2 = dup(fd1);
	CHECK(fd2 >= 0 && fd2 != fd1, "dup for reverse close");
	CHECK(close(fd2) == 0, "close duplicated fd first");

	val = 11;
	CHECK(write(fd1, &val, sizeof(val)) == sizeof(val),
	      "write original after duplicated fd closed");
	val = 0;
	CHECK(read(fd1, &val, sizeof(val)) == sizeof(val) && val == 11,
	      "original still functional after duplicated fd closed");
	CHECK(close(fd1) == 0, "close original fd");

	/* A dup() must share the object's file status flags (O_NONBLOCK here):
	 * reading an empty non-blocking eventfd through the new fd must fail
	 * with EAGAIN instead of blocking. */
	fd1 = eventfd(0, EFD_NONBLOCK);
	CHECK(fd1 >= 0, "eventfd create (nonblock inherit)");
	fd2 = dup(fd1);
	CHECK(fd2 >= 0, "dup for nonblock inherit");
	CHECK(fcntl(fd2, F_GETFL) & O_NONBLOCK,
	      "duplicated fd reports O_NONBLOCK via fcntl");
	errno = 0;
	CHECK(read(fd2, &val, sizeof(val)) == -1 && errno == EAGAIN,
	      "duplicated fd inherits O_NONBLOCK");
	CHECK(close(fd1) == 0, "close original nonblock fd");
	CHECK(close(fd2) == 0, "close duplicated nonblock fd");

	/* Multiple dup()s of one fd must all share the object; closing any one
	 * of them leaves the others fully functional. */
	fd1 = eventfd(0, EFD_NONBLOCK);
	CHECK(fd1 >= 0, "eventfd create (multi dup)");
	fd2 = dup(fd1);
	fd3 = dup(fd1);
	CHECK(fd2 >= 0 && fd3 >= 0 && fd2 != fd1 && fd3 != fd1 && fd2 != fd3,
	      "multi dup returns distinct fds");
	CHECK(close(fd2) == 0, "close middle duplicated fd");

	val = 23;
	CHECK(write(fd1, &val, sizeof(val)) == sizeof(val),
	      "write fd1 after closing middle dup");
	val = 0;
	CHECK(read(fd3, &val, sizeof(val)) == sizeof(val) && val == 23,
	      "fd3 still shares object after middle dup closed");
	CHECK(close(fd1) == 0, "close remaining fd1");
	CHECK(close(fd3) == 0, "close remaining fd3");

	/* dup() of a descriptor that has already been closed must fail. */
	fd1 = eventfd(0, EFD_NONBLOCK);
	CHECK(fd1 >= 0, "eventfd create (closed fd)");
	CHECK(close(fd1) == 0, "close fd before dup");
	errno = 0;
	CHECK(dup(fd1) == -1 && errno == EBADF,
	      "dup on already closed fd fails with EBADF");

	/* Out-of-range descriptor numbers must fail cleanly too. */
	errno = 0;
	CHECK(dup(INT_MAX) == -1 && errno == EBADF,
	      "dup(INT_MAX) fails with EBADF");

	printf("dup() sample %s (%d failures)\n",
	       failures ? "FAILED" : "PASSED", failures);

	return failures ? 1 : 0;
}
