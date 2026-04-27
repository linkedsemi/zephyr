/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_POSIX_POLL_COMPAT_H_
#define ZEPHYR_INCLUDE_POSIX_POLL_COMPAT_H_

#include <zephyr/net/socket.h>

#ifndef POLLPRI
#define POLLPRI ZSOCK_POLLPRI
#endif

#ifndef POLLRDNORM
#define POLLRDNORM 0x0040 /* non-OOB/URG data available */
#endif

#ifndef POLLWRNORM
#define POLLWRNORM POLLOUT /* no write type differentiation */
#endif

#ifndef POLLRDBAND
#define POLLRDBAND 0x0080 /* OOB/Urgent readable data */
#endif

#ifndef POLLWRBAND
#define POLLWRBAND 0x0100 /* OOB/Urgent data can be written */
#endif

typedef unsigned int nfds_t;

#endif /* ZEPHYR_INCLUDE_POSIX_POLL_COMPAT_H_ */
