/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_POSIX_SIGNAL_COMPAT_H_
#define ZEPHYR_INCLUDE_POSIX_SIGNAL_COMPAT_H_

/* Supplemental signal definitions for OpenBMC compatibility */

#ifndef SA_THIRTYTWO
#define SA_THIRTYTWO 0x02000000
#endif

#ifndef SA_RESTORER
#define SA_RESTORER 0x04000000
#endif

#ifndef SA_ONSTACK
#define SA_ONSTACK 0x08000000
#endif

#ifndef SA_RESTART
#define SA_RESTART 0x10000000
#endif

#ifndef SA_NOMASK
#define SA_NOMASK SA_NODEFER
#endif

#ifndef SA_ONESHOT
#define SA_ONESHOT SA_RESETHAND
#endif

#endif /* ZEPHYR_INCLUDE_POSIX_SIGNAL_COMPAT_H_ */
