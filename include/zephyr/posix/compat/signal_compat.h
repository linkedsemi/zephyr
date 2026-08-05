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

/* Bits in `sa_flags'.  */
#define SA_NOCLDSTOP  1		 /* Don't send SIGCHLD when children stop.  */
#define SA_NOCLDWAIT  2		 /* Don't create zombie on child death.  */
#define SA_SIGINFO    4		 /* Invoke signal-catching function with
				    three arguments instead of one.  */

#endif /* ZEPHYR_INCLUDE_POSIX_SIGNAL_COMPAT_H_ */
