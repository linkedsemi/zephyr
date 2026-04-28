/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_POSIX_SOCKET_COMPAT_H_
#define ZEPHYR_INCLUDE_POSIX_SOCKET_COMPAT_H_

#include <zephyr/net/socket.h>

/* Socket message flags from POSIX compat */
#ifndef MSG_OOB
#define MSG_OOB 0x01
#endif

#ifndef MSG_PEEK
#define MSG_PEEK 0x02
#endif

#ifndef MSG_DONTROUTE
#define MSG_DONTROUTE 0x04
#endif

#ifndef MSG_CTRUNC
#define MSG_CTRUNC 0x08
#endif

#ifndef MSG_PROXY
#define MSG_PROXY 0x10
#endif

#ifndef MSG_TRUNC
#define MSG_TRUNC 0x20
#endif

#ifndef MSG_DONTWAIT
#define MSG_DONTWAIT 0x40
#endif

#ifndef MSG_EOR
#define MSG_EOR 0x80
#endif

#ifndef MSG_WAITALL
#define MSG_WAITALL 0x100
#endif

#ifndef MSG_FIN
#define MSG_FIN 0x200
#endif

#ifndef MSG_SYN
#define MSG_SYN 0x400
#endif

#ifndef MSG_CONFIRM
#define MSG_CONFIRM 0x800
#endif

#ifndef MSG_RST
#define MSG_RST 0x1000
#endif

#ifndef MSG_ERRQUEUE
#define MSG_ERRQUEUE 0x2000
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0x4000
#endif

#ifndef MSG_MORE
#define MSG_MORE 0x8000
#endif

#ifndef MSG_WAITFORONE
#define MSG_WAITFORONE 0x10000
#endif

#ifndef MSG_BATCH
#define MSG_BATCH 0x40000
#endif

#ifndef MSG_ZEROCOPY
#define MSG_ZEROCOPY 0x4000000
#endif

#ifndef MSG_FASTOPEN
#define MSG_FASTOPEN 0x20000000
#endif

#ifndef MSG_CMSG_CLOEXEC
#define MSG_CMSG_CLOEXEC 0x40000000
#endif

/* IPv4 multicast request structure for Boost.Asio compatibility */
#ifndef _IP_MREQ_DEFINED
#define _IP_MREQ_DEFINED

/**
 * @brief Struct used when setting a IPv4 multicast network interface.
 */
struct ip_mreq {
	struct in_addr imr_multiaddr;
	struct in_addr imr_interface;
};

#endif /* _IP_MREQ_DEFINED */

#endif /* ZEPHYR_INCLUDE_POSIX_SOCKET_COMPAT_H_ */
