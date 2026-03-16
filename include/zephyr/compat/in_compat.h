/*
 * SPDX-License-Identifier: Apache-2.0
 */

 #ifndef ZEPHYR_INCLUDE_COMPAT_IN_COMPAT_H_
 #define ZEPHYR_INCLUDE_COMPAT_IN_COMPAT_H_

 #ifndef IPV6_PKTINFO
 #define IPV6_PKTINFO		50
 #endif

 #ifndef IPV6_RECVPKTINFO
 #define IPV6_RECVPKTINFO	49
 #endif

 #ifndef IPV6_RECVHOPLIMIT
 #define IPV6_RECVHOPLIMIT	51
 #endif

 #ifndef IPV6_HOPLIMIT
 #define IPV6_HOPLIMIT		52
 #endif

 #ifndef IPV6_RECVTCLASS
 #define IPV6_RECVTCLASS		66
 #endif

 #ifndef IPV6_TCLASS
 #define IPV6_TCLASS		67
 #endif

 #ifndef SOL_IPV6
 #define SOL_IPV6		41
 #endif

 #ifndef SOL_IP
 #define SOL_IP			0
 #endif

 #ifndef IP_PKTINFO
 #define IP_PKTINFO		8
 #endif

 #ifndef IP_RECVPKTINFO
 #define IP_RECVPKTINFO		IP_PKTINFO
 #endif

 #ifndef IP_PMTUDISC_DONT
 #define IP_PMTUDISC_DONT	0
 #endif

 #ifndef IP_PMTUDISC_WANT
 #define IP_PMTUDISC_WANT	1
 #endif

 #ifndef IP_PMTUDISC_DO
 #define IP_PMTUDISC_DO		2
 #endif

 #ifndef IP_PMTUDISC_PROBE
 #define IP_PMTUDISC_PROBE	3
 #endif

 #ifndef IPV6_PMTUDISC_DONT
 #define IPV6_PMTUDISC_DONT	0
 #endif

 #ifndef IPV6_PMTUDISC_WANT
 #define IPV6_PMTUDISC_WANT	1
 #endif

 #ifndef IPV6_PMTUDISC_DO
 #define IPV6_PMTUDISC_DO	2
 #endif

 #endif /* ZEPHYR_INCLUDE_COMPAT_IN_COMPAT_H_ */
