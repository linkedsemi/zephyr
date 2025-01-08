/*
 * Copyright (c) 1984-1999, 2012 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 1982, 1986 Regents of the University of California.
 * All rights reserved.  The Berkeley software License Agreement
 * specifies the terms and conditions for redistribution.
 *
 *	@(#)errno.h	7.1 (Berkeley) 6/4/86
 */

/**
 * @file
 * @brief System error numbers
 */

#ifndef ZEPHYR_NEWLIB_INCLUDE_ERRNO_H_
#define ZEPHYR_NEWLIB_INCLUDE_ERRNO_H_

/**
 * @brief System error numbers
 *        Error codes returned by functions.
 *        Includes a list of those defined by IEEE Std 1003.1-2017.
 * @defgroup system_errno Error numbers
 * @{
 */

#include <zephyr/sys/errno_private.h>

#ifdef __cplusplus
extern "C" {
#endif

#define errno (*z_errno())

#define EPERM 1         /**< Not owner */
#define ENOENT 2        /**< No such file or directory */
#define ESRCH 3         /**< No such context */
#define EINTR 4         /**< Interrupted system call */
#define EIO 5           /**< I/O error */
#define ENXIO 6         /**< No such device or address */
#define E2BIG 7         /**< Arg list too long */
#define ENOEXEC 8       /**< Exec format error */
#define EBADF 9         /**< Bad file number */
#define ECHILD 10       /**< No children */
#define EAGAIN 11       /**< No more contexts */
#define ENOMEM 12       /**< Not enough core */
#define EACCES 13       /**< Permission denied */
#define EFAULT 14       /**< Bad address */
#define ENOTBLK 15      /**< Block device required */
#define EBUSY 16        /**< Mount device busy */
#define EEXIST 17       /**< File exists */
#define EXDEV 18        /**< Cross-device link */
#define ENODEV 19       /**< No such device */
#define ENOTDIR 20      /**< Not a directory */
#define EISDIR 21       /**< Is a directory */
#define EINVAL 22       /**< Invalid argument */
#define ENFILE 23       /**< File table overflow */
#define EMFILE 24       /**< Too many open files */
#define ENOTTY 25       /**< Not a typewriter */
#define ETXTBSY 26      /**< Text file busy */
#define EFBIG 27        /**< File too large */
#define ENOSPC 28       /**< No space left on device */
#define ESPIPE 29       /**< Illegal seek */
#define EROFS 30        /**< Read-only file system */
#define EMLINK 31       /**< Too many links */
#define EPIPE 32        /**< Broken pipe */
#define EDOM 33         /**< Argument too large */
#define ERANGE 34       /**< Result too large */
#define ENOMSG 35       /**< Unexpected message type */
#define EDEADLK 45      /**< Resource deadlock avoided */
#define ENOLCK 46       /**< No locks available */
// #define ENOSTR 60       /**< STREAMS device required */
// #define ENODATA 61      /**< Missing expected message data */
// #define ETIME 62        /**< STREAMS timeout occurred */
// #define ENOSR 63        /**< Insufficient memory */
// #define EPROTO 71       /**< Generic STREAMS error */
// #define EBADMSG 77      /**< Invalid STREAMS message */
// #define ENOSYS 88       /**< Function not implemented */
// #define ENOTEMPTY 90    /**< Directory not empty */
#define ENAMETOOLONG 91 /**< File name too long */
// #define ELOOP 92        /**< Too many levels of symbolic links */
// #define EOPNOTSUPP 95   /**< Operation not supported on socket */
// #define EPFNOSUPPORT 96 /**< Protocol family not supported */
// #define ECONNRESET 104   /**< Connection reset by peer */
// #define ENOBUFS 105      /**< No buffer space available */
// #define EAFNOSUPPORT 106 /**< Addr family not supported */
// #define EPROTOTYPE 107   /**< Protocol wrong type for socket */
// #define ENOTSOCK 108     /**< Socket operation on non-socket */
// #define ENOPROTOOPT 109  /**< Protocol not available */
// #define ESHUTDOWN 110    /**< Can't send after socket shutdown */
// #define ECONNREFUSED 111 /**< Connection refused */
// #define EADDRINUSE 112   /**< Address already in use */
// #define ECONNABORTED 113 /**< Software caused connection abort */
// #define ENETUNREACH 114  /**< Network is unreachable */
// #define ENETDOWN 115     /**< Network is down */
// #define ETIMEDOUT 116    /**< Connection timed out */
// #define EHOSTDOWN 117    /**< Host is down */
// #define EHOSTUNREACH 118 /**< No route to host */
// #define EINPROGRESS 119  /**< Operation now in progress */
// #define EALREADY 120     /**< Operation already in progress */
// #define EDESTADDRREQ 121 /**< Destination address required */
// #define EMSGSIZE 122        /**< Message size */
// #define EPROTONOSUPPORT 123 /**< Protocol not supported */
// #define ESOCKTNOSUPPORT 124 /**< Socket type not supported */
// #define EADDRNOTAVAIL 125   /**< Can't assign requested address */
// #define ENETRESET 126       /**< Network dropped connection on reset */
// #define EISCONN 127         /**< Socket is already connected */
// #define ENOTCONN 128        /**< Socket is not connected */
// #define ETOOMANYREFS 129    /**< Too many references: can't splice */
#define ENOTSUP 134         /**< Unsupported value */
// #define EILSEQ 138          /**< Illegal byte sequence */
// #define EOVERFLOW 139       /**< Value overflow */
// #define ECANCELED 140       /**< Operation canceled */
// #define	ENOMEDIUM	123	/* No medium found */
// #define	EMEDIUMTYPE	124	/* Wrong medium type */
// #define EWOULDBLOCK EAGAIN /**< Operation would block */
// #define	EDEADLOCK	EDEADLK

#define	EBFONT		59	/* Bad font file format */
#define	ENOSYS		38	/* Invalid system call number */

#define	ENOTEMPTY	39	/* Directory not empty */
#define	ELOOP		40	/* Too many symbolic links encountered */
#define	EWOULDBLOCK	EAGAIN	/* Operation would block */
// #define	ENOMSG		42	/* No message of desired type */
#define	EIDRM		43	/* Identifier removed */
#define	ECHRNG		44	/* Channel number out of range */
#define	EL2NSYNC	45	/* Level 2 not synchronized */
#define	EL3HLT		46	/* Level 3 halted */
#define	EL3RST		47	/* Level 3 reset */
#define	ELNRNG		48	/* Link number out of range */
#define	EUNATCH		49	/* Protocol driver not attached */
#define	ENOCSI		50	/* No CSI structure available */
#define	EL2HLT		51	/* Level 2 halted */
#define	EBADE		52	/* Invalid exchange */
#define	EBADR		53	/* Invalid request descriptor */
#define	EXFULL		54	/* Exchange full */
#define	ENOANO		55	/* No anode */
#define	EBADRQC		56	/* Invalid request code */
#define	EBADSLT		57	/* Invalid slot */

#define	EDEADLOCK	EDEADLK

#define	EBFONT		59	/* Bad font file format */
#define	ENOSTR		60	/* Device not a stream */
#define	ENODATA		61	/* No data available */
#define	ETIME		62	/* Timer expired */
#define	ENOSR		63	/* Out of streams resources */
#define	ENONET		64	/* Machine is not on the network */
#define	ENOPKG		65	/* Package not installed */
#define	EREMOTE		66	/* Object is remote */
#define	ENOLINK		67	/* Link has been severed */
#define	EADV		68	/* Advertise error */
#define	ESRMNT		69	/* Srmount error */
#define	ECOMM		70	/* Communication error on send */
#define	EPROTO		71	/* Protocol error */
#define	EMULTIHOP	72	/* Multihop attempted */
#define	EDOTDOT		73	/* RFS specific error */
#define	EBADMSG		74	/* Not a data message */
#define	EOVERFLOW	75	/* Value too large for defined data type */
#define	ENOTUNIQ	76	/* Name not unique on network */
#define	EBADFD		77	/* File descriptor in bad state */
#define	EREMCHG		78	/* Remote address changed */
#define	ELIBACC		79	/* Can not access a needed shared library */
#define	ELIBBAD		80	/* Accessing a corrupted shared library */
#define	ELIBSCN		81	/* .lib section in a.out corrupted */
#define	ELIBMAX		82	/* Attempting to link in too many shared libraries */
#define	ELIBEXEC	83	/* Cannot exec a shared library directly */
#define	EILSEQ		84	/* Illegal byte sequence */
#define	ERESTART	85	/* Interrupted system call should be restarted */
#define	ESTRPIPE	86	/* Streams pipe error */
#define	EUSERS		87	/* Too many users */
#define	ENOTSOCK	88	/* Socket operation on non-socket */
#define	EDESTADDRREQ	89	/* Destination address required */
#define	EMSGSIZE	90	/* Message too long */
#define	EPROTOTYPE	91	/* Protocol wrong type for socket */
#define	ENOPROTOOPT	92	/* Protocol not available */
#define	EPROTONOSUPPORT	93	/* Protocol not supported */
#define	ESOCKTNOSUPPORT	94	/* Socket type not supported */
#define	EOPNOTSUPP	95	/* Operation not supported on transport endpoint */
#define	EPFNOSUPPORT	96	/* Protocol family not supported */
#define	EAFNOSUPPORT	97	/* Address family not supported by protocol */
#define	EADDRINUSE	98	/* Address already in use */
#define	EADDRNOTAVAIL	99	/* Cannot assign requested address */
#define	ENETDOWN	100	/* Network is down */
#define	ENETUNREACH	101	/* Network is unreachable */
#define	ENETRESET	102	/* Network dropped connection because of reset */
#define	ECONNABORTED	103	/* Software caused connection abort */
#define	ECONNRESET	104	/* Connection reset by peer */
#define	ENOBUFS		105	/* No buffer space available */
#define	EISCONN		106	/* Transport endpoint is already connected */
#define	ENOTCONN	107	/* Transport endpoint is not connected */
#define	ESHUTDOWN	108	/* Cannot send after transport endpoint shutdown */
#define	ETOOMANYREFS	109	/* Too many references: cannot splice */
#define	ETIMEDOUT	110	/* Connection timed out */
#define	ECONNREFUSED	111	/* Connection refused */
#define	EHOSTDOWN	112	/* Host is down */
#define	EHOSTUNREACH	113	/* No route to host */
#define	EALREADY	114	/* Operation already in progress */
#define	EINPROGRESS	115	/* Operation now in progress */
#define	ESTALE		116	/* Stale file handle */
#define	EUCLEAN		117	/* Structure needs cleaning */
#define	ENOTNAM		118	/* Not a XENIX named type file */
#define	ENAVAIL		119	/* No XENIX semaphores available */
#define	EISNAM		120	/* Is a named type file */
#define	EREMOTEIO	121	/* Remote I/O error */
#define	EDQUOT		122	/* Quota exceeded */

#define	ENOMEDIUM	123	/* No medium found */
#define	EMEDIUMTYPE	124	/* Wrong medium type */
#define	ECANCELED	125	/* Operation Canceled */
#define	ENOKEY		126	/* Required key not available */
#define	EKEYEXPIRED	127	/* Key has expired */
#define	EKEYREVOKED	128	/* Key has been revoked */
#define	EKEYREJECTED	129	/* Key was rejected by service */

/* for robust mutexes */
#define	EOWNERDEAD	130	/* Owner died */
#define	ENOTRECOVERABLE	131	/* State not recoverable */

#define ERFKILL		132	/* Operation not possible due to RF-kill */

#define EHWPOISON	133	/* Memory page has hardware error */
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_NEWLIB_INCLUDE_ERRNO_H_ */
