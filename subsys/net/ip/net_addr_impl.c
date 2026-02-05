/* SPDX-License-Identifier: Apache-2.0 */

#include "net_addr_compat.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Protocol family definitions */
#define AF_UNSPEC	0
#define AF_INET		2
#define AF_INET6	10

/* Address family type */
typedef unsigned short sa_family_t;

/* Forward declarations for network structures */
struct in_addr {
	uint8_t s4_addr[4];
};

struct in6_addr {
	union {
		uint8_t s6_addr[16];
		uint16_t s6_addr16[8];
		uint32_t s6_addr32[4];
	};
};

/* Implementation of net_addr_ntop */
static char *net_addr_ntop_impl(sa_family_t family, const void *src,
				char *dst, size_t size)
{
	struct in_addr *addr = NULL;
	struct in6_addr *addr6 = NULL;
	uint16_t *w = NULL;
	int i;
	uint8_t longest = 1U;
	int pos = -1;
	char delim = ':';
	uint8_t zeros[8] = { 0 };
	char *ptr = dst;
	int len = -1;
	uint16_t value;
	bool needcolon = false;
	bool mapped = false;

	if (family == AF_INET6) {
		addr6 = (struct in6_addr *)src;
		w = (uint16_t *)addr6->s6_addr16;
		len = 8;

		if (addr6->s6_addr[0] == 0 && addr6->s6_addr[1] == 0 &&
		    addr6->s6_addr[2] == 0 && addr6->s6_addr[3] == 0 &&
		    addr6->s6_addr[4] == 0 && addr6->s6_addr[5] == 0 &&
		    addr6->s6_addr[6] == 0 && addr6->s6_addr[7] == 0 &&
		    addr6->s6_addr[8] == 0 && addr6->s6_addr[9] == 0 &&
		    (addr6->s6_addr[10] == 0xff || addr6->s6_addr[10] == 0xFF)) {
			mapped = true;
		}

		for (i = 0; i < 8; i++) {
			for (int j = i; j < 8; j++) {
				if (w[j] != 0) {
					break;
				}
				zeros[i]++;
			}
		}

		for (i = 0; i < 8; i++) {
			if (zeros[i] > longest) {
				longest = zeros[i];
				pos = i;
			}
		}

		if (longest == 1U) {
			pos = -1;
		}

	} else if (family == AF_INET) {
		addr = (struct in_addr *)src;
		len = 4;
		delim = '.';
	} else {
		return NULL;
	}

print_mapped:
	for (i = 0; i < len; i++) {
		if (len == 4) {
			uint8_t l;
			value = addr->s4_addr[i];

			if (value == 0U) {
				*ptr++ = '0';
				*ptr++ = delim;
				continue;
			}

			l = snprintf(ptr, size - (ptr - dst), "%u", value);
			if (l <= 0) break;
			ptr += l;
			*ptr++ = delim;
			continue;
		}

		if (mapped && (i > 5)) {
			delim = '.';
			len = 4;
			addr = (struct in_addr *)(&addr6->s6_addr32[3]);
			*ptr++ = ':';
			family = AF_INET;
			goto print_mapped;
		}

		if (i == pos) {
			if (needcolon || i == 0U) {
				*ptr++ = ':';
			}
			*ptr++ = ':';
			needcolon = false;
			i += (int)longest - 1;
			continue;
		}

		if (needcolon) {
			*ptr++ = ':';
		}

		value = (w[i] >> 8) | (w[i] << 8);
		uint8_t bh = value >> 8;
		uint8_t bl = value & 0xff;

		if (bh) {
			ptr += snprintf(ptr, size - (ptr - dst), "%x", bh);
			ptr += snprintf(ptr, size - (ptr - dst), "%02x", bl);
		} else {
			ptr += snprintf(ptr, size - (ptr - dst), "%x", bl);
		}

		needcolon = true;
	}

	if (!(ptr - dst)) {
		return NULL;
	}

	if (family == AF_INET) {
		*(ptr - 1) = '\0';
	} else {
		*ptr = '\0';
	}

	return dst;
}

/* Implementation of net_addr_pton */
static int net_addr_pton_impl(sa_family_t family, const char *src, void *dst)
{
	if (family == AF_INET) {
		struct in_addr *addr = (struct in_addr *)dst;
		size_t i, len;

		len = strlen(src);
		for (i = 0; i < len; i++) {
			if (!(src[i] >= '0' && src[i] <= '9') &&
			    src[i] != '.') {
				return -EINVAL;
			}
		}

		memset(addr, 0, sizeof(struct in_addr));

		for (i = 0; i < sizeof(struct in_addr); i++) {
			char *endptr;
			addr->s4_addr[i] = strtol(src, &endptr, 10);
			src = ++endptr;
		}

	} else if (family == AF_INET6) {
		int expected_groups = strchr(src, '.') ? 6 : 8;
		struct in6_addr *addr = (struct in6_addr *)dst;
		int i, len;

		if (*src == ':') {
			src++;
		}

		len = strlen(src);
		for (i = 0; i < len; i++) {
			if (!(src[i] >= '0' && src[i] <= '9') &&
			    !(src[i] >= 'A' && src[i] <= 'F') &&
			    !(src[i] >= 'a' && src[i] <= 'f') &&
			    src[i] != '.' && src[i] != ':') {
				return -EINVAL;
			}
		}

		for (i = 0; i < expected_groups; i++) {
			char *tmp;

			if (!src || *src == '\0') {
				return -EINVAL;
			}

			if (*src != ':') {
				addr->s6_addr16[i] = ((strtol(src, NULL, 16) >> 8) |
						      (strtol(src, NULL, 16) << 8));
				src = strchr(src, ':');
				if (src) {
					src++;
				} else {
					if (i < expected_groups - 1) {
						return -EINVAL;
					}
				}
				continue;
			}

			for (; i < expected_groups; i++) {
				addr->s6_addr16[i] = 0;
			}

			tmp = strrchr(src, ':');
			if (src == tmp && (expected_groups == 6 || !src[1])) {
				src++;
				break;
			}

			if (expected_groups == 6) {
				tmp--;
			}

			i = expected_groups - 1;
			do {
				if (*tmp == ':') {
					i--;
				}
				if (i < 0) {
					return -EINVAL;
				}
			} while (tmp-- != src);

			src++;
		}

		if (expected_groups == 6) {
			for (i = 0; i < 4; i++) {
				if (!src || !*src) {
					return -EINVAL;
				}
				addr->s6_addr[12 + i] = strtol(src, NULL, 10);
				src = strchr(src, '.');
				if (src) {
					src++;
				} else {
					if (i < 3) {
						return -EINVAL;
					}
				}
			}
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

/* Public wrappers */
char *net_addr_ntop(sa_family_t family, const void *src,
		     char *dst, size_t size)
{
	return net_addr_ntop_impl(family, src, dst, size);
}

int net_addr_pton(sa_family_t family, const char *src,
		  void *dst)
{
	return net_addr_pton_impl(family, src, dst);
}
