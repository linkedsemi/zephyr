#ifndef _ZEPHYR_LINKEDSEMI_SEMIHOST_H
#define _ZEPHYR_LINKEDSEMI_SEMIHOST_H

#include <semihosting.h>

#define SH_PRINTF_STR_MAX_SIZE 100

int sh_printf(const char *ZRESTRICT format, ...);

#endif
