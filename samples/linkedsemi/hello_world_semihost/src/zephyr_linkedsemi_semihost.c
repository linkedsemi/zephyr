#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <zephyr/sys/cbprintf.h>

#include "zephyr_linkedsemi_semihost.h"

char sh_printf_str[SH_PRINTF_STR_MAX_SIZE];

struct emitter {
    char *ptr;
    int len;
};

static int sprintf_out(int c, struct emitter *p)
{
    if (p->len > 1) { /* need to reserve a byte for EOS */
        *(p->ptr) = c;
        p->ptr += 1;
        p->len -= 1;
    }
    return 0; /* indicate keep going so we get the total count */
}

int sh_printf(const char *ZRESTRICT format, ...)
{
    char *ZRESTRICT str = sh_printf_str;

    va_list vargs;

    struct emitter p;
    int r;

    p.ptr = str;
    p.len = (int)0x7fffffff; /* allow up to "maxint" characters */

    va_start(vargs, format);
    r = cbvprintf(sprintf_out, (void *)(&p), format, vargs);
    va_end(vargs);

    *(p.ptr) = 0;

    if (get_semihosting_state()) {
        semihosting_puts(sh_printf_str, r);
    }

    return r;
}
