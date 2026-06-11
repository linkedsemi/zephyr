#include <stdio.h>

FILE *__real_fopen(const char *file, const char *mode);
FILE *__wrap_fopen(const char *file, const char *mode)
{
    FILE *fp = __real_fopen(file, mode);
    if (fp) {
        fp->_flags |= __SNBF;
    }
    return fp;
}
