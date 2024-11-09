#include <stdio.h>

void __stack_chk_guard(void)
{
    printf("\n%s\n", __func__);
    while(1) {;}
}

void __stack_chk_fail(void)
{
    printf("\n%s\n", __func__);
    while(1) {;}
}
