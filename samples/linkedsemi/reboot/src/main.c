/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <soc_reset.h>

char *reset_reason_str[] = {
    [NO_RESET_REASON] = "NO_RESET_REASON ",
    [PWR_FULL_RESET] = "PWR_FULL_RESET ",
    [SOFT_FULL_RESET] = "SOFT_FULL_RESET ",
    [CPU_FULL_RESET] = "CPU_FULL_RESET ",
    [SYS_IWDT_FULL_RESET] = "SYS_IWDT_FULL_RESET",
    [EXT_FULL_RESET] = "EXT_FULL_RESET ",
    [SEC_IWDT_FULL_RESET] = "SEC_IWDT_FULL_RESET",
    [SEC_WWDT_FULL_RESET] = "SEC_WWDT_FULL_RESET",
    [SEC_IWDT_HART_RESET] = "SEC_IWDT_HART_RESET",
    [SEC_WWDT_HART_RESET] = "SEC_WWDT_HART_RESET",
    [SOFT_HART_RESET] = "SOFT_HART_RESET ",
    [APP_IWDT_HART_RESET] = "APP_IWDT_HART_RESET",
    [APP_WWDT_HART_RESET] = "APP_WWDT_HART_RESET",
    [EMUL_SOFT_RESET] = "EMUL_SOFT_RESET",
};

int main()
{
    printf("\n\n\nHello World! %s\n", reset_reason_str[reset_reason_get()]);
    printf("reset_reason %d\n", reset_reason_get());

    sys_reboot(SYS_REBOOT_WARM);

    return 0;
}
