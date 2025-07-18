/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <soc_reset.h>
#include <ls_hal_iwdgv2.h>

#define BOOT_WDG_VALUE_BASE_S  (32768)
#define BOOT_WDG_VALUE_BASE_MS ((32768) / 1000)

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

int main(void)
{
    uint32_t reset_reason = reset_reason_get();
    printf("\n\n\nHello World! %s\n", reset_reason_str[reset_reason]);
    printf("reset_reason %d\n", reset_reason);

    HAL_IWDG_Init(APP_IWDG, BOOT_WDG_VALUE_BASE_S * 1);
    while(1) {
        printf("wait for reset..\n");
        k_msleep(300);
    }

    return 0;
}
