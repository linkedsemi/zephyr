/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
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
    [SEC_IWDT_PARTIAL_RESET] = "SEC_IWDT_PARTIAL_RESET",
    [SEC_WWDT_PARTIAL_RESET] = "SEC_WWDT_PARTIAL_RESET",
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

    struct wdt_reset_en *sec_iwdt_reset_en = wdt_reset_en_val_get();
    /* memset sec_iwdt_reset_en is an essential step */
    memset(sec_iwdt_reset_en, 0xff, sizeof(struct wdt_reset_en));
    sec_iwdt_reset_en->UART1 = 1;
    sec_iwdt_reset_en->QSPI1 = 0;

    HAL_IWDG_Init(SEC_IWDG, BOOT_WDG_VALUE_BASE_S * 2);
    // HAL_IWDG_Init(SEC_PMU_IWDG, BOOT_WDG_VALUE_BASE_S * 2);

    sec_iwdt_reset_en_set(sec_iwdt_reset_en);
    sys_cache_data_flush_and_invd_range(sec_iwdt_reset_en, sizeof(struct wdt_reset_en));

    while(1) {
        printf("wait for reset..\n");
        k_msleep(300);
    }

    return 0;
}
