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

char reset_reason_str[][15] = {
    [NO_RESET_REASON] = "NO_RESET_REASON",
    [COLD_RESET] = "COLD_RESET",
    [GLOBAL_RESET] = "GLOBAL_RESET",
    [HART_RESET] = "HART_RESET",
    [PASSIVE_RESET] = "PASSIVE_RESET",
};

int main(void)
{
    printf("\n\n\nHello World! %s\n", reset_reason_str[reset_reason_get()]);
    printf("reset_reason %d\n", reset_reason_get());

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    global_reset_reason_clean();
#endif
    reset_reason_magic_set();
    reset_reason_clean();
    reset_reason_flush_cache();
    HAL_IWDG_Init(APP_IWDG, BOOT_WDG_VALUE_BASE_S * 1);
    while(1) {
        printf("wait for reset..\n");
        k_msleep(300);
    }

    return 0;
}
