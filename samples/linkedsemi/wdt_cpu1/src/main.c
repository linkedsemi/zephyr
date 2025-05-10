/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <ls_hal_iwdgv2.h>

#define BOOT_WDG_VALUE_BASE_S  (32768)
#define BOOT_WDG_VALUE_BASE_MS ((32768) / 1000)

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    HAL_IWDG_Init(SEC_IWDG, BOOT_WDG_VALUE_BASE_S * 2);
    while(1) {
        printf("wait for reset..\n");
        k_msleep(300);
    }

    return 0;
}
