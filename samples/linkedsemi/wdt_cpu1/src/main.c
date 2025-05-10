/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <ls_hal_iwdgv2.h>

#define BOOT_WDG_VALUE_BASE_S  (1000000 * 10)
#define BOOT_WDG_VALUE_BASE_MS ((1000000 * 10) / 1000)

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    HAL_IWDG_Init(SEC_IWDG, BOOT_WDG_VALUE_BASE_S * 2);

    return 0;
}
