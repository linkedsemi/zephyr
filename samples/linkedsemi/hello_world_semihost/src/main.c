/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr_linkedsemi_semihost.h"

int main(void)
{
    if (semihosting_enabled()) {
        semihosting_init();
    }

    sh_printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    return 0;
}
