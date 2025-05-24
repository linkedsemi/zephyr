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

extern volatile uint8_t reset_reason;

char reset_reason_str[][15] = {
    [NO_RESET_REASON] = "NO_RESET_REASON",
    [COLD_RESET] = "COLD_RESET",
    [GLOBAL_RESET] = "GLOBAL_RESET",
    [HART_RESET] = "HART_RESET",
    [PASSIVE_RESET] = "PASSIVE_RESET",
};

int main()
{
    printf("\n\n\nHello World! %s\n", reset_reason_str[reset_reason_get()]);
    printf("reset_reason %d\n", reset_reason);

    sys_reboot(SYS_REBOOT_WARM);

    return 0;
}
