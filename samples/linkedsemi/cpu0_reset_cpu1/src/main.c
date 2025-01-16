/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <reg_sysc_sec_cpu.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	k_msleep(1000);
	while(1) {
		printf("reset cpu1\n");
		SYSC_SEC_CPU->APP_CPU_SRST = 0x2; /* reset */
		printf("sleep 1s\n");
		k_msleep(1000);
		printf("dereset cpu1\n");
		SYSC_SEC_CPU->APP_CPU_ADDR_CFG = 0x10080000; /* set cpu1 pc addr */
		SYSC_SEC_CPU->APP_CPU_SRST = 0x1; /* release reset */
		printf("sleep 1s\n");
		printf("\n");
		k_msleep(1000);
	}

	return 0;
}
