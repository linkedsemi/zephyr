/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/jtag.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static uint8_t tdi_buffer[512];
uint8_t id_code[4] = {0x3d,0x56,0x0,0x10};  //GD32 id-code：0x1000563d
uint8_t ir_scan_value = 0x1;
uint8_t dr_scan_value = 0x0;
bool state = false;
int main(void)
{
	printf("start jtag master test! %s\n", CONFIG_BOARD_TARGET);

	const struct device *const jtag = DEVICE_DT_GET(DT_ALIAS(testjtag));

	if (!device_is_ready(jtag))
	{
		__ASSERT(0,"JTAG device is not ready");
	}

	state = true;
	if(!jtag_tap_set(jtag, TAP_RESET))
		state = false;
	if(state == false)
	{
		state = true;
		if(!jtag_ir_scan(jtag, 0x5, &ir_scan_value, tdi_buffer,TAP_IDLE))
			state = false;
	}

	if(state == false)
	{
		state = true;
		if(!jtag_dr_scan(jtag, 0x20, &dr_scan_value, tdi_buffer,TAP_IDLE))
		{
			state = false;
		}
		if (!memcmp(tdi_buffer, id_code, sizeof(id_code))) {
			printf("jtag master test pass!\n");
		}
	}

	return 0;
}
 