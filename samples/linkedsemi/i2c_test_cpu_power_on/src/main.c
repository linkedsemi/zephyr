/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>

int main(void)
{
    const struct device *const i2c2 = DEVICE_DT_GET(DT_NODELABEL(i2c2));

    uint16_t dev_addr = 0x10;
    uint8_t wdata[] = {0x1, 0x30, 0x2};

    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    if (!device_is_ready(i2c2)) {
        __ASSERT(0,"I2C device is not ready\n");
    }
    if (i2c_configure(i2c2, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER) )
    {
        __ASSERT(0,"I2C device config failed\n");
    }

    i2c_write(i2c2, wdata, sizeof(wdata), dev_addr);

    return 0;
}
