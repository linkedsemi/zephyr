/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>

int main(void)
{
    const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash_controller));

    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    uint8_t id[3] = { 0 };
    int rc = 0;

    rc = flash_read_jedec_id(flash_dev, id);
    if (rc == 0) {
        printf("jedec-id = [%02x %02x %02x];\n", id[0], id[1], id[2]);
    } else {
        printf("JEDEC ID read failed: %d\n", rc);
    }

    return 0;
}
