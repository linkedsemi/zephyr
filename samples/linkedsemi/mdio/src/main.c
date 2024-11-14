/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/mdio.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>

int main(void)
{
    const struct device *const mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio));

    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    uint16_t reg;
    int rc = 0;

    printf("phy addr: 0x0  id: ");
    rc = mdio_read(mdio_dev, 0x0, 0x2, &reg);
    printf("%4.4x", reg);
    rc = mdio_read(mdio_dev, 0x0, 0x3, &reg);
    printf("%4.4x\n", reg);

    printf("phy addr: 0x1  id: ");
    rc = mdio_read(mdio_dev, 0x1, 0x2, &reg);
    printf("%4.4x", reg);
    rc = mdio_read(mdio_dev, 0x1, 0x3, &reg);
    printf("%4.4x\n", reg);

    return rc;
}
