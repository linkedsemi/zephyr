/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(flash_spi_example, LOG_LEVEL_INF);

#define FLASH_SIZE        0x10000
#define FLASH_OFFSET      0x0000

static uint8_t data_write[] = {0x9f,0x9e,0x9c};  // The data to be written
static uint8_t data_read[3];  //Data for reading

/*
 *@method:Write the data externally to the flash,
 *        and then open the logic analyzer to check
 *        if the SPI pins have responded with the device ID.
*/
void flash_test(const struct device *dev)
{
    int rc;
    size_t len = sizeof(data_write);
	size_t read_len = sizeof(data_read);
    if (!dev) {
        LOG_INF("Flash device not found");
        return;
    }

    // Erase the data in Flash
    rc = flash_erase(dev, FLASH_OFFSET, FLASH_SIZE);  // Erase the specified area
    if (rc != 0) {
        LOG_INF("Flash erase failed: %d", rc);
        return;
    }
    LOG_INF("Flash sector erased");

    // Write data to Flash
    rc = flash_write(dev, FLASH_OFFSET, data_write, len);
    if (rc != 0) {
        LOG_INF("Flash write failed: %d", rc);
        return;
    }
    LOG_INF("Data written to Flash");

    // // Reading data
    rc = flash_read(dev, FLASH_OFFSET, data_read, read_len);
    if (rc != 0) {
        LOG_INF("Flash read failed: %d", rc);
        return;
    }
    LOG_INF("Data read from Flash: ");
    for (int i = 0; i < read_len; i++) {
        LOG_INF("0x%02X ", data_read[i]);
    }
}

void main(void)
{
	LOG_INF("flash3");
    //get external flash
    const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash3));
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    // obtain the ID of the external flash
    uint8_t id[3] = { 0 };
    int rc = 0;

    rc = flash_read_jedec_id(flash_dev, id);
    if (rc == 0) {
        printf("jedec-id = [%02x %02x %02x];\n", id[0], id[1], id[2]);
    } else {
        printf("JEDEC ID read failed: %d\n", rc);
    }

    flash_test(flash_dev);
}