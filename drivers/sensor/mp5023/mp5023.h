/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MP5023_MP5023_H_
#define ZEPHYR_DRIVERS_SENSOR_MP5023_MP5023_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

/* Default configuration values */
#define MP5023_DEFAULT_PAGE                  CONFIG_MP5023_DEFAULT_PAGE
#define MP5023_DEFAULT_TIMEOUT_MS            CONFIG_MP5023_TIMEOUT_MS
#define MP5023_DEFAULT_PEC_EN                CONFIG_MP5023_PEC_EN

/* Manufacturer-specific commands */
#define PMBUS_CMD_MFR_CTRL                   0xF0
#define PMBUS_CMD_MFR_ADDR_PMBUS             0xF1
#define PMBUS_CMD_CONFIG_ID                  0xF2
#define PMBUS_CMD_MFR_SPECIFIC_STARTUP_CURRENT_LIMIT   0xF6
#define PMBUS_CMD_MFR_OTP_LEFT               0xFE

struct mp5023_data {
    uint8_t current_page;              /* Current PMBus page */
    uint16_t status_word;              /* Status word from device */
    uint8_t status_input;              /* Input voltage status */
    uint16_t status_temp;              /* Over-temperature fault or warning */
    uint8_t status_cml;                /* Command, data, PEC communication faults */
    float vout;                        /* Measured value of the output voltage (V) */
    float iout;                        /* Measured value of the output current. (A) */
    float temperature;                 /* Internal sensed temperature (°C) */
    uint32_t last_update;              /* Last update timestamp */
    uint32_t timeout_ms;               /* 添加timeout_ms成员 */
};

struct mp5023_config {
    struct smbus_dt_spec smbus;        /* SMBus specification from DT */
    uint8_t default_page;              /* Default PMBus page */
    uint32_t timeout_ms;               /* Operation timeout */
    bool pec_en;                       /* PEC mode enalbe */
}; 
                  

/* Public API Functions */
int mp5023_init(const struct device *dev);
int mp5023_read_byte(const struct device *dev, uint8_t cmd, uint8_t *value);
int mp5023_read_word(const struct device *dev, uint8_t cmd, uint16_t *value);
int mp5023_write_word(const struct device *dev, uint8_t cmd, uint16_t value);
int mp5023_write_byte(const struct device *dev, uint8_t cmd, uint8_t value);
int mp5023_select_page(const struct device *dev, uint8_t page);
int mp5023_clear_faults(const struct device *dev);

/* MP5023-specific direct format conversion function */
static float mp5023_convert_direct(uint16_t raw_value, uint8_t cmd);

/* Sensor API Functions */
int mp5023_sample_fetch(const struct device *dev, enum sensor_channel chan);
int mp5023_channel_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val);
int mp5023_attr_set(const struct device *dev,
                      enum sensor_channel chan,
                      enum sensor_attribute attr,
                      const struct sensor_value *val);
int mp5023_attr_get(const struct device *dev,
                      enum sensor_channel chan,
                      enum sensor_attribute attr,
                      struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_MP5023_MP5023_H_ */