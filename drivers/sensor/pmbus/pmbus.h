/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

/* PMBus Command Definitions */
#define PMBUS_CMD_PAGE                       0x00
#define PMBUS_CMD_OPERATION                  0x01
#define PMBUS_CMD_CLEAR_FAULTS               0x03
#define PMBUS_CMD_STORE_USER_ALL             0x15
#define PMBUS_CMD_RESTORE_USER_ALL           0x16
#define PMBUS_CMD_CAPABILITY                 0x19
#define PMBUS_CMD_IOUT_CAL_GAIN              0x38
#define PMBUS_CMD_IOUT_CAL_OFFSET            0x39
#define PMBUS_CMD_IOUT_OC_WARN_LIMIT         0x4A
#define PMBUS_CMD_OT_FAULT_LIMIT             0x4F
#define PMBUS_CMD_OT_WARN_LIMIT              0x51
#define PMBUS_CMD_VIN_OV_WARN_LIMIT          0x57
#define PMBUS_CMD_VIN_UV_WARN_LIMIT          0x58
#define PMBUS_CMD_POWER_GOOD_ON              0x5E
#define PMBUS_CMD_POWER_GOOD_OFF             0x5F
#define PMBUS_CMD_STATUS_BYTE                0x78
#define PMBUS_CMD_STATUS_WORD                0x79
#define PMBUS_CMD_STATUS_INPUT               0x7C
#define PMBUS_CMD_STATUS_TEMP                0x7D
#define PMBUS_CMD_STATUS_CML                 0x7E
#define PMBUS_CMD_READ_EIN                   0x86
#define PMBUS_CMD_READ_VIN                   0x88
#define PMBUS_CMD_READ_VOUT                  0x8B
#define PMBUS_CMD_READ_IOUT                  0x8C
#define PMBUS_CMD_READ_TEMPERATURE_1         0x8D
#define PMBUS_CMD_READ_PIN                   0x97
#define PMBUS_CMD_PMBUS_REVISION             0x98
#define PMBUS_CMD_MFR_ID                     0x99
#define PMBUS_CMD_MFR_MODEL                  0x9A
#define PMBUS_CMD_MFR_REVISION               0x9B
#define PMBUS_CMD_MFR_DATE                   0x9D

/* PMBus Data Format Definitions */
#define PMBUS_LINEAR11_EXPONENT_MASK         0xE000
#define PMBUS_LINEAR11_EXPONENT_SHIFT        13
#define PMBUS_LINEAR11_MANTISSA_MASK         0x1FFF
#define PMBUS_LINEAR16_EXPONENT_MASK         0xFF00
#define PMBUS_LINEAR16_EXPONENT_SHIFT        8
#define PMBUS_LINEAR16_MANTISSA_MASK         0x00FF

/* PMBus constants */
#define PMBUS_MAX_READ_RETRIES               3
#define PMBUS_MAX_BLOCK_DATA_SIZE            32

/* 自定义的2的幂函数实现 */
float my_powf(float base, int exponent);
