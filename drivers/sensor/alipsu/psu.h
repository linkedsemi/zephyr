/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ALIPSU_PSU_H_
#define ZEPHYR_DRIVERS_SENSOR_ALIPSU_PSU_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include "../ls_pmbus/ls_pmbus.h"


#define ALI_PS_REG_OPERATION_BYTE 0x01 //len: 2
#define ALI_PS_REG_ON_OFF_CONFIG_BYTE 0x02 //len: 2
#define ALI_PS_REG_AUTO_TURN_ON_DELAY_WORD 0x60 //len: 2
#define ALI_PS_REG_READ_VIN1 0x75 //len: 2
#define ALI_PS_REG_STATUS_WORD 0x79 //len: 2
#define ALI_PS_REG_STATUS_VOUT 0x7A //len: 1
#define ALI_PS_REG_STATUS_IOUT 0x7B //len: 1
#define ALI_PS_REG_STATUS_INPUT 0x7C //len: 1
#define ALI_PS_REG_STATUS_TEMP 0x7D //len: 1
#define ALI_PS_REG_STATUS_CML 0x7E //len: 1
#define ALI_PS_REG_STATUS_OTHER 0x7F //len: 1
#define ALI_PS_REG_STATUS_MFR_SPEC 0x80 //len: 1
#define ALI_PS_REG_STATUS_FANS12 0x81 //len: 1
#define ALI_PS_REG_STATUS_INPUT_B 0x84 //len: 1
#define ALI_PS_REG_READ_VIN 0x88 //len: 2
#define ALI_PS_REG_READ_IIN 0x89 //len: 2
#define ALI_PS_REG_READ_VCAP 0x8A //len: 2
#define ALI_PS_REG_READ_VOUT 0x8B //len: 2
#define ALI_PS_REG_READ_IOUT 0x8C //len: 2
#define ALI_PS_REG_READ_TEMP1 0x8D //len: 2
#define ALI_PS_REG_READ_TEMP2 0x8E //len: 2
#define ALI_PS_REG_READ_TEMP3 0x8F //len: 2
#define ALI_PS_REG_READ_FAN_SPEED1 0x90 //len: 2
#define ALI_PS_REG_READ_POUT 0x96 //len: 2
#define ALI_PS_REG_READ_PIN 0x97 //len: 2
#define ALI_PS_REG_MFR_ID 0x99 //len: 12
#define ALI_PS_REG_MFR_MODEL 0x9A //real length
#define ALI_PS_REG_MFR_HW_REV 0x9B //len: 2
#define ALI_PS_REG_MFR_LOCATION 0x9C //len: 16
#define ALI_PS_REG_MFR_DATE 0x9D //len: 4
#define ALI_PS_REG_MFR_SN 0x9E //len: 15
#define ALI_BRICK_PS_REG_FW_REV 0xAF //len: 8
#define ALI_PS_REG_MFR_PSU_DEFINED 0xD2 //len: 1
#define ALI_PS_REG_FW_REV 0xD5 //len: 8
#define ALI_PS_REG_LINE_STATUS 0xD8 //len: 1
#define ALI_PS_REG_PSU_MATCH_STATUS 0xD9 //len: 1
#define ALI_PS_REG_INPUT_CH_STATUS 0xDA //len: 1
#define ALI_PS_REG_POWER_SUPPLY_MODE 0xE0 //len: 1
#define ALI_PS_REG_MFR_PAGE 0xE4 //len: 1
#define ALI_PS_REG_MFR_POS_TOTAL 0xE5 //len: 4
#define ALI_PS_REG_MFR_POS_LAST 0xE6 //len: 4
#define ALI_PS_REG_BOOTLOADER_KEY 0xF0 //len: 3
#define ALI_PS_REG_BOOTLOADER_STATUS 0xF1 //len: 1
#define ALI_PS_REG_BOOTLOADER_MEMORY 0xF2 //len: 16/32
#define ALI_PS_REG_BOOTLOADER_PRODUCT_KEY 0xF3 //len: 16

#define ALI_PS_REG_CURR_SHARE_ADJUST_EN           0xDB //len: 1
#define ALI_PS_REG_CURR_SHARE_ADJUST              0xDC //len: 1
#define ALI_PS_REG_CURR_SHARE_COMPENSATION_READ   0xDD //len: 2
#define ALI_PS_REG_CURR_SHARE_COMPENSATION_WRITE  0xDE //len: 2
#define ALI_PS_REG_CURR_SHARE_IOUT_READ_FILTER    0xDF //len: 2 [linear11]

#define ALI_PWR_DELTA_BLACKBOX_READ_DATA       0xD0 //len: 32
#define ALI_PWR_DELTA_BLACKBOX_HIS_EVENT_INDEX 0xD2 //len: 2

#define ALI_PWR_ACEP_BLACKBOX_READ_DATA        0xDB //len: 32
#define ALI_PWR_ACEP_BLACKBOX_HIS_EVENT_INDEX  0xD7 //len: 1


enum ali_attr {
	operation,
	on_off_config,
	auto_turn_on_delay,

	fan_config_1_2,
	fan_command_1,
	fan_command_2,

	status_word,
	status_vout,
	status_iout,
	status_input,
	status_input_b,
	status_temp,
	status_cml,
	status_other,
	status_mfr_spec,
	status_fans12,
	read_vin,
	read_iin,
	read_vcap,
	read_vout,
	read_vin1,
	read_iout,
	read_temp1,
	read_temp2,
	read_temp3,
	read_fan_speed1,
	read_pout,
	read_pin,
	mfr_id,
	mfr_model,
	mfr_hw_rev,
	mfr_location,
	mfr_date,
	mfr_sn,
	mfr_psu_defined,
	delta_event_index,
	delta_read_event,
	acep_event_index,
	acep_read_event,

	fw_rev,
	line_status,
	match_status,
	input_channel_status,
	power_supply_mode,
	mfr_page,
	mfr_pos_total,
	mfr_pos_last,
	bootloader_key,
	bootloader_status,
	bootloader_memory16,
	bootloader_memory32,
	bootloader_product_key,

	curr_share_adjust_en,
	curr_share_adjust,
	curr_share_compensation_read,
	curr_share_compensation_write,
	curr_iout_filter,

	num_sysfs,
};

#if 0
static uint8_t ali_attr_len[num_sysfs] = {
	[operation] = 1,
	[on_off_config] = 1,
	[auto_turn_on_delay] = 2,
	[fan_config_1_2] = 1,
	[fan_command_1] = 2,
	[fan_command_2] = 2,
	[status_word] = 2,
	[status_vout] = 1,
	[status_iout] = 1,
	[status_input] = 1,
	[status_input_b] = 1,
	[status_temp] = 1,
	[status_cml] = 1,
	[status_other] = 1,
	[status_mfr_spec] = 1,
	[status_fans12] = 1,
	[read_vin] = 2,
	[read_iin] = 2,
	[read_vcap] = 2,
	[read_vout] = 2,
	[read_vin1] = 2,
	[read_iout] = 2,
	[read_temp1] = 2,
	[read_temp2] = 2,
	[read_temp3] = 2,
	[read_fan_speed1] = 2,
	[read_pout] = 2,
	[read_pin] = 2,
	[mfr_id] = 12,
	[mfr_model] = 32,
	[mfr_hw_rev] = 2,
	[mfr_location] = 16,
	[mfr_date] = 4,
	[mfr_sn] = 15,
	[mfr_psu_defined] = 1,
	[delta_event_index] = 2,
	[delta_read_event] = 32,
	[acep_event_index] = 1,
	[acep_read_event] = 32,
	[fw_rev] = 8,
	[line_status] = 1,
	[match_status] = 1,
	[input_channel_status] = 1,
	[power_supply_mode] = 1,
	[mfr_page] = 1,
	[mfr_pos_total] = 4,
	[mfr_pos_last] = 4,
	[bootloader_key] = 3,
	[bootloader_status] = 1,
	[bootloader_memory16] = 16,
	[bootloader_memory32] = 32,
	[bootloader_product_key] = 16,

	[curr_share_adjust_en] = 1,
	[curr_share_adjust] = 1,
	[curr_share_compensation_read] = 2,
	[curr_share_compensation_write] = 2,
	[curr_iout_filter] = 2,
};
#endif

static uint8_t ali_psu_regs[num_sysfs] = {
	[operation] = ALI_PS_REG_OPERATION_BYTE,
	[on_off_config] = ALI_PS_REG_ON_OFF_CONFIG_BYTE,
	[auto_turn_on_delay] = ALI_PS_REG_AUTO_TURN_ON_DELAY_WORD,
	[fan_config_1_2] = PMBUS_CMD_FAN_CONFIG_12,
	[fan_command_1] = PMBUS_CMD_FAN_COMMAND_1,
	[fan_command_2] = PMBUS_CMD_FAN_COMMAND_2,
	[status_word] = ALI_PS_REG_STATUS_WORD,
	[status_vout] = ALI_PS_REG_STATUS_VOUT,
	[status_iout] = ALI_PS_REG_STATUS_IOUT,
	[status_input] = ALI_PS_REG_STATUS_INPUT,
	[status_input_b] = ALI_PS_REG_STATUS_INPUT_B,
	[status_temp] = ALI_PS_REG_STATUS_TEMP,
	[status_cml] = ALI_PS_REG_STATUS_CML,
	[status_other] = ALI_PS_REG_STATUS_OTHER,
	[status_mfr_spec] = ALI_PS_REG_STATUS_MFR_SPEC,
	[status_fans12] = ALI_PS_REG_STATUS_FANS12,
	[read_vin] = ALI_PS_REG_READ_VIN,
	[read_iin] = ALI_PS_REG_READ_IIN,
	[read_vcap] = ALI_PS_REG_READ_VCAP,
	[read_vout] = ALI_PS_REG_READ_VOUT,
	[read_vin1] = ALI_PS_REG_READ_VIN1,
	[read_iout] = ALI_PS_REG_READ_IOUT,
	[read_temp1] = ALI_PS_REG_READ_TEMP1,
	[read_temp2] = ALI_PS_REG_READ_TEMP2,
	[read_temp3] = ALI_PS_REG_READ_TEMP3,
	[read_fan_speed1] = ALI_PS_REG_READ_FAN_SPEED1,
	[read_pout] = ALI_PS_REG_READ_POUT,
	[read_pin] = ALI_PS_REG_READ_PIN,
	[mfr_id] = ALI_PS_REG_MFR_ID,
	[mfr_model] = ALI_PS_REG_MFR_MODEL,
	[mfr_hw_rev] = ALI_PS_REG_MFR_HW_REV,
	[mfr_location] = ALI_PS_REG_MFR_LOCATION,
	[mfr_date] = ALI_PS_REG_MFR_DATE,
	[mfr_sn] = ALI_PS_REG_MFR_SN,
	[mfr_psu_defined] = ALI_PS_REG_MFR_PSU_DEFINED,
	[delta_event_index] = ALI_PWR_DELTA_BLACKBOX_HIS_EVENT_INDEX,
	[delta_read_event] = ALI_PWR_DELTA_BLACKBOX_READ_DATA,
	[acep_event_index] = ALI_PWR_ACEP_BLACKBOX_HIS_EVENT_INDEX,
	[acep_read_event] = ALI_PWR_ACEP_BLACKBOX_READ_DATA,
	[fw_rev] = ALI_PS_REG_FW_REV,
	[line_status] = ALI_PS_REG_LINE_STATUS,
	[match_status] = ALI_PS_REG_PSU_MATCH_STATUS,
	[input_channel_status] = ALI_PS_REG_INPUT_CH_STATUS,
	[power_supply_mode] = ALI_PS_REG_POWER_SUPPLY_MODE,
	[mfr_page] = ALI_PS_REG_MFR_PAGE,
	[mfr_pos_total] = ALI_PS_REG_MFR_POS_TOTAL,
	[mfr_pos_last] = ALI_PS_REG_MFR_POS_LAST,
	[bootloader_key] = ALI_PS_REG_BOOTLOADER_KEY,
	[bootloader_status] = ALI_PS_REG_BOOTLOADER_STATUS,
	[bootloader_memory16] = ALI_PS_REG_BOOTLOADER_MEMORY,
	[bootloader_memory32] = ALI_PS_REG_BOOTLOADER_MEMORY,
	[bootloader_product_key] = ALI_PS_REG_BOOTLOADER_PRODUCT_KEY,

	[curr_share_adjust_en] = ALI_PS_REG_CURR_SHARE_ADJUST_EN,
	[curr_share_adjust] = ALI_PS_REG_CURR_SHARE_ADJUST,
	[curr_share_compensation_read] =
		ALI_PS_REG_CURR_SHARE_COMPENSATION_READ,
	[curr_share_compensation_write] =
		ALI_PS_REG_CURR_SHARE_COMPENSATION_WRITE,
	[curr_iout_filter] = ALI_PS_REG_CURR_SHARE_IOUT_READ_FILTER,
};

enum ali_psus { alipsu, powerbrick, delta800, gw800, gw1300 };
enum psu_status {
	psu_fw_update = 1,
	psu_fw_blackbox = 2,
};

struct ali_psu_data {
    struct k_mutex update_lock;
    int model;
    uint8_t mfr_page;
    uint8_t fw_update;
	uint32_t vin;
	uint32_t vin1;
	uint32_t vout;
	uint32_t iin;
	uint32_t iout;
	uint32_t pin;
	uint32_t temp1;
	uint32_t fan1;
	uint32_t fan2;
};

struct ali_psu_config {
    struct smbus_dt_spec smbus;        /* SMBus specification from DT */
}; 
                  
/* Public API Functions */
int ali_psu_init(const struct device *dev);
int ali_psu_read_word_data(const struct device *dev, uint8_t cmd, uint16_t *value);
int ali_psu_write_word_data(const struct device *dev, uint8_t cmd, uint16_t value);
int ali_psu_read_byte_data(const struct device *dev, uint8_t cmd, uint8_t *value);
int ali_psu_write_byte_data(const struct device *dev, uint8_t cmd, uint8_t value);
int ali_psu_write_byte(const struct device *dev, uint8_t cmd);
int ali_psu_read_block_data(const struct device *dev, uint8_t cmd, uint8_t *buffer, uint8_t *len);
int ali_psu_write_block_data(const struct device *dev, uint8_t cmd, uint8_t *buffer, uint8_t len);

/* Additional Public API Functions */
int ali_psu_read_byte(const struct device *dev, uint8_t cmd, uint8_t *value);
int ali_psu_read_word(const struct device *dev, uint8_t cmd, uint16_t *value);
int ali_psu_write_word(const struct device *dev, uint8_t cmd, uint16_t value);

/* Show and Store Functions */
int ali_psu_word_show(const struct device *dev, uint8_t reg, uint16_t *word);
int ali_psu_word_store(const struct device *dev, uint8_t reg, uint16_t word);
int ali_psu_byte_show(const struct device *dev, uint8_t reg, uint8_t *byte);
int ali_psu_byte_store(const struct device *dev, uint8_t reg, uint8_t byte);
int ali_psu_sensor_show(const struct device *dev, uint8_t reg, int64_t *val);
int ali_psu_mfr_page_show(const struct device *dev, uint8_t *page);
int ali_psu_mfr_page_store(const struct device *dev, uint8_t num);
int ali_psu_ac_cycle_store(const struct device *dev);
int ali_psu_block_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);
int ali_psu_fw_version_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);
int ali_psu_bootloader_str_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);
int ali_psu_bootloader_str_store(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t len);
int ali_psu_bootloader_hex_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);
int ali_psu_bootloader_hex_store(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t len);

/* History Functions */
int ali_psu_byte_word_read_his(const struct device *dev, uint8_t reg, uint8_t *buf, int mode);
int ali_psu_byte_word_read_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, int mode);
int ali_psu_word_his_show(const struct device *dev, uint8_t reg, uint8_t *buf);
int ali_psu_byte_his_show(const struct device *dev, uint8_t reg, uint8_t *buf);
int ali_psu_sensor_his_show(const struct device *dev, uint8_t reg, int64_t *val);
int ali_psu_block_hex_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);
int ali_powerbrick_block_hex_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len);

/* direct format conversion function */
float ali_psu_convert_direct(uint16_t raw_value, uint8_t cmd);

/* Sensor API Functions */
int ali_psu_sample_fetch(const struct device *dev, enum sensor_channel chan);
int ali_psu_channel_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_ALIPSU_PSU_H_ */
