/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ali_psu

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <stdio.h>
#include <string.h>

#include "psu.h"
// #include "../ls_pmbus/ls_pmbus.h"

/* Log configuration */
LOG_MODULE_REGISTER(ALIPSU, CONFIG_SENSOR_LOG_LEVEL);

#define I2C_SMBUS_BLOCK_MAX  32

static const struct sensor_driver_api ali_psu_api = {
	.sample_fetch = ali_psu_sample_fetch,
    .channel_get  = ali_psu_channel_get,
};

int ali_psu_init(const struct device *dev)
{
    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    struct ali_psu_data *data = dev->data;
    const struct ali_psu_config *config = dev->config;

    LOG_DBG("Initializing ALIPSU PMBus sensor");

    /* Check if i2c device is ready */
    if (!device_is_ready(config->smbus.bus)) {
        LOG_ERR("i2c device not ready: %s", dev->name);
        return -ENODEV;
    }

    data->model = alipsu;
    data->fw_update = psu_fw_update;
    data->mfr_page = 0x00;
    k_mutex_init(&data->update_lock);

    LOG_INF("ALIPSU PMBus sensor initialized successfully");
    return 0;
}

int ali_psu_read_byte(const struct device *dev, uint8_t cmd, uint8_t *value)
{
    const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    if (!value) {
        return -EINVAL;
    }

    config = dev->config;
    ret = ls_pmbus_read_byte(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to read command 0x%02X: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Read command 0x%02X: 0x%02X", cmd, *value);
    return 0;
}

int ali_psu_read_word(const struct device *dev, uint8_t cmd, uint16_t *value)
{
    const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    if (!value) {
        return -EINVAL;
    }

    config = dev->config;
    ret = ls_pmbus_read_word(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to read command 0x%02X: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Read command 0x%02X: 0x%04X", cmd, *value);
    return 0;
}

int ali_psu_write_word(const struct device *dev, uint8_t cmd, uint16_t value)
{
    const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    config = dev->config;
    ret = ls_pmbus_write_word(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to write command 0x%02X: 0x%04X, error: %d", cmd, value, ret);
        return ret;
    }

    LOG_DBG("Wrote command 0x%02X: 0x%04X", cmd, value);
    return 0;
}

int ali_psu_write_byte(const struct device *dev, uint8_t cmd)
{
    const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    config = dev->config;
    ret = ls_pmbus_write_byte(&config->smbus, cmd);
    if (ret < 0) {
        LOG_ERR("Failed to write byte command 0x%02X, error: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Wrote byte command 0x%02X", cmd);
    return 0;
}

int ali_psu_write_byte_data(const struct device *dev, uint8_t cmd, uint8_t value)
{
    const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    config = dev->config;
    ret = ls_pmbus_write_byte_data(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to write byte data command 0x%02X, value 0x%02X, error: %d", cmd, value, ret);
        return ret;
    }

    LOG_DBG("Wrote byte data command 0x%02X: 0x%02X", cmd, value);
    return 0;
}

int ali_psu_read_block_data(const struct device *dev, uint8_t cmd, uint8_t *buffer, uint8_t *len)
{
   const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    config = dev->config;
    ret = ls_pmbus_read_block(&config->smbus, cmd, len, buffer);
    if (ret < 0) {
        LOG_ERR("Failed to read block command 0x%02X, error: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Read block command 0x%02X: 0x%04X", cmd, *buffer);
    return 0;
}

int ali_psu_write_block_data(const struct device *dev, uint8_t cmd, uint8_t *buffer, uint8_t len)
{
   const struct ali_psu_config *config;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    config = dev->config;
    ret = ls_pmbus_write_block(&config->smbus, cmd, len, buffer);
    if (ret < 0) {
        LOG_ERR("Failed to write block command 0x%02X, error: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Wrote block command 0x%02X: 0x%04X", cmd, *buffer);
    return 0;
}

#if 0
static const char *buf2str_extended(const uint8_t *buf, int len,
				    const char *sep)
{
	static char str[4096];
	char *cur;
	int i;
	int sz;
	int left;
	int sep_len;

	if (!buf) {
		LOG_ERR("Buffer pointer is NULL");
		return (const char *)str;
	}
	cur = str;
	left = sizeof(str);
	if (sep)
		sep_len = strlen(sep);
	else
		sep_len = 0;

	for (i = 0; i < len; i++) {
		/* may return more than 2, depending on locale */
		sz = snprintf(cur, left, "%2.2x", buf[i]);
		if (sz >= left) {
			/* buffer overflow, truncate */
			break;
		}
		cur += sz;
		left -= sz;
		/* do not write separator after last byte */
		if (sep && i != (len - 1)) {
			if (sep_len >= left)
				break;

			strncpy(cur, sep, left - sz);
			cur += sep_len;
			left -= sep_len;
		}
	}
	*cur = '\0';

	return (const char *)str;
}
#endif

int ali_psu_read_word_data(const struct device *dev, uint8_t cmd, uint16_t *value)
{
	struct ali_psu_data *data = dev->data;
	if (!data || data->fw_update)
		return -EPERM;

	if (cmd >= PMBUS_VIRT_BASE)
		return -ENODATA;
    return ali_psu_read_word(dev, cmd, value);;
}

int ali_psu_write_word_data(const struct device *dev, uint8_t cmd, uint16_t value)
{
	struct ali_psu_data *data = dev->data;
	if (!data || data->fw_update)
		return -EPERM;

	if (cmd >= PMBUS_VIRT_BASE)
		return -ENODATA;
	return ali_psu_write_word(dev, cmd, value);
}

int ali_psu_read_byte_data(const struct device *dev, uint8_t cmd, uint8_t *value)
{
	struct ali_psu_data *data = dev->data;
	if (!data || data->fw_update)
		return -EPERM;

	return ali_psu_read_byte(dev, cmd, value);
}

int ali_psu_word_show(const struct device *dev, uint8_t reg, uint16_t *word)
{
	int rc;

	struct ali_psu_data *data = dev->data;
	k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_word_data(dev, reg, word);
	k_mutex_unlock(&data->update_lock);
	if (rc < 0) {
        LOG_ERR("Failed to read word data command 0x%02X, error: %d", reg, rc);
	}
    LOG_DBG("show psu_word: 0x%04x", *word);
    return rc;
}

int ali_psu_word_store(const struct device *dev, uint8_t reg, uint16_t word)
{
	int rc;

	struct ali_psu_data *data = dev->data;
    k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_write_word_data(dev, reg, word);
	k_mutex_unlock(&data->update_lock);
	if (rc < 0) {
		LOG_ERR("Failed to write word data command 0x%02X, error: %d", reg, rc);
	}
    return rc;
}

int ali_psu_byte_show(const struct device *dev, uint8_t reg, uint8_t *byte)
{
	int rc;

	struct ali_psu_data *data = dev->data;
    k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_byte_data(dev, reg, byte);
	k_mutex_unlock(&data->update_lock);
	if (rc < 0) {
		LOG_ERR("Failed to read byte data command 0x%02X, error: %d", reg, rc);
	}
    LOG_DBG("show psu_byte: 0x%02x", *byte);
    return rc;
}

int ali_psu_byte_store(const struct device *dev, uint8_t reg, uint8_t byte)
{
	int rc;

	struct ali_psu_data *data = dev->data;
    k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_write_byte_data(dev, reg, byte);
	k_mutex_unlock(&data->update_lock);
	if (rc < 0) {
		LOG_ERR("Failed to write byte data command 0x%02X, error: %d", reg, rc);
	}
    return rc;
}

int ali_psu_sensor_show(const struct device *dev, uint8_t reg, int64_t *val)
{
	int rc;
    uint16_t word;

	struct ali_psu_data *data = dev->data;
    k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_word_data(dev, reg, &word);
	k_mutex_unlock(&data->update_lock);

	if (rc < 0) {
        LOG_ERR("Failed to read word data command 0x%02X, error: %d", reg, rc);
		return rc;
	}

	/* Now we only support LINEAR11 */
	int16_t exponent = ((int16_t)word) >> 11;
	int32_t mantissa = ((int16_t)((word & 0x7ff) << 5)) >> 5;
	*val = mantissa;

	/* scale result to milli-units for all sensors except fans */
	// if (sensor->class != PSC_FAN)
	*val = *val * 1000LL;

	/* scale result to micro-units for power sensors */
	//if (sensor->class == PSC_POWER)
	//val = val * 1000LL;

	*val = (exponent >= 0) ? (*val << exponent) : (*val >> -exponent);
    LOG_DBG("show sensor: 0x%lld", *val);
	return rc;
}

int ali_psu_mfr_page_show(const struct device *dev, uint8_t *page)
{
	int ret;

	struct ali_psu_data *data = dev->data;
    k_mutex_lock(&data->update_lock, K_FOREVER);
	ret = ali_psu_read_byte_data(dev, ALI_PS_REG_MFR_PAGE, page);
	k_mutex_unlock(&data->update_lock);
	if (ret < 0)
        LOG_ERR("Failed to read MFR_PAGE, error: %d", ret);

	LOG_DBG("MFR_PAGE: 0x%02x", *page);
    return ret;
}

static inline int switch_page(const struct device *dev, uint8_t value);

int ali_psu_mfr_page_store(const struct device *dev, uint8_t num)
{
	int rc;
	struct ali_psu_data *data = dev->data;

	if ((num >= 0 && num < 15) || num == 0xff) {
		rc = switch_page(dev, num);
		if (rc < 0)
        {
            LOG_ERR("Failed to switch MFR_PAGE to 0x%02x, error: %d", num, rc);
            return rc;
        }
		data->mfr_page = num;
	} 
    else {
		LOG_ERR("alipsu mfr_page: %d, beyond range!\n", num);
		return -EINVAL;
	}

	return rc;
}

int ali_psu_ac_cycle_store(const struct device *dev)
{
	int rc;
    uint8_t read;
	struct ali_psu_data *data = dev->data;
	if (data->fw_update){
		return -EPERM;
    }

    k_mutex_lock(&data->update_lock, K_FOREVER);
	// Restore to default mode
	rc = ali_psu_write_byte_data(
		dev, ali_psu_regs[on_off_config], 0x88);
	if (rc < 0)
		goto exit;
	rc = ali_psu_write_byte_data(dev, ali_psu_regs[operation],
				       0x00);
	if (rc < 0)
		goto exit;

	// Wait 10ms for PSU to handle.
	k_msleep(10);

	// Check if psu is in default mode
	rc = ali_psu_read_byte_data(dev, ali_psu_regs[on_off_config], &read);
	if (rc < 0 || read != 0x88) {
		LOG_ERR("Failed to read on_off_config reg, error: %d", rc);
		goto exit;
	}
	rc = ali_psu_read_byte_data(dev, ali_psu_regs[operation], &read);
	if (rc < 0 || read != 0x0) {
		LOG_ERR("Failed to read operation reg, error: %d", rc);
		goto exit;
	}

	// Set auto turn on mode as PSU required
	rc = ali_psu_write_byte_data(
		dev, ali_psu_regs[on_off_config], 0x48);
	if (rc < 0)
		goto exit;
	rc = ali_psu_write_byte_data(dev, ali_psu_regs[operation],
				       0x40);
	if (rc < 0)
		goto exit;

	rc = 0;
exit:
	k_mutex_unlock(&data->update_lock);
	return rc;
}

int ali_psu_block_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;
    struct ali_psu_data *data = dev->data;
    if (!len || *len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu block show: len is NULL or beyond max size");
        return -EINVAL;
    }

	k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_block_data(dev, reg, buf, len);
	k_mutex_unlock(&data->update_lock);
	if (rc < 0) {
		LOG_ERR(
			"alipsu read block fail: cmd %d, rc %d\n", reg, rc);
    }
    LOG_DBG("show psu_block: 0x%x, len %d", *buf, *len);
    return rc;
}

int ali_psu_fw_version_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;
	struct ali_psu_data *psu_data = dev->data;

	/* We don't check the return value given FW version reg always return
	 * negative value no matter i2c reading succeeds or not.
	 */
    if (!len || *len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu fw version show: len is NULL or beyond max size");
        return -EINVAL;
    }

	k_mutex_lock(&psu_data->update_lock, K_FOREVER);
	if (psu_data->model == powerbrick) {
		reg = ALI_BRICK_PS_REG_FW_REV;
		rc = ali_psu_read_block_data(dev, reg, buf, len);
		/* Data format in PowerBrick: [length][value].
		 * i2c_smbus_read_block_data returns the length of the value.
		 */
		// *len = *len + 1;
	} else {
		rc = ali_psu_read_block_data(dev, reg, buf, len);
	}
	k_mutex_unlock(&psu_data->update_lock);

	if (rc < 0){
		LOG_ERR("Failed to read FW version, error: %d", rc);
    }

	LOG_DBG("FW version: 0x%x, len %d", *buf, *len);
    return rc;
}

int ali_psu_bootloader_str_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;

	/* We don't check the return value given some reg always return
	 * negative value no matter i2c reading succeeds or not.
	 */
    if (!len || *len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu bootloader str show: len is NULL or beyond max size");
        return -EINVAL;
    }
    
    struct ali_psu_data *psu_data = dev->data;
    k_mutex_lock(&psu_data->update_lock, K_FOREVER);
	rc = ali_psu_read_block_data(dev, reg, buf, len);
	if (rc < 0) {
		LOG_ERR("Failed to read bootloader string, error: %d", rc);
		return rc;
	}
	k_mutex_unlock(&psu_data->update_lock);

	if (rc < 0){
		LOG_ERR("Failed to read bootloader string, error: %d", rc);
    }

	// if (reg == ALI_PS_REG_MFR_POS_TOTAL || reg == ALI_PS_REG_MFR_POS_LAST)
	// 	return snprintf(buf, PAGE_SIZE, "0x%08x\n", *(uint32_t*)(data));
    LOG_DBG("show psu_bootloader_str: 0x%x, len %d", *buf, *len);
	return rc;
}

int ali_psu_bootloader_str_store(const struct device *dev,uint8_t reg, uint8_t *buf, uint8_t len)
{
	int rc;

	/* We don't check the return value given some reg always return
	 * negative value no matter i2c writing succeeds or not.
	 */
    if (len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu bootloader str store: len is beyond max size");
        return -EINVAL;
    }

	struct ali_psu_data *psu_data = dev->data;
	k_mutex_lock(&psu_data->update_lock, K_FOREVER);
	rc = ali_psu_write_block_data(dev, reg, buf, len);
	k_mutex_unlock(&psu_data->update_lock);

	if (rc < 0){
        LOG_ERR("Failed to write bootloader string, error: %d", rc);
    }

	return rc;
}

int ali_psu_bootloader_hex_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;

	/* We don't check the return value given some reg always return
	 * negative value no matter i2c reading succeeds or not.
	 */
    if (!len || *len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu bootloader hex show: len is NULL or beyond max size");
        return -EINVAL;
    }

	struct ali_psu_data *psu_data = dev->data;
	k_mutex_lock(&psu_data->update_lock, K_FOREVER);
	rc = ali_psu_read_block_data(dev, reg, buf, len);
	k_mutex_unlock(&psu_data->update_lock);

	if (rc < 0){
        LOG_ERR("Failed to read bootloader hex, error: %d", rc);
    }

    LOG_DBG("show psu_bootloader_hex: 0x%x, len %d", *buf, *len);
    return rc;
}

int ali_psu_bootloader_hex_store(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
	int rc;

    if (len > I2C_SMBUS_BLOCK_MAX) {
        LOG_ERR("alipsu bootloader hex store: len is beyond max size");
        return -EINVAL;
    }

	/* We don't check the return value given some reg always return
	 * negative value no matter i2c writing succeeds or not.
	 */
    struct ali_psu_data *psu_data = dev->data;
	k_mutex_lock(&psu_data->update_lock, K_FOREVER);
	rc = ali_psu_write_block_data(dev, reg, buf, len);
	k_mutex_unlock(&psu_data->update_lock);

	if (rc != 0){
        LOG_ERR("Failed to write bootloader hex, error: %d", rc);
    }
		
	return rc;
}

static inline int switch_page(const struct device *dev, uint8_t value)
{
	int ret;
    uint8_t current_page;
	struct ali_psu_data *data = dev->data;

	k_mutex_lock(&data->update_lock, K_FOREVER);
	ret = ali_psu_read_byte_data(dev, ALI_PS_REG_MFR_PAGE, &current_page);
	if (ret < 0)
		goto exit;
	if (current_page == value) {
		ret = 0;
		goto exit;
	}

	ret = ali_psu_write_byte_data(dev, ALI_PS_REG_MFR_PAGE, value);
	k_msleep(100);

exit:
	k_mutex_unlock(&data->update_lock);
	return ret;
}

int ali_psu_byte_word_read_his(const struct device *dev, uint8_t reg, uint8_t *buf, int mode)
{
	int rc;
	struct ali_psu_data *data = dev->data;

	if ((data->fw_update != psu_fw_blackbox) || (data->mfr_page >= 15))
		return -EPERM;

	k_mutex_lock(&data->update_lock, K_FOREVER);
	// read data
	if (mode == 0)
	{
		rc = ali_psu_read_byte_data(dev, reg, buf);
	}
	else
	{
		rc = ali_psu_read_word_data(dev, reg, (uint16_t *)buf);
	}
	k_mutex_unlock(&data->update_lock);

	if (rc < 0){
        LOG_ERR("Failed to read history data, mode: %d, error: %d", mode, rc);
    }

	LOG_DBG("show psu_bootloader_hex: 0x%x, mode %d", *buf, mode);
	return rc;
}

int ali_psu_byte_word_read_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, int mode)
{
	int rc = ali_psu_byte_word_read_his(dev, reg, buf, mode);

	if (rc < 0)
		return rc;

	if (mode == 0)
	{
		LOG_DBG("show byte: 0x%02x", *buf);
	}
	else
	{
		LOG_DBG("show word: 0x%04x", *buf);
	}
	return rc;
}

int ali_psu_word_his_show(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	return ali_psu_byte_word_read_his_show(dev, reg, buf, 1);
}

int ali_psu_byte_his_show(const struct device *dev, uint8_t reg, uint8_t *buf)
{
	return ali_psu_byte_word_read_his_show(dev, reg, buf, 0);
}

int ali_psu_sensor_his_show(const struct device *dev, uint8_t reg, int64_t *val)
{
	int rc;
    uint8_t byte;
	int16_t exponent;
    uint16_t read_word;
	int32_t mantissa;
	struct ali_psu_data *data = dev->data;

	if ((data->fw_update != psu_fw_blackbox) || (data->mfr_page >= 15))
		return -EPERM;

	rc = ali_psu_byte_word_read_his(dev, reg, (uint8_t *)&read_word, 1);
	if (rc < 0)
		return rc;

	if (reg == ALI_PS_REG_READ_VOUT) { /* LINEAR16 */
        rc = ali_psu_read_byte_data(dev, PMBUS_CMD_VOUT_MODE, &byte);
		exponent = byte & 0x1F;
		mantissa = read_word;
	} else {				/* LINEAR11 */
		exponent = read_word >> 11;
		mantissa = ((read_word & 0x7ff) << 5) >> 5;
	}

	*val = mantissa;

	/* scale result to milli-units for all sensors except fans */
	if (reg != ALI_PS_REG_READ_FAN_SPEED1)
		*val = *val * 1000LL;

	/* scale result to micro-units for power sensors */
	if (reg == ALI_PS_REG_READ_POUT || reg == ALI_PS_REG_READ_PIN)
		*val = *val * 1000LL;

	if (exponent >= 0)
		*val <<= exponent;
	else
		*val >>= -exponent;

	LOG_DBG("psu sensor hist show: val = %lld", *val);
    return rc;
}

int ali_psu_block_hex_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;
	struct ali_psu_data *data = dev->data;

	if ((data->fw_update != psu_fw_blackbox) || (data->mfr_page >= 15))
		return -EPERM;

	k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_block_data(dev, reg, buf, len);
	k_mutex_unlock(&data->update_lock);

	if (rc < 0){
        LOG_ERR("psu_block_hex_his_show failed to read: cmd %d, rc %d", reg, rc);
    }
            
    LOG_DBG("psu_block_hex_his_show: block data 0x%x, len %d", *buf, *len);
	return rc;
}

int ali_powerbrick_block_hex_his_show(const struct device *dev, uint8_t reg, uint8_t *buf, uint8_t *len)
{
	int rc;
	struct ali_psu_data *data = dev->data;

	k_mutex_lock(&data->update_lock, K_FOREVER);
	rc = ali_psu_read_block_data(dev, reg, buf, len);
	k_mutex_unlock(&data->update_lock);

	if (rc < 0){
        LOG_ERR("Failed to read block data, reg: 0x%02x, error: %d", reg, rc);
	}
	
	LOG_DBG("psu_block_hex_his_show: block data 0x%x, len %d", *buf, *len);
	return rc;
}

/* Alipsu-specific direct format conversion function, m = 1 for all now. */
float ali_psu_convert_direct(uint16_t raw_value, uint8_t cmd)
{
    int8_t m, b, R;
    float result;
    m = b = R = 0;

    switch (cmd)
    {
        case PMBUS_CMD_READ_VOUT:
        case PMBUS_CMD_READ_VIN:
		case PMBUS_CMD_READ_VIN1:
            m = 1 /*32*/;
            break;
        case PMBUS_CMD_READ_IOUT:
		case PMBUS_CMD_READ_IIN:
            m = 1  /*16*/;
            break;
        case PMBUS_CMD_READ_PIN:
            m = 1;
            break;
        case PMBUS_CMD_READ_TEMPERATURE_1:
            m = 1 /*2*/;
            break;
        case PMBUS_CMD_FAN_COMMAND_1:
		case PMBUS_CMD_FAN_COMMAND_2:
            m = 1;
            break;
        default:
            break;
    }

    /* Calculate result = (raw * 10^-R - b) / m */
    if (m != 0)
    {
        result = (my_powf(10.0f, -R) * (float)raw_value - (float)b) / (float)m;
    }
    else
    {
        LOG_ERR("Command %d not supported", cmd);
        result = 0.0f;
    }
    
    return result;	
}

/* Sensor API Functions */
int ali_psu_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ali_psu_data *data;
    uint16_t raw_value;
    int ret;

    if (!dev) {
        LOG_ERR("Device pointer is NULL");
        return -ENODEV;
    }

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE &&
        chan != SENSOR_CHAN_GAUGE_TEMP && chan != SENSOR_CHAN_POWER &&
		chan != SENSOR_CHAN_CURRENT) {
        return -ENOTSUP;
    }

    data = dev->data;
    /* Read voltage if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
        ret = ali_psu_read_word(dev, PMBUS_CMD_READ_VIN, &raw_value);
        data->vin = ls_pmbus_parse_linear11(raw_value) * 1000;

		ret = ali_psu_read_word(dev, PMBUS_CMD_READ_VIN1, &raw_value);
        data->vin1 = ls_pmbus_parse_linear11(raw_value) * 1000;

		ret = ali_psu_read_word(dev, PMBUS_CMD_READ_VOUT, &raw_value);
        data->vout = ls_pmbus_parse_linear16(raw_value) * 1000;
    }
	
    /* Read current if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {
		ret = ali_psu_read_word(dev, PMBUS_CMD_READ_IIN, &raw_value);
        data->iin = ls_pmbus_parse_linear11(raw_value) * 1000;
		
		ret = ali_psu_read_word(dev, PMBUS_CMD_READ_IOUT, &raw_value);
        data->iout = ls_pmbus_parse_linear11(raw_value) * 1000;
    }

	/* Read power if requested or all channels */
	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POWER) {
		ret = ali_psu_read_word(dev, PMBUS_CMD_READ_PIN, &raw_value);
        data->pin = ls_pmbus_parse_linear11(raw_value) * 1000 * 1000;
	}

    /* Read temperature if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_GAUGE_TEMP) {
        ret = ali_psu_read_word(dev, PMBUS_CMD_READ_TEMPERATURE_1, &raw_value);
        data->temp1 = ls_pmbus_parse_linear11(raw_value) * 1000;
    }

    /* Read status registers */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_RPM) {
        ret = ali_psu_read_word(dev, PMBUS_CMD_FAN_COMMAND_1, &raw_value);
		data->fan1 = ls_pmbus_parse_linear11(raw_value);
        
        ret = ali_psu_read_word(dev, PMBUS_CMD_FAN_COMMAND_2, &raw_value);
		data->fan2 = ls_pmbus_parse_linear11(raw_value);
    }

	if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_GAUGE_STATE_OF_HEALTH) {
		ret = ali_psu_read_word(dev, PMBUS_CMD_STATUS_WORD, &data->status_word);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_CML, &data->status_cml);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_FAN_12, &data->status_fans12);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_INPUT, &data->status_input);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_IOUT, &data->status_iout);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_MFR_SPECIFIC, &data->status_mfr_spec);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_OTHER, &data->status_other);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_TEMP, &data->status_temp);
		ret = ali_psu_read_byte(dev, PMBUS_CMD_STATUS_VOUT, &data->status_vout);
	}

    LOG_DBG("Fetched samples: input voltage=%dmV, input current=%dmA, input voltage1=%dmV, \
		     output voltage=%dmV, output current=%dmA, power=%dvW, temp=%dm°C, fan1=%dmHz, fan2=%dmHz.\n",
             data->vin, data->iin, data->vin1, data->vout, data->iout, data->pin, data->temp1, data->fan1, data->fan2);

	LOG_DBG("Fetched status: status_word=0x%04x, status_cml=0x%02x, status_fans12=0x%02x, \
		     status_input=0x%02x, status_iout=0x%02x, status_mfr_spec=0x%02x, status_other=0x%02x, \
			 status_temp=0x%02x, status_vout=0x%02x.\n",
             data->status_word, data->status_cml, data->status_fans12, data->status_input, 
			 data->status_iout, data->status_mfr_spec, data->status_other, data->status_temp, data->status_vout);

    return 0;
}

int ali_psu_channel_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val)
{
    struct ali_psu_data *data = dev->data;

    if (!val) {
        return -EINVAL;
    }

    switch (chan) {
    case SENSOR_CHAN_VOLTAGE:
		if (val->val2 == 0) {
			val->val1 = data->vin;
			val->val2 = 0;
		} else if (val->val2 == 1) {
			val->val1 = data->vin1;
			val->val2 = 0;
		} else if (val->val2 == 2) {
			val->val1 = data->vout;
			val->val2 = 0;
		}
        break;
    case SENSOR_CHAN_CURRENT:
		if (val->val2 == 0) {
			val->val1 = data->iin;
			val->val2 = 0;
		} else if (val->val2 == 1) {
			val->val1 = data->iout;
			val->val2 = 0;
		}
        break;
    case SENSOR_CHAN_POWER:
        val->val1 = data->pin;
        val->val2 = 0;
        break;
    case SENSOR_CHAN_GAUGE_TEMP:
        val->val1 = data->temp1;
        val->val2 = 0;
        break;
	case SENSOR_CHAN_RPM:
		if (val->val2 == 0) {
			val->val1 = data->fan1;
			val->val2 = 0;
		} else if (val->val2 == 1) {
			val->val1 = data->fan2;
			val->val2 = 0;
		}
		break;
	case SENSOR_CHAN_GAUGE_STATE_OF_HEALTH:
		if (val->val2 == 0) {
			val->val1 = data->status_word;
			val->val2 = 0;
		} else if (val->val2 == 1) {
			val->val1 = data->status_cml;
			val->val2 = 0;
		} else if (val->val2 == 2) {
			val->val1 = data->status_fans12;
			val->val2 = 0;
		} else if (val->val2 == 3) {
			val->val1 = data->status_input;
			val->val2 = 0;
		} else if (val->val2 == 4) {
			val->val1 = data->status_iout;
			val->val2 = 0;
		} else if (val->val2 == 5) {
			val->val1 = data->status_mfr_spec;
			val->val2 = 0;
		} else if (val->val2 == 6) {
			val->val1 = data->status_other;
			val->val2 = 0;
		} else if (val->val2 == 7) {
			val->val1 = data->status_temp;
			val->val2 = 0;
		} else if (val->val2 == 8) {
			val->val1 = data->status_vout;
			val->val2 = 0;
		}
		break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

/* Device registration */
#define ALIPSU_INIT(inst)                                                \
    static struct ali_psu_data ali_psu_data_##inst;                        \
    static const struct ali_psu_config ali_psu_config_##inst = {           \
        .smbus = SMBUS_DT_SPEC_INST_GET(inst),                           \
    };                                                                   \
    DEVICE_DT_INST_DEFINE(inst, ali_psu_init, NULL,                       \
                          &ali_psu_data_##inst,                           \
                          &ali_psu_config_##inst, POST_KERNEL,            \
                          CONFIG_SENSOR_INIT_PRIORITY, &ali_psu_api);

DT_INST_FOREACH_STATUS_OKAY(ALIPSU_INIT)
