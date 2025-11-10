/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mps_mp5023

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>

#include "mp5023.h"
#include "../pmbus/pmbus.h"

/* Log configuration */
LOG_MODULE_REGISTER(MP5023, CONFIG_SENSOR_LOG_LEVEL);

/* PMBus protocol implementation functions */
extern int pmbus_read_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t *value);
extern int pmbus_read_byte(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t *value);
extern int pmbus_write_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t value);
extern int pmbus_write_byte(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t value);
extern int pmbus_select_page(const struct smbus_dt_spec *smbus, uint8_t page);
extern int pmbus_clear_faults(const struct smbus_dt_spec *smbus);
extern int pmbus_verify_device(const struct smbus_dt_spec *smbus);
extern int pmbus_configure_pec(const struct smbus_dt_spec *smbus, bool enable);

/* MP5023-specific direct format conversion function */
static float mp5023_convert_direct(uint16_t raw_value, uint8_t cmd)
{
    int8_t m, b, R;
    float result;
    m = b = R = 0;

    switch (cmd)
    {
        case PMBUS_CMD_READ_VOUT:
        case PMBUS_CMD_READ_VIN:
        case PMBUS_CMD_VIN_OV_WARN_LIMIT:
        case PMBUS_CMD_VIN_UV_WARN_LIMIT:
            m = 32;
            break;
        case PMBUS_CMD_READ_IOUT:
        case PMBUS_CMD_IOUT_OC_WARN_LIMIT:
            m = 16;
            break;
        case PMBUS_CMD_READ_EIN:
        case PMBUS_CMD_READ_PIN:
            m = 1;
            break;
        case PMBUS_CMD_OT_FAULT_LIMIT:
        case PMBUS_CMD_OT_WARN_LIMIT:
        case PMBUS_CMD_READ_TEMPERATURE_1:
            m = 2;
            break;
        case PMBUS_CMD_MFR_SPECIFIC_STARTUP_CURRENT_LIMIT:
            m = 4;
            b = 50;
            R = -1;
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

static const struct sensor_driver_api mp5023_api = {
    .sample_fetch = mp5023_sample_fetch,
    .channel_get = mp5023_channel_get,
};

int mp5023_init(const struct device *dev)
{
    struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    LOG_DBG("Initializing MP5023 PMBus sensor");

    /* Check if SMBus device is ready */
    if (!device_is_ready(config->smbus.bus)) {
        LOG_ERR("SMBus bus device not ready: %s", config->smbus.bus->name);
        return -ENODEV;
    }

    /* Copy configuration to data structure */
    // data->smbus = config->smbus;
    data->current_page = config->default_page;
    data->timeout_ms = config->timeout_ms;

    /* Verify device presence */
    ret = pmbus_verify_device(&config->smbus);
    if (ret < 0) {
        LOG_ERR("Failed to verify MP5023 device: %d", ret);
        return ret;
    }

    /* Select default page */
    ret = pmbus_select_page(&config->smbus, config->default_page);
    if (ret < 0) {
        LOG_ERR("Failed to select default page: %d", ret);
        return ret;
    }

    ret = pmbus_configure_pec(&config->smbus, MP5023_DEFAULT_PEC_EN);
    if (ret < 0) {
        LOG_ERR("Failed to configure PEC: %d", ret);
        return ret;
    }

    /* Clear any existing faults */
    ret = pmbus_clear_faults(&config->smbus);
    if (ret < 0) {
        LOG_WRN("Failed to clear faults: %d", ret);
        /* Continue initialization despite warning */
    }

    /* Read initial status */
    ret = pmbus_read_word(&config->smbus, PMBUS_CMD_STATUS_WORD, &data->status_word);
    if (ret < 0) {
        LOG_WRN("Failed to read initial status: %d", ret);
    }

    LOG_INF("MP5023 PMBus sensor initialized successfully");
    return 0;
}

int mp5023_read_byte(const struct device *dev, uint8_t cmd, uint8_t *value)
{
    // struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    if (!value) {
        return -EINVAL;
    }

    ret = pmbus_read_byte(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to read command 0x%02X: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Read command 0x%02X: 0x%04X", cmd, *value);
    return 0;
}

int mp5023_read_word(const struct device *dev, uint8_t cmd, uint16_t *value)
{
    // struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    if (!value) {
        return -EINVAL;
    }

    ret = pmbus_read_word(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to read command 0x%02X: %d", cmd, ret);
        return ret;
    }

    LOG_DBG("Read command 0x%02X: 0x%04X", cmd, *value);
    return 0;
}

int mp5023_write_word(const struct device *dev, uint8_t cmd, uint16_t value)
{
    // struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    ret = pmbus_write_word(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to write command 0x%02X: 0x%04X, error: %d", cmd, value, ret);
        return ret;
    }

    LOG_DBG("Wrote command 0x%02X: 0x%04X", cmd, value);
    return 0;
}

int mp5023_write_byte(const struct device *dev, uint8_t cmd, uint8_t value)
{
    // struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    ret = pmbus_write_byte(&config->smbus, cmd, value);
    if (ret < 0) {
        LOG_ERR("Failed to write byte command 0x%02X: 0x%02X, error: %d", cmd, value, ret);
        return ret;
    }

    LOG_DBG("Wrote byte command 0x%02X: 0x%02X", cmd, value);
    return 0;
}

int mp5023_select_page(const struct device *dev, uint8_t page)
{
    struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    ret = pmbus_select_page(&config->smbus, page);
    if (ret < 0) {
        LOG_ERR("Failed to select page %d: %d", page, ret);
        return ret;
    }

    data->current_page = page;
    LOG_DBG("Selected page %d", page);
    return 0;
}

int mp5023_clear_faults(const struct device *dev)
{
    struct mp5023_data *data = dev->data;
    const struct mp5023_config *config = dev->config;
    int ret;

    ret = pmbus_clear_faults(&config->smbus);
    if (ret < 0) {
        LOG_ERR("Failed to clear faults: %d", ret);
        return ret;
    }

    /* Clear cached status values */
    data->status_word = 0;
    data->status_input = 0;
    data->status_temp = 0;
    data->status_cml = 0;

    LOG_DBG("Cleared all faults");
    return 0;
}

int mp5023_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct mp5023_data *data = dev->data;
    uint16_t raw_value;
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE &&
        chan != SENSOR_CHAN_CURRENT && chan != SENSOR_CHAN_GAUGE_TEMP) {
        return -ENOTSUP;
    }

    /* Read voltage if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE) {
        ret = mp5023_read_word(dev, PMBUS_CMD_READ_VOUT, &raw_value);
        if (ret < 0) {
            return ret;
        }
        
        data->vout = mp5023_convert_direct(raw_value, PMBUS_CMD_READ_VOUT);
    }

    /* Read current if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_CURRENT) {
        ret = mp5023_read_word(dev, PMBUS_CMD_READ_IOUT, &raw_value);
        if (ret < 0) {
            return ret;
        }
        
        data->iout = mp5023_convert_direct(raw_value, PMBUS_CMD_READ_IOUT);
    }

    /* Read temperature if requested or all channels */
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_GAUGE_TEMP) {
        ret = mp5023_read_word(dev, PMBUS_CMD_READ_TEMPERATURE_1, &raw_value);
        if (ret < 0) {
            return ret;
        }
        
        data->temperature = mp5023_convert_direct(raw_value, PMBUS_CMD_READ_TEMPERATURE_1);
    }

    /* Read status registers */
    if (chan == SENSOR_CHAN_ALL) {
        mp5023_read_word(dev, PMBUS_CMD_STATUS_WORD, &data->status_word);
        mp5023_read_byte(dev, PMBUS_CMD_STATUS_INPUT, &data->status_input);
        mp5023_read_word(dev, PMBUS_CMD_STATUS_TEMP, &data->status_temp);
        mp5023_read_byte(dev, PMBUS_CMD_STATUS_CML, &data->status_cml);
    }

    data->last_update = k_uptime_get_32();
    LOG_DBG("Fetched samples: voltage=%fV, current=%fA, temp=%f°C",
            (double)data->vout, (double)data->iout, (double)data->temperature);

    return 0;
}

int mp5023_channel_get(const struct device *dev, enum sensor_channel chan,
                       struct sensor_value *val)
{
    struct mp5023_data *data = dev->data;

    if (!val) {
        return -EINVAL;
    }

    /* Check if data is fresh */
    if (k_uptime_get_32() - data->last_update > data->timeout_ms) {
        LOG_WRN("Data is stale, consider calling sample_fetch first");
    }

    /* Sensor value consists of val1 (integer part) and val2 (fractional part) */
    switch (chan) {
    case SENSOR_CHAN_VOLTAGE:
        val->val1 = (int32_t)data->vout;
        val->val2 = (int32_t)(val->val1 - data->vout) >= 0 ? 
            (int32_t)(val->val1 - data->vout) * 1000000 : (int32_t)(data->vout - val->val1) * 1000000;
        break;
    
    case SENSOR_CHAN_CURRENT:
        val->val1 = (int32_t)data->iout;
        val->val2 = (int32_t)(val->val1 - data->iout) >= 0 ? 
            (int32_t)(val->val1 - data->iout) * 1000000 : (int32_t)(data->iout - val->val1) * 1000000;
        break;
    
    case SENSOR_CHAN_GAUGE_TEMP:
        val->val1 = (int32_t)data->temperature;
        val->val2 = (int32_t)(val->val1 - data->temperature) >= 0 ? 
            (int32_t)(val->val1 - data->temperature) * 1000000 : (int32_t)(data->temperature - val->val1) * 1000000;
        break;
    
    default:
        LOG_ERR("Unsupported sensor channel: %d", chan);
        return -ENOTSUP;
    }

    return 0;
}

/* Device registration */
#define MP5023_INIT(inst)                                                \
    static struct mp5023_data mp5023_data_##inst;                        \
    static const struct mp5023_config mp5023_config_##inst = {           \
        .smbus = SMBUS_DT_SPEC_INST_GET(inst),                           \
        .default_page = MP5023_DEFAULT_PAGE,                             \
        .timeout_ms = MP5023_DEFAULT_TIMEOUT_MS,                         \
        .pec_en = MP5023_DEFAULT_PEC_EN,                                 \
    };                                                                   \
    DEVICE_DT_INST_DEFINE(inst, mp5023_init, NULL,                       \
                          &mp5023_data_##inst,                           \
                          &mp5023_config_##inst, POST_KERNEL,            \
                          CONFIG_SENSOR_INIT_PRIORITY, &mp5023_api);

DT_INST_FOREACH_STATUS_OKAY(MP5023_INIT)