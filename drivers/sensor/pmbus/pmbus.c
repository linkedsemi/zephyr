/*
 * Copyright (c) 2023 Jory Engineering
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pmbus.h"

/* Log configuration */
LOG_MODULE_REGISTER(PMBUS, CONFIG_SENSOR_LOG_LEVEL);

float my_powf(float base, int exponent)
{
    float result = 1.0f;
    
    if (exponent > 0) {
        for (int i = 0; i < exponent; i++) {
            result *= base;
        }
    } else if (exponent < 0) {
        for (int i = 0; i < -exponent; i++) {
            result /= base;
        }
    }
    
    return result;
}

/* Read a 16-bit value using PMBus protocol */
int pmbus_read_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t *value)
{
    int ret;
    uint8_t retry = 0;

    if (!smbus || !value) {
        return -EINVAL;
    }

    /* Try up to MAX_READ_RETRIES times */
    do {
         /* 直接使用SMBus API，避免通过宏间接调用 */
        const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
        if (api->smbus_word_data_read == NULL) {
            ret = -ENOSYS;
            break;
        }
        /* Use SMBus Word Data Read protocol */
        ret = api->smbus_word_data_read(smbus->bus, smbus->addr, cmd, value);
        if (ret == 0) {
            break;
        }
        retry++;
        k_msleep(1);
    } while (retry < PMBUS_MAX_READ_RETRIES);

    if (ret < 0) {
        LOG_ERR("PMBus read word failed after %d retries: %d", PMBUS_MAX_READ_RETRIES, ret);
    }

    return ret;
}

/* Write a 16-bit value using PMBus protocol */
int pmbus_write_word(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t value)
{
    /* 直接使用SMBus API，避免通过宏间接调用 */
    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->smbus_word_data_write == NULL) {
        return -ENOSYS;
    }
    /* Use SMBus Word Data Write protocol */
    return api->smbus_word_data_write(smbus->bus, smbus->addr, cmd, value);
}

/* Write a byte command using PMBus protocol */
int pmbus_write_byte(const struct smbus_dt_spec *smbus, uint8_t cmd)
{
    /* 直接使用SMBus API，避免通过宏间接调用 */
    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->smbus_byte_write == NULL) {
        return -ENOSYS;
    }
    /* Use SMBus Byte Write protocol */
    return api->smbus_byte_write(smbus->bus, smbus->addr, cmd);
}

/* Write a byte data using PMBus protocol */
int pmbus_write_byte_data(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t value)
{
    /* 直接使用SMBus API，避免通过宏间接调用 */
    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->smbus_byte_data_write == NULL) {
        return -ENOSYS;
    }
    /* Use SMBus Byte Data Write protocol */
    return api->smbus_byte_data_write(smbus->bus, smbus->addr, cmd, value);
}

/* Read a byte using PMBus protocol */
int pmbus_read_byte(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t *value)
{
    int ret;
    uint8_t retry = 0;

    /* Try up to MAX_READ_RETRIES times */
    do {
        /* 直接使用SMBus API，避免通过宏间接调用 */
        const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
        if (api->smbus_byte_data_read == NULL) {
            return -ENOSYS;
        }
        /* Use SMBus Byte Data Read protocol */
        ret = api->smbus_byte_data_read(smbus->bus, smbus->addr, cmd, value);
        if (ret == 0) {
            break;
        }
        retry++;
        k_msleep(1);
    } while (retry < PMBUS_MAX_READ_RETRIES);

    return ret;
}

/* Read a block of data using PMBus protocol */
int pmbus_read_block(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t *len, uint8_t *data)
{
    int ret;
    uint8_t retry = 0;

    /* Use SMBus Block Read protocol */
    do {
        /* 直接使用SMBus API，避免通过宏间接调用 */
        const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
        if (api->smbus_block_read == NULL) {
            return -ENOSYS;
        }
        ret = api->smbus_block_read(smbus->bus, smbus->addr, cmd, len, data);
        if (ret == 0) {
            break;
        }
        k_msleep(1);
        retry++;
    } while (retry < PMBUS_MAX_READ_RETRIES);

    return ret;
}

/* Write a block of data using PMBus protocol */
int pmbus_write_block(const struct smbus_dt_spec *smbus, uint8_t cmd, uint8_t len, const uint8_t *data)
{
    /* 直接使用SMBus API，避免通过宏间接调用 */
    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->smbus_block_write == NULL) {
        return -ENOSYS;
    }
    return api->smbus_block_write(smbus->bus, smbus->addr, cmd, len, (uint8_t *)data);
}

/* Process call using PMBus protocol */
int pmbus_process_call(const struct smbus_dt_spec *smbus, uint8_t cmd, uint16_t send_word, uint16_t *recv_word)
{
    /* 直接使用SMBus API，避免通过宏间接调用 */
    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->smbus_pcall == NULL) {
        return -ENOSYS;
    }
    return api->smbus_pcall(smbus->bus, smbus->addr, cmd, send_word, recv_word);
}

/* Select PMBus page */
int pmbus_select_page(const struct smbus_dt_spec *smbus, uint8_t page)
{
    int ret;

    if (!smbus) {
        return -EINVAL;
    }

    LOG_DBG("Selecting PMBus page: %d", page);
    ret = pmbus_write_byte_data(smbus, PMBUS_CMD_PAGE, page);
    if (ret < 0) {
        LOG_ERR("Failed to select page: %d", page);
        return ret;
    }

    /* Small delay to ensure page selection takes effect */
    k_msleep(1);
    return 0;
}

/* Clear all faults */
int pmbus_clear_faults(const struct smbus_dt_spec *smbus)
{
    int ret;

    if (!smbus) {
        return -EINVAL;
    }

    LOG_DBG("Clearing PMBus faults");
    ret = pmbus_write_byte(smbus, PMBUS_CMD_CLEAR_FAULTS);
    if (ret < 0) {
        LOG_ERR("Failed to clear faults, ret: %d", ret);
        return ret;
    }

    /* Small delay to ensure fault clearing takes effect */
    k_msleep(1);
    return 0;
}

/* Verify device presence and identity */
int pmbus_verify_device(const struct smbus_dt_spec *smbus)
{
    uint16_t value;
    int ret;

    if (!smbus) {
        return -EINVAL;
    }

    /* Try to read STATUS_WORD as a basic presence check */
    ret = pmbus_read_word(smbus, PMBUS_CMD_STATUS_WORD, &value);
    if (ret < 0) {
        LOG_ERR("Device not responding to STATUS_WORD command");
        return ret;
    }

    LOG_DBG("Device responded with STATUS_WORD: 0x%04X", value);
    
    /* Optional: Read and verify manufacturer ID, model, etc. */
    /* This would require knowing the expected values for MP5023 */
    
    return 0;
}

/* Parse Linear-11 format value */
float pmbus_parse_linear11(uint16_t value)
{
    int8_t exponent;
    int16_t mantissa;
    float result;

    /* Extract exponent (3 bits) and mantissa (13 bits) */
    exponent = (int8_t)((value & PMBUS_LINEAR11_EXPONENT_MASK) >> PMBUS_LINEAR11_EXPONENT_SHIFT);
    mantissa = (int16_t)(value & PMBUS_LINEAR11_MANTISSA_MASK);

    /* Sign extend the 13-bit mantissa to 16 bits */
    if (mantissa & 0x1000) {
        mantissa |= 0xE000;  /* 扩展符号位 */
    }

    /* Calculate result: mantissa * 2^exponent */
    result = (float)mantissa * my_powf(2.0f, exponent);
    
    return result;
}

/* Parse Linear-16 format value */
float pmbus_parse_linear16(uint16_t value)
{
    int8_t exponent;
    int8_t mantissa;
    float result;

    /* Extract exponent (8 bits) and mantissa (8 bits) */
    exponent = (int8_t)((value & PMBUS_LINEAR16_EXPONENT_MASK) >> PMBUS_LINEAR16_EXPONENT_SHIFT);
    mantissa = (int8_t)(value & PMBUS_LINEAR16_MANTISSA_MASK);

    /* Calculate result: mantissa * 2^exponent */
    result = (float)mantissa * my_powf(2.0f, exponent);
    
    return result;
}

/* Configure SMBus PEC (Packet Error Checking) */
int pmbus_configure_pec(const struct smbus_dt_spec *smbus, bool enable)
{
    int ret;
    uint32_t config;

    const struct smbus_driver_api *api = (const struct smbus_driver_api *)smbus->bus->api;
    if (api->get_config == NULL) {
        return -ENOSYS;
    }
    /* Get current SMBus configuration */
    ret = api->get_config(smbus->bus, &config);
    if (ret != 0) {
        LOG_ERR("Failed to get SMBus config: %d", ret);
        return ret;
    }

    /* Update PEC setting */
    if (enable) {
        config |= SMBUS_MODE_PEC;
    } else {
        config &= ~SMBUS_MODE_PEC;
    }

    if (api->configure == NULL) {
        return -ENOSYS;
    }
    /* Apply new configuration */
    ret = api->configure(smbus->bus, config);
    if (ret != 0) {
        LOG_ERR("Failed to set SMBus config: %d", ret);
        return ret;
    }

    return 0;
}