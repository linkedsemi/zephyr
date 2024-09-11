/*
 * Copyright (c) 2023 SILA Embedded Solutions GmbH
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/drivers/smbus.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <soc.h>

#include "smbus_utils.h"

LOG_MODULE_REGISTER(linkedsemi_smbus, CONFIG_SMBUS_LOG_LEVEL);

struct smbus_linkedsemi_config {
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    const struct device *i2c_dev;
};

struct smbus_linkedsemi_data {
    uint32_t config;
    const struct device *dev;
#ifdef CONFIG_SMBUS_LINKEDSEMI_SMBALERT
    sys_slist_t smbalert_callbacks;
    struct k_work smbalert_work;
#endif /* CONFIG_SMBUS_LINKEDSEMI_SMBALERT */
};

#ifdef CONFIG_SMBUS_LINKEDSEMI_SMBALERT
static void smbus_linkedsemi_smbalert_isr(const struct device *dev)
{
    struct smbus_linkedsemi_data *data = dev->data;

    k_work_submit(&data->smbalert_work);
}

static void smbus_linkedsemi_smbalert_work(struct k_work *work)
{
    struct smbus_linkedsemi_data *data = CONTAINER_OF(work, struct smbus_linkedsemi_data, smbalert_work);
    const struct device *dev = data->dev;

    LOG_DBG("%s: got SMB alert", dev->name);

    smbus_loop_alert_devices(dev, &data->smbalert_callbacks);
}

static int smbus_linkedsemi_smbalert_set_cb(const struct device *dev, struct smbus_callback *cb)
{
    struct smbus_linkedsemi_data *data = dev->data;

    return smbus_callback_set(&data->smbalert_callbacks, cb);
}

static int smbus_linkedsemi_smbalert_remove_cb(const struct device *dev, struct smbus_callback *cb)
{
    struct smbus_linkedsemi_data *data = dev->data;

    return smbus_callback_remove(&data->smbalert_callbacks, cb);
}
#endif /* CONFIG_SMBUS_LINKEDSEMI_SMBALERT */

static int smbus_linkedsemi_init(const struct device *dev)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    struct smbus_linkedsemi_data *data = dev->data;
#if defined(CONFIG_PINCTRL)
    int result;
#endif
    data->dev = dev;

    if (!device_is_ready(config->i2c_dev)) {
        LOG_ERR("%s: I2C device is not ready", dev->name);
        return -ENODEV;
    }
#if defined(CONFIG_PINCTRL)
    result = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (result < 0) {
        LOG_ERR("%s: pinctrl setup failed (%d)", dev->name, result);
        return result;
    }
#endif
#ifdef CONFIG_SMBUS_LINKEDSEMI_SMBALERT
    // k_work_init(&data->smbalert_work, smbus_linkedsemi_smbalert_work);

    // i2c_linkedsemi_smbalert_set_callback(config->i2c_dev, smbus_linkedsemi_smbalert_isr, dev);
#endif /* CONFIG_SMBUS_LINKEDSEMI_SMBALERT */

    return 0;
}

static int smbus_linkedsemi_configure(const struct device *dev, uint32_t config_value)
{
    // const struct smbus_linkedsemi_config *config = dev->config;
    struct smbus_linkedsemi_data *data = dev->data;

    if (config_value & SMBUS_MODE_PEC) {
        LOG_ERR("%s: not implemented", dev->name);
        return -EINVAL;
    }

    if (config_value & SMBUS_MODE_HOST_NOTIFY) {
        LOG_ERR("%s: not available", dev->name);
        return -EINVAL;
    }

    if (config_value & SMBUS_MODE_CONTROLLER) {
        LOG_DBG("%s: configuring SMB in host mode", dev->name);
        // i2c_linkedsemi_set_smbus_mode(config->i2c_dev, I2CLINKEDSEMIMODE_SMBUSHOST);
    } else {
        LOG_DBG("%s: configuring SMB in device mode", dev->name);
        // i2c_linkedsemi_set_smbus_mode(config->i2c_dev, I2CLINKEDSEMIMODE_SMBUSDEVICE);
    }

    if (config_value & SMBUS_MODE_SMBALERT) {
        LOG_DBG("%s: activating SMB alert", dev->name);
        // i2c_linkedsemi_smbalert_enable(config->i2c_dev);
    } else {
        LOG_DBG("%s: deactivating SMB alert", dev->name);
        // i2c_linkedsemi_smbalert_disable(config->i2c_dev);
    }

    data->config = config_value;
    return 0;
}

static int smbus_linkedsemi_get_config(const struct device *dev, uint32_t *config)
{
    struct smbus_linkedsemi_data *data = dev->data;
    *config = data->config;
    return 0;
}

static int smbus_linkedsemi_quick(const struct device *dev, uint16_t periph_addr,
                 enum smbus_direction rw)
{
    const struct smbus_linkedsemi_config *config = dev->config;

    switch (rw) {
    case SMBUS_MSG_WRITE:
        return i2c_write(config->i2c_dev, NULL, 0, periph_addr);
    case SMBUS_MSG_READ:
        return i2c_read(config->i2c_dev, NULL, 0, periph_addr);
    default:
        LOG_ERR("%s: invalid smbus direction %i", dev->name, rw);
        return -EINVAL;
    }
}

static int smbus_linkedsemi_byte_write(const struct device *dev, uint16_t periph_addr, uint8_t command)
{
    const struct smbus_linkedsemi_config *config = dev->config;

    return i2c_write(config->i2c_dev, &command, sizeof(command), periph_addr);
}

static int smbus_linkedsemi_byte_read(const struct device *dev, uint16_t periph_addr, uint8_t *byte)
{
    const struct smbus_linkedsemi_config *config = dev->config;

    return i2c_read(config->i2c_dev, byte, sizeof(*byte), periph_addr);
}

static int smbus_linkedsemi_byte_data_write(const struct device *dev, uint16_t periph_addr,
                       uint8_t command, uint8_t byte)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    uint8_t buffer[] = {
        command,
        byte,
    };

    return i2c_write(config->i2c_dev, buffer, ARRAY_SIZE(buffer), periph_addr);
}

static int smbus_linkedsemi_byte_data_read(const struct device *dev, uint16_t periph_addr,
                      uint8_t command, uint8_t *byte)
{
    const struct smbus_linkedsemi_config *config = dev->config;

    return i2c_write_read(config->i2c_dev, periph_addr, &command, sizeof(command), byte,
                  sizeof(*byte));
}

static int smbus_linkedsemi_word_data_write(const struct device *dev, uint16_t periph_addr,
                       uint8_t command, uint16_t word)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    uint8_t buffer[sizeof(command) + sizeof(word)];

    buffer[0] = command;
    sys_put_le16(word, buffer + 1);

    return i2c_write(config->i2c_dev, buffer, ARRAY_SIZE(buffer), periph_addr);
}

static int smbus_linkedsemi_word_data_read(const struct device *dev, uint16_t periph_addr,
                      uint8_t command, uint16_t *word)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    int result;

    result = i2c_write_read(config->i2c_dev, periph_addr, &command, sizeof(command), word,
                  sizeof(*word));
    *word = sys_le16_to_cpu(*word);

    return result;
}

static int smbus_linkedsemi_pcall(const struct device *dev, uint16_t periph_addr, uint8_t command,
                 uint16_t send_word, uint16_t *recv_word)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    uint8_t buffer[sizeof(command) + sizeof(send_word)];
    int result;

    buffer[0] = command;
    sys_put_le16(send_word, buffer + 1);

    result = i2c_write_read(config->i2c_dev, periph_addr, buffer, ARRAY_SIZE(buffer), recv_word,
                  sizeof(*recv_word));
    *recv_word = sys_le16_to_cpu(*recv_word);

    return result;
}

static int smbus_linkedsemi_block_write(const struct device *dev, uint16_t periph_addr, uint8_t command,
                   uint8_t count, uint8_t *buf)
{
    const struct smbus_linkedsemi_config *config = dev->config;
    struct i2c_msg messages[] = {
        {
            .buf = &command,
            .len = sizeof(command),
            .flags = 0,
        },
        {
            .buf = buf,
            .len = count,
            .flags = 0,
        },
    };

    return i2c_transfer(config->i2c_dev, messages, ARRAY_SIZE(messages), periph_addr);
}

static const struct smbus_driver_api smbus_linkedsemi_api = {
    .configure = smbus_linkedsemi_configure,
    .get_config = smbus_linkedsemi_get_config,
    .smbus_quick = smbus_linkedsemi_quick,
    .smbus_byte_write = smbus_linkedsemi_byte_write,
    .smbus_byte_read = smbus_linkedsemi_byte_read,
    .smbus_byte_data_write = smbus_linkedsemi_byte_data_write,
    .smbus_byte_data_read = smbus_linkedsemi_byte_data_read,
    .smbus_word_data_write = smbus_linkedsemi_word_data_write,
    .smbus_word_data_read = smbus_linkedsemi_word_data_read,
    .smbus_pcall = smbus_linkedsemi_pcall,
    .smbus_block_write = smbus_linkedsemi_block_write,
#ifdef CONFIG_SMBUS_LINKEDSEMI_SMBALERT
    .smbus_smbalert_set_cb = smbus_linkedsemi_smbalert_set_cb,
    .smbus_smbalert_remove_cb = smbus_linkedsemi_smbalert_remove_cb,
#else
    .smbus_smbalert_set_cb = NULL,
    .smbus_smbalert_remove_cb = NULL,
#endif /* CONFIG_SMBUS_LINKEDSEMI_SMBALERT */
    .smbus_block_read = NULL,
    .smbus_block_pcall = NULL,
    .smbus_host_notify_set_cb = NULL,
    .smbus_host_notify_remove_cb = NULL,
};

#define DT_DRV_COMPAT linkedsemi_smbus

#if defined(CONFIG_PINCTRL)
#define SMBUS_LINKEDSEMI_DEVICE_INIT(n)                                                         \
    PINCTRL_DT_INST_DEFINE(n);                                                                  \
    static struct smbus_linkedsemi_config smbus_linkedsemi_config_##n = {                       \
        .i2c_dev = DEVICE_DT_GET(DT_INST_PROP(n, i2c)),                                         \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                              \
    };                                                                                          \
                                                                                                \
    static struct smbus_linkedsemi_data smbus_linkedsemi_data_##n;                              \
                                                                                                \
    SMBUS_DEVICE_DT_INST_DEFINE(n, smbus_linkedsemi_init, NULL, &smbus_linkedsemi_data_##n,     \
                    &smbus_linkedsemi_config_##n, POST_KERNEL,                                  \
                    CONFIG_SMBUS_INIT_PRIORITY, &smbus_linkedsemi_api);
#else
    #define SMBUS_LINKEDSEMI_DEVICE_INIT(n)                                                     \
        static struct smbus_linkedsemi_config smbus_linkedsemi_config_##n = {                   \
            .i2c_dev = DEVICE_DT_GET(DT_INST_PROP(n, i2c)),                                     \
        };                                                                                      \
                                                                                                \
        static struct smbus_linkedsemi_data smbus_linkedsemi_data_##n;                          \
                                                                                                \
        SMBUS_DEVICE_DT_INST_DEFINE(n, smbus_linkedsemi_init, NULL, &smbus_linkedsemi_data_##n, \
                        &smbus_linkedsemi_config_##n, POST_KERNEL,                              \
                        CONFIG_SMBUS_INIT_PRIORITY, &smbus_linkedsemi_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(SMBUS_LINKEDSEMI_DEVICE_INIT)
