/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mp5023_app, LOG_LEVEL_INF);

// 定义驱动兼容性字符串
#define DT_DRV_COMPAT monolithic_power_mp5023

// 定义设备标签名称
#define MP5023_DEVICE_NAME "mp5023"

void main(void) {
  const struct device *dev = device_get_binding(MP5023_DEVICE_NAME);
  struct sensor_value voltage, current, temp;
  uint8_t retry = 0;

  // mp5023 sensor API
  /* const struct sensor_driver_api *mp5023_api =
      (const struct sensor_driver_api *)dev->api; */
  int ret;

  if (!device_is_ready(dev)) {
    LOG_ERR("Device %s is not ready", dev->name);
    return;
  }

  LOG_INF("MP5023 sensor example started");

  while (retry < 100) {
    /* Fetch all sensor data - 5023 specific sensor driver */
    // ret = mp5023_api->sample_fetch(dev, SENSOR_CHAN_ALL);
    ret = sensor_sample_fetch(dev); /* general sensor api */
    if (ret < 0) {
      LOG_ERR("Failed to fetch samples: %d", ret);
      continue;
    }

    /* Get voltage, current and temperature data */
    // ret = mp5023_api->channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
    ret = sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &voltage);
    if (ret < 0) {
      LOG_ERR("Failed to get voltage: %d", ret);
    }

    // ret = mp5023_api->channel_get(dev, SENSOR_CHAN_CURRENT, &current);
    ret = sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &current);
    if (ret < 0) {
      LOG_ERR("Failed to get current: %d", ret);
    }

    // ret = mp5023_api->channel_get(dev, SENSOR_CHAN_GAUGE_TEMP, &temp);
    ret = sensor_channel_get(dev, SENSOR_CHAN_GAUGE_TEMP, &temp);
    if (ret < 0) {
      LOG_ERR("Failed to get temperature: %d", ret);
    }

    LOG_INF("Voltage: %d.%06d V", voltage.val1, voltage.val2);
    LOG_INF("Current: %d.%06d A", current.val1, current.val2);
    LOG_INF("Temperature: %d.%06d °C", temp.val1, temp.val2);

    k_sleep(K_SECONDS(1));
    retry++;
  }
}