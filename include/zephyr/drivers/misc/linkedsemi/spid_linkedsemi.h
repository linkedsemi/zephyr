/*
 * Copyright (c) 2023 Tokita, Hiroshi <tokita.hiroshi@fujitsu.com>
 * Copyright (c) 2023 Yonatan Schachter
 * Copyright (c) 2023 Ionut Pavel <iocapa@iocapa.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_SPID_LINKEDSEMI_H_
#define ZEPHYR_DRIVERS_MISC_SPID_LINKEDSEMI_H_

#include <zephyr/device.h>
#include <stdint.h>

typedef void (*spid_callback_t)(const struct device *dev,
                uint32_t callback_idx, void *user_data,
                void *drv_data);

int spid_linkedsemi_register_callback(const struct device *dev,
        uint32_t callback_idx, spid_callback_t cb, void *user_data);

#endif /* ZEPHYR_DRIVERS_MISC_SPID_LINKEDSEMI_H_ */
