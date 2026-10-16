/*
 * Copyright (c) 2026 Linkedsemi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Extended public API for the Linkedsemi case-open (chassis intrusion)
 * detector (APP_PMU CASEOPEN_CTRL).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_CASEOPEN_LS_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_CASEOPEN_LS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_attribute_caseopen_ls {
	/**
	 * Rearm the intrusion latch: clears the latched status (stt).
	 * Value is ignored (attr_set only).
	 */
	SENSOR_ATTR_CASEOPEN_REARM = SENSOR_ATTR_PRIV_START,
	/**
	 * Enable/disable case-open detection.
	 * attr_set: val1 = 0 (disable) / 1 (enable).
	 * attr_get: val1 returns current enable state.
	 */
	SENSOR_ATTR_CASEOPEN_ENABLE,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_CASEOPEN_LS_H_ */
