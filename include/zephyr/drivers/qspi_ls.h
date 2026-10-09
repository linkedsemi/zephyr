/*
 * Copyright (c) 2026 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_QSPI_LS_H_
#define ZEPHYR_INCLUDE_DRIVERS_QSPI_LS_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief QSPI capture timing calibration status 
 *
 * QSPI_TIMING_CALIB_UNUSED  - calibration disabled or not yet attempted
 * QSPI_TIMING_CALIB_SUCCESS - calibration completed successfully
 * QSPI_TIMING_CALIB_FAILED  - calibration attempted but failed / skipped with error
 */
enum qspi_timing_calib_status {
	QSPI_TIMING_CALIB_UNUSED = 0,
	QSPI_TIMING_CALIB_SUCCESS = 1,
	QSPI_TIMING_CALIB_FAILED = 2,
};

/**
 * @brief Read QSPI timing calibration status.
 *
 * @param dev QSPI controller device 
 *
 * @retval >=0 One of enum qspi_timing_calib_status
 * @retval -EINVAL Invalid device 
 */
int qspi_timing_calibration_status(const struct device *dev);

/**
 * @brief Capture timing currently programmed on the QSPI controller.
 *
 * cap_neg / cap_dly are read from QSPI_CTRL1 (the values in use).
 */
struct qspi_timing_calib_value {
	uint8_t cap_neg;
	uint8_t cap_dly;
};

/**
 * @brief Read the QSPI capture timing currently in use.
 *
 * @param dev QSPI controller device
 * @param value Output; must not be NULL
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid device or value
 */
int qspi_timing_calibration_value(const struct device *dev,
				  struct qspi_timing_calib_value *value);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_QSPI_LS_H_ */
