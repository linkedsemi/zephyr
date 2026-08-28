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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_QSPI_LS_H_ */
