/*
 * Copyright (c) 2026 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SPI_DW_API_H_
#define ZEPHYR_INCLUDE_DRIVERS_SPI_DW_API_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Capture timing currently programmed on the DesignWare SPI controller.
 *
 * rx_sample_dly is the value saved for that slave. 0xFF means this
 * chip-select has no calibration result yet. RX_SAMPLE_DLY is one register
 * shared by every device on the controller, so it only shows the slave that
 * was configured last.
 */
struct spi_dw_timing_calib_value {
	uint8_t rx_sample_dly;
};

/**
 * @brief Read the calibration result stored for one chip-select.
 *
 * @param dev SPI controller device
 * @param slave Chip-select index, the same value as the flash reg / spi_config.slave
 * @param value Output; must not be NULL
 *
 * @retval 0 Success
 * @retval -EINVAL Invalid device, value, or slave, or dev is not a DesignWare SPI controller
 * @retval -ENODEV Device is not ready
 */
int spi_dw_timing_calibration_value(const struct device *dev, uint8_t slave,
				    struct spi_dw_timing_calib_value *value);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SPI_DW_API_H_ */
