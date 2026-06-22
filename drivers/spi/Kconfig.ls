# Linkedsemi LS SPI configuration options

# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config SPI_LS
	bool "linkedsemi ls SPI driver"
    default y
	depends on DT_HAS_LINKEDSEMI_LS_SPI_ENABLED
	help
	  Enable support for the linkedsemi ls spi peripheral

config QSPI_LS
	bool "LinkedSemi QSPIv2 driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_QSPI_ENABLED
	help
	  Enable support for the LinkedSemi QSPIv2 controller used as an
	  SPI NOR bus master.
