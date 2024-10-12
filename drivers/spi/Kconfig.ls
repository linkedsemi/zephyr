# Linkedsemi LS SPI configuration options

# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config SPI_LS
	bool "linkedsemi ls SPI driver"
    default y
	depends on DT_HAS_LINKEDSEMI_LS_SPI_ENABLED
	help
	  Enable support for the linkedsemi ls spi peripheral

config SPI_LS_INTERRUPT
	bool "LS SPI Interrupt Support"
	help
	  Enable Interrupt support for the SPI Driver of LS family.