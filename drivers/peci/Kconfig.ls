# Linkedsemi LS PECI configuration options

# Copyright (c) 2024 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config PECI_LS
	bool "linkedsemi ls PECI driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_PECI_ENABLED
    select PECI_INTERRUPT_DRIVEN
	help
	  Enable the Linkedsemi PECI IO driver.
