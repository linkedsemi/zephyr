# Linkedsemi LS Clock controller driver configuration options

# Copyright (c) 2024 Linkedsemi.
# SPDX-License-Identifier: Apache-2.0

config CLOCK_CONTROL_LS
	bool "linkedsemi ls clock control"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_CCTL_ENABLED
	help
	  Enable support for Linkedsemi clock controller driver.