# Linkedsemi LS GPIO configuration options

# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config GPIO_LS
	bool "Linkedsemi ls GPIO driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_GPIO_ENABLED
	help
	  Enable the Linkedsemi ls GPIO driver.
