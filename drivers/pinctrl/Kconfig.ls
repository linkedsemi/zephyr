# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config PINCTRL_LS
	bool "linkedsemi ls pinctrl driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_PINCTRL_ENABLED
	help
	  Enable the linkedsemi ls pinctrl driver
