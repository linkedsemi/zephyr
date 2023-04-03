# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config SOC_FLASH_LS
	bool "linkedsemi ls flash driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_FLASH_CONTROLLER_ENABLED
	select FLASH_HAS_PAGE_LAYOUT
	select FLASH_HAS_DRIVER_ENABLED
	help
	  Enables linkedsemi ls flash driver.