# Copyright (c) 2023 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config SOC_FLASH_LS
	bool "linkedsemi ls flash driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_FLASH_CONTROLLER_ENABLED
	select FLASH_HAS_PAGE_LAYOUT
	select FLASH_HAS_DRIVER_ENABLED
	select FLASH_JESD216
	select FLASH_JESD216_API
	select FLASH_HAS_EXPLICIT_ERASE
	select FLASH_HAS_EX_OP
	select FLASH_EX_OP_ENABLED
	select FLASH_PAGE_LAYOUT
	help
	  Enables linkedsemi ls flash driver.

if SOC_FLASH_LS

config FLASH_SWINT_PRIORITY
	int "Flash Software Interrupt Priority"
	default 1

endif

config FLASH_OP_DELEGATION_SERVER
	bool "Flash Operation Delegation Server"

config SOC_FLASH_LS_DELEGATION_CLIENT
	default y
	bool "linkedsemi ls flash delegation client"
	depends on DT_HAS_LINKEDSEMI_LS_FLASH_DELEGATION_CLIENT_ENABLED
	select FLASH_HAS_PAGE_LAYOUT
	select FLASH_HAS_DRIVER_ENABLED
	select FLASH_JESD216
	select FLASH_JESD216_API
	select FLASH_HAS_EXPLICIT_ERASE
	select FLASH_HAS_EX_OP
	select FLASH_EX_OP_ENABLED
	select FLASH_OP_DELEGATION_SERVER
