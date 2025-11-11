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

config FLASH_SWINT_PRIORITY
	int "Flash Software Interrupt Priority"
	default 1
	depends on SOC_FLASH_LS

config FLASH_OP_DELEGATION_SERVER
	bool "Flash Operation Delegation Server"
	depends on SOC_FLASH_LS

config FLASH_DELEGATION_SYNC_TIMEOUT
	int "Flash Delegation Sync Timeout (ms)"
	default 100
	depends on FLASH_OP_DELEGATION_SERVER || SOC_FLASH_LS_DELEGATION_CLIENT

config FLASH_DELEGATION_SUSPEND_TIMEOUT
	int "Flash Delegation Suspend Timeout (us)"
	default 5000
	depends on FLASH_OP_DELEGATION_SERVER || SOC_FLASH_LS_DELEGATION_CLIENT


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

config FLASH_DELEGATION_CLIENT_SUSPEND_REQUEST
	default y
	bool "delegation client flash suspend request"
	depends on SOC_FLASH_LS_DELEGATION_CLIENT
