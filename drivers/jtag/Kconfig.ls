# Linkedsemi LS JTAG configuration options

# Copyright (c) 2024 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config JTAG_LS
    bool "linkdesemi ls JTAG driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_JTAG_ENABLED
    help
	  This option enables the JTAG driver for Linkedsemi family of
	  processors.
	  Say y if you wish to use JTAG channels on Linkedsemi MCU.