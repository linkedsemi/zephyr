# Linkedsemi LS CAP configuration options

# Copyright (c) 2024 Linkedsemi
# SPDX-License-Identifier: Apache-2.0

config CAP_LS
    bool "linkdesemi ls CAP driver"
    default y
    depends on DT_HAS_LINKEDSEMI_CAP_ENABLED
    help
	  This option enables the CAP driver for Linkedsemi family of
	  processors.
	  Say y if you wish to use CAP channels on Linkedsemi MCU.