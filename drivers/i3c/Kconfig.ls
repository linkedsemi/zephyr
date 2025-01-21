# Copyright (c) 2022 Meta Platforms, Inc. and its affiliates.
#
# SPDX-License-Identifier: Apache-2.0


config I3C_LS
	bool "LS I3C driver"
	select I3C_IBI_WORKQUEUE if I3C_USE_IBI
	depends on DT_HAS_LS_I3C_ENABLED
	default y
	help
	  Enable LS I3C driver.
