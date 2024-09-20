/*
 * Copyright (c) 2024 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _LS_SOC_CLOCK_H_
#define _LS_SOC_CLOCK_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Common clock control device node for all LS series */
#define LS_CLK_CTRL_NODE   DT_NODELABEL(cctl)

struct ls_clk_cfg {
	uint16_t offest:5;
	uint16_t bit:5;
};

#define LS_DT_CLK_CFG_ITEM(inst)                                             \
	{                                                                      \
	  .offest = DT_PHA(DT_DRV_INST(inst), clocks, offest),                      \
	  .bit  = DT_PHA(DT_DRV_INST(inst), clocks, bit),                      \
	}

#ifdef __cplusplus
}
#endif
#endif /* _LS_SOC_CLOCK_H_ */