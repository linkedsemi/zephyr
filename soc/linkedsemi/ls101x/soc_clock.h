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

struct ls_clk_cfg {
	uint16_t cctl_addr_offest:5;
	uint16_t set_bit:5;
	uint16_t clr_bit:5;
};

#define LS_DT_CLK_CFG_ITEM(inst)                                             \
	{                                                                      \
	  .cctl_addr_offest = DT_PHA(DT_DRV_INST(inst), clocks, cctl_addr_offest),         \
	  .set_bit  = DT_PHA(DT_DRV_INST(inst), clocks, set_bit),                      \
	  .clr_bit  = DT_PHA(DT_DRV_INST(inst), clocks, clr_bit),                      \
	}
	
#ifdef __cplusplus
}
#endif
#endif /* _LS_SOC_CLOCK_H_ */