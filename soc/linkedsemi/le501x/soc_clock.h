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

#define DT_HAS_CLOCKS(inst)    DT_NODE_HAS_PROP(DT_DRV_INST(inst), clocks)

struct ls_clk_cfg {
    const struct device *cctl_dev;
    uint8_t bus;
    uint8_t reset;
    uint8_t pos;
};

#define LS_DT_CLK_CFG_ITEM(inst)                                             \
    {                                                                      \
      .cctl_dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(DT_DRV_INST(inst),clocks,0)), \
      .bus = DT_PHA(DT_DRV_INST(inst), clocks, bus),         \
      .reset  = DT_PHA(DT_DRV_INST(inst), clocks, reset),                      \
      .pos  = DT_PHA(DT_DRV_INST(inst), clocks, pos),                      \
    }

#ifdef __cplusplus
}
#endif
#endif /* _LS_SOC_CLOCK_H_ */