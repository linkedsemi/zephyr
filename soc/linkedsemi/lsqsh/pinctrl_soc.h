/*
 * Copyright (c) 2024 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * LS SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_RISCV_LINKEDSEMI_PINCTRL_SOC_H_
#define ZEPHYR_SOC_RISCV_LINKEDSEMI_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    uint32_t pin_attr_val;
    struct {
        uint32_t pin        : 8,  // LS_PIN_POS (4), LS_PIN_MASK (0xff)
                 pull_down  : 1,  // LS_PULL_DOWN_POS (8), LS_PULL_DOWN_MASK (0x1)
                 pull_up    : 1,  // LS_PULL_UP_POS (9), LS_PULL_UP_MASK (0x1)
                 push_pull  : 1,  // LS_PUSH_PULL_POS (10), LS_PUSH_PULL_MASK (0x1)
                 open_drain : 1,  // LS_OPEN_DRAIN_POS (11), LS_OPEN_DRAIN_MASK (0x1)
                 cfg_input  : 1,  // LS_CFG_INPUT_POS (12), LS_CFG_INPUT_MASK (0x1)
                 cfg_output : 1,  // LS_CFG_OUTPUT_POS (13), LS_CFG_OUTPUT_MASK (0x1)
                 out_high   : 1,  // LS_OUT_HIGH_POS (14), LS_OUT_HIGH_MASK (0x1)
                 out_low    : 1,  // LS_OUT_LOW_POS (15), LS_OUT_LOW_MASK (0x1)
                 drive      : 2,  // LS_DRIVE_POS (16), LS_DRIVE_MASK (0x3)
                 din        : 1,  // LS_DIN_POS (18), LS_DIN_MASK (0x1)
                 alt        : 8,  // LS_ALT_POS (19), LS_ALT_MASK (0xff)
                 func       : 2;  // LS_FUNC_POS (27), LS_FUNC_MASK (0x3)
    } pin_attr_st;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node, pr, idx)                                           \
	(DT_PROP_BY_PHANDLE_IDX(node, pr, idx, pinmux) |                                      \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, bias_pull_down) << LS_PULL_DOWN_POS |          \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, bias_pull_up) << LS_PULL_UP_POS |              \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, drive_push_pull) << LS_PUSH_PULL_POS |         \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, drive_open_drain) << LS_OPEN_DRAIN_POS |       \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_high) << LS_OUT_HIGH_POS |              \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_low) << LS_OUT_LOW_POS |                \
	 DT_PROP_BY_PHANDLE_IDX(node, pr, idx, input_enable) << LS_CFG_INPUT_POS |            \
     DT_PROP_BY_PHANDLE_IDX(node, pr, idx, output_enable) << LS_CFG_OUTPUT_POS |          \
	 DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength) << LS_DRIVE_POS),			  

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)			       \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_LINKEDSEMI_PINCTRL_SOC_H_ */