/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * LS SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_ARM_LINKEDSEMI_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_LINKEDSEMI_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/ls-pinctrl.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t pinctrl_soc_pin_t;

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
	 DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, pr, idx), drive_strength) << LS_DRIVE_POS)

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

#endif /* ZEPHYR_SOC_ARM_LINKEDSEMI_PINCTRL_SOC_H_ */