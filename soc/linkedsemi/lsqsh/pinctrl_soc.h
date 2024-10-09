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

typedef struct {
    union {
        uint16_t pinmux_val;
        struct {
            uint16_t pin  : 8,
                     alt  : 6,
                     func : 2;
        } pinmux_st;
    } pinmux_un;
    struct {
        uint16_t pull_down  : 1,
                 pull_up    : 1,
                 push_pull  : 1,
                 open_drain : 1,
                 cfg_input  : 1,
                 cfg_output : 1,
                 out_high   : 1,
                 out_low    : 1,
                 drive      : 2;
    } pin_attr_st;
}  __attribute__((packed)) pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node, prop, idx) \
    { \
        .pinmux_un.pinmux_val    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, pinmux), \
        .pin_attr_st.pull_down   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_down), \
        .pin_attr_st.pull_up     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up), \
        .pin_attr_st.push_pull   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_push_pull), \
        .pin_attr_st.open_drain  = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_open_drain), \
        .pin_attr_st.cfg_input   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, input_enable), \
        .pin_attr_st.cfg_output  = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_enable), \
        .pin_attr_st.out_high    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_high), \
        .pin_attr_st.out_low     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_low), \
        .pin_attr_st.drive       = DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, prop, idx), drive_strength), \
    },

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    {DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_LINKEDSEMI_PINCTRL_SOC_H_ */
