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
        volatile uint16_t value;
        struct {
            volatile uint16_t pin  : 8, /*[0-7]*/
                              alt  : 6, /*[8-13]*/
                              func : 2; /*[14-15]*/
        } field;
    } pinmux_un;
    union {
        volatile uint16_t value;
        struct {
            volatile uint16_t pull_down  : 1, /*[0]*/
                              pull_up    : 1, /*[1]*/
                              push_pull  : 1, /*[2]*/
                              open_drain : 1, /*[3]*/
                              cfg_input  : 1, /*[4]*/
                              cfg_output : 1, /*[5]*/
                              out_high   : 1, /*[6]*/
                              out_low    : 1, /*[7]*/
                              drive      : 2, /*[8-9]*/
                              reserve0   : 6; /*[10-15]*/
        } field;
    } pin_attr_un;
}  __attribute__((packed)) pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node, prop, idx) \
    { \
        .pinmux_un.value               = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, pinmux), \
        .pin_attr_un.field.pull_down   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_down), \
        .pin_attr_un.field.pull_up     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up), \
        .pin_attr_un.field.push_pull   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_push_pull), \
        .pin_attr_un.field.open_drain  = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_open_drain), \
        .pin_attr_un.field.cfg_input   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, input_enable), \
        .pin_attr_un.field.cfg_output  = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_enable), \
        .pin_attr_un.field.out_high    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_high), \
        .pin_attr_un.field.out_low     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_low), \
        .pin_attr_un.field.drive       = DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, prop, idx), drive_strength), \
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
