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

typedef struct __attribute__((packed)) {
    union {
        uint16_t value;
        struct {
            uint16_t
                pin        : 8, /*[0-7]*/
                func       : 2, /*[8-9]*/
                alt        : 5, /*[10-14]*/
                func_valid : 1; /*[15]*/
        };
    } pinmux;
    union {
        uint32_t value;
        struct {
            uint32_t
                lock             : 1,
                bias_pull_up     : 1,
                bias_pull_up0    : 1,
                bias_pull_up1    : 1,
                bias_pull_up2    : 1,
                bias_pull_down   : 1,
                drive_push_pull  : 1,
                drive_open_drain : 1,
                input_enable     : 1,
                input_1v8_enable : 1,
                output_enable    : 1,
                analog           : 1,
                input_filter     : 1,
                st               : 1,
                sl               : 1,
                drive_strength   : 3,
                output_high      : 1,
                output_low       : 1,
                gpio             : 1,
                disable_all      : 1;
        };
    } pin_attr;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize pincfg field in #pinctrl_pin_t.
 *
 * @param node_id Node identifier.
 */
#define Z_PINCTRL_STATE_PIN_INIT(node, prop, idx) \
    { \
        .pinmux.value              = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, pinmux), \
        .pin_attr.lock             = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, lock), \
        .pin_attr.bias_pull_up     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up), \
        .pin_attr.bias_pull_up0    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up0), \
        .pin_attr.bias_pull_up1    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up1), \
        .pin_attr.bias_pull_up2    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_up2), \
        .pin_attr.bias_pull_down   = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, bias_pull_down), \
        .pin_attr.drive_push_pull  = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_push_pull), \
        .pin_attr.drive_open_drain = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, drive_open_drain), \
        .pin_attr.input_enable     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, input_enable), \
        .pin_attr.input_1v8_enable = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, input_1v8_enable), \
        .pin_attr.output_enable    = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_enable), \
        .pin_attr.analog           = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, analog), \
        .pin_attr.input_filter     = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, input_filter), \
        .pin_attr.st               = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, st), \
        .pin_attr.sl               = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, sl), \
        .pin_attr.drive_strength   = DT_ENUM_IDX(DT_PHANDLE_BY_IDX(node, prop, idx), drive_strength), \
        .pin_attr.output_high      = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_high), \
        .pin_attr.output_low       = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, output_low), \
        .pin_attr.gpio             = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, gpio), \
        .pin_attr.disable_all      = DT_PROP_BY_PHANDLE_IDX(node, prop, idx, disable_all), \
    },

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop) \
    {DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

static inline uint16_t pinctrl_pin2code(const pinctrl_soc_pin_t *pin)
{
    return pin->pinmux.pin;
}

/** @endcond */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_RISCV_LINKEDSEMI_PINCTRL_SOC_H_ */
