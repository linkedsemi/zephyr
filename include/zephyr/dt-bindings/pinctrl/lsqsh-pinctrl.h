/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_

#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/per_func_mux.h"
#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/ls_soc_gpio_def.h"

#define FUNC_NULL 0x0
#define FUNC_GPIO 0xff

#define PINMUX_FUNC0 0
#define PINMUX_FUNC1 1
#define PINMUX_FUNC2 2
#define PINMUX_FUNC3 3
#define PINMUX_FUNC_START PINMUX_FUNC0
#define PINMUX_FUNC_END PINMUX_FUNC3

/**
 * @brief Bit Masks
 */
#define LS_PIN_POS  0
#define LS_PIN_MASK 0xff

#define LS_PULL_DOWN_POS  8
#define LS_PULL_DOWN_MASK 0x1

#define LS_PULL_UP_POS  9
#define LS_PULL_UP_MASK 0x1

#define LS_PUSH_PULL_POS  10
#define LS_PUSH_PULL_MASK 0x1

#define LS_OPEN_DRAIN_POS  11
#define LS_OPEN_DRAIN_MASK 0x1

#define LS_CFG_INPUT_POS  12
#define LS_CFG_INPUT_MASK 0x1

#define LS_CFG_OUTPUT_POS  13
#define LS_CFG_OUTPUT_MASK 0x1

#define LS_OUT_HIGH_POS  14
#define LS_OUT_HIGH_MASK 0x1

#define LS_OUT_LOW_POS  15
#define LS_OUT_LOW_MASK 0x1

#define LS_DRIVE_POS  16
#define LS_DRIVE_MASK 0x3

#define LS_DIN_POS  18
#define LS_DIN_MASK 0x1

#define LS_ALT_POS  19
#define LS_ALT_MASK 0xff

#define LS_FUNC_POS  27
#define LS_FUNC_MASK 0x3

#define LSFUNC(func) (((func) << LS_FUNC_POS) & LS_FUNC_MASK)

/* Set and get macro definitions for the pin reuse configuration */
#define LS_PINMUX_SET(pin, alt_fun) \
    ((pin) << LS_PIN_POS | (alt_fun) << LS_ALT_POS)

#define LS_PINMUX_GET_PIN(pmx)        ((pmx >> LS_PIN_POS) & LS_PIN_MASK)
#define LS_PINMUX_GET_PULL_DOWN(pmx)  ((pmx >> LS_PULL_DOWN_POS) & LS_PULL_DOWN_MASK)
#define LS_PINMUX_GET_PULL_UP(pmx)    ((pmx >> LS_PULL_UP_POS) & LS_PULL_UP_MASK)
#define LS_PINMUX_GET_PUSH_PULL(pmx)  ((pmx >> LS_PUSH_PULL_POS) & LS_PUSH_PULL_MASK)
#define LS_PINMUX_GET_OPEN_DRAIN(pmx) ((pmx >> LS_OPEN_DRAIN_POS) & LS_OPEN_DRAIN_MASK)
#define LS_PINMUX_GET_OUT_HIGH(pmx)   ((pmx >> LS_OUT_HIGH_POS) & LS_OUT_HIGH_MASK)
#define LS_PINMUX_GET_OUT_LOW(pmx)    ((pmx >> LS_OUT_LOW_POS) & LS_OUT_LOW_MASK)
#define LS_PINMUX_GET_INPUT(pmx)      ((pmx >> LS_CFG_INPUT_POS) & LS_CFG_INPUT_MASK)
#define LS_PINMUX_GET_OUTPUT(pmx)     ((pmx >> LS_CFG_OUTPUT_POS) & LS_CFG_OUTPUT_MASK)
#define LS_PINMUX_GET_DRIVE(pmx)      ((pmx >> LS_DRIVE_POS) & LS_DRIVE_MASK)
#define LS_PINMUX_GET_ALT(pmx)        ((pmx >> LS_ALT_POS) & LS_ALT_MASK)
#define LS_PINMUX_GET_FUNC(pmx)        ((pmx >> LS_FUNC_POS) & LS_FUNC_MASK)
#define LS_PINMUX_GET_ANALOG(pmx)     ((pmx >> LS_ANALOG_EN_POS) & LS_ANALOG_EN_MASK)
#define LS_PINMUX_GET_LOCK(pmx)       ((pmx >> LS_LOCK_POS) & LS_LOCK_MASK)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_ */
