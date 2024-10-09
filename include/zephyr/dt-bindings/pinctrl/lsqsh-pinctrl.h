/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_

#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/per_func_mux.h"
#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/ls_soc_gpio_def.h"

/**
 * @brief Bit Masks
 */
#define LS_PIN_POS  0
#define LS_PIN_MASK 0xff

#define LS_ALT_POS  8
#define LS_ALT_MASK 0x63

#define LS_FUNC_POS  14
#define LS_FUNC_MASK 0x3

#define LSFUNC(func) (((func) & LS_FUNC_MASK) << (LS_FUNC_POS - LS_ALT_POS))

/* Set and get macro definitions for the pin reuse configuration */
#define LS_PINMUX_SET(pin, alt_fun) \
    ((pin) << LS_PIN_POS | ((alt_fun) << LS_ALT_POS))

#define FUNC_NULL 0x0
#define FUNC_GPIO LS_ALT_MASK

#define PINMUX_FUNC0 0
#define PINMUX_FUNC1 1
#define PINMUX_FUNC2 2
#define PINMUX_FUNC3 3
#define PINMUX_FUNC_START PINMUX_FUNC0
#define PINMUX_FUNC_END PINMUX_FUNC3

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_ */
