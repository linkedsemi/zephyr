/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_

#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/include/per_func_mux.h"
#include "../../../../../modules/hal/linkedsemi/soc/rv32/qsh/include/ls_soc_gpio_def.h"

/**
 * @brief Bit Masks
 */
#define LS_PIN_POS  0
#define LS_PIN_MASK 0xff

#define LS_FUNC_POS  8
#define LS_FUNC_MASK 0x3

#define LS_ALT_POS  10
#define LS_ALT_MASK 0x3f

#include "lsqsh-pinctrl_func_1.h"
#include "lsqsh-pinctrl_func_2_3_4.h"

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_ */


/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * This header provides constants for most IRQ bindings.
 *
 * Most IRQ bindings include a flags cell as part of the IRQ specifier.
 * In most cases, the format of the flags cell uses the standard values
 * defined in this header.
 */

#ifndef _DT_BINDINGS_INTERRUPT_CONTROLLER_IRQ_H
#define _DT_BINDINGS_INTERRUPT_CONTROLLER_IRQ_H

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	1
#define IRQ_TYPE_EDGE_FALLING	2
#define IRQ_TYPE_EDGE_BOTH	(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH	4
#define IRQ_TYPE_LEVEL_LOW	8

#endif
