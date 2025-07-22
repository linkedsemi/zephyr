/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_H_

#include "per_func_mux.h"
#include "ls_soc_gpio_def.h"

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

#define DEFINE_PINCTRL_PRODUCER_I2C(NAME, PIN_SCL, PIN_SDA, PINMUX_SCL, PINMUX_SDA)\
    /omit-if-no-ref/NAME##_scl_##PIN_SCL: NAME##_scl_##PIN_SCL {\
        pinmux = <PINMUX_SCL>;\
        drive-strength = "quarter max driver";\
        bias-pull-up;\
        drive-open-drain;\
        input-enable;\
    };\
    /omit-if-no-ref/NAME##_sda_##PIN_SDA: NAME##_sda_##PIN_SDA {\
        pinmux = <PINMUX_SDA>;\
        drive-strength = "quarter max driver";\
        bias-pull-up;\
        drive-open-drain;\
        input-enable;\
    };\
    /omit-if-no-ref/NAME##_scl_gpio_##PIN_SCL: NAME##_scl_gpio_##PIN_SCL {\
        pinmux = <PIN_SCL>;\
        gpio;\
        bias-pull-up;\
        drive-open-drain;\
        input-enable;\
    };\
    /omit-if-no-ref/NAME##_sda_gpio_##PIN_SDA: NAME##_sda_gpio_##PIN_SDA {\
        pinmux = <PIN_SDA>;\
        gpio;\
        bias-pull-up;\
        drive-open-drain;\
        input-enable;\
    };

#define DEFINE_PINCTRL_CONSUMER_I2C(NAME, PIN_SCL, PIN_SDA, GPIO_PORT_SCL, GPIO_NUM_SCL, GPIO_PORT_SDA, GPIO_NUM_SDA)\
    scl-gpios = <&GPIO_PORT_SCL GPIO_NUM_SCL (GPIO_OPEN_DRAIN | GPIO_PULL_UP)>;\
    sda-gpios = <&GPIO_PORT_SDA GPIO_NUM_SDA (GPIO_OPEN_DRAIN | GPIO_PULL_UP)>;\
    pinctrl-0 = <&NAME##_scl_##PIN_SCL &NAME##_sda_##PIN_SDA>;\
    pinctrl-1 = <&NAME##_scl_gpio_##PIN_SCL &NAME##_sda_gpio_##PIN_SDA>;\
    pinctrl-names = "default", "priv_start";

#endif
