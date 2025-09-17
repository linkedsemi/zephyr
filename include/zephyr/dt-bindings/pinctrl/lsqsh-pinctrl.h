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

//GPIOA
#define GPIO_PORT_PA00 gpioa
#define GPIO_PORT_PA01 gpioa
#define GPIO_PORT_PA02 gpioa
#define GPIO_PORT_PA03 gpioa
#define GPIO_PORT_PA04 gpioa
#define GPIO_PORT_PA05 gpioa
#define GPIO_PORT_PA06 gpioa
#define GPIO_PORT_PA07 gpioa
#define GPIO_PORT_PA08 gpioa
#define GPIO_PORT_PA09 gpioa
#define GPIO_PORT_PA10 gpioa
#define GPIO_PORT_PA11 gpioa
#define GPIO_PORT_PA12 gpioa
#define GPIO_PORT_PA13 gpioa
#define GPIO_PORT_PA14 gpioa
#define GPIO_PORT_PA15 gpioa
//GPIOB
#define GPIO_PORT_PB00 gpiob
#define GPIO_PORT_PB01 gpiob
#define GPIO_PORT_PB02 gpiob
#define GPIO_PORT_PB03 gpiob
#define GPIO_PORT_PB04 gpiob
#define GPIO_PORT_PB05 gpiob
#define GPIO_PORT_PB06 gpiob
#define GPIO_PORT_PB07 gpiob
#define GPIO_PORT_PB08 gpiob
#define GPIO_PORT_PB09 gpiob
#define GPIO_PORT_PB10 gpiob
#define GPIO_PORT_PB11 gpiob
#define GPIO_PORT_PB12 gpiob
#define GPIO_PORT_PB13 gpiob
#define GPIO_PORT_PB14 gpiob
#define GPIO_PORT_PB15 gpiob
//GPIOC
#define GPIO_PORT_PC00 gpioc
#define GPIO_PORT_PC01 gpioc
#define GPIO_PORT_PC02 gpioc
#define GPIO_PORT_PC03 gpioc
#define GPIO_PORT_PC04 gpioc
#define GPIO_PORT_PC05 gpioc
#define GPIO_PORT_PC06 gpioc
#define GPIO_PORT_PC07 gpioc
#define GPIO_PORT_PC08 gpioc
#define GPIO_PORT_PC09 gpioc
#define GPIO_PORT_PC10 gpioc
#define GPIO_PORT_PC11 gpioc
#define GPIO_PORT_PC12 gpioc
#define GPIO_PORT_PC13 gpioc
#define GPIO_PORT_PC14 gpioc
#define GPIO_PORT_PC15 gpioc
//GPIOD
#define GPIO_PORT_PD00 gpiod
#define GPIO_PORT_PD01 gpiod
#define GPIO_PORT_PD02 gpiod
#define GPIO_PORT_PD03 gpiod
#define GPIO_PORT_PD04 gpiod
#define GPIO_PORT_PD05 gpiod
#define GPIO_PORT_PD06 gpiod
#define GPIO_PORT_PD07 gpiod
#define GPIO_PORT_PD08 gpiod
#define GPIO_PORT_PD09 gpiod
#define GPIO_PORT_PD10 gpiod
#define GPIO_PORT_PD11 gpiod
#define GPIO_PORT_PD12 gpiod
#define GPIO_PORT_PD13 gpiod
#define GPIO_PORT_PD14 gpiod
#define GPIO_PORT_PD15 gpiod
//GPIOE
#define GPIO_PORT_PE00 gpioe
#define GPIO_PORT_PE01 gpioe
#define GPIO_PORT_PE02 gpioe
#define GPIO_PORT_PE03 gpioe
#define GPIO_PORT_PE04 gpioe
#define GPIO_PORT_PE05 gpioe
#define GPIO_PORT_PE06 gpioe
#define GPIO_PORT_PE07 gpioe
#define GPIO_PORT_PE08 gpioe
#define GPIO_PORT_PE09 gpioe
#define GPIO_PORT_PE10 gpioe
#define GPIO_PORT_PE11 gpioe
#define GPIO_PORT_PE12 gpioe
#define GPIO_PORT_PE13 gpioe
#define GPIO_PORT_PE14 gpioe
#define GPIO_PORT_PE15 gpioe
//GPIOF
#define GPIO_PORT_PF00 gpiof
#define GPIO_PORT_PF01 gpiof
#define GPIO_PORT_PF02 gpiof
#define GPIO_PORT_PF03 gpiof
#define GPIO_PORT_PF04 gpiof
#define GPIO_PORT_PF05 gpiof
#define GPIO_PORT_PF06 gpiof
#define GPIO_PORT_PF07 gpiof
#define GPIO_PORT_PF08 gpiof
#define GPIO_PORT_PF09 gpiof
#define GPIO_PORT_PF10 gpiof
#define GPIO_PORT_PF11 gpiof
#define GPIO_PORT_PF12 gpiof
#define GPIO_PORT_PF13 gpiof
#define GPIO_PORT_PF14 gpiof
#define GPIO_PORT_PF15 gpiof
//GPIOG
#define GPIO_PORT_PG00 gpiog
#define GPIO_PORT_PG01 gpiog
#define GPIO_PORT_PG02 gpiog
#define GPIO_PORT_PG03 gpiog
#define GPIO_PORT_PG04 gpiog
#define GPIO_PORT_PG05 gpiog
#define GPIO_PORT_PG06 gpiog
#define GPIO_PORT_PG07 gpiog
#define GPIO_PORT_PG08 gpiog
#define GPIO_PORT_PG09 gpiog
#define GPIO_PORT_PG10 gpiog
#define GPIO_PORT_PG11 gpiog
#define GPIO_PORT_PG12 gpiog
#define GPIO_PORT_PG13 gpiog
#define GPIO_PORT_PG14 gpiog
#define GPIO_PORT_PG15 gpiog
//GPIOH
#define GPIO_PORT_PH00 gpioh
#define GPIO_PORT_PH01 gpioh
#define GPIO_PORT_PH02 gpioh
#define GPIO_PORT_PH03 gpioh
#define GPIO_PORT_PH04 gpioh
#define GPIO_PORT_PH05 gpioh
#define GPIO_PORT_PH06 gpioh
#define GPIO_PORT_PH07 gpioh
#define GPIO_PORT_PH08 gpioh
#define GPIO_PORT_PH09 gpioh
#define GPIO_PORT_PH10 gpioh
#define GPIO_PORT_PH11 gpioh
#define GPIO_PORT_PH12 gpioh
#define GPIO_PORT_PH13 gpioh
#define GPIO_PORT_PH14 gpioh
#define GPIO_PORT_PH15 gpioh
//GPIOI
#define GPIO_PORT_PI00 gpioi
#define GPIO_PORT_PI01 gpioi
#define GPIO_PORT_PI02 gpioi
#define GPIO_PORT_PI03 gpioi
#define GPIO_PORT_PI04 gpioi
#define GPIO_PORT_PI05 gpioi
#define GPIO_PORT_PI06 gpioi
#define GPIO_PORT_PI07 gpioi
#define GPIO_PORT_PI08 gpioi
#define GPIO_PORT_PI09 gpioi
#define GPIO_PORT_PI10 gpioi
#define GPIO_PORT_PI11 gpioi
#define GPIO_PORT_PI12 gpioi
#define GPIO_PORT_PI13 gpioi
#define GPIO_PORT_PI14 gpioi
#define GPIO_PORT_PI15 gpioi
//GPIOJ
#define GPIO_PORT_PJ00 gpioj
#define GPIO_PORT_PJ01 gpioj
#define GPIO_PORT_PJ02 gpioj
#define GPIO_PORT_PJ03 gpioj
#define GPIO_PORT_PJ04 gpioj
#define GPIO_PORT_PJ05 gpioj
#define GPIO_PORT_PJ06 gpioj
#define GPIO_PORT_PJ07 gpioj
#define GPIO_PORT_PJ08 gpioj
#define GPIO_PORT_PJ09 gpioj
#define GPIO_PORT_PJ10 gpioj
#define GPIO_PORT_PJ11 gpioj
#define GPIO_PORT_PJ12 gpioj
#define GPIO_PORT_PJ13 gpioj
#define GPIO_PORT_PJ14 gpioj
#define GPIO_PORT_PJ15 gpioj
//GPIOK
#define GPIO_PORT_PK00 gpiok
#define GPIO_PORT_PK01 gpiok
#define GPIO_PORT_PK02 gpiok
#define GPIO_PORT_PK03 gpiok
#define GPIO_PORT_PK04 gpiok
#define GPIO_PORT_PK05 gpiok
#define GPIO_PORT_PK06 gpiok
#define GPIO_PORT_PK07 gpiok
#define GPIO_PORT_PK08 gpiok
#define GPIO_PORT_PK09 gpiok
#define GPIO_PORT_PK10 gpiok
#define GPIO_PORT_PK11 gpiok
#define GPIO_PORT_PK12 gpiok
#define GPIO_PORT_PK13 gpiok
#define GPIO_PORT_PK14 gpiok
#define GPIO_PORT_PK15 gpiok
//GPIOM
#define GPIO_PORT_PM00 gpiom
#define GPIO_PORT_PM01 gpiom
#define GPIO_PORT_PM02 gpiom
#define GPIO_PORT_PM03 gpiom
#define GPIO_PORT_PM04 gpiom
#define GPIO_PORT_PM05 gpiom
#define GPIO_PORT_PM06 gpiom
#define GPIO_PORT_PM07 gpiom
#define GPIO_PORT_PM08 gpiom
#define GPIO_PORT_PM09 gpiom
#define GPIO_PORT_PM10 gpiom
#define GPIO_PORT_PM11 gpiom
#define GPIO_PORT_PM12 gpiom
#define GPIO_PORT_PM13 gpiom
#define GPIO_PORT_PM14 gpiom
#define GPIO_PORT_PM15 gpiom
//GPION
#define GPIO_PORT_PN00 gpion
#define GPIO_PORT_PN01 gpion
#define GPIO_PORT_PN02 gpion
#define GPIO_PORT_PN03 gpion
#define GPIO_PORT_PN04 gpion
#define GPIO_PORT_PN05 gpion
#define GPIO_PORT_PN06 gpion
#define GPIO_PORT_PN07 gpion
#define GPIO_PORT_PN08 gpion
#define GPIO_PORT_PN09 gpion
#define GPIO_PORT_PN10 gpion
#define GPIO_PORT_PN11 gpion
#define GPIO_PORT_PN12 gpion
#define GPIO_PORT_PN13 gpion
#define GPIO_PORT_PN14 gpion
#define GPIO_PORT_PN15 gpion
//GPIOQ
#define GPIO_PORT_PQ00 gpioq
#define GPIO_PORT_PQ01 gpioq
#define GPIO_PORT_PQ02 gpioq
#define GPIO_PORT_PQ03 gpioq
#define GPIO_PORT_PQ04 gpioq
#define GPIO_PORT_PQ05 gpioq
#define GPIO_PORT_PQ06 gpioq
#define GPIO_PORT_PQ07 gpioq
#define GPIO_PORT_PQ08 gpioq
#define GPIO_PORT_PQ09 gpioq
#define GPIO_PORT_PQ10 gpioq
#define GPIO_PORT_PQ11 gpioq
#define GPIO_PORT_PQ12 gpioq
#define GPIO_PORT_PQ13 gpioq
#define GPIO_PORT_PQ14 gpioq
#define GPIO_PORT_PQ15 gpioq
//GPIOT
#define GPIO_PORT_PT00 gpiot
#define GPIO_PORT_PT01 gpiot
#define GPIO_PORT_PT02 gpiot
#define GPIO_PORT_PT03 gpiot
#define GPIO_PORT_PT04 gpiot
#define GPIO_PORT_PT05 gpiot
#define GPIO_PORT_PT06 gpiot
#define GPIO_PORT_PT07 gpiot
#define GPIO_PORT_PT08 gpiot
#define GPIO_PORT_PT09 gpiot
#define GPIO_PORT_PT10 gpiot
#define GPIO_PORT_PT11 gpiot
#define GPIO_PORT_PT12 gpiot
#define GPIO_PORT_PT13 gpiot
#define GPIO_PORT_PT14 gpiot
#define GPIO_PORT_PT15 gpiot

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
        bias-pull-up;\
        drive-open-drain;\
        input-enable;\
    };\
    /omit-if-no-ref/NAME##_sda_##PIN_SDA: NAME##_sda_##PIN_SDA {\
        pinmux = <PINMUX_SDA>;\
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

#define GPIO_NUM_GEN(PIN) (PIN & 0xf)

#define DEFINE_PINCTRL_CONSUMER_I2C(NAME, PIN_SCL, PIN_SDA)\
    scl-gpios = <&GPIO_PORT_##PIN_SCL GPIO_NUM_GEN(PIN_SCL) (GPIO_OPEN_DRAIN | GPIO_PULL_UP)>;\
    sda-gpios = <&GPIO_PORT_##PIN_SDA GPIO_NUM_GEN(PIN_SDA) (GPIO_OPEN_DRAIN | GPIO_PULL_UP)>;\
    pinctrl-0 = <&NAME##_scl_##PIN_SCL &NAME##_sda_##PIN_SDA>;\
    pinctrl-1 = <&NAME##_scl_gpio_##PIN_SCL &NAME##_sda_gpio_##PIN_SDA>;\
    pinctrl-names = "default", "priv_start";

#endif
