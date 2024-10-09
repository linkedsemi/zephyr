/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <reg_sysc_awo.h>
#include <reg_sysc_per.h>
#include <ls_soc_gpio.h>

#define DT_DRV_COMPAT linkedsemi_lsqsh_pinctrl

static void pinctrl_configure_pin_func(uint8_t pin, uint8_t func, uint32_t alt)
{
    switch (func) {
    case PINMUX_FUNC0:
        per_func0_set(pin, alt);
        __fallthrough;
    case PINMUX_FUNC1:
        __fallthrough;
    case PINMUX_FUNC2:
        __fallthrough;
    case PINMUX_FUNC3:
        switch (alt) {
        case FUNC_NULL:
            io_cfg_disable(pin);
            __fallthrough;
        case FUNC_GPIO:
            for (uint8_t i = PINMUX_FUNC_START; i <= PINMUX_FUNC_END; i++) {
                per_func_disable(pin, PINMUX_FUNC0);
            }
            break;
        default:
            for (uint8_t i = PINMUX_FUNC_START; i <= PINMUX_FUNC_END; i++) {
                if (func == i) {
                    per_func_enable(pin, i);
                } else {
                    per_func_disable(pin, i);
                }
            }
            break;
        }
        break;
    default:
        break;
    }
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pinmux)
{
    uint8_t pin = 0;

    pin = pinmux.pinmux_un.pinmux_st.pin;

    if (pinmux.pin_attr_st.pull_down) {
        io_pull_write(pin, IO_PULL_DOWN);
    }

    if (pinmux.pin_attr_st.pull_up) {
        io_pull_write(pin, IO_PULL_UP);
    }

    if (pinmux.pin_attr_st.cfg_input) {
        io_cfg_input(pin);
    }

    if (pinmux.pin_attr_st.cfg_output) {
        io_cfg_output(pin);
    }

    if (pinmux.pin_attr_st.open_drain) {
        io_cfg_opendrain(pin);
    }

    if (pinmux.pin_attr_st.push_pull) {
        io_cfg_pushpull(pin);
    }

    /* only has effect if mode is push_pull */
    if (pinmux.pin_attr_st.out_high) {
        io_set_pin(pin);
    }

    /* only has effect if mode is push_pull */
    if (pinmux.pin_attr_st.out_low) {
        io_clr_pin(pin);
    }

    /* only has effect if mode is push_pull */
    io_drive_capacity_write(pin, pinmux.pin_attr_st.drive);

    pinctrl_configure_pin_func(pin,
                                pinmux.pinmux_un.pinmux_st.func,
                                pinmux.pinmux_un.pinmux_st.alt);

    return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
    ARG_UNUSED(reg);

    for (uint8_t i = 0U; i < pin_cnt; i++) {
        int ret = pinctrl_configure_pin(*pins++);

        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}
