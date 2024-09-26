/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <ls_soc_gpio.h>

#define DT_DRV_COMPAT linkedsemi_lsqsh_pinctrl

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pinmux)
{
    uint8_t port_id, pin_id, pin, driv_stren;

    port_id = LS_PINMUX_GET_PORT(pinmux);
    __ASSERT(((port_id != 'L' - 'A')
              && (port_id != 'O' - 'A')
              && (port_id != 'P' - 'A')),
             "PLXX/POXX/PPXX is not exist");
    if (port_id >= ('O' - 'A')) {
        port_id -= 3;
    } else if (port_id >= ('L' - 'A')) {
        port_id--;
    }

    pin_id = LS_PINMUX_GET_PIN(pinmux);
    pin = LSPIN(port_id, pin_id);

    if (LS_PINMUX_GET_PULL_DOWN(pinmux)) {
        io_pull_write(pin, IO_PULL_DOWN);
    }

    if (LS_PINMUX_GET_PULL_UP(pinmux)) {
        io_pull_write(pin, IO_PULL_UP);
    }

    if (LS_PINMUX_GET_INPUT(pinmux)) {
        io_cfg_input(pin);
    }

    if (LS_PINMUX_GET_OUTPUT(pinmux)) {
        io_cfg_output(pin);
    }

    if (LS_PINMUX_GET_OPEN_DRAIN(pinmux)) {
        io_cfg_opendrain(pin);
    }

    if (LS_PINMUX_GET_PUSH_PULL(pinmux)) {
        io_cfg_pushpull(pin);
    }

    /* only has effect if mode is push_pull */
    if (LS_PINMUX_GET_OUT_HIGH(pinmux)) {
        io_set_pin(pin);
    }

    /* only has effect if mode is push_pull */
    if (LS_PINMUX_GET_OUT_LOW(pinmux)) {
        io_clr_pin(pin);
    }

    /* only has effect if mode is push_pull */
    driv_stren = LS_PINMUX_GET_DRIVE(pinmux);
    io_drive_capacity_write(pin, driv_stren);

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
