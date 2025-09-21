/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/lsqsh-pinctrl.h>
#include <ls_soc_gpio.h>

#define DT_DRV_COMPAT linkedsemi_lsqsh_pinctrl

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pin_desc)
{
    uint8_t pin = 0;

    pin = pin_desc.pinmux.pin;

    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_cfg_lock(pin, false);))
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_cfg_app_input_lock(pin, false);))
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_func_cfg_lock(pin, false);))

    if (pin_desc.pin_attr.bias_pull_down) {
        io_pull_write(pin, IO_PULL_DOWN);
    }

    if (pin_desc.pin_attr.bias_pull_up) {
        io_pull_write(pin, IO_PULL_UP);
    }

    if (pin_desc.pin_attr.bias_pull_up0) {
        io_pull_write(pin, IO_PULL_UP0);
    }

    if (pin_desc.pin_attr.bias_pull_up1) {
        io_pull_write(pin, IO_PULL_UP1);
    }

    if (pin_desc.pin_attr.bias_pull_up2) {
        io_pull_write(pin, IO_PULL_UP2);
    }

    if (pin_desc.pin_attr.input_enable) {
        io_cfg_input_pure(pin);
    } else {
        io_cfg_disable_input(pin);
    }

    // if (pin_desc.pin_attr.input_1v8_enable) {
    //     io_cfg_input_1v8_pure(pin);
    // }

    if (pin_desc.pin_attr.output_enable) {
        io_cfg_output(pin);
    } else {
        io_cfg_disable_output(pin);
    }

    if (pin_desc.pin_attr.drive_open_drain) {
        io_cfg_opendrain(pin);
    }

    if (pin_desc.pin_attr.drive_push_pull) {
        io_cfg_pushpull(pin);
    }

    /* only has effect if mode is drive_push_pull */
    if (pin_desc.pin_attr.output_high) {
        io_set_pin(pin);
    }

    /* only has effect if mode is drive_push_pull */
    if (pin_desc.pin_attr.output_low) {
        io_clr_pin(pin);
    }

    if (pin_desc.pin_attr.analog) {
        gpio_ana_init(pin);
    } else {
        gpio_ana_deinit(pin);
    }

    /* only has effect if mode is drive_push_pull */
    io_drive_capacity_write(pin, pin_desc.pin_attr.drive_strength);

    if (pin_desc.pinmux.func_valid) {
        pinmux_cfg_pin_func_alt(pin,
                                    pin_desc.pinmux.func,
                                    pin_desc.pinmux.alt);
    } else {
        per_func_disable_all(pin);
    }

    /* TODO: check */
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_func_cfg_lock(pin, true);))
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_cfg_lock(pin, true);))
    IF_ENABLED(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay), (io_cfg_app_input_lock(pin, true);))

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
