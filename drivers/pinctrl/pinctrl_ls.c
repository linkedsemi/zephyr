/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/ls-pinctrl.h>

#include <ls_soc_gpio.h>
#include <field_manipulate.h>

#define DT_DRV_COMPAT linkedsemi_ls_pinctrl

static void gpio_afs_init(gpio_pins_t *pin, uint8_t af)
{
    reg_lsgpio_t *port = GPIO_GetPort(pin->port);
    switch(pin->num)
    {
        case 0: case 1: case 2: case 3:
            MODIFY_REG(port->AF0, GPIO_IO0_AF_MASK << pin->num * 8, af << pin->num * 8);
        break;
        case 4: case 5: case 6: case 7:
            MODIFY_REG(port->AF1, GPIO_IO4_AF_MASK << (pin->num - 4) * 8, af << (pin->num - 4) * 8);
        break;
        case 8: case 9: case 10: case 11:
            MODIFY_REG(port->AF2, GPIO_IO8_AF_MASK << (pin->num - 8) * 8, af << (pin->num - 8) * 8);
        break;
        case 12: case 13: case 14: case 15:
            MODIFY_REG(port->AF3, GPIO_IO12_AF_MASK << (pin->num - 12) * 8, af << (pin->num - 12) * 8);
        break;
    }
    MODIFY_REG(port->MODE, GPIO_MODE0_MASK << (pin->num << 1u), SET_GPIO_MODE_AF << (pin->num << 1u));
}

static void gpio_analog_init(gpio_pins_t *pin, uint8_t ana)
{
   reg_lsgpio_t *port = GPIO_GetPort(pin->port);
   MODIFY_REG(port->AE, GPIO_AE0_MASK << (pin->num << 1u), ana << (pin->num << 1u));
   MODIFY_REG(port->MODE, GPIO_MODE0_MASK << (pin->num << 1u), SET_GPIO_MODE_ANALOG << (pin->num << 1u));
  
}

static int pinctrl_configure_pin(const pinctrl_soc_pin_t pinmux)
{
	uint8_t port_id, pin_id, alt_fun, pin, driv_stren;

    port_id = LS_PINMUX_GET_PORT(pinmux); 
    pin_id = LS_PINMUX_GET_PIN(pinmux);
    pin = LSPIN(port_id, pin_id);                        

	if (LS_PINMUX_GET_PULL_DOWN(pinmux)) {
		io_pull_write(pin, IO_PULL_DOWN);
	}

	if (LS_PINMUX_GET_PULL_UP(pinmux)) {
		io_pull_write(pin,IO_PULL_UP);
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

	alt_fun = LS_PINMUX_GET_ALT(pinmux);

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

    if (alt_fun < 64 ) {
        gpio_afs_init((gpio_pins_t *)&pin, alt_fun);
    } else {
        gpio_analog_init((gpio_pins_t *)&pin, (alt_fun - 64));
    }

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
