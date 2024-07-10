/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <field_manipulate.h>
#include <ls_soc_gpio.h>

#if CONFIG_SOC_SERIES_LE501X == 1

#include <zephyr/dt-bindings/pinctrl/ls-pinctrl.h>

#define DT_DRV_COMPAT linkedsemi_ls_pinctrl

static void gpio_afs_init(gpio_port_pin_t *pin, uint8_t af)
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

static void gpio_analog_init(gpio_port_pin_t *pin, uint8_t ana)
{
   reg_lsgpio_t *port = GPIO_GetPort(pin->port);
   MODIFY_REG(port->AE, GPIO_AE0_MASK << (pin->num << 1u), ana << (pin->num << 1u));
   MODIFY_REG(port->MODE, GPIO_MODE0_MASK << (pin->num << 1u), SET_GPIO_MODE_ANALOG << (pin->num << 1u));
  
}

#elif CONFIG_SOC_SERIES_LS101X == 1

#include <zephyr/dt-bindings/pinctrl/ls101x-pinctrl.h>
#include "reg_sysc_awo.h"
#include "reg_sysc_per.h"

#define DT_DRV_COMPAT linkedsemi_ls101x_pinctrl

static void per_func_enable(uint8_t func_io_num,uint8_t per_func)
{
    MODIFY_REG(SYSC_PER->FUNC_SEL[func_io_num/4],0xff<<8*(func_io_num%4),per_func<<8*(func_io_num%4));
    if(func_io_num >= 96)
    {
        SYSC_AWO->PIN_SEL4 |= 1<<(func_io_num-96);
    }else if(func_io_num >= 64)
    {
        SYSC_AWO->PIN_SEL3 |= 1<<(func_io_num-64);
    }else if(func_io_num >= 32)
    {
        SYSC_AWO->PIN_SEL2 |= 1<<(func_io_num-32);
    }else
    {
        SYSC_AWO->PIN_SEL1 |= 1<<func_io_num;
    }
}

void gpio_analog_func1_init(uint8_t pin)
{
    gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
    SYSC_AWO->IO[x->port].AE |= 1<<16<<x->num;
}

void gpio_analog_func2_init(uint8_t pin)
{
    gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
    SYSC_AWO->IO[x->port].AE |= 1<<x->num;
}
#endif


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

#if CONFIG_SOC_SERIES_LE501X == 1
    if (alt_fun < 64 ) {
        gpio_afs_init((gpio_port_pin_t *)&pin, alt_fun);
    } else {
        gpio_analog_init((gpio_port_pin_t *)&pin, (alt_fun - 64));
    }

#elif CONFIG_SOC_SERIES_LS101X == 1
	if(alt_fun < 164){
        per_func_enable(pin, alt_fun);
    }else{
        gpio_analog_func1_init(pin);
    }
#endif

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
