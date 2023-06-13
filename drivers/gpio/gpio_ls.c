/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/linkedsemi-ls-gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include <ls_soc_gpio.h>
#include <reg_gpio.h>

#include <zephyr/irq.h>
#include <errno.h>

#define LSPIN(_port, _pin)      (_port << 4 | _pin)

struct gpio_ls_config {
	struct gpio_driver_config common;
	uint32_t *base;
};

struct gpio_ls_data {
	struct gpio_driver_data common;
	const struct device *dev;
	sys_slist_t callbacks;
};

void io_wkup_en_clr_set(uint8_t pin);
static int gpio_ls_port_set_bits_raw(const struct device *dev,
					gpio_port_pins_t pins);
static int gpio_ls_port_clear_bits_raw(const struct device *dev,
					gpio_port_pins_t pins);

static int get_gpio_port_id(uint32_t port)
{
    uint8_t port_id;

    switch (port) {
    /* port A base */
    case 0x48000000:
        port_id = 0;
        break;
    /* port B base */
    case 0x48000400:
        port_id = 1;
        break;
    /* port C base */
    case 0x48000800:
        port_id = 2;
        break;
    default:
        return -ENOTSUP;
    }

    return port_id;
}

static int gpio_ls_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;
    uint8_t pinval, port;
    
    port = get_gpio_port_id((uint32_t)gpio);
    pinval = LSPIN(port, pin);
	
    switch (flags & (GPIO_INPUT | GPIO_OUTPUT)) {
	case GPIO_INPUT:
		io_cfg_input(pinval);
		break;
	case GPIO_OUTPUT:
		io_cfg_output(pinval);
		break;
	case GPIO_DISCONNECTED:
        io_pull_write(pinval, IO_PULL_DISABLE);
		io_cfg_disable(pinval);
        break;
	default:
		return -ENOTSUP;
	}

    switch (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
	case 0:
		io_pull_write(pinval, IO_PULL_DISABLE);
		break;
	case GPIO_PULL_UP:
		io_pull_write(pinval, IO_PULL_UP);
		break;
	case GPIO_PULL_DOWN:
		io_pull_write(pinval, IO_PULL_DOWN);
		break;
	default:
		return -EINVAL;
	}

    switch (flags & LS_GPIO_DS_MASK) {
    case LS_GPIO_DS_DEFAULT_DRIVE:
	case LS_GPIO_DS_QUARTER_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_QUARTER_DRIVER);
		break;
	case LS_GPIO_DS_HALF_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_HALF_DRIVER);
		break;
    case LS_GPIO_DS_MAX_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_MAX_DRIVER);
    break;
	default:
		return -ENOTSUP;
	}

    if ((flags & GPIO_OUTPUT) != 0) {
        if ((flags & GPIO_SINGLE_ENDED) != 0) {
            if (flags & GPIO_LINE_OPEN_DRAIN) {
                io_cfg_opendrain(pinval);
            } else {
                /* Output can't be open source */
                return -ENOTSUP;
            }
        } else {
            io_cfg_pushpull(pinval);
        }

		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			gpio_ls_port_set_bits_raw(dev, BIT(pin));
		} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			gpio_ls_port_clear_bits_raw(dev, BIT(pin));
		}
	}

	return 0;
}

static void gpio_ls_isr(const struct device *dev)
{
	struct gpio_ls_data *data = dev->data;
    
    uint8_t port, pinval;
    uint16_t status = EXTI->EEIFM;

    for (uint8_t i = 0; i < 16; i++) {
        if (BIT(i) & status) {
            port = i < 8 ? EXTI->EICFG0 >> 4 * i : EXTI->EICFG1 >> 4 * (i - 8);
            pinval = port << 4 | i;
            EXTI->EICR = BIT(i);
            io_wkup_en_clr_set(pinval);
        }
    }
   
	gpio_fire_callbacks(&data->callbacks, dev, status);
}

static int gpio_ls_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;

	*value = gpio->DIN;

	return 0;
}

static int gpio_ls_manage_callback(const struct device *dev,
					      struct gpio_callback *callback,
					      bool set)
{
	struct gpio_ls_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_ls_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
				       gpio_port_value_t value)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;
	uint32_t port_value;

	port_value = gpio->DOUT;
	gpio->DOUT = (value & mask) | (port_value & ~mask);
    
	return 0;
}

static int gpio_ls_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;

	gpio->BSBR = pins;

	return 0;
}

static int gpio_ls_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;

	gpio->BSBR = pins << 16;

	return 0;
}

static int gpio_ls_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;

    gpio->DOUT ^= pins;

	return 0;
}

static int gpio_ls_pin_interrupt_configure(const struct device *dev,
						      gpio_pin_t pin,
						      enum gpio_int_mode mode,
						      enum gpio_int_trig trig)
{
    const struct gpio_ls_config *cfg = dev->config;
	reg_lsgpio_t *gpio = (reg_lsgpio_t *)cfg->base;
	uint8_t config, pinval, port;
  
    port = get_gpio_port_id((uint32_t)gpio);
    pinval = LSPIN(port, pin);

	if (mode != GPIO_INT_MODE_DISABLED) {
		if (mode == GPIO_INT_MODE_EDGE) {
			if (trig == GPIO_INT_TRIG_BOTH) {
				config = INT_EDGE_BOTH;
			} else if (trig == GPIO_INT_TRIG_HIGH) {
				config = INT_EDGE_RISING;
			} else { 
				config = INT_EDGE_FALLING;
			}
		} else {
			return -ENOTSUP;
		}
	} else {
        config = INT_EDGE_DISABLE;
	}

    io_exti_config(pinval, config);

	return 0;
}

static int gpio_ls_init(const struct device *dev) 
{
    /* Enable IRQ */
    IRQ_CONNECT(0, DT_INST_IRQ(0, priority),
		    gpio_ls_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(0);

    return 0; 
}

static const struct gpio_driver_api gpio_ls_driver_api = {
	.pin_configure = gpio_ls_pin_configure,
	.port_get_raw = gpio_ls_port_get_raw,
	.port_set_masked_raw = gpio_ls_port_set_masked_raw,
	.port_set_bits_raw = gpio_ls_port_set_bits_raw,
	.port_clear_bits_raw = gpio_ls_port_clear_bits_raw,
	.port_toggle_bits = gpio_ls_port_toggle_bits,
	.pin_interrupt_configure = gpio_ls_pin_interrupt_configure,
	.manage_callback = gpio_ls_manage_callback,
};

#define GPIO_LS_DEFINE(index)                                   \
	static struct gpio_ls_data ls_data_##index;                                     \
                                                                    \
	static const struct gpio_ls_config ls_config_##index = {                         \
		.base = (uint32_t *)DT_INST_REG_ADDR(index),                                \
		.common = {.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index)}};              \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, gpio_ls_init, NULL, &ls_data_##index,               \
			      &ls_config_##index, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,     \
			      &gpio_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_LS_DEFINE)
