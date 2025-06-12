/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * NOTE: If you are looking into an implementation of button0 events with
 * debouncing, check out `input` subsystem and `samples/subsys/input/input_dump`
 * example instead.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#define SLEEP_TIME_MS	1

/*
 * Get button0 configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
/*
 * Get button1 configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW1_NODE	DT_ALIAS(sw1)
#if !DT_NODE_HAS_STATUS_OKAY(SW1_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW1_NODE, gpios,
							      {0});
static struct gpio_callback button0_cb_data;
static struct gpio_callback button1_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button0 is pressed.
 */
static struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});
static struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios,
						     {0});
static struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios,
						     {0});

void button0_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button0 pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void button1_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button1 pressed at %" PRIu32 "\n", k_cycle_get_32());
}

#define LED_DEVICE_PROCESS(node_id)                            \
	 {                                                       \
		 .gpio_dev = GPIO_DT_SPEC_GET_OR(node_id, gpios, {0}), \
		 .label = DT_PROP(node_id, label),  \
		 .opened = ATOMIC_INIT(0), \
	},

struct led_device_info {
	// const struct device *dev;
	const struct gpio_dt_spec gpio_dev;
	const char *label;
	atomic_t opened;
};

struct led_device_info led_devices[] = {
	 DT_FOREACH_CHILD_STATUS_OKAY(DT_PATH(leds), LED_DEVICE_PROCESS)};

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&button0)) {
		printk("Error: button0 device %s is not ready\n",
		       button0.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button0.port->name, button0.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button0,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button0.port->name, button0.pin);
		return 0;
	}

	gpio_init_callback(&button0_cb_data, button0_pressed, BIT(button0.pin));
	gpio_add_callback(button0.port, &button0_cb_data);
	printk("Set up button0 at %s pin %d\n", button0.port->name, button0.pin);

	if (!gpio_is_ready_dt(&button1)) {
		printk("Error: button1 device %s is not ready\n",
		       button1.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button1.port->name, button1.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button1.port->name, button1.pin);
		return 0;
	}

	gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &button1_cb_data);
	printk("Set up button1 at %s pin %d\n", button1.port->name, button1.pin);

	printk("led %p\n", &led_devices[0]);

	if (led0.port && !gpio_is_ready_dt(&led0)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led0.port->name);
		led0.port = NULL;
	}
	if (led0.port) {
		ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led0.port->name, led0.pin);
			led0.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led0.port->name, led0.pin);
		}
	}

	if (led1.port && !gpio_is_ready_dt(&led1)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led1.port->name);
		led1.port = NULL;
	}
	if (led1.port) {
		ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led1.port->name, led1.pin);
			led1.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led1.port->name, led1.pin);
		}
	}

	if (led2.port && !gpio_is_ready_dt(&led2)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led2.port->name);
		led2.port = NULL;
	}
	if (led2.port) {
		ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led2.port->name, led2.pin);
			led2.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led2.port->name, led2.pin);
		}
	}

	printk("Press the button\n");
	if ((led0.port) || (led1.port) || (led2.port)) {
		while (1) {
			/* If we have an LED, match its state to the buttons's. */
			int val = gpio_pin_get_dt(&button0) & gpio_pin_get_dt(&button1);

			if (led0.port) {
				if (val >= 0) {
					gpio_pin_set_dt(&led0, val);
				}
			}
			if (led1.port) {
				if (val >= 0) {
					gpio_pin_set_dt(&led1, val);
				}
			}
			if (led2.port) {
				if (val >= 0) {
					gpio_pin_set_dt(&led2, val);
				}
			}
			k_msleep(SLEEP_TIME_MS);
		}
	}

	return 0;
}
