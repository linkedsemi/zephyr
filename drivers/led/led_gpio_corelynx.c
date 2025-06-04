#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led/led_gpio_corelynx.h>
#include <zephyr/kernel.h>

static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
                                                     {0});
struct k_timer led_timer;
static int led_state = 0;

void led_timer_callback(struct k_timer* timer_id)
{
    if (led.port)
    {
        gpio_pin_set_dt(&led, led_state ? 0 : 1);
        led_state = !led_state;
    }
}

void led_state_set(int state)
{
    if (led.port)
    {
        gpio_pin_set_dt(&led, state);
    }
}

void led_state_init(void)
{
    if (led.port)
    {
        if (!gpio_is_ready_dt(&led))
        {
            printk("Error: LED device %s is not ready; ignoring it\n",
                   led.port->name);
            led.port = NULL;
        }

        int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
        if (ret != 0)
        {
            printk("Error %d: failed to configure LED device %s pin %d\n", ret,
                   led.port->name, led.pin);
            led.port = NULL;
        }
        else
        {
            printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
        }
    }

    k_timer_init(&led_timer, led_timer_callback, NULL);
    k_timer_start(&led_timer, K_NO_WAIT, K_MSEC(250));
}
