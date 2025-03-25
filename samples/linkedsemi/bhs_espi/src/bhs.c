
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "ls_soc_gpio.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_NODELABEL(bmc_boot_done_btn)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// GPIO19
#define ONCTL PD03
// GPIO69
// #define SLPS3 PM06
// #define SLPS3 PH06
#define SLPS3 PH09


int bhs_bmc_ready(void)
{
    int ret;

    printf("Hello World! %s\n", CONFIG_BOARD);

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    ret = gpio_pin_set_dt(&led, 1);
    if (ret < 0) {
        return 0;
    }

    io_cfg_input(ONCTL);
    io_cfg_input(SLPS3);

    while(io_read_pin(SLPS3) == 1);

    io_cfg_output(ONCTL);
    io_write_pin(ONCTL, 0);

    return 0;
}
