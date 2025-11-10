
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include <stdlib.h>


const static struct device *trng = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_entropy));


int ls_trng_get_random(uint8_t *buf, uint16_t need_size)
{
    // return entropy_get_entropy(trng, buf, need_size);
    return rand();
}
