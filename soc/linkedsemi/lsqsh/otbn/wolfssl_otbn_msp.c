
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>



const static struct device *trng = DEVICE_DT_GET(DT_NODELABEL(trng0));


int ls_trng_get_random(uint8_t *buf, uint16_t need_size)
{
    return entropy_get_entropy(trng, buf, need_size);
}
