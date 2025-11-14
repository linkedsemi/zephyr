
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

#if DT_HAS_CHOSEN(zephyr_entropy)
#if !defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

const static struct device *trng = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
int ls_wolfssl_get_random(uint8_t *buf, uint16_t need_size)
{
    return entropy_get_entropy(trng, buf, need_size);
}
#endif
#else
extern void z_impl_sys_rand_get(void *dst, size_t outlen);
int ls_wolfssl_get_random(uint8_t *buf, uint16_t need_size)
{
    z_impl_sys_rand_get(buf, need_size);
    return 0;
}

#endif