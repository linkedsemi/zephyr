
#include <zephyr/kernel.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>

/*wolfssl return code*/
#ifndef RNG_FAILURE_E
#define RNG_FAILURE_E (-199)
#endif

#if DT_HAS_CHOSEN(zephyr_entropy)
#if !defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

const static struct device *trng = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
int ls_wolfssl_get_random(uint8_t *buf, uint16_t need_size)
{
    uint8_t retry_count = 0;   
    do
    {
        if(entropy_get_entropy(trng, buf, need_size) != 0)
        {
            return RNG_FAILURE_E;
        }
        if(((buf[0] == 0x0) && (buf[need_size-1] == 0x0)))
        {
            retry_count++;
        }else
        {
            break;
        }
    } while (retry_count < 3);
    
    if(retry_count == 3)
    {
        return RNG_FAILURE_E;
    }else
    {
        return 0;
    }
}
#endif
#else
extern void z_impl_sys_rand_get(void *dst, size_t outlen);
int ls_wolfssl_get_random(uint8_t *buf, uint16_t need_size)
{
    uint8_t retry_count = 0;   
    do
    {
        z_impl_sys_rand_get(buf, need_size);
        if(((buf[0] == 0x0) && (buf[need_size-1] == 0x0)))
        {
            retry_count++;
        }else
        {
            break;
        }
    } while (retry_count < 3);
    
    if(retry_count == 3)
    {
        return RNG_FAILURE_E;
    }else
    {
        return 0;
    }
}

#endif