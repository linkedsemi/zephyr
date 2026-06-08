#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "core_rv32.h"
#include "platform.h"
#include "co_math.h"
#include "qsh.h"
#include "ls_otbn_config.h"
#include <stdio.h>
#include <math.h>

static uint32_t ls_otbn_default_prng_cb(void)
{
    return (uint32_t)rand();
}

static otbn_rand_cb s_trng_cb = ls_otbn_default_prng_cb;
static otbn_rand_cb s_prng_cb = ls_otbn_default_prng_cb;
static uint32_t EDN_URND_BUS_IN;

void ls_otbn_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb)
{
    s_trng_cb = trng_cb ? trng_cb : ls_otbn_default_prng_cb;
    s_prng_cb = prng_cb ? prng_cb : ls_otbn_default_prng_cb;
}

void LS_OTBN_SYSC_IRQHandler(void)
{
    uint32_t intr = SYSC_SEC_CPU->OTBN_INTR_STT;
    if (intr & CO_BIT(0))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(0);
    }
    if (intr & CO_BIT(1))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(1);
    }
    if (intr & CO_BIT(2))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(2);
    }
    if (intr & CO_BIT(3))
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(3);
    }
    if (intr & 0xf0)
    {
        SYSC_SEC_CPU->INTR_CLR_MSK = 0xf0;
    }
    if (intr & CO_BIT(8))
    {
        SYSC_SEC_CPU->EDN_RND_BUS = s_trng_cb();
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(8)|CO_BIT(9);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(8);
    }
    if (intr & CO_BIT(9))
    {
        SYSC_SEC_CPU->EDN_URND_BUS = s_prng_cb();
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(10)|CO_BIT(11);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(9);
    }
    if (intr & CO_BIT(10))
    {
        SYSC_SEC_CPU->OTBN_OTP_KEY_0 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_1 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_2 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_OTP_KEY_3 = ++EDN_URND_BUS_IN;
        SYSC_SEC_CPU->OTBN_CTRL2 |= CO_BIT(12);
        SYSC_SEC_CPU->INTR_CLR_MSK = CO_BIT(10);
    }
}