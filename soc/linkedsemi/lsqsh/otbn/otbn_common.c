#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "reg_otbn_type.h"
#include "ls_msp_otbn.h"
#include "co_math.h"
#include "qsh.h"
#include "ls_otbn_config.h"
#include <stdio.h>
#include <math.h>
#include <zephyr/irq.h>

#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN)&&defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_ENABLE)
BUILD_ASSERT(false,"otbn is reused");
#endif
static uint32_t ls_otbn_default_prng_cb(void)
{
    return (uint32_t)rand();
}

static void *s_otbn_param = NULL;
static otbn_done_callback_t s_otbn_done_handler  = NULL;
static otbn_rand_cb s_trng_cb = ls_otbn_default_prng_cb;
static otbn_rand_cb s_prng_cb = ls_otbn_default_prng_cb;
static uint32_t EDN_URND_BUS_IN;

static void LS_OTBN_IRQHandler(void)
{
    if(s_otbn_done_handler)
    {
        s_otbn_done_handler(s_otbn_param);
    }
    else
    {
        __ASSERT_NO_MSG(0);
    }   
}

static void LS_OTBN_SYSC_IRQHandler(void)
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

void ls_otbn_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb)
{
    s_trng_cb = trng_cb ? trng_cb : ls_otbn_default_prng_cb;
    s_prng_cb = prng_cb ? prng_cb : ls_otbn_default_prng_cb;
}

void ls_otbn_done_callback_register(otbn_done_callback_t handler, void *param)
{
    s_otbn_done_handler = handler;
    s_otbn_param = param;
}

void ls_otbn_done_callback_unregister(void)
{
    s_otbn_done_handler = NULL;
    s_otbn_param = NULL;
}

void ls_otbn_module_init(void)
{
    REG_FIELD_WR(SYSC_SEC_CPU->INTR_CTRL_INTR_MSK, SYSC_SEC_CPU_I_EDN_URND_REQ, 0);
    SYSC_SEC_CPU->PD_CPU_CLKG[1] = SYSC_SEC_CPU_CLKG_CLR_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_SRST[1] = SYSC_SEC_CPU_SRST_CLR_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_SRST[1] = SYSC_SEC_CPU_SRST_SET_OTBN_MASK;
    SYSC_SEC_CPU->PD_CPU_CLKG[1] = SYSC_SEC_CPU_CLKG_SET_OTBN_MASK;
    for (uint8_t i = 0; i < 16; i++)
    {
        while (!REG_FIELD_RD(SYSC_SEC_CPU->OTBN_INTR_RAW, SYSC_SEC_CPU_I_EDN_URND_REQ)) ;
        SYSC_SEC_CPU->EDN_URND_BUS = ++EDN_URND_BUS_IN;
        REG_FIELD_WR(SYSC_SEC_CPU->OTBN_CTRL2, SYSC_SEC_CPU_EDN_URND_ACK, 1);
        REG_FIELD_WR(SYSC_SEC_CPU->OTBN_CTRL2, SYSC_SEC_CPU_EDN_URND_ACK, 0);
        SYSC_SEC_CPU->INTR_CLR_MSK = SYSC_SEC_CPU_I_EDN_URND_REQ_MASK;
    }
    SYSC_SEC_CPU->INTR_CLR_MSK = FIELD_BUILD(SYSC_SEC_CPU_I_EDN_RND_REQ, 1) |
                            FIELD_BUILD(SYSC_SEC_CPU_I_EDN_URND_REQ, 1) |
                            FIELD_BUILD(SYSC_SEC_CPU_I_OTBN_OTP_REQ, 1);
    SYSC_SEC_CPU->INTR_CTRL_INTR_MSK = FIELD_BUILD(SYSC_SEC_CPU_I_EDN_RND_REQ, 1) |
                              FIELD_BUILD(SYSC_SEC_CPU_I_EDN_URND_REQ, 1) |
                              FIELD_BUILD(SYSC_SEC_CPU_I_OTBN_OTP_REQ, 1);

    // otbn system irq
    IRQ_CONNECT(OTBN_SYSC_IRQN, 3, LS_OTBN_SYSC_IRQHandler,NULL, 0);
    irq_enable(OTBN_SYSC_IRQN);
    // otbn status irq
    IRQ_CONNECT(OBTN_IRQN, 3, LS_OTBN_IRQHandler,NULL, 0);
    irq_enable(OBTN_IRQN);

}