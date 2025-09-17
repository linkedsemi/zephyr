

#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "platform.h"

#include <zephyr/irq.h>

static void (*p_otbn_func)(void *);
static void *p_otbn_param;
static void MBEDTLS_LS_OTBN_IRQHandler(void *parm)
{
    if(p_otbn_func)
        p_otbn_func(NULL);
}

void ls_otbn_mbedtls_update_callback(void (*func)(void*),void *param)
{
    p_otbn_func = func;
    p_otbn_param = param;
}

void mbedtls_ls_otbn_moudle_init(void)
{
    uint32_t EDN_URND_BUS_IN = 0;
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

    IRQ_CONNECT(OTBN_SYSC_IRQN, 3, HAL_OTBN_SYSC_IRQHandler,NULL, 0);
    csi_vic_clear_pending_irq(OTBN_SYSC_IRQN);
    irq_enable(OTBN_SYSC_IRQN);
    IRQ_CONNECT(OBTN_IRQN, 3, MBEDTLS_LS_OTBN_IRQHandler,NULL, 0);
    csi_vic_clear_pending_irq(OBTN_IRQN);
    irq_enable(OBTN_IRQN);

}

void mbedtls_ls_otbn_moudle_deinit(void)
{
    HAL_LSOTBN_MSP_DeInit();
}
