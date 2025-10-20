

#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "platform.h"
#include "stdio.h"
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

#define MBEDTLS_ERR_LS_OTBN_BUSY -0x135

static struct k_mutex mbedtls_otbn_doneLock;
static ls_otbn_fireware_t current_obtn_fireware = OTBN_UNUSED;
static bool mbedtls_enabled = false;
static bool mbedtls_otbn_mutex_initailed = false;
static void (*p_otbn_func)(void *);
static void *p_otbn_param;
volatile struct k_thread *current_otbn_thread = NULL;
void mbedtls_ls_otbn_moudle_init(void);

static void MBEDTLS_LS_OTBN_IRQHandler(void *parm)
{
    if(p_otbn_func)
        p_otbn_func(p_otbn_param);
}

void ls_otbn_mbedtls_update_callback(void (*func)(void*),void *param)
{
    p_otbn_func = func;
    p_otbn_param = param;
}

/* 
不同的线程，调用otbn的时候，这里返回true，通过互斥量mbedtls_otbn_doneLock来管理进程
当相同线程同时调用otbn时，这里返回false，表示在某个otbn释放前，不允许这个线程调用otbn其他相关接口。
如hash算法，在start和finish的接口之间，起计算数据中间数据保存在otbn的内存区域，在此期间不允许被其
他otbn应用覆盖。
*/

void mbedtls_ls_otbn_threading_release(void)
{
    if(current_otbn_thread == 0)
    {
        while(1);
    }
    current_otbn_thread = NULL;
    current_obtn_fireware = OTBN_UNUSED;
    k_mutex_unlock(&mbedtls_otbn_doneLock);
}

bool mbedtls_ls_otbn_is_operation_current_thread(void)
{
    struct k_thread *current_thread;
    current_thread = k_current_get();
    // printf("current thread: 0x%x, otbn thread: 0x%x\n",current_thread,current_otbn_thread);
    if(current_otbn_thread != current_thread)
    {
        return false;
    }
    return true;
}

int mbedtls_ls_otbn_operation_init(ls_otbn_fireware_t fireware_id)
{
    struct k_thread *current_thread;
    bool is_busy = false;
    unsigned int key;
    key = irq_lock();
    current_thread = k_current_get();
    // printf("current thread: 0x%x, otbn thread: 0x%x\n",current_thread,current_otbn_thread);

    if(current_otbn_thread == current_thread)
    {
        /* Two applications that are both based on otbn 
        are repeatedly called in the same thread. This 
        operation is not supported. Please wait for the 
        current otbn application to release.*/
        is_busy = true;
    }
    

    irq_unlock(key);
    if(is_busy)
    {
        return MBEDTLS_ERR_LS_OTBN_BUSY;
    }

    if(mbedtls_otbn_mutex_initailed == false)
    {
        k_mutex_init(&mbedtls_otbn_doneLock);
        mbedtls_otbn_mutex_initailed = true;
    }

    k_mutex_lock(&mbedtls_otbn_doneLock,K_FOREVER);

    if(current_otbn_thread == NULL)
    {
        current_otbn_thread = current_thread;
        current_obtn_fireware = fireware_id;
    }else
    {
        while(1);
    }
    mbedtls_ls_otbn_moudle_init();
    return 0;
}


void mbedtls_ls_otbn_moudle_init(void)
{
    // unsigned int key;
    if(mbedtls_enabled)
    {
        // if(mbedtls_otbn_mutex_initailed == true)
        // {
            // mbedtls_mutex_lock(&mbedtls_otbn_doneLock);
        // }
        return;
    }
    // key = irq_lock();
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
    // irq_unlock(key);
    IRQ_CONNECT(OTBN_SYSC_IRQN, 3, HAL_OTBN_SYSC_IRQHandler,NULL, 0);
    csi_vic_clear_pending_irq(OTBN_SYSC_IRQN);
    irq_enable(OTBN_SYSC_IRQN);
    IRQ_CONNECT(OBTN_IRQN, 3, MBEDTLS_LS_OTBN_IRQHandler,NULL, 0);
    csi_vic_clear_pending_irq(OBTN_IRQN);
    irq_enable(OBTN_IRQN);

    mbedtls_enabled = true;

}

void mbedtls_ls_otbn_moudle_deinit(void)
{
    unsigned int key;
    key = irq_lock();
    if(mbedtls_enabled)
    {
        HAL_LSOTBN_MSP_DeInit();
        mbedtls_enabled = false;
    }
    irq_unlock(key);
}
