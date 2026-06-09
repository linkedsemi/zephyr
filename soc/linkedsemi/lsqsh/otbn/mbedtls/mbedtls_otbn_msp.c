
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "otbn/ls_otbn_config.h"

#define MBEDTLS_ERR_LS_OTBN_BUSY -0x135

static struct k_sem wait_complete;
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
    
    if (LSOTBN->INTR_STATE)
    {
        LSOTBN->INTR_STATE = OTBN_INTR_STATE_DONE_MASK;
        k_sem_give(&wait_complete);

    }else
    {
        printf("unexpected state: LSOTBN->INTR_STATE : 0x%x",LSOTBN->INTR_STATE);
    }
}

void ls_otbn_mbedtls_update_callback(void (*func)(void*),void *param)
{
    p_otbn_func = func;
    p_otbn_param = param;
}

void ls_otbn_cmd(enum HAL_OTBN_CMD cmd)
{
    if (LSOTBN->INTR_STATE)
        LSOTBN->INTR_STATE = OTBN_INTR_STATE_DONE_MASK;
    LSOTBN->TNSN_CNT = 0;
    LSOTBN->INTR_ENABLE = OTBN_INTR_ENABLE_EN_MASK;
    LSOTBN->CMD = cmd;
    (void)k_sem_take(&wait_complete, K_FOREVER);
}

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
        k_sem_init(&wait_complete,0,1);
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

void ls_otbn_module_init(void);
void mbedtls_ls_otbn_moudle_init(void)
{
    if(mbedtls_enabled)
    {
        return;
    }
    ls_otbn_done_callback_register(MBEDTLS_LS_OTBN_IRQHandler,NULL);
    ls_otbn_module_init();
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
