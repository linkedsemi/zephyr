/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 LINKEDSEMI Technology Inc.
 */
#define DT_DRV_COMPAT linkedsemi_otbn_crypto

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/crypto/ecdsa.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include <zephyr/crypto/ecdsa.h>
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>
#include "qsh.h"
LOG_MODULE_REGISTER(otbn,LOG_LEVEL_DBG);
#define OTBN_IMEM_OFFSET_PTR(offset) (uint32_t *)(OTBN_IMEM_ADDR+offset)
#define OTBN_DMEM_OFFSET_PTR(offset) (uint32_t *)(OTBN_DMEM_ADDR+offset)


struct ls_otbn_config
{
    reg_otbn_t *otbn_reg_addr;     // SEC_OTBN_ADDR

    void (*irq_config_func)(const struct device *);
};

static bool otbn_assert_idle(const struct device *dev)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;
    uint32_t status = cfg_info->otbn_reg_addr->STATUS;
    if(status == kOtbnStatusIdle)
    {
        return true;
    }
    
    return false;
}


status_t ls_otbn_imem_sec_wipe(const struct device *dev)
{
    /*nothing to do*/
    return 0;
    if(!otbn_assert_idle(dev))
    {
        return -1;
    }

    HAL_OTBN_CMD_Write_Polling(HAL_OTBN_CMD_SEC_WIPE_IMEM);

    return 0;
}

status_t ls_otbn_dmem_sec_wipe(const struct device *dev)
{
    /*nothing to do*/
    return 0;
    if(!otbn_assert_idle(dev))
    {
        return -1;
    }

    HAL_OTBN_CMD_Write_Polling(HAL_OTBN_CMD_SEC_WIPE_DMEM);

    return 0;
}

status_t ls_otbn_execute(const struct device *dev)
{
    if(!otbn_assert_idle(dev))
    {
        return -1;
    }

    HAL_OTBN_CMD_Write_Polling(HAL_OTBN_CMD_EXECUTE);

    return 0;
}

uint32_t otbn_err_bits_get(const struct device *dev)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;
    return cfg_info->otbn_reg_addr->ERR_BITS;
}



int ls_otbn_busy_wait_for_done(const struct device *dev)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;
    uint32_t status;
    do{
        status = cfg_info->otbn_reg_addr->STATUS;
    }while(status != kOtbnStatusIdle && status != kOtbnStatusLocked);

    uint32_t err_bits = otbn_err_bits_get(dev);
    if(status == kOtbnStatusIdle && err_bits == kOtbnErrBitsNoError)
    {
        return 0;
    }

    // If OTBN is idle (not locked), then return a recoverable error.
    if(status == kOtbnStatusIdle)
    {
        return -1;
    }

    // OTBN is locked; return a fatal error.
    return -2;
}


int ls_otbn_dmem_read(const struct device *dev, uint16_t num_words, otbn_addr_t src, uint32_t *dest)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;

    if (cfg_info->otbn_reg_addr->STATUS != HAL_OTBN_STATUS_IDLE)
        return -1;

    memcpy32(dest, OTBN_DMEM_OFFSET_PTR(src), num_words);
    return 0;
}

int ls_otbn_dmem_set(const struct device *dev, uint16_t num_words, const uint32_t data, otbn_addr_t dest)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;

    if (cfg_info->otbn_reg_addr->STATUS != HAL_OTBN_STATUS_IDLE)
        return HAL_BUSY;

    if(dest+(4*num_words)>OTBN_DMEM_SIZE)
        return HAL_INVALIAD_PARAM;

    memset32(OTBN_DMEM_OFFSET_PTR(dest), data, num_words);
    return HAL_OK;
}

int ls_otbn_dmem_write(const struct device *dev, uint16_t num_words, const uint32_t *src, otbn_addr_t dest)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;

    if (cfg_info->otbn_reg_addr->STATUS != HAL_OTBN_STATUS_IDLE)
        return HAL_BUSY;

    if(dest+(4*num_words)>OTBN_DMEM_SIZE)
        return HAL_INVALIAD_PARAM;

    memcpy32(OTBN_DMEM_OFFSET_PTR(dest), src, num_words);
    return HAL_OK;
}

int ls_otbn_imem_write(const struct device *dev, uint16_t num_words, const uint32_t *src, otbn_addr_t dest)
{
    struct ls_otbn_config *cfg_info = (struct ls_otbn_config *)dev->config;

    if (cfg_info->otbn_reg_addr->STATUS != HAL_OTBN_STATUS_IDLE)
        return HAL_BUSY;

    if(dest+(4*num_words)>OTBN_IMEM_SIZE)
        return HAL_INVALIAD_PARAM;

    memcpy32(OTBN_IMEM_OFFSET_PTR(dest), src, num_words);
    return HAL_OK;
}

status_t ls_otbn_load_app(const struct device *dev, const otbn_app_t *app_info)
{
    uint32_t status;
    status = otbn_assert_idle(dev);
    if(!status)
    {
        LOG_ERR("OTBN is busy");
    }

    ls_otbn_imem_sec_wipe(dev);
    ls_otbn_dmem_sec_wipe(dev);

    ls_otbn_dmem_set(dev,(app_info->kOtbnAppDmemEnd+3)/4,0,0);
    ls_otbn_imem_write(dev,(app_info->kOtbnAppImemSize+3)/4,(uint32_t *)app_info->imem_image,0);
    ls_otbn_dmem_write(dev,(app_info->kOtbnAppDmemSize+3)/4,(uint32_t *)app_info->dmem_image,0);

    return 0;
}

extern void HAL_OTBN_SYSC_IRQHandler(void);
static int ls_otbn_init(const struct device *dev)
{
    // const struct ls_otbn_config *config = dev->config;
    struct ls_otbn_data *data = dev->data;
    static uint32_t EDN_URND_BUS_IN;
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

    irq_enable(OTBN_SYSC_IRQN);

    k_sem_init(&data->mutex, 1, K_SEM_MAX_LIMIT);

    return 0;
}

void ls_otbn_isr(const struct device *dev)
{
    struct ls_otbn_data *data = dev->data;
    if (LSOTBN->INTR_STATE)
    {
        LSOTBN->INTR_STATE = OTBN_INTR_STATE_DONE_MASK;
        data->app_callback(dev);
    }
}


int otbn_get_random(uint8_t *buf, uint16_t buf_len)
{
    for(uint16_t i = 0; i < buf_len; i++)
    {
        buf[i] = i * 0xfc;
    }
    return 0;
} 

static struct otbn_ops_api_t  otbn_ops_api = {
    .otbn_imem_sec_wipe = ls_otbn_imem_sec_wipe,
    .otbn_dmem_sec_wipe = ls_otbn_dmem_sec_wipe,
    .otbn_err_bits_get = otbn_err_bits_get,
    .otbn_execute = ls_otbn_execute,
    .otbn_busy_wait_for_done = ls_otbn_busy_wait_for_done,
    .otbn_dmem_read = ls_otbn_dmem_read,
    .otbn_dmem_set = ls_otbn_dmem_set,
    .otbn_dmem_write = ls_otbn_dmem_write,
    .otbn_load_app = ls_otbn_load_app,
};


#define LS_OTBN_INIT(idx)                    \
    \
	static void otbn_irq_config_func_##idx(const struct device *dev)              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), ls_otbn_isr,  \
			    DEVICE_DT_INST_GET(idx), 0);                                           \
		irq_enable(DT_INST_IRQN(idx));                                                     \
	}                                                                                          \
	static struct ls_otbn_data otbn_data_##idx;                         \
	static const struct ls_otbn_config otbn_config_##idx = {            \
		.otbn_reg_addr = (void *)DT_INST_REG_ADDR(idx), \
		.irq_config_func = otbn_irq_config_func_##idx,                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_otbn_init, NULL, &otbn_data_##idx,    \
			      &otbn_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&otbn_ops_api);

DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_INIT)