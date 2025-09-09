/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 LINKEDSEMI Technology Inc.
 */
#define DT_DRV_COMPAT linkedsemi_otbn_p256_ecdsa

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
// #include <zephyr/crypto/p256_ecdsa.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>
#include "qsh.h"

#include <zephyr/crypto/ls_otbn_p256_ecdsa.h>

LOG_MODULE_DECLARE(otbn,LOG_LEVEL_DBG);

#define SM2_MSG_DIGSET_BYTES 32

struct otbn_p256_ecdsa_data{
    struct k_sem *otbn_mutex;
    // struct aspeed_p256_ecdsa_ctx key;
};


struct otbn_p256_ecdsa_config
{
    // OTBN app.
    otbn_app_t kOtbnAppP256Ecdsa;
    // Record offsets for input and output buffers of imem or dmem.
    otbn_addr_t kOtbnVarEcdsaMode;
    otbn_addr_t kOtbnVarEcdsaMsg;
    otbn_addr_t kOtbnVarEcdsaR;
    otbn_addr_t kOtbnVarEcdsaS;
    otbn_addr_t kOtbnVarEcdsaX;
    otbn_addr_t kOtbnVarEcdsaY;
    otbn_addr_t kOtbnVarEcdsaD0;
    otbn_addr_t kOtbnVarEcdsaD1;
    otbn_addr_t kOtbnVarEcdsaXr;
    otbn_addr_t kOtbnVarEcdsaOk;
    otbn_addr_t kOtbnVarEcdsaRandomSeed;
    // mode constants.
    uint32_t kOtbnEcdsaModeKeygen;
    uint32_t kOtbnEcdsaModeSign;
    uint32_t kOtbnEcdsaModeVerify;
    uint32_t kOtbnEcdsaModeSideloadKeygen;
    uint32_t kOtbnEcdsaModeSideloadSign;

    reg_otbn_t *otbn_reg_addr;     // SEC_OTBN_ADDR
    uint32_t otbn_imem_addr;    // otbn_reg_addr + 0x4000
    uint32_t otbn_dmem_addr;    // otbn_reg_addr + 0x8000
    const struct device *otbn;
    void (*irq_config_func)(const struct device *);
};




static int ls_otbn_p256_ecdsa_sign(struct p256_ecdsa_ctx *ctx, struct p256_ecdsa_key *key, struct p256_ecdsa_pkt *pkt)
{
    int error = 0;
    struct otbn_p256_ecdsa_config *cfg_info = (struct otbn_p256_ecdsa_config *)ctx->device->config;
    struct otbn_p256_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeSign;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_P256_ECDSA)
    {
        return -1;
    }

    if(pkt->m == NULL || pkt->m_len != SM2_MSG_DIGSET_BYTES)
    {
        LOG_ERR("input the message digest len is not 32");
        return -1;
    }

    if(pkt->r == NULL || pkt->s == NULL)
    {
        LOG_ERR("no buffer to load r or s");
        return -1;
    }

    if(key->d == NULL || key->qx == NULL || key->qy == NULL)
    {
        LOG_ERR("The SM2 key massage pointer is NULL");
        return -1;
    }

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarEcdsaMode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->d,cfg_info->kOtbnVarEcdsaD0);
    uint8_t rand[256];
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xfc+i;
    }
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,cfg_info->kOtbnVarEcdsaRandomSeed);

    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,cfg_info->kOtbnVarEcdsaMsg);

    error |= otbn_func->otbn_execute(otbn);
    if(error != 0)
    {
        LOG_ERR("otbn is running");
        goto exit;
    }

    error = otbn_func->otbn_err_bits_get(otbn);
    if(error != 0)
    {
        LOG_DBG("otbn error :otbn err bits = 0x%x",error);
        goto exit;
    }

    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaR,(uint32_t *)pkt->r);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaS,(uint32_t *)pkt->s);
    
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_p256_ecdsa_verify(struct p256_ecdsa_ctx *ctx, struct p256_ecdsa_key *key, struct p256_ecdsa_pkt *pkt)
{
    int error = 0;
    struct otbn_p256_ecdsa_config *cfg_info = (struct otbn_p256_ecdsa_config *)ctx->device->config;
    struct otbn_p256_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeVerify;
    struct ls_otbn_data *otbn_data = otbn->data;
    uint8_t r_x[32] = {0};
    if(otbn_data->mode != OTBN_P256_ECDSA)
    {
        return -1;
    }

    if(pkt->m == NULL || pkt->m_len != SM2_MSG_DIGSET_BYTES)
    {
        LOG_ERR("input the message digest len is not 32");
        return -1;
    }

    if(pkt->r == NULL || pkt->s == NULL)
    {
        LOG_ERR("no buffer to load r or s");
        return -1;
    }

    if(key->d == NULL || key->qx == NULL || key->qy == NULL)
    {
        LOG_ERR("The SM2 key massage pointer is NULL");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarEcdsaMode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }
    
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qy,cfg_info->kOtbnVarEcdsaY);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qx,cfg_info->kOtbnVarEcdsaX);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->r,cfg_info->kOtbnVarEcdsaR);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->s,cfg_info->kOtbnVarEcdsaS);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,cfg_info->kOtbnVarEcdsaMsg);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)r_x,cfg_info->kOtbnVarEcdsaXr);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }
    
    error = otbn_func->otbn_execute(otbn);
    if(error != 0)
    {
        LOG_ERR("otbn is running");
        goto exit;
    }

    error = otbn_func->otbn_err_bits_get(otbn);
    if(error != 0)
    {
        LOG_DBG("otbn error :otbn err bits = 0x%x",error);
        goto exit;
    }


    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaXr,(uint32_t *)r_x);

    if(memcmp(r_x, pkt->r, 32))
    {
        error = -1;
    }
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_p256_ecdsa_keygen(struct p256_ecdsa_ctx *ctx, struct p256_ecdsa_key *key)
{
    int error = 0;
    struct otbn_p256_ecdsa_config *cfg_info = (struct otbn_p256_ecdsa_config *)ctx->device->config;
    struct otbn_p256_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeKeygen;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_P256_ECDSA)
    {
        return -1;
    }

    if(key->qx == NULL || key->qy == NULL || key->d == NULL)
    {
        LOG_ERR("no buffer to load key");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    otbn_func->otbn_dmem_set(otbn,32,0,cfg_info->kOtbnVarEcdsaX);
    otbn_func->otbn_dmem_set(otbn,32,0,cfg_info->kOtbnVarEcdsaY);
    otbn_func->otbn_dmem_set(otbn,32,0,cfg_info->kOtbnVarEcdsaD0);

    uint8_t rand[256];
    memset(rand,0xff,256);
    // rand[0] = 0x33;
    // rand[10] = 0x33;
    // rand[255] = 0x1;
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xfe + i;
    }
    otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,cfg_info->kOtbnVarEcdsaRandomSeed);

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarEcdsaMode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }

    error = otbn_func->otbn_err_bits_get(otbn);
    if(error != 0)
    {
        LOG_DBG("otbn error :otbn err bits = 0x%x",error);
        goto exit;
    }

    error = otbn_func->otbn_execute(otbn);
    if(error != 0)
    {
        LOG_ERR("otbn is running");
        goto exit;
    }

    error = otbn_func->otbn_err_bits_get(otbn);
    if(error != 0)
    {
        LOG_DBG("otbn error :otbn err bits = 0x%x",error);
        goto exit;
    }

    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaD0,(uint32_t *)key->d);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaX,(uint32_t *)key->qx);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarEcdsaY,(uint32_t *)key->qy);
    LOG_INF("get p256_ecdsa key succeed");

exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}


static int ls_otbn_p256_ecdsa_session_setup(const struct device *dev,
				      struct p256_ecdsa_ctx *ctx,
				      struct p256_ecdsa_key *key)
{
    int error = 0;
    struct otbn_p256_ecdsa_config *cfg_info = (struct otbn_p256_ecdsa_config *)dev->config;
    const struct otbn_p256_ecdsa_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    struct ls_otbn_data *otbn_data = otbn->data;
    k_sem_take(data->otbn_mutex, K_FOREVER);

    otbn_func->otbn_dmem_sec_wipe(otbn);
    otbn_func->otbn_imem_sec_wipe(otbn);
    error = otbn_func->otbn_load_app(otbn,&cfg_info->kOtbnAppP256Ecdsa);
    if(error != 0)
    {
        LOG_ERR("%s trigger error,please check the app image info !",__func__);
        goto exti;
    }

    otbn_data->mode = OTBN_P256_ECDSA;
    ctx->device = dev;
    ctx->ops.keygen = ls_otbn_p256_ecdsa_keygen;
    ctx->ops.sign = ls_otbn_p256_ecdsa_sign;
    ctx->ops.verify = ls_otbn_p256_ecdsa_verify;
// {kOtbnAppP256Ecdsa = {imem_start = 0x1000cd8c, 
//     imem_end = 0x1000d680, 
//     dmem_data_start = 0x1000ac40, 
//     dmem_data_end = 0x1000af60, 
//     dmem_data_start_addr = 0x0, 
//     checksum = 0xf731f8c4}, 
//     kOtbnVarEcdsaMode = 0x0, 
//     kOtbnVarEcdsaMsg = 0xa0, 
//     kOtbnVarEcdsaR = 0xc0, 
//     kOtbnVarEcdsaS = 0xe0, 
//     kOtbnVarEcdsaX = 0x100, 
//     kOtbnVarEcdsaY = 0x120, 
//     kOtbnVarEcdsaD0 = 0x20, 
//     kOtbnVarEcdsaXr = 0x140, 
//     kOtbnVarEcdsaRand1 = 0x0, 
//     kOtbnEcdsaModeKeygen = 0x3d4, 
//     kOtbnEcdsaModeSign = 0x15b, 
//     kOtbnEcdsaModeVerify = 0x727, 
//     otbn_reg_addr = 0x0, 
//     otbn_imem_addr = 0x4000, 
//     otbn_dmem_addr = 0x8000, 
//     otbn = 0x1000a4e8 <__device_dts_ord_195>, 
//     irq_config_func = 0x100060a2 <p256_ecdsa_irq_config_func_0>}

exti:
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_p256_ecdsa_init(const struct device *dev)
{
    struct otbn_p256_ecdsa_config *cfg_info = (struct otbn_p256_ecdsa_config *)dev->config;
    struct otbn_p256_ecdsa_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    struct ls_otbn_data *otbn_data = otbn->data;

    /*通过指针获取otbn的锁，多个加密驱动共用这个锁*/
    data->otbn_mutex = &otbn_data->mutex;

    return 0;
}



static struct p256_ecdsa_driver_api  ls_p256_ecdsa_driver_api = {
    .begin_session = ls_otbn_p256_ecdsa_session_setup,
    // .free_session = ls_otbn_p256_ecdsa_session_free,
    .query_hw_caps = NULL,
};


#define LS_OTBN_P256_ECDSA_INIT(idx)                    \
    \
	static void p256_ecdsa_irq_config_func_##idx(const struct device *dev){}              \
	static struct otbn_p256_ecdsa_data otbn_p256_ecdsa_data_##idx;                         \
    OTBN_DECLARE_APP_SYMBOLS(p256_ecdsa);\
    \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, mode);  \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, msg);   \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, r);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, s);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, x);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, y);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa,d0);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa,d1);     \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, x_r);   \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, ok);    \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, random_seed0);    \
    \
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, MODE_KEYGEN);\
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, MODE_SIGN);\
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, MODE_VERIFY);\
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, MODE_SIDELOAD_KEYGEN);\
    OTBN_DECLARE_SYMBOL_ADDR(p256_ecdsa, MODE_SIDELOAD_SIGN);\
    \
	static const struct otbn_p256_ecdsa_config otbn_p256_ecdsa_config_##idx = {            \
        .kOtbnAppP256Ecdsa =  OTBN_APP_T_INIT(p256_ecdsa),                      \
        .kOtbnVarEcdsaMode = OTBN_ADDR_T_INIT(p256_ecdsa, mode),            \
        .kOtbnVarEcdsaMsg  = OTBN_ADDR_T_INIT(p256_ecdsa, msg),         \
        .kOtbnVarEcdsaR    =   OTBN_ADDR_T_INIT(p256_ecdsa, r),         \
        .kOtbnVarEcdsaS    =   OTBN_ADDR_T_INIT(p256_ecdsa, s),         \
        .kOtbnVarEcdsaX    =   OTBN_ADDR_T_INIT(p256_ecdsa, x),         \
        .kOtbnVarEcdsaY    =   OTBN_ADDR_T_INIT(p256_ecdsa, y),         \
        .kOtbnVarEcdsaD0   =   OTBN_ADDR_T_INIT(p256_ecdsa, d0),            \
        .kOtbnVarEcdsaD1   =   OTBN_ADDR_T_INIT(p256_ecdsa, d1),            \
        .kOtbnVarEcdsaXr   =   OTBN_ADDR_T_INIT(p256_ecdsa, x_r),           \
        .kOtbnVarEcdsaOk   =   OTBN_ADDR_T_INIT(p256_ecdsa, ok),            \
        .kOtbnVarEcdsaRandomSeed = OTBN_ADDR_T_INIT(p256_ecdsa, random_seed0),\
        .kOtbnEcdsaModeKeygen = OTBN_ADDR_T_INIT(p256_ecdsa, MODE_KEYGEN),\
        .kOtbnEcdsaModeSign = OTBN_ADDR_T_INIT(p256_ecdsa, MODE_SIGN),\
        .kOtbnEcdsaModeVerify = OTBN_ADDR_T_INIT(p256_ecdsa, MODE_VERIFY),\
        .kOtbnEcdsaModeSideloadKeygen = OTBN_ADDR_T_INIT(p256_ecdsa, MODE_SIDELOAD_KEYGEN),\
        .kOtbnEcdsaModeSideloadSign = OTBN_ADDR_T_INIT(p256_ecdsa, MODE_SIDELOAD_SIGN),\
        .otbn_imem_addr = 0x4000,   \
        .otbn_dmem_addr = 0x8000,   \
		.irq_config_func = p256_ecdsa_irq_config_func_##idx,                        \
        .otbn = DEVICE_DT_GET(DT_INST_PARENT(idx)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_p256_ecdsa_init, NULL, &otbn_p256_ecdsa_data_##idx,    \
			      &otbn_p256_ecdsa_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&ls_p256_ecdsa_driver_api);



DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_P256_ECDSA_INIT)