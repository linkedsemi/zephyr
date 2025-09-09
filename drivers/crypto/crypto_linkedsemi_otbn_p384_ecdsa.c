/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 LINKEDSEMI Technology Inc.
 */
#define DT_DRV_COMPAT linkedsemi_otbn_p384_ecdsa

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
// #include <zephyr/crypto/p384_ecdsa.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>
#include "qsh.h"

#include <zephyr/crypto/ls_otbn_p384_ecdsa.h>

LOG_MODULE_DECLARE(otbn,LOG_LEVEL_DBG);

#define SM2_MSG_DIGSET_BYTES 48

struct otbn_p384_ecdsa_data{
    struct k_sem *otbn_mutex;
    // struct aspeed_p384_ecdsa_ctx key;
};


struct otbn_p384_ecdsa_config
{
    // OTBN app.
    otbn_app_t kOtbnAppP384Ecdsa;
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




static int ls_otbn_p384_ecdsa_sign(struct p384_ecdsa_ctx *ctx, struct p384_ecdsa_key *key, struct p384_ecdsa_pkt *pkt)
{
    int error = 0;
    struct otbn_p384_ecdsa_config *cfg_info = (struct otbn_p384_ecdsa_config *)ctx->device->config;
    struct otbn_p384_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeSign;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_P384_ECDSA)
    {
        return -1;
    }

    if(pkt->m == NULL || pkt->m_len != SM2_MSG_DIGSET_BYTES)
    {
        LOG_ERR("input the message digest len is not 48");
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

    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)key->d,cfg_info->kOtbnVarEcdsaD0);
    error |= otbn_func->otbn_dmem_set(otbn,12,0,cfg_info->kOtbnVarEcdsaD1);
    uint8_t rand[384];
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xfc+i;
    }
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,cfg_info->kOtbnVarEcdsaRandomSeed);

    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)pkt->m,cfg_info->kOtbnVarEcdsaMsg);

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

    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaR,(uint32_t *)pkt->r);
    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaS,(uint32_t *)pkt->s);
    
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_p384_ecdsa_verify(struct p384_ecdsa_ctx *ctx, struct p384_ecdsa_key *key, struct p384_ecdsa_pkt *pkt)
{
    int error = 0;
    struct otbn_p384_ecdsa_config *cfg_info = (struct otbn_p384_ecdsa_config *)ctx->device->config;
    struct otbn_p384_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeVerify;
    struct ls_otbn_data *otbn_data = otbn->data;
    uint8_t r_x[48] = {0};
    if(otbn_data->mode != OTBN_P384_ECDSA)
    {
        return -1;
    }

    if(pkt->m == NULL || pkt->m_len != SM2_MSG_DIGSET_BYTES)
    {
        LOG_ERR("input the message digest len is not 48");
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
    
    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)key->qy,cfg_info->kOtbnVarEcdsaY);
    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)key->qx,cfg_info->kOtbnVarEcdsaX);
    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)pkt->r,cfg_info->kOtbnVarEcdsaR);
    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)pkt->s,cfg_info->kOtbnVarEcdsaS);
    error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)pkt->m,cfg_info->kOtbnVarEcdsaMsg);
    // error |= otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)r_x,cfg_info->kOtbnVarEcdsaXr);
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


    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaXr,(uint32_t *)r_x);

    if(memcmp(r_x, pkt->r, 48))
    {
        error = -1;
    }
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_p384_ecdsa_keygen(struct p384_ecdsa_ctx *ctx, struct p384_ecdsa_key *key)
{
    int error = 0;
    struct otbn_p384_ecdsa_config *cfg_info = (struct otbn_p384_ecdsa_config *)ctx->device->config;
    struct otbn_p384_ecdsa_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnEcdsaModeKeygen;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_P384_ECDSA)
    {
        return -1;
    }

    if(key->qx == NULL || key->qy == NULL || key->d == NULL)
    {
        LOG_ERR("no buffer to load key");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    // otbn_func->otbn_dmem_set(otbn,48,0,cfg_info->kOtbnVarEcdsaX);
    // otbn_func->otbn_dmem_set(otbn,48,0,cfg_info->kOtbnVarEcdsaY);
    // otbn_func->otbn_dmem_set(otbn,48,0,cfg_info->kOtbnVarEcdsaD0);

    uint8_t rand[384];
    memset(rand,0xff,384);
    // rand[0] = 0x33;
    // rand[10] = 0x33;
    // rand[255] = 0x1;
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xfe + i;
    }
    otbn_func->otbn_dmem_write(otbn,12,(uint32_t *)rand,cfg_info->kOtbnVarEcdsaRandomSeed);

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

    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaD0,(uint32_t *)key->d);
    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaX,(uint32_t *)key->qx);
    otbn_func->otbn_dmem_read(otbn,12,cfg_info->kOtbnVarEcdsaY,(uint32_t *)key->qy);
    LOG_INF("get p384_ecdsa key succeed");

exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

int ls_otbn_p384_ecdsa_session_free(const struct device *dev, struct p384_ecdsa_ctx *ctx)
{
    // NOT TO DO
    return 0;
}

static int ls_otbn_p384_ecdsa_session_setup(const struct device *dev,
				      struct p384_ecdsa_ctx *ctx,
				      struct p384_ecdsa_key *key)
{
    int error = 0;
    struct otbn_p384_ecdsa_config *cfg_info = (struct otbn_p384_ecdsa_config *)dev->config;
    const struct otbn_p384_ecdsa_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    struct ls_otbn_data *otbn_data = otbn->data;
    k_sem_take(data->otbn_mutex, K_FOREVER);

    otbn_func->otbn_dmem_sec_wipe(otbn);
    otbn_func->otbn_imem_sec_wipe(otbn);
    error = otbn_func->otbn_load_app(otbn,&cfg_info->kOtbnAppP384Ecdsa);
    if(error != 0)
    {
        LOG_ERR("%s trigger error,please check the app image info !",__func__);
        goto exti;
    }

    otbn_data->mode = OTBN_P384_ECDSA;
    ctx->device = dev;
    ctx->ops.keygen = ls_otbn_p384_ecdsa_keygen;
    ctx->ops.sign = ls_otbn_p384_ecdsa_sign;
    ctx->ops.verify = ls_otbn_p384_ecdsa_verify;

exti:
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_p384_ecdsa_init(const struct device *dev)
{
    struct otbn_p384_ecdsa_config *cfg_info = (struct otbn_p384_ecdsa_config *)dev->config;
    struct otbn_p384_ecdsa_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    struct ls_otbn_data *otbn_data = otbn->data;

    /*通过指针获取otbn的锁，多个加密驱动共用这个锁*/
    data->otbn_mutex = &otbn_data->mutex;
    // LOG_INF("P384 INFO");
    // LOG_INF("imem size = 0x%x, dmem size = 0x%x",4*(cfg_info->kOtbnAppP384Ecdsa.imem_end - cfg_info->kOtbnAppP384Ecdsa.imem_start),
    // 4*(cfg_info->kOtbnAppP384Ecdsa.dmem_data_end - cfg_info->kOtbnAppP384Ecdsa.dmem_data_start));
    // LOG_HEXDUMP_INF((uint8_t *)cfg_info->kOtbnAppP384Ecdsa.imem_start,4*(cfg_info->kOtbnAppP384Ecdsa.imem_end - cfg_info->kOtbnAppP384Ecdsa.imem_start),"imem:");
    // LOG_HEXDUMP_INF((uint8_t *)cfg_info->kOtbnAppP384Ecdsa.dmem_data_start,4*(cfg_info->kOtbnAppP384Ecdsa.dmem_data_end - cfg_info->kOtbnAppP384Ecdsa.dmem_data_start),"dmem:");

    // LOG_INF("checksum : 0x%lx",cfg_info->kOtbnAppP384Ecdsa.checksum);
    return 0;
}



static struct p384_ecdsa_driver_api  ls_p384_ecdsa_driver_api = {
    .begin_session = ls_otbn_p384_ecdsa_session_setup,
    .free_session = ls_otbn_p384_ecdsa_session_free,
    .query_hw_caps = NULL,
};


#define LS_OTBN_P384_ECDSA_INIT(idx)                    \
    \
	static void p384_ecdsa_irq_config_func_##idx(const struct device *dev){}              \
	static struct otbn_p384_ecdsa_data otbn_p384_ecdsa_data_##idx;                         \
    OTBN_DECLARE_APP_SYMBOLS(p384_ecdsa);\
    \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, mode);  \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, msg);   \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, r);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, s);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, x);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, y);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa,d0);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa,d1);     \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, x_r);   \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, random_seed);    \
    \
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, MODE_KEYGEN);\
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, MODE_SIGN);\
    OTBN_DECLARE_SYMBOL_ADDR(p384_ecdsa, MODE_VERIFY);\
    \
	static const struct otbn_p384_ecdsa_config otbn_p384_ecdsa_config_##idx = {            \
        .kOtbnAppP384Ecdsa =  OTBN_APP_T_INIT(p384_ecdsa),                      \
        .kOtbnVarEcdsaMode = OTBN_ADDR_T_INIT(p384_ecdsa, mode),            \
        .kOtbnVarEcdsaMsg  = OTBN_ADDR_T_INIT(p384_ecdsa, msg),         \
        .kOtbnVarEcdsaR    =   OTBN_ADDR_T_INIT(p384_ecdsa, r),         \
        .kOtbnVarEcdsaS    =   OTBN_ADDR_T_INIT(p384_ecdsa, s),         \
        .kOtbnVarEcdsaX    =   OTBN_ADDR_T_INIT(p384_ecdsa, x),         \
        .kOtbnVarEcdsaY    =   OTBN_ADDR_T_INIT(p384_ecdsa, y),         \
        .kOtbnVarEcdsaD0   =   OTBN_ADDR_T_INIT(p384_ecdsa, d0),            \
        .kOtbnVarEcdsaD1   =   OTBN_ADDR_T_INIT(p384_ecdsa, d1),            \
        .kOtbnVarEcdsaXr   =   OTBN_ADDR_T_INIT(p384_ecdsa, x_r),           \
        .kOtbnVarEcdsaRandomSeed = OTBN_ADDR_T_INIT(p384_ecdsa, random_seed),\
        .kOtbnEcdsaModeKeygen = OTBN_ADDR_T_INIT(p384_ecdsa, MODE_KEYGEN),\
        .kOtbnEcdsaModeSign = OTBN_ADDR_T_INIT(p384_ecdsa, MODE_SIGN),\
        .kOtbnEcdsaModeVerify = OTBN_ADDR_T_INIT(p384_ecdsa, MODE_VERIFY),\
        .otbn_imem_addr = 0x4000,   \
        .otbn_dmem_addr = 0x8000,   \
		.irq_config_func = p384_ecdsa_irq_config_func_##idx,                        \
        .otbn = DEVICE_DT_GET(DT_INST_PARENT(idx)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_p384_ecdsa_init, NULL, &otbn_p384_ecdsa_data_##idx,    \
			      &otbn_p384_ecdsa_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&ls_p384_ecdsa_driver_api);



DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_P384_ECDSA_INIT)