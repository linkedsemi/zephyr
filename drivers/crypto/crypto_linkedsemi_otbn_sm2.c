/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 LINKEDSEMI Technology Inc.
 */
#define DT_DRV_COMPAT linkedsemi_otbn_sm2

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
// #include <zephyr/crypto/sm2.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>
#include "qsh.h"

#include <zephyr/crypto/ls_otbn_sm2.h>

LOG_MODULE_DECLARE(otbn,LOG_LEVEL_DBG);

#define SM2_MSG_DIGSET_BYTES 32

struct otbn_sm2_data{
    struct k_sem *otbn_mutex;
    // struct aspeed_sm2_ctx key;
};



struct otbn_sm2_config
{
    // OTBN app.
    otbn_app_t kOtbnAppSm2;
    // Record offsets for input and output buffers of imem or dmem.
    otbn_addr_t kOtbnVarSm2Mode;
    otbn_addr_t kOtbnVarSm2Msg;
    otbn_addr_t kOtbnVarSm2R;
    otbn_addr_t kOtbnVarSm2S;
    otbn_addr_t kOtbnVarSm2X;
    otbn_addr_t kOtbnVarSm2Y;
    otbn_addr_t kOtbnVarSm2D0;
    otbn_addr_t kOtbnVarSm2Xr;
    otbn_addr_t kOtbnVarSm2Rand1;
    // mode constants.
    uint32_t kOtbnSm2ModeKeygen;
    uint32_t kOtbnSm2ModeSign;
    uint32_t kOtbnSm2ModeVerify;

    reg_otbn_t *otbn_reg_addr;     // SEC_OTBN_ADDR
    uint32_t otbn_imem_addr;    // otbn_reg_addr + 0x4000
    uint32_t otbn_dmem_addr;    // otbn_reg_addr + 0x8000
    const struct device *otbn;
    void (*irq_config_func)(const struct device *);
};




static int ls_otbn_sm2_sign(struct sm2_ctx *ctx, struct sm2_key *key, struct sm2_pkt *pkt)
{
    int error = 0;
    const struct otbn_sm2_config *cfg_info = (struct otbn_sm2_config *)ctx->device->config;
    struct otbn_sm2_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnSm2ModeSign;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_SM2)
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarSm2Mode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->d,cfg_info->kOtbnVarSm2D0);
    uint8_t rand[256];
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xfc+i;
    }
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,cfg_info->kOtbnVarSm2Rand1);

    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,cfg_info->kOtbnVarSm2Msg);

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

    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2R,(uint32_t *)pkt->r);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2S,(uint32_t *)pkt->s);
    
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_sm2_verify(struct sm2_ctx *ctx, struct sm2_key *key, struct sm2_pkt *pkt)
{
    int error = 0;
    const struct otbn_sm2_config *cfg_info = (struct otbn_sm2_config *)ctx->device->config;
    struct otbn_sm2_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnSm2ModeVerify;
    struct ls_otbn_data *otbn_data = otbn->data;
    uint8_t r_x[32] = {0};
    if(otbn_data->mode != OTBN_SM2)
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarSm2Mode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }
    
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qy,cfg_info->kOtbnVarSm2Y);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qx,cfg_info->kOtbnVarSm2X);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->r,cfg_info->kOtbnVarSm2R);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->s,cfg_info->kOtbnVarSm2S);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,cfg_info->kOtbnVarSm2Msg);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)r_x,cfg_info->kOtbnVarSm2Xr);
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


    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2Xr,(uint32_t *)r_x);

    if(memcmp(r_x, pkt->r, 32))
    {
        error = -1;
    }
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_sm2_keygen(struct sm2_ctx *ctx, struct sm2_key *key)
{
    int error = 0;
    const struct otbn_sm2_config *cfg_info = (struct otbn_sm2_config *)ctx->device->config;
    struct otbn_sm2_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = cfg_info->kOtbnSm2ModeKeygen;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_SM2)
    {
        return -1;
    }

    if(key->qx == NULL || key->qy == NULL || key->d == NULL)
    {
        LOG_ERR("no buffer to load key");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    uint8_t rand[256];
    memset(rand,0xff,256);
    rand[0] = 0x1;
    rand[255] = 0x1;
    for(uint8_t i =0; i<255;i++)
    {
        rand[i] = 0xff;
    }
    otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,cfg_info->kOtbnVarSm2Rand1);

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,cfg_info->kOtbnVarSm2Mode);
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

    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2D0,(uint32_t *)key->d);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2X,(uint32_t *)key->qx);
    otbn_func->otbn_dmem_read(otbn,8,cfg_info->kOtbnVarSm2Y,(uint32_t *)key->qy);
    LOG_INF("get sm2 key succeed");

exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

int ls_otbn_sm2_session_free(const struct device *dev, struct sm2_ctx *ctx)
{
    // NOT TO DO
    return 0;
}

static int ls_otbn_sm2_session_setup(const struct device *dev,
				      struct sm2_ctx *ctx,
				      struct sm2_key *key)
{
    int error = 0;
    const struct otbn_sm2_config *cfg_info = (struct otbn_sm2_config *)dev->config;
    struct otbn_sm2_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    struct ls_otbn_data *otbn_data = otbn->data;
    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_load_app(otbn,&cfg_info->kOtbnAppSm2);
    if(error != 0)
    {
        LOG_ERR("%s trigger error,please check the app image info !",__func__);
        return -1;
    }

    otbn_data->mode = OTBN_SM2;
    ctx->device = dev;
    ctx->ops.keygen = ls_otbn_sm2_keygen;
    ctx->ops.sign = ls_otbn_sm2_sign;
    ctx->ops.verify = ls_otbn_sm2_verify;

    k_sem_give(data->otbn_mutex);

    return error;
}

static int ls_sm2_init(const struct device *dev)
{
    const struct otbn_sm2_config *cfg_info = (struct otbn_sm2_config *)dev->config;
    struct otbn_sm2_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    struct ls_otbn_data *otbn_data = otbn->data;

    /*通过指针获取otbn的锁，多个加密驱动共用这个锁*/
    data->otbn_mutex = &otbn_data->mutex;
    // LOG_INF("SM2 INFO");
    // LOG_HEXDUMP_INF((uint8_t *)cfg_info->kOtbnAppSm2.imem_start,4*(cfg_info->kOtbnAppSm2.imem_end - cfg_info->kOtbnAppSm2.imem_start),"imem:");
    // LOG_HEXDUMP_INF((uint8_t *)cfg_info->kOtbnAppSm2.dmem_data_start,4*(cfg_info->kOtbnAppSm2.dmem_data_end - cfg_info->kOtbnAppSm2.dmem_data_start),"dmem:");

    // LOG_INF("checksum : 0x%lx",cfg_info->kOtbnAppSm2.checksum);
    return 0;
}



static struct sm2_driver_api ls_sm2_driver_api = {
    .begin_session = ls_otbn_sm2_session_setup,
    .free_session = ls_otbn_sm2_session_free,
    .query_hw_caps = NULL,
};


#define LS_OTBN_SM2_INIT(idx)                    \
    \
	static void sm2_irq_config_func_##idx(const struct device *dev){}              \
	static struct otbn_sm2_data otbn_sm2_data_##idx;                         \
    OTBN_DECLARE_APP_SYMBOLS(sm2);\
    \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, mode);  \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, msg);   \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, r);     \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, s);     \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, x);     \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, y);     \
    OTBN_DECLARE_SYMBOL_ADDR(sm2,d0);     \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, x_r);   \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, rand1);   \
    \
    OTBN_DECLARE_SYMBOL_ADDR(sm2, MODE_KEYGEN);\
    OTBN_DECLARE_SYMBOL_ADDR(sm2, MODE_SIGN);\
    OTBN_DECLARE_SYMBOL_ADDR(sm2, MODE_VERIFY);\
    \
	static const struct otbn_sm2_config otbn_sm2_config_##idx = {            \
        .kOtbnAppSm2 =  OTBN_APP_T_INIT(sm2),                      \
        .kOtbnVarSm2Mode = OTBN_ADDR_T_INIT(sm2, mode),            \
        .kOtbnVarSm2Msg  = OTBN_ADDR_T_INIT(sm2, msg),         \
        .kOtbnVarSm2R    =   OTBN_ADDR_T_INIT(sm2, r),         \
        .kOtbnVarSm2S    =   OTBN_ADDR_T_INIT(sm2, s),         \
        .kOtbnVarSm2X    =   OTBN_ADDR_T_INIT(sm2, x),         \
        .kOtbnVarSm2Y    =   OTBN_ADDR_T_INIT(sm2, y),         \
        .kOtbnVarSm2D0   =   OTBN_ADDR_T_INIT(sm2, d0),            \
        .kOtbnVarSm2Xr   =   OTBN_ADDR_T_INIT(sm2, x_r),           \
        .kOtbnVarSm2Rand1 = OTBN_ADDR_T_INIT(sm2, rand1),           \
        .kOtbnSm2ModeKeygen = OTBN_ADDR_T_INIT(sm2, MODE_KEYGEN),\
        .kOtbnSm2ModeSign = OTBN_ADDR_T_INIT(sm2, MODE_SIGN),\
        .kOtbnSm2ModeVerify = OTBN_ADDR_T_INIT(sm2, MODE_VERIFY),\
        .otbn_imem_addr = 0x4000,   \
        .otbn_dmem_addr = 0x8000,   \
		.irq_config_func = sm2_irq_config_func_##idx,                        \
        .otbn = DEVICE_DT_GET(DT_INST_PARENT(idx)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_sm2_init, NULL, &otbn_sm2_data_##idx,    \
			      &otbn_sm2_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&ls_sm2_driver_api);



DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_SM2_INIT)