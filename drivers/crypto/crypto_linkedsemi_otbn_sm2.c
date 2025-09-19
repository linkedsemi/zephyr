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
#include "ls_otbn_ecc.h"
LOG_MODULE_DECLARE(otbn,LOG_LEVEL_DBG);
static int otbn_get_random(uint8_t *buf, uint16_t buf_len)
{
    static uint16_t c = 0xff;
    for(uint16_t i = 0; i < buf_len; i++)
    {
        buf[i] = i*c + c;
        c++;
    }
    return 0;
}
#define SM2_MSG_DIGSET_BYTES 32

struct otbn_sm2_data{
    struct k_sem *otbn_mutex;

    // OTBN app.
    // otbn_app_t kOtbnAppP256Sm2;
    // Record offsets for input and output buffers of imem or dmem.
    otbn_addr_t kOtbnVarSm2Mode;
    otbn_addr_t kOtbnVarSm2Msg;
    otbn_addr_t kOtbnVarSm2R;
    otbn_addr_t kOtbnVarSm2S;
    otbn_addr_t kOtbnVarSm2X;
    otbn_addr_t kOtbnVarSm2Y;
    otbn_addr_t kOtbnVarSm2D0;
    otbn_addr_t kOtbnVarSm2D1;
    otbn_addr_t kOtbnVarSm2Xr;
    otbn_addr_t kOtbnVarSm2Ok;
    otbn_addr_t kOtbnVarSm2RandomSeed;
    // mode constants.
    uint32_t kOtbnSm2ModeKeygen;
    uint32_t kOtbnSm2ModeSign;
    uint32_t kOtbnSm2ModeVerify;
    uint32_t kOtbnSm2ModeSharedKey;

    uint32_t kOtbnAppImemSize;
    uint32_t kOtbnAppDmemSize;
    uint32_t kOtbnAppDmemEnd;

    otbn_app_t app_info;
};

struct otbn_sm2_config
{
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
    uint32_t mode = data->kOtbnSm2ModeSign;
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarSm2Mode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->d,data->kOtbnVarSm2D0);
    uint8_t rand[256];
    otbn_get_random(rand,256);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,data->kOtbnVarSm2RandomSeed);

    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,data->kOtbnVarSm2Msg);

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

    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2R,(uint32_t *)pkt->r);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2S,(uint32_t *)pkt->s);
    
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
    uint32_t mode = data->kOtbnSm2ModeVerify;
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarSm2Mode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }
    
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qy,data->kOtbnVarSm2Y);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qx,data->kOtbnVarSm2X);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->r,data->kOtbnVarSm2R);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->s,data->kOtbnVarSm2S);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,data->kOtbnVarSm2Msg);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)r_x,data->kOtbnVarSm2Xr);
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


    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2Xr,(uint32_t *)r_x);

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
    uint32_t mode = data->kOtbnSm2ModeKeygen;
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
    otbn_get_random(rand,256);
    otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,data->kOtbnVarSm2RandomSeed);

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarSm2Mode);
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

    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2D0,(uint32_t *)key->d);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2X,(uint32_t *)key->qx);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarSm2Y,(uint32_t *)key->qy);
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

    error = otbn_func->otbn_load_app(otbn,&data->app_info);
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

    data->kOtbnVarSm2Mode = LS_OTBN_SM2_MODE_OFFSET;
    data->kOtbnVarSm2Msg = LS_OTBN_SM2_MSG_OFFSET;
    data->kOtbnVarSm2R = LS_OTBN_SM2_R_OFFSET;
    data->kOtbnVarSm2S = LS_OTBN_SM2_S_OFFSET;
    data->kOtbnVarSm2X = LS_OTBN_SM2_X_OFFSET;
    data->kOtbnVarSm2Y = LS_OTBN_SM2_Y_OFFSET;
    data->kOtbnVarSm2D0 = LS_OTBN_SM2_D0_OFFSET;
    data->kOtbnVarSm2D1 = LS_OTBN_SM2_D1_OFFSET;
    data->kOtbnVarSm2Xr = LS_OTBN_SM2_X_R_OFFSET;
    data->kOtbnVarSm2Ok = LS_OTBN_SM2_OK;
    data->kOtbnVarSm2RandomSeed = LS_OTBN_SM2_RANDOM_SEED_OFFSET;
    // mode constants.
    data->kOtbnSm2ModeKeygen = LS_OTBN_SM2_MODE_KEYGEN;
    data->kOtbnSm2ModeSign = LS_OTBN_SM2_MODE_SIGN;
    data->kOtbnSm2ModeVerify = LS_OTBN_SM2_MODE_VERIFY;
    data->kOtbnSm2ModeSharedKey =LS_OTBN_SM2_MODE_SHARED_KEY;

    data->app_info.curve = OTBN_ECC_P384;
    data->app_info.kOtbnAppImemSize = LS_OTBN_SM2_IMEM_SIZE;
    data->app_info.kOtbnAppDmemSize = LS_OTBN_SM2_DMEM_SIZE;
    data->app_info.kOtbnAppDmemEnd = LS_OTBN_SM2_DMEM_END;
    data->app_info.dmem_image = (uint8_t *)sm2_dmem;
    data->app_info.imem_image = (uint8_t *)sm2_imem;
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
	static const struct otbn_sm2_config otbn_sm2_config_##idx = {            \
        .otbn_imem_addr = 0x4000,   \
        .otbn_dmem_addr = 0x8000,   \
		.irq_config_func = sm2_irq_config_func_##idx,                        \
        .otbn = DEVICE_DT_GET(DT_INST_PARENT(idx)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_sm2_init, NULL, &otbn_sm2_data_##idx,    \
			      &otbn_sm2_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&ls_sm2_driver_api);



DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_SM2_INIT)