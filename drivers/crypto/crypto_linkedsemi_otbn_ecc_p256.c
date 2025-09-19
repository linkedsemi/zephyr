/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2025 LINKEDSEMI Technology Inc.
 */
#define DT_DRV_COMPAT linkedsemi_otbn_ecc_p256

#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
// #include <zephyr/crypto/ecc_p256.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include <zephyr/logging/log.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>
#include "qsh.h"

#include <zephyr/crypto/ls_otbn_ecc_p256.h>
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

struct otbn_ecc_p256_data{
    struct k_sem *otbn_mutex;

    // OTBN app.
    // otbn_app_t kOtbnAppP256Ecdsa;
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
    uint32_t kOtbnEcdsaModeSharedKey;

    otbn_app_t app_info;
};


struct otbn_ecc_p256_config
{
    reg_otbn_t *otbn_reg_addr;     // SEC_OTBN_ADDR
    uint32_t otbn_imem_addr;    // otbn_reg_addr + 0x4000
    uint32_t otbn_dmem_addr;    // otbn_reg_addr + 0x8000
    const struct device *otbn;
    void (*irq_config_func)(const struct device *);
};

static int ls_otbn_ecc_p256_sign(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key, struct ecc_p256_pkt *pkt)
{
    int error = 0;
    struct otbn_ecc_p256_config *cfg_info = (struct otbn_ecc_p256_config *)ctx->device->config;
    struct otbn_ecc_p256_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = data->kOtbnEcdsaModeSign;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_ECC_P256)
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarEcdsaMode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    error = otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->d,data->kOtbnVarEcdsaD0);
    uint8_t rand[256];
    otbn_get_random(rand,256);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,data->kOtbnVarEcdsaRandomSeed);

    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,data->kOtbnVarEcdsaMsg);

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

    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaR,(uint32_t *)pkt->r);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaS,(uint32_t *)pkt->s);
    
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_ecc_p256_verify(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key, struct ecc_p256_pkt *pkt)
{
    int error = 0;
    struct otbn_ecc_p256_config *cfg_info = (struct otbn_ecc_p256_config *)ctx->device->config;
    struct otbn_ecc_p256_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = data->kOtbnEcdsaModeVerify;
    struct ls_otbn_data *otbn_data = otbn->data;
    uint8_t r_x[32] = {0};
    if(otbn_data->mode != OTBN_ECC_P256)
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

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarEcdsaMode);
    if(error != 0)
    {
        LOG_ERR("otbn is running, do not write data to dmem");
        goto exit;
    }
    
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qy,data->kOtbnVarEcdsaY);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)key->qx,data->kOtbnVarEcdsaX);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->r,data->kOtbnVarEcdsaR);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->s,data->kOtbnVarEcdsaS);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)pkt->m,data->kOtbnVarEcdsaMsg);
    error |= otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)r_x,data->kOtbnVarEcdsaXr);
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


    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaXr,(uint32_t *)r_x);

    if(memcmp(r_x, pkt->r, 32))
    {
        error = -1;
    }
exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_otbn_ecc_p256_keygen(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key)
{
    int error = 0;
    struct otbn_ecc_p256_config *cfg_info = (struct otbn_ecc_p256_config *)ctx->device->config;
    struct otbn_ecc_p256_data *data = ctx->device->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    uint32_t mode = data->kOtbnEcdsaModeKeygen;
    struct ls_otbn_data *otbn_data = otbn->data;
    if(otbn_data->mode != OTBN_ECC_P256)
    {
        return -1;
    }

    if(key->qx == NULL || key->qy == NULL || key->d == NULL)
    {
        LOG_ERR("no buffer to load key");
        return -1;
    }

    k_sem_take(data->otbn_mutex, K_FOREVER);

    otbn_func->otbn_dmem_set(otbn,32,0,data->kOtbnVarEcdsaX);
    otbn_func->otbn_dmem_set(otbn,32,0,data->kOtbnVarEcdsaY);
    otbn_func->otbn_dmem_set(otbn,32,0,data->kOtbnVarEcdsaD0);

    uint8_t rand[256];
    otbn_get_random(rand,256);
    otbn_func->otbn_dmem_write(otbn,8,(uint32_t *)rand,data->kOtbnVarEcdsaRandomSeed);

    error = otbn_func->otbn_dmem_write(otbn,1,(uint32_t *)&mode,data->kOtbnVarEcdsaMode);
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

    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaD0,(uint32_t *)key->d);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaX,(uint32_t *)key->qx);
    otbn_func->otbn_dmem_read(otbn,8,data->kOtbnVarEcdsaY,(uint32_t *)key->qy);
    LOG_INF("get ecc_p256 key succeed");

exit:
    otbn_func->otbn_dmem_sec_wipe(otbn);
    k_sem_give(data->otbn_mutex);
    return error;
}


static int ls_otbn_ecc_p256_session_setup(const struct device *dev,
				      struct ecc_p256_ctx *ctx,
				      struct ecc_p256_key *key)
{
    int error = 0;
    struct otbn_ecc_p256_config *cfg_info = (struct otbn_ecc_p256_config *)dev->config;
    const struct otbn_ecc_p256_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    const struct otbn_ops_api_t *otbn_func = otbn->api;
    struct ls_otbn_data *otbn_data = otbn->data;
    k_sem_take(data->otbn_mutex, K_FOREVER);

    otbn_func->otbn_dmem_sec_wipe(otbn);
    otbn_func->otbn_imem_sec_wipe(otbn);
    error = otbn_func->otbn_load_app(otbn,&data->app_info);
    if(error != 0)
    {
        LOG_ERR("%s trigger error,please check the app image info !",__func__);
        goto exti;
    }

    otbn_data->mode = OTBN_ECC_P256;
    ctx->device = dev;
    ctx->ops.keygen = ls_otbn_ecc_p256_keygen;
    ctx->ops.sign = ls_otbn_ecc_p256_sign;
    ctx->ops.verify = ls_otbn_ecc_p256_verify;
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
//     irq_config_func = 0x100060a2 <ecc_p256_irq_config_func_0>}

exti:
    k_sem_give(data->otbn_mutex);
    return error;
}

static int ls_ecc_p256_init(const struct device *dev)
{
    struct otbn_ecc_p256_config *cfg_info = (struct otbn_ecc_p256_config *)dev->config;
    struct otbn_ecc_p256_data *data = dev->data;   
    const struct device *otbn = cfg_info->otbn;
    struct ls_otbn_data *otbn_data = otbn->data;

    /*通过指针获取otbn的锁，多个加密驱动共用这个锁*/
    data->otbn_mutex = &otbn_data->mutex;

    data->kOtbnVarEcdsaMode = LS_OTBN_ECDSA_P256_MODE_OFFSET;
    data->kOtbnVarEcdsaMsg = LS_OTBN_ECDSA_P256_MSG_OFFSET;
    data->kOtbnVarEcdsaR = LS_OTBN_ECDSA_P256_R_OFFSET;
    data->kOtbnVarEcdsaS = LS_OTBN_ECDSA_P256_S_OFFSET;
    data->kOtbnVarEcdsaX = LS_OTBN_ECDSA_P256_X_OFFSET;
    data->kOtbnVarEcdsaY = LS_OTBN_ECDSA_P256_Y_OFFSET;
    data->kOtbnVarEcdsaD0 = LS_OTBN_ECDSA_P256_D0_OFFSET;
    data->kOtbnVarEcdsaD1 = LS_OTBN_ECDSA_P256_D1_OFFSET;
    data->kOtbnVarEcdsaXr = LS_OTBN_ECDSA_P256_X_R_OFFSET;
    data->kOtbnVarEcdsaOk = LS_OTBN_ECDSA_P256_OK;
    data->kOtbnVarEcdsaRandomSeed = LS_OTBN_ECDSA_P256_RANDOM_SEED_OFFSET;
    // mode constants.
    data->kOtbnEcdsaModeKeygen = LS_OTBN_ECDSA_P256_MODE_KEYGEN;
    data->kOtbnEcdsaModeSign = LS_OTBN_ECDSA_P256_MODE_SIGN;
    data->kOtbnEcdsaModeVerify = LS_OTBN_ECDSA_P256_MODE_VERIFY;
    data->kOtbnEcdsaModeSharedKey =LS_OTBN_ECDSA_P256_MODE_SHARED_KEY;

    data->app_info.curve = OTBN_ECC_P256;
    data->app_info.kOtbnAppImemSize = LS_OTBN_ECDSA_P256_IMEM_SIZE;
    data->app_info.kOtbnAppDmemSize = LS_OTBN_ECDSA_P256_DMEM_SIZE;
    data->app_info.kOtbnAppDmemEnd = LS_OTBN_ECDSA_P256_DMEM_END;
    data->app_info.dmem_image = (uint8_t *)p256_dmem;
    data->app_info.imem_image = (uint8_t *)p256_imem;
    return 0;
}



static struct ecc_p256_driver_api  ls_ecc_p256_driver_api = {
    .begin_session = ls_otbn_ecc_p256_session_setup,
    // .free_session = ls_otbn_ecc_p256_session_free,
    .query_hw_caps = NULL,
};


#define LS_OTBN_P256_ECDSA_INIT(idx)                    \
    \
	static void ecc_p256_irq_config_func_##idx(const struct device *dev){}              \
	static struct otbn_ecc_p256_data otbn_ecc_p256_data_##idx;                         \
	static const struct otbn_ecc_p256_config otbn_ecc_p256_config_##idx = {            \
        .otbn_imem_addr = 0x4000,   \
        .otbn_dmem_addr = 0x8000,   \
		.irq_config_func = ecc_p256_irq_config_func_##idx,                        \
        .otbn = DEVICE_DT_GET(DT_INST_PARENT(idx)), \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(idx, ls_ecc_p256_init, NULL, &otbn_ecc_p256_data_##idx,    \
			      &otbn_ecc_p256_config_##idx, POST_KERNEL,                        \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, (void *)&ls_ecc_p256_driver_api);



DT_INST_FOREACH_STATUS_OKAY(LS_OTBN_P256_ECDSA_INIT)