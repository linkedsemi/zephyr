#define DT_DRV_COMPAT linkedsemi_ls_kcs

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <zephyr/drivers/kcs.h>
#include <zephyr/logging/log.h>
#include "espi_lpc_common.h"
LOG_MODULE_REGISTER(kcs_ls, LOG_LEVEL_DBG);

struct kcs_ls_config {
    struct host_kcs_env *kcs_env;
    struct host_bmc_msg_exch hb_exch;
};

struct kcs_ls_data {
    ibf_callback_t callback;
    void *param;
};

void bmc_kcs_rx_callback(const struct device *dev,void *msg)
{
    struct kcs_ls_data *data = dev->data;
    if(data->callback)
    {
        data->callback(dev,data->param);
    }
}

static int kcs_ls_init(const struct device *dev)
{
    const struct kcs_ls_config *cfg = dev->config;
    host_bmc_msg_exch_init(&cfg->hb_exch);
    return 0;
}

static int kcs_ls_read_data(const struct device *dev,uint8_t *data)
{
    const struct kcs_ls_config *cfg = dev->config;
    int ret = 0;
    uint32_t key = kcs_env_lock(cfg->kcs_env);
    if(cfg->kcs_env->status & KCS_IBF)
    {
        cfg->kcs_env->status &= ~KCS_IBF;
        *data = cfg->kcs_env->data_in;
    }else
    {
        ret = -EIO;
    }
    kcs_env_unlock(cfg->kcs_env,key);
    return ret;
}

static int kcs_ls_write_data(const struct device *dev,uint8_t data)
{
    const struct kcs_ls_config *cfg = dev->config;
    int ret = 0;
    uint32_t key = kcs_env_lock(cfg->kcs_env);
    if(cfg->kcs_env->status & KCS_OBF)
    {
        ret = -EIO;
    }else
    {
        cfg->kcs_env->status |= KCS_OBF;
        cfg->kcs_env->data_out = data;
    }
    kcs_env_unlock(cfg->kcs_env,key);
    if(ret == 0)
    {
        kcs_b2h_send_obf(&cfg->hb_exch);
    }
    return ret;
}

static int kcs_ls_read_status(const struct device *dev,uint8_t *status)
{
    const struct kcs_ls_config *cfg = dev->config;
    *status = cfg->kcs_env->status;
    return 0;
}

static int kcs_ls_update_status(const struct device *dev,uint8_t mask,uint8_t val)
{
    const struct kcs_ls_config *cfg = dev->config;
    uint32_t key = kcs_env_lock(cfg->kcs_env);
    cfg->kcs_env->status = (cfg->kcs_env->status & ~mask) | val;
    kcs_env_unlock(cfg->kcs_env,key);
    return 0;
}

static int kcs_ls_set_ibf_callback(const struct device *dev,ibf_callback_t callback,void *param)
{
    struct kcs_ls_data *dev_data = dev->data;
    dev_data->callback = callback;
    dev_data->param = param;
    return 0;
}

static const struct kcs_driver_api kcs_ls_driver_api = {
    .read_data = kcs_ls_read_data,
    .write_data = kcs_ls_write_data,
    .read_status = kcs_ls_read_status,
    .update_status = kcs_ls_update_status,
    .set_ibf_callback = kcs_ls_set_ibf_callback,
};

#define LS_KCS_INIT(idx)\
    IF_ENABLED(DT_HAS_UP_IRQ(idx),(UPSTREAM_IRQ_DT_INST_DEFINE(idx)))\
    static struct kcs_ls_data kcs_ls_data_##idx;\
    static const struct kcs_ls_config kcs_ls_cfg_##idx = {\
        .kcs_env = (struct host_kcs_env *)DT_INST_PROP(idx,kcs_env_addr),\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,bmc_kcs_rx_callback,host_kcs_rx_callback),\
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        &kcs_ls_init,\
        NULL,\
        &kcs_ls_data_##idx,&kcs_ls_cfg_##idx,\
        POST_KERNEL,CONFIG_KCS_INIT_PRIORITY,\
        &kcs_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LS_KCS_INIT)

