#define DT_DRV_COMPAT linkedsemi_ls_pcc

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/pcc.h>
#include <zephyr/logging/log.h>
#include "espi_lpc_common.h"
LOG_MODULE_REGISTER(pcc_ls, LOG_LEVEL_DBG);

struct pcc_ls_config {
    struct host_pcc_env *pcc_env;
    struct host_bmc_msg_exch hb_exch;
};

struct pcc_ls_data {
    pcc_callback_t callback;
};

/*
 * Invoked when the host side captured a post code, either directly in single
 * image builds or through the mbox in dual core builds. The message carries
 * the struct pcc_data that host_pcc_port_io_write() just pushed to the shared
 * FIFO.
 */
void bmc_pcc_rx_callback(const struct device *dev,void *msg)
{
    struct pcc_ls_data *data = dev->data;
    if(data->callback)
    {
        data->callback((struct pcc_data *)msg);
    }
}

static int pcc_ls_read_post_code(const struct device *dev,struct pcc_data *buf,size_t *length)
{
    const struct pcc_ls_config *cfg = dev->config;
    size_t count = 0;
    if(buf == NULL || length == NULL)
    {
        return -EINVAL;
    }
    while(count < *length && general_fifo_get(&cfg->pcc_env->fifo,&buf[count]))
    {
        count++;
    }
    *length = count;
    return 0;
}

static int pcc_ls_set_callback(const struct device *dev,pcc_callback_t callback)
{
    struct pcc_ls_data *dev_data = dev->data;
    dev_data->callback = callback;
    return 0;
}

static const struct pcc_driver_api pcc_ls_driver_api = {
    .read_post_code = pcc_ls_read_post_code,
    .set_callback = pcc_ls_set_callback,
};

static int pcc_ls_init(const struct device *dev)
{
    const struct pcc_ls_config *cfg = dev->config;
    host_bmc_msg_exch_init(&cfg->hb_exch);
    return 0;
}

#define LS_PCC_INIT(idx)\
    static struct pcc_ls_data pcc_ls_data_##idx;\
    static const struct pcc_ls_config pcc_ls_cfg_##idx = {\
        .pcc_env = (struct host_pcc_env *)DT_INST_PROP(idx,pcc_env_addr),\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,bmc_pcc_rx_callback,host_pcc_rx_callback),\
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        &pcc_ls_init,\
        NULL,\
        &pcc_ls_data_##idx,&pcc_ls_cfg_##idx,\
        POST_KERNEL,CONFIG_PCC_INIT_PRIORITY,\
        &pcc_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LS_PCC_INIT)
