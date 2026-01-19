#define DT_DRV_COMPAT linkedsemi_ls_host_kcs
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/kcs.h>
#include <zephyr/logging/log.h>
#include "espi_lpc_common.h"
LOG_MODULE_REGISTER(ls_host_kcs, LOG_LEVEL_DBG);

struct host_kcs_ls_config {
    struct peri_ioport_content data;
    struct peri_ioport_content cmd_stt;
    const struct device *espi_lpc;
    const struct upstream_irq_type *up_irq;
    struct host_kcs_env *kcs_env;
    struct host_bmc_msg_exch hb_exch;
};

struct host_kcs_ls_data {
    struct peri_ioport io_data;
    struct peri_ioport io_cmd_stt;
};

static void data_cmd_stt_iowr(const struct device *dev,uint8_t size,uint8_t *data,bool is_data)
{
    const struct host_kcs_ls_config *cfg = dev->config;
    kcs_env_lock(cfg->kcs_env);
    cfg->kcs_env->data_in = data[0];
    cfg->kcs_env->status |= KCS_IBF;
    if(is_data)
    {
        cfg->kcs_env->status &= ~KCS_CMD_DAT;
    }else
    {
        cfg->kcs_env->status |= KCS_CMD_DAT;
    }
    kcs_env_unlock(cfg->kcs_env);
    kcs_h2b_send_ibf(&cfg->hb_exch);
}

static void data_io_read(const struct peri_ioport_content *ioport,uint8_t size,void *res)
{
    uint8_t *val = res;
    struct device *dev = ioport->ctx;
    const struct host_kcs_ls_config *cfg = dev->config;
    kcs_env_lock(cfg->kcs_env);
	*val = cfg->kcs_env->data_out;
	cfg->kcs_env->status &= ~KCS_OBF;
    kcs_env_unlock(cfg->kcs_env);
}

static void data_io_write(const struct peri_ioport_content *ioport,uint8_t size,uint8_t *data)
{
    data_cmd_stt_iowr(ioport->ctx,size,data,true);
}

static void cmd_stt_io_read(const struct peri_ioport_content *ioport,uint8_t size,void *res)
{
    uint8_t *val = res;
    struct device *dev = ioport->ctx;
    const struct host_kcs_ls_config *cfg = dev->config;
    kcs_env_lock(cfg->kcs_env);
	*val = cfg->kcs_env->status;
    kcs_env_unlock(cfg->kcs_env);
}

static void cmd_stt_io_write(const struct peri_ioport_content *ioport,uint8_t size,uint8_t *data)
{
    data_cmd_stt_iowr(ioport->ctx,size,data,false);
}

void host_kcs_rx_callback(const struct device *dev,void *msg)
{
    const struct host_kcs_ls_config *cfg = dev->config;
    if(cfg->up_irq)
    {
        espi_lpc_raise_edge_irq(cfg->espi_lpc,cfg->up_irq->idx);
    }
}

static int host_kcs_ls_init(const struct device *dev)
{
    struct host_kcs_ls_data *data = dev->data;
    const struct host_kcs_ls_config *cfg = dev->config;
    if(!device_is_ready(cfg->espi_lpc))
    {
		LOG_DBG("%s device not ready", cfg->espi_lpc->name);
		return -ENODEV;
    }
    host_bmc_msg_exch_init(&cfg->hb_exch);
    memset(cfg->kcs_env,0,sizeof(*cfg->kcs_env));
    data->io_data.content = &cfg->data;
    data->io_cmd_stt.content = &cfg->cmd_stt;
    int key = arch_irq_lock();
    espi_lpc_add_ioport(cfg->espi_lpc,&data->io_data);
    espi_lpc_add_ioport(cfg->espi_lpc,&data->io_cmd_stt);
    arch_irq_unlock(key);
    return 0;
}

#define HOST_KCS_INIT(idx)\
    IF_ENABLED(DT_HAS_UP_IRQ(idx),(UPSTREAM_IRQ_DT_INST_DEFINE(idx)))\
    static struct host_kcs_ls_data host_kcs_ls_data_##idx;\
    static const struct host_kcs_ls_config host_kcs_ls_cfg_##idx = {\
        .data = {\
            .io_read = data_io_read,\
            .io_write = data_io_write,\
            .ctx = (void *)DEVICE_DT_INST_GET(idx),\
            .addr = DT_INST_PROP_BY_IDX(idx,port,0),\
        },\
        .cmd_stt = {\
            .io_read = cmd_stt_io_read,\
            .io_write = cmd_stt_io_write,\
            .ctx = (void *)DEVICE_DT_INST_GET(idx),\
            .addr = DT_INST_PROP_BY_IDX(idx,port,1),\
        },\
        .espi_lpc = DEVICE_DT_GET(DT_INST_PHANDLE(idx,espi_lpc)),\
        .kcs_env = (struct host_kcs_env *)DT_INST_PROP(idx,kcs_env_addr),\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,host_kcs_rx_callback,bmc_kcs_rx_callback,1),\
        IF_ENABLED(DT_HAS_UP_IRQ(idx),(.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(idx)))\
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        &host_kcs_ls_init,\
        NULL,\
        &host_kcs_ls_data_##idx,&host_kcs_ls_cfg_##idx,\
        POST_KERNEL,CONFIG_KCS_INIT_PRIORITY,\
        NULL);

DT_INST_FOREACH_STATUS_OKAY(HOST_KCS_INIT)
