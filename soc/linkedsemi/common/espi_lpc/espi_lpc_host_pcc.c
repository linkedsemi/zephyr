#define DT_DRV_COMPAT linkedsemi_ls_host_pcc
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>
#include "espi_lpc_common.h"
LOG_MODULE_REGISTER(ls_host_pcc, LOG_LEVEL_DBG);

struct host_pcc_ls_config {
    const struct device *espi_lpc;
    struct host_pcc_env *pcc_env;
    struct host_bmc_msg_exch hb_exch;
    const struct peri_ioport_content *ports_content;
    struct peri_ioport *ports;
    uint8_t port_num;
};

struct host_pcc_ls_data {
    uint8_t dummy;
};

static void host_pcc_port_io_read(const struct peri_ioport_content *ioport,uint8_t size,void *res)
{
    __ASSERT_NO_MSG(0);
}

static void host_pcc_port_io_write(const struct peri_ioport_content *ioport,uint8_t size,uint8_t *data)
{
    struct device *dev = ioport->ctx;
    const struct host_pcc_ls_config *cfg = dev->config;
    struct pcc_data post_code = {
        .post_code = *data,
        .port = ioport->addr,
    };
    general_fifo_put(&cfg->pcc_env->fifo,&post_code);
    post_code_msg_send(&cfg->hb_exch,&post_code);
}


void host_pcc_rx_callback(const struct device *dev,void *msg)
{

}

static int host_pcc_ls_init(const struct device *dev)
{
    const struct host_pcc_ls_config *cfg = dev->config;
    struct host_pcc_ls_data *ptr_data = dev->data;
    if(!device_is_ready(cfg->espi_lpc))
    {
        LOG_DBG("%s device not ready", cfg->espi_lpc->name);
        return -ENODEV;
    }
    host_bmc_msg_exch_init(&cfg->hb_exch);
    sw_fifo_init(&cfg->pcc_env->fifo,cfg->pcc_env->data,CONFIG_ESPI_LPC_PCC_FIFO_LENGTH,sizeof(struct pcc_data));
    for(uint8_t i = 0; i < cfg->port_num; i++)
    {
        cfg->ports[i].content = &cfg->ports_content[i];
        espi_lpc_add_ioport(cfg->espi_lpc,&cfg->ports[i]);
    }
    return 0;
}

#define HOST_PCC_PORT_CONTENT_INIT(node_id, prop, elem) {\
    .io_read = host_pcc_port_io_read,\
    .io_write = host_pcc_port_io_write,\
    .ctx = (void *)DEVICE_DT_GET(node_id),\
    .addr = DT_PROP_BY_IDX(node_id,prop,elem),},

#define HOST_PCC_INIT(idx)\
    static const struct peri_ioport_content host_pcc_ports_content_##idx[] = {\
        DT_INST_FOREACH_PROP_ELEM(idx,ports,HOST_PCC_PORT_CONTENT_INIT)\
    };\
    static struct peri_ioport host_pcc_ports_##idx[DT_INST_PROP_LEN(idx,ports)];\
    static struct host_pcc_ls_data host_pcc_ls_data_##idx;\
    static const struct host_pcc_ls_config host_pcc_ls_cfg_##idx = {\
        .espi_lpc = DEVICE_DT_GET(DT_INST_PHANDLE(idx,espi_lpc)),\
        .pcc_env = (struct host_pcc_env *)DT_INST_PROP(idx,pcc_env_addr),\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,host_pcc_rx_callback,bmc_pcc_rx_callback),\
        .ports_content = host_pcc_ports_content_##idx,\
        .ports = host_pcc_ports_##idx,\
        .port_num = DT_INST_PROP_LEN(idx,ports),\
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        host_pcc_ls_init,\
        NULL,\
        &host_pcc_ls_data_##idx,&host_pcc_ls_cfg_##idx,\
        POST_KERNEL,CONFIG_PCC_INIT_PRIORITY,\
        NULL);

DT_INST_FOREACH_STATUS_OKAY(HOST_PCC_INIT)
