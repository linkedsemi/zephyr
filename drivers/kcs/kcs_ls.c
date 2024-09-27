#define DT_DRV_COMPAT linkedsemi_ls_kcs

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <zephyr/drivers/kcs.h>
#include <zephyr/logging/log.h>
#include "espi_lpc_common.h"
LOG_MODULE_REGISTER(kcs_ls, LOG_LEVEL_DBG);

struct kcs_ls_config {
    struct peri_ioport_content data;
    struct peri_ioport_content cmd_stt;
    const struct device *parent;
};

struct kcs_ls_data {
    struct peri_ioport io_data;
    struct peri_ioport io_cmd_stt;
    ibf_callback_t callback;
    struct k_spinlock lock;
    uint8_t status;
    uint8_t data_out;
    uint8_t data_in;
};

static int kcs_ls_init(const struct device *dev)
{
    struct kcs_ls_data *data = dev->data;
    struct kcs_ls_config *cfg = dev->config;
    if(!device_is_ready(cfg->parent))
    {
		LOG_DBG("%s device not ready", cfg->parent->name);
		return -ENODEV;
    }
    data->io_data.content = &cfg->data;
    data->io_cmd_stt.content = &cfg->cmd_stt;
    espi_lpc_add_ioport(cfg->parent,&data->io_data);
    espi_lpc_add_ioport(cfg->parent,&data->io_cmd_stt);
    return 0;
}

static int kcs_ls_read_data(const struct device *dev,uint8_t *data)
{
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
    dev_data->status &= ~KCS_IBF;
    *data = dev_data->data_in;
    k_spin_unlock(&dev_data->lock,key);
    return 0;
}

static int kcs_ls_write_data(const struct device *dev,uint8_t data)
{
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
    dev_data->status |= KCS_OBF;
    dev_data->data_out = data;
    k_spin_unlock(&dev_data->lock,key);
    //up irq
    return 0;
}

static int kcs_ls_read_status(const struct device *dev,uint8_t *status)
{
    struct kcs_ls_data *dev_data = dev->data;
    *status = dev_data->status;
    return 0;
}

static int kcs_ls_update_status(const struct device *dev,uint8_t mask,uint8_t val)
{
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
    dev_data->status = (dev_data->status & ~mask) | val;
    k_spin_unlock(&dev_data->lock,key);
    return 0;
}

static int kcs_ls_set_ibf_callback(const struct device *dev,ibf_callback_t callback)
{
    struct kcs_ls_data *dev_data = dev->data;
    dev_data->callback = callback;
    return 0;
}

static const struct kcs_driver_api kcs_ls_driver_api = {
    .read_data = kcs_ls_read_data,
    .write_data = kcs_ls_write_data,
    .read_status = kcs_ls_read_status,
    .update_status = kcs_ls_update_status,
    .set_ibf_callback = kcs_ls_set_ibf_callback,
};

static void data_cmd_stt_iowr(const struct device *dev,uint8_t size,uint8_t *data,bool is_data)
{
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
	dev_data->data_in = data[0];
	dev_data->status |= KCS_IBF;
	if(is_data)
	{
		dev_data->status &= ~KCS_CMD_DAT;
	}else
	{
		dev_data->status |= KCS_CMD_DAT;
	}
    k_spin_unlock(&dev_data->lock,key);
    if(dev_data->callback)
    {
        dev_data->callback(dev);
    }
}

static void data_io_read(struct peri_ioport_content *ioport,uint8_t size,void *res)
{
    uint8_t *val = res;
    struct device *dev = ioport->ctx;
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
	*val = dev_data->data_out;
	dev_data->status &= ~KCS_OBF;
    k_spin_unlock(&dev_data->lock,key);
    //level irq inactive
}

static void data_io_write(struct peri_ioport_content *ioport,uint8_t size,uint8_t *data)
{
    data_cmd_stt_iowr(ioport->ctx,size,data,true);
}

static void cmd_stt_io_read(struct peri_ioport_content *ioport,uint8_t size,void *res)
{
    uint8_t *val = res;
    struct device *dev = ioport->ctx;
    struct kcs_ls_data *dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);
	*val = dev_data->status;
    k_spin_unlock(&dev_data->lock,key);
}

static void cmd_stt_io_write(struct peri_ioport_content *ioport,uint8_t size,uint8_t *data)
{
    data_cmd_stt_iowr(ioport->ctx,size,data,false);
}

#define LS_KCS_INIT(idx)\
    static struct kcs_ls_data kcs_ls_data_##idx;\
    static const struct kcs_ls_config kcs_ls_cfg_##idx = {\
        .data = {\
            .io_read = data_io_read,\
            .io_write = data_io_write,\
            .ctx = DEVICE_DT_INST_GET(idx),\
            .addr = DT_INST_PROP_BY_IDX(idx,port,0),\
        },\
        .cmd_stt = {\
            .io_read = cmd_stt_io_read,\
            .io_write = cmd_stt_io_write,\
            .ctx = DEVICE_DT_INST_GET(idx),\
            .addr = DT_INST_PROP_BY_IDX(idx,port,1),\
        },\
        .parent = DT_INST_PARENT(idx),\
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        &kcs_ls_init,\
        NULL,\
        &kcs_ls_data_##idx,&kcs_ls_cfg_##idx,\
        POST_KERNEL,CONFIG_KCS_INIT_PRIORITY,\
        &kcs_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LS_KCS_INIT)

