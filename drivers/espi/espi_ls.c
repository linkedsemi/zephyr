#define DT_DRV_COMPAT linkedsemi_ls_espi

#include <zephyr/drivers/espi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include "reg_espi_type.h"
LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

struct espi_ls_config{
	void (*irq_config_func)(const struct device *);   //函数指针类型要重新定义
    reg_espi_t *reg;    
    const struct pinctrl_dev_config *pcfg;
};

struct espi_ls_data{
	sys_slist_t callbacks;

};

static int espi_ls_configure(const struct device *dev, struct espi_cfg *cfg)
{
    return 0;
}

static bool espi_ls_get_channel_status(const struct device *dev,enum espi_channel ch)
{
    return false;
}

static int espi_ls_read_request(const struct device *dev,struct espi_request_packet *req)
{
    return 0;
}

static int espi_ls_write_request(const struct device *dev,struct espi_request_packet *req)
{
    return 0;
}

static int espi_ls_read_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    return 0;
}

static int espi_ls_write_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    return 0;
}

static int espi_ls_send_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t level)
{
    return 0;
}

static int espi_ls_receive_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t *level)
{
    return 0;
}

static int espi_ls_send_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_receive_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_read(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_write(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_erase(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_manage_callback(const struct device *dev,struct espi_callback *callback,bool set)
{
    return 0;
}

static const struct espi_driver_api espi_ls_driver_api = {
    .config = espi_ls_configure,
    .get_channel_status = espi_ls_get_channel_status,
    .read_request = espi_ls_read_request,
    .write_request = espi_ls_write_request,
    .read_lpc_request = espi_ls_read_lpc_request,
    .write_lpc_request = espi_ls_write_lpc_request,
    .send_vwire = espi_ls_send_vwire,
    .receive_vwire = espi_ls_receive_vwire,
    .send_oob = espi_ls_send_oob,
    .receive_oob = espi_ls_receive_oob,
    .flash_read = espi_ls_flash_read,
    .flash_write = espi_ls_flash_write,
    .flash_erase = espi_ls_flash_erase,
    .manage_callback = espi_ls_manage_callback,
};

static int espi_ls_init(const struct device *dev)
{
	const struct espi_ls_config *const cfg = dev->config;
	struct espi_ls_data *const data = dev->data;
    int ret;
    cfg->irq_config_func(dev);
    ret = pinctrl_apply_state(cfg->pcfg,PINCTRL_STATE_DEFAULT);


    return 0;
}

static void ls_espi_isr(void *arg)
{

}

#define LS_ESPI_INIT(idx)\
    PINCTRL_DT_INST_DEFINE(idx);\
    static void espi_ls_irq_config_func_##idx(const struct device *dev)	\
    {\
        IRQ_CONNECT(DT_INST_IRQN(idx),DT_INST_IRQ(idx, priority),\
                ls_espi_isr,DEVICE_DT_INST_GET(idx), 0);\
        irq_enable(DT_INST_IRQN(idx));\
    }\
    static const struct espi_ls_config espi_ls_cfg_##idx = {\
        .reg = (reg_espi_t *)DT_INST_REG_ADDR(idx),\
        .irq_config_func = espi_ls_irq_config_func_##idx,\
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),\
    };\
    static struct espi_ls_data espi_ls_data_##idx;\
    DEVICE_DT_INST_DEFINE(idx,espi_ls_init,NULL,&espi_ls_data_##idx,\
        &espi_ls_cfg_##idx,PRE_KERNEL_2,CONFIG_ESPI_INIT_PRIORITY,\
        &espi_ls_driver_api);
            
DT_INST_FOREACH_STATUS_OKAY(LS_ESPI_INIT)