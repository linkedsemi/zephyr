#define DT_DRV_COMPAT linkedsemi_ls_lpc

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include <zephyr/drivers/lpc.h>
#include <zephyr/logging/log.h>
#include "reg_lpc_type.h"
#include "espi_lpc_common.h"

LOG_MODULE_REGISTER(lpc_ls, LOG_LEVEL_DBG);
#define LPC_TYPE_IO_READ                0x00
#define LPC_TYPE_IO_WRITE               0x20
#define LPC_TYPE_MEM_READ               0x40
#define LPC_TYPE_MEM_WRITE              0x60

static void lpc_reg_init(const struct device *dev)
{
	const struct espi_lpc_ls_config *const cfg = dev->config;
    reg_lpc_t *reg = cfg->reg;
    reg->INTR_CLR = LPC_INTR_STT_CMD_VLD_MASK|LPC_INTR_STT_SYNC_TO_MASK|LPC_INTR_STT_SERIRQ_STOP_MASK|LPC_INTR_STT_SERIRQ_STOP_TO_MASK|LPC_INTR_STT_SERIRQ_STOP_IVLD_MASK;
    reg->INTR_MSK = LPC_INTR_STT_CMD_VLD_MASK|LPC_INTR_STT_SYNC_TO_MASK|LPC_INTR_STT_SERIRQ_STOP_MASK|LPC_INTR_STT_SERIRQ_STOP_TO_MASK|LPC_INTR_STT_SERIRQ_STOP_IVLD_MASK;
}

static int lpc_ls_init(const struct device *dev)
{
	const struct espi_lpc_ls_config *const cfg = dev->config;
	struct espi_lpc_ls_data *const data = dev->data;
    int ret;
	sys_slist_init(&data->peri_io);
	sys_slist_init(&data->peri_mem);
	sys_slist_init(&data->callbacks);
    cfg->irq_config_func(dev);
    if(cfg->cctl_cfg.cctl_dev)
    {
		const struct device *clk_dev = cfg->cctl_cfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_DBG("%s device not ready", clk_dev->name);
			return -ENODEV;
		}
		clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->cctl_cfg);
    }
    ret = pinctrl_apply_state(cfg->pcfg,PINCTRL_STATE_DEFAULT);
    if(ret)
    {
        return ret;
    }
    lpc_reg_init(dev);
    return 0;
}

static int lpc_ls_read_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{

    return 0;
}

static int lpc_ls_write_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{

    return 0;
}

static const struct lpc_driver_api lpc_ls_driver_api = {
    .read_request = lpc_ls_read_request,
    .write_request = lpc_ls_write_request,
};

static void serirq_int_set(const struct device *dev,bool unmask)
{
	const struct espi_lpc_ls_config *cfg = dev->config;
	struct espi_lpc_ls_data *dev_data = dev->data;
    reg_lpc_t *reg = cfg->reg;
    k_spinlock_key_t key = k_spin_lock(&dev_data->u.lpc.int_msk_lock);
    uint32_t int_msk = reg->INTR_MSK;
    if(!unmask)
    {
        int_msk &= ~LPC_INTR_STT_SERIRQ_STOP_MASK;
    }else
    {
        int_msk |= LPC_INTR_STT_SERIRQ_STOP_MASK;
    }
    reg->INTR_MSK = int_msk;
    k_spin_unlock(&dev_data->u.lpc.int_msk_lock,key);
}

static void ls_lpc_isr(void *arg)
{
	struct device *dev = (struct device *) arg;
	const struct espi_lpc_ls_config *cfg = dev->config;
	struct espi_lpc_ls_data *dev_data = dev->data;
    reg_lpc_t *reg = cfg->reg;
    uint32_t stt = reg->INTR_STT;
    if(stt&LPC_INTR_STT_CMD_VLD_MASK)
    {
        uint32_t addr = reg->LPC_ADDR;
        uint32_t rx = reg->LPC_RX;
        uint8_t start_cycle_dir = rx & 0xff;
        uint8_t data = rx >> 8;
        uint32_t res = 0;
        reg->INTR_CLR = LPC_INTR_STT_CMD_VLD_MASK;
        switch(start_cycle_dir)
        {
        case LPC_TYPE_IO_READ:
            iord_short(dev_data,1,addr,&res);
        break;
        case LPC_TYPE_IO_WRITE:
            iowr_short(dev_data,1,addr,&data);
        break;
        case LPC_TYPE_MEM_READ:
            memrd_short(dev_data,1,addr,&res);
        break;
        case LPC_TYPE_MEM_WRITE:
            memwr_short(dev_data,1,addr,&data);
        break;
        default:
            while(1);
        break;
        }
        reg->LPC_CTRL1 = res;
        reg->LPC_CTRL3 = LPC_LPC_SYNC_VLD_T_MASK;
    }
    if(stt&LPC_INTR_STT_SYNC_TO_MASK)
    {
        reg->INTR_CLR = LPC_INTR_STT_SYNC_TO_MASK;
        while(1);
    }
    if(stt&LPC_INTR_STT_SERIRQ_STOP_MASK)
    {
        uint32_t serirq_sent = reg->INTR_SEND;
        uint32_t serirq_src;
        k_spinlock_key_t key = k_spin_lock(&dev_data->u.lpc.serirq_src_lock);
        serirq_src = reg->LPC_CTRL2;
        serirq_src ^= (serirq_sent & dev_data->u.lpc.serirq_edge_mask);
        reg->LPC_CTRL2 = serirq_src;
        k_spin_unlock(&dev_data->u.lpc.serirq_src_lock,key);
        reg->INTR_CLR = LPC_INTR_STT_SERIRQ_STOP_MASK;
        if(!(serirq_src & dev_data->u.lpc.serirq_edge_mask))
        {
            serirq_int_set(dev,false);
        }
    }
    if(stt&LPC_INTR_STT_SERIRQ_STOP_TO_MASK)
    {
        reg->INTR_CLR = LPC_INTR_STT_SERIRQ_STOP_TO_MASK;
        while(1);
    }
    if(stt&LPC_INTR_STT_SERIRQ_STOP_IVLD_MASK)
    {
        reg->INTR_CLR = LPC_INTR_STT_SERIRQ_STOP_IVLD_MASK;
        while(1);
    }
}

static void lpc_send_edge_irq(const struct device *dev,uint8_t idx)
{
	const struct espi_lpc_ls_config *cfg = dev->config;
	struct espi_lpc_ls_data *dev_data = dev->data;
    reg_lpc_t *reg = cfg->reg;
    dev_data->u.lpc.serirq_edge_mask |= 1<<idx;
    k_spinlock_key_t key = k_spin_lock(&dev_data->u.lpc.serirq_src_lock);
    uint32_t serirq = reg->LPC_CTRL2;
    serirq |= 1<<idx;
    reg->LPC_CTRL2 = serirq;
    k_spin_unlock(&dev_data->u.lpc.serirq_src_lock,key);
    serirq_int_set(dev,true);
}

static void lpc_send_level_irq(const struct device *dev,uint8_t idx,uint8_t active)
{
    k_spinlock_key_t key = k_spin_lock(&dev_data->u.lpc.serirq_src_lock);
    uint32_t serirq = reg->LPC_CTRL2;
    if(active)
    {
        serirq |= 1<<idx;
    }else
    {
        serirq &= ~1<<idx;
    }
    reg->LPC_CTRL2 = serirq;
    k_spin_unlock(&dev_data->u.lpc.serirq_src_lock,key);
}

#define LS_LPC_INIT(idx)\
	IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(idx);))\
    static void lpc_ls_irq_config_func_##idx(const struct device *dev)	\
    {\
        IRQ_CONNECT(DT_INST_IRQN(idx),DT_INST_IRQ(idx, priority),\
                ls_lpc_isr,DEVICE_DT_INST_GET(idx), 0);\
        irq_enable(DT_INST_IRQN(idx));\
    }\
    static struct lpc_ls_data lpc_ls_data_##idx;\
    static const struct lpc_ls_config lpc_ls_cfg_##idx = {\
        .reg = (reg_espi_t *)DT_INST_REG_ADDR(idx),\
        .irq_config_func = lpc_ls_irq_config_func_##idx,\
        .raise_edge_irq = lpc_send_edge_irq,\
        .set_level_irq = lpc_send_level_irq,\
    	IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),)) \
    	IF_ENABLED(DT_HAS_CLOCKS(idx), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(idx),))	 \
    };\
    DEVICE_DT_INST_DEFINE(idx,\
        lpc_ls_init,\
        NULL,\
        &lpc_ls_data_##idx,&lpc_ls_cfg_##idx,\
        PRE_KERNEL_2,CONFIG_LPC_INIT_PRIORITY,\
        &lpc_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LS_LPC_INIT)
