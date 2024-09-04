#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/kernel.h>
#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ls);

#include "platform.h"
#include "field_manipulate.h"
#include "reg_i2c_type.h"
#define DT_DRV_COMPAT linkedsemi_ls_i2c

#define MASTER_NACK_RECVIED BIT(0)

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct i2c_ls_config{
	irq_cfg_func_t irq_config_func;   //函数指针类型要重新定义
    reg_i2c_t *reg;    
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
};

struct i2c_ls_data {
	struct k_sem device_sync_sem;
	struct k_sem bus_mutex;
	uint32_t dev_config;
	struct i2c_target_config *slave_cfg;
	struct i2c_msg *current;
	uint8_t errs;

};

struct i2c_speed_config_t
{
    uint32_t scll     : 8;
    uint32_t sclh     : 8;
    uint32_t sdadel   : 4;
    uint32_t scldel   : 4;
    uint32_t role 	  : 4;
    uint32_t presc    : 4;
};

static void i2c_slave_addr_reenable(reg_i2c_t *reg)
{
    uint32_t oar1 = reg->OAR1;
    uint32_t oar2 = reg->OAR2;
    reg->OAR1 = oar1 & ~I2C_OAR1_OA1EN_MASK;
    reg->OAR2 = oar2 & ~I2C_OAR2_OA2EN_MASK;
    reg->OAR1 = oar1;
    reg->OAR2 = oar2;
}

void ls_i2c_isr(void *arg)
{
	struct device *dev = (struct device *) arg;
	const struct i2c_ls_config *cfg = dev->config;
	struct i2c_ls_data *data = dev->data;
	uint32_t irq = cfg->reg->IFM;
	if(irq&I2C_IFM_TXEFM_MASK)
	{
		cfg->reg->ICR = I2C_ICR_TXEIC_MASK;
		#ifdef CONFIG_I2C_TARGET
		if(!data->current)
		{
			uint8_t val;
			if(data->slave_cfg->callbacks->read_processed(data->slave_cfg,&val))
			{
				cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
			}
			cfg->reg->TXDR = val;
		}else
		#endif
		{
			cfg->reg->TXDR = *data->current->buf++;
			if(--data->current->len == 0)
			{
				cfg->reg->IDR = I2C_IDR_TXEID_MASK;
				cfg->reg->IER = I2C_IER_TCRIE_MASK|I2C_IER_TCIE_MASK;
			}
		}
	}
	if(irq&I2C_IFM_RXNEFM_MASK)
	{
		i2c_slave_addr_reenable(cfg->reg);
		do
		{
			#ifdef CONFIG_I2C_TARGET
			if(!data->current)
			{
				if(data->slave_cfg->callbacks->write_received(data->slave_cfg,cfg->reg->RXDR))
				{
					cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
					cfg->reg->IDR = I2C_IDR_RXNEID_MASK;
					break;
				}
			}else
			#endif
			{
				*data->current->buf++ = cfg->reg->RXDR;
				if(--data->current->len == 0)
				{
					cfg->reg->IDR = I2C_IDR_RXNEID_MASK;
					cfg->reg->IER = I2C_IER_TCRIE_MASK|I2C_IER_TCIE_MASK;
					break;
				}
			}
		}while(cfg->reg->SR&I2C_SR_RXNE_MASK);
		cfg->reg->ICR = I2C_ICR_RXNEIC_MASK;
	}
	if(irq&I2C_IFM_ADDRFM_MASK)
	{
		#ifdef CONFIG_I2C_TARGET
	    uint32_t status = cfg->reg->SR;
		cfg->reg->ICR = I2C_ICR_ADDRIC_MASK;
		if(status&I2C_SR_DIR_MASK)
		{
			uint8_t val;
			if(data->slave_cfg->callbacks->read_requested(data->slave_cfg,&val))
			{
				cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
			}
			cfg->reg->TXDR = val;
			cfg->reg->IER = I2C_IER_TXEIE_MASK;
		}else
		{
			if(data->slave_cfg->callbacks->write_requested(data->slave_cfg))
			{
				cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
			}
			cfg->reg->IER = I2C_IER_RXNEIE_MASK;
		}
		#endif
	}
	if(irq&I2C_IFM_NACKFM_MASK)
	{
		cfg->reg->ICR = I2C_ICR_NACKIC_MASK;
		if(data->current)
		{
			data->errs |= MASTER_NACK_RECVIED;
			k_sem_give(&data->device_sync_sem);
		}
	}
	if(irq&I2C_IFM_STOPFM_MASK)
	{
    	cfg->reg->ICR = I2C_ICR_STOPIC_MASK;
		#ifdef CONFIG_I2C_TARGET
		if(!data->current)
		{
			cfg->reg->IDR = I2C_IDR_TXEID_MASK | I2C_IDR_RXNEID_MASK;
			cfg->reg->SR = 1;
			while(cfg->reg->SR&I2C_SR_RXNE_MASK)
			{
				cfg->reg->RXDR;
			}
			data->slave_cfg->callbacks->stop(data->slave_cfg);
		}else
		#endif
		{
			k_sem_give(&data->device_sync_sem);
		}
	}
	if(irq&I2C_IFM_TCFM_MASK)
	{
		cfg->reg->ICR = I2C_ICR_TCIC_MASK;
		cfg->reg->IDR = I2C_IDR_TCRID_MASK|I2C_IDR_TCID_MASK;
		k_sem_give(&data->device_sync_sem);
	}
	if(irq&I2C_IFM_TCRFM_MASK)
	{
		cfg->reg->ICR = I2C_ICR_TCRIC_MASK;
		cfg->reg->IDR = I2C_IDR_TCRID_MASK|I2C_IDR_TCID_MASK;
		k_sem_give(&data->device_sync_sem);
	}
	if(irq&I2C_IFM_BERRFM_MASK)
	{
		__ASSERT(0,"i2c bus err\n");
	}
	if(irq&I2C_IFM_ARLOFM_MASK)
	{
		__ASSERT(0,"i2c arb loss\n");
	}
	if(irq&I2C_IFM_OVRFM_MASK)
	{
		__ASSERT(0,"i2c overrun err\n");
	}
	if(irq&I2C_IFM_PECEFM_MASK)
	{
		__ASSERT(0,"i2c pec err\n");
	}
	if(irq&I2C_IFM_TOUTFM_MASK)
	{
		__ASSERT(0,"i2c timeout err\n");
	}
	if(irq&I2C_IFM_ALERTFM_MASK)
	{
		__ASSERT(0,"i2c smbus alert\n");
	}
}


static int i2c_ls_transfer(const struct device *dev, struct i2c_msg *msg,
			      uint8_t num_msgs, uint16_t slave)
{
	struct i2c_ls_data *data = dev->data;
	const struct i2c_ls_config *config = dev->config;
	int ret = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);
	data->errs = 0;
	config->reg->SR = I2C_SR_TXE_MASK;//clear tx fifo
	uint32_t cr2_0_1 = msg->flags&I2C_MSG_ADDR_10_BITS? I2C_CR2_SADD10_MASK|slave<<I2C_CR2_SADD0_POS :slave<<I2C_CR2_SADD1_7_POS;
	for(data->current = msg;data->current<&msg[num_msgs];data->current++)
	{
		bool read = (data->current->flags&I2C_MSG_RW_MASK)==I2C_MSG_READ;
		if(read)
		{
			cr2_0_1 |= I2C_CR2_RD_WEN_MASK;
		}else
		{
			cr2_0_1 &= ~I2C_CR2_RD_WEN_MASK;
		}

		if(data->current->flags&I2C_MSG_RESTART||data->current==msg)
		{
			config->reg->CR2_3 &= ~I2C_CR2_RELOAD_MASK;
			config->reg->CR2_2 = data->current->len;
			if((data->current->flags&I2C_MSG_STOP)==0)
			{
				config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
			}
			config->reg->CR2_0_1 = cr2_0_1 | I2C_CR2_START_MASK;
		}else
		{
			config->reg->CR2_2 = data->current->len;
		}
		if(read)
		{
			config->reg->IER = I2C_IER_RXNEIE_MASK;
		}else
		{
			config->reg->IER = I2C_IER_TXEIE_MASK;
		}
		k_sem_take(&data->device_sync_sem, K_FOREVER);
		if(data->errs)
		{
			ret = -EIO;
			break;
		}
		if(data->current->flags&I2C_MSG_STOP)
		{
			config->reg->CR2_0_1 |= I2C_CR2_STOP_MASK;
			k_sem_take(&data->device_sync_sem, K_FOREVER);
		}
	}
	data->current = NULL;
	k_sem_give(&data->bus_mutex);

	return ret;
}

static void i2c_timing_param_set(const struct i2c_ls_config *config,uint32_t i2c_clk)
{
	uint32_t pclk = 144000000;
	uint16_t cycle_count;
	uint8_t prescalar = 1;
	do{
		prescalar += 1;
		cycle_count = pclk/i2c_clk/prescalar;
	}while(cycle_count>256);
	__ASSERT(cycle_count>=16&&prescalar<=16,"Invalid i2c timing");
    int16_t scll, sclh, scldel, sdadel;
	scll = cycle_count*2/3;
    sclh = cycle_count/3;
	scldel = scll>16?15:scll-2;
	sdadel = 1;
	struct i2c_speed_config_t param;
    param.presc = prescalar - 1;
    param.scll = scll;
    param.sclh = sclh;
    param.scldel = scldel;
    param.sdadel = sdadel;
   	MODIFY_REG(config->reg->TIMINGR, (I2C_TIMINGR_PRESC_MASK |I2C_TIMINGR_SCLH_MASK | I2C_TIMINGR_SCLL_MASK | I2C_TIMINGR_SDADEL_MASK | I2C_TIMINGR_SCLDEL_MASK), 
        param.presc<<I2C_TIMINGR_PRESC_POS|param.sclh<<I2C_TIMINGR_SCLH_POS|param.scll<<I2C_TIMINGR_SCLL_POS|param.sdadel<<I2C_TIMINGR_SDADEL_POS|param.scldel<<I2C_TIMINGR_SCLDEL_POS);	

}

static void i2c_reenable(const struct i2c_ls_config *config ,uint32_t i2c_clk)
{
	config->reg->CR1 &= ~I2C_CR1_PE_MASK;
	i2c_timing_param_set(config,i2c_clk);
    config->reg->CFR = 0xffff;
	config->reg->CR1 |= I2C_CR1_PE_MASK;
}

static int i2c_runtime_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_ls_config *config = dev->config;
	struct i2c_ls_data *data = (struct i2c_ls_data *const)(dev)->data;  
	uint32_t i2c_clk = 0;
	switch(I2C_SPEED_GET(dev_config))
	{
	case I2C_SPEED_STANDARD:
		i2c_clk = 100000;
	break;
	case I2C_SPEED_FAST:
		i2c_clk = 400000;
	break;
	case I2C_SPEED_FAST_PLUS:
		i2c_clk = 1000000;
	break;
	default:
		__ASSERT(0,"i2c speed not supported\n");
	break;
	}
	data->dev_config = dev_config;
	i2c_reenable(config,i2c_clk);
    return  0;
}

static int i2c_ls_init(const struct device *dev)
{
	const struct i2c_ls_config *cfg = dev->config;
#if defined(CONFIG_PINCTRL)
	int ret;
#endif
	struct i2c_ls_data *data = dev->data;
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->bus_mutex, 1, 1);
	cfg->irq_config_func(dev);
#if defined(CONFIG_PINCTRL)
	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);   //pin
	if (ret < 0) {
		// LOG_ERR("I2C pinctrl setup failed (%d)", ret);
		return ret;
	}
#endif
	i2c_reenable(cfg,100000);
	cfg->reg->ICR = 0xffff;
	cfg->reg->IER = I2C_IER_STOPIE_MASK|I2C_IER_NACKIE_MASK|I2C_IER_BERRIE_MASK
		|I2C_IER_ARLOIE_MASK|I2C_IER_OVRIE_MASK|I2C_IER_PECEIE_MASK
		|I2C_IER_TOUTIE_MASK|I2C_IER_ALERTIE_MASK;

	return 0;
}

static int i2c_ls_get_config(const struct device *dev,uint32_t *dev_config)
{
	struct i2c_ls_data *data = dev->data;
	*dev_config = data->dev_config;
	return 0;
}

#ifdef CONFIG_I2C_TARGET
static int i2c_ls_target_register(const struct device *dev,struct i2c_target_config *target_cfg)
{
	const struct i2c_ls_config *cfg = dev->config;
	struct i2c_ls_data *data = dev->data;
	data->slave_cfg = target_cfg;
	if(target_cfg->flags&I2C_TARGET_FLAGS_ADDR_10_BITS)
	{
		cfg->reg->OAR1 = I2C_OAR1_OA1EN_MASK|I2C_OAR1_OA1MODE_MASK|target_cfg->address<<I2C_OAR1_OA10_POS;
	}else
	{
		cfg->reg->OAR1 = I2C_OAR1_OA1EN_MASK|target_cfg->address<<I2C_OAR1_OA11_7_POS;
	}
	cfg->reg->IER = I2C_IER_ADDRIE_MASK;
	return 0;
}

static int i2c_ls_target_unregister(const struct device *dev,struct i2c_target_config *target_cfg)
{
	struct i2c_ls_data *data = dev->data;
	const struct i2c_ls_config *cfg = dev->config;
	cfg->reg->IDR = I2C_IDR_ADDRID_MASK;
	cfg->reg->OAR1 = 0;
	data->slave_cfg = NULL;
	return 0;
}
#endif

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_runtime_configure,
	.get_config = i2c_ls_get_config,
	.transfer = i2c_ls_transfer,
#ifdef CONFIG_I2C_TARGET
	.target_register = i2c_ls_target_register,
	.target_unregister = i2c_ls_target_unregister,
#endif
};

#define LS_I2C_IRQ_HANDLER(index)					\
static void i2c_ls_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),			\
			DT_INST_IRQ(index, priority),		\
			ls_i2c_isr,			\
			DEVICE_DT_INST_GET(index), 0);		\
	irq_enable(DT_INST_IRQN(index));			\
}

#if defined(CONFIG_PINCTRL)
	#define LS_I2C_INIT(index)\
		PINCTRL_DT_INST_DEFINE(index);\
		LS_I2C_IRQ_HANDLER(index)\
		static const struct i2c_ls_config i2c_ls_cfg_##index = {\
			.reg = (reg_i2c_t *)DT_INST_REG_ADDR(index),\
			.irq_config_func = i2c_ls_irq_config_func_##index,\
			.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),\
		};\
		static struct i2c_ls_data i2c_ls_dev_data_##index;\
		I2C_DEVICE_DT_INST_DEFINE(index, i2c_ls_init,\
					NULL, &i2c_ls_dev_data_##index,\
					&i2c_ls_cfg_##index,\
					POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,\
					&api_funcs);
#else
	#define LS_I2C_INIT(index)\
		LS_I2C_IRQ_HANDLER(index)\
		static const struct i2c_ls_config i2c_ls_cfg_##index = {\
			.reg = (reg_i2c_t *)DT_INST_REG_ADDR(index),\
			.irq_config_func = i2c_ls_irq_config_func_##index,\
		};\
		static struct i2c_ls_data i2c_ls_dev_data_##index;\
		I2C_DEVICE_DT_INST_DEFINE(index, i2c_ls_init,\
					NULL, &i2c_ls_dev_data_##index,\
					&i2c_ls_cfg_##index,\
					POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,\
					&api_funcs);
#endif

DT_INST_FOREACH_STATUS_OKAY(LS_I2C_INIT)
