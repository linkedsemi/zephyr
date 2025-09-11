#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/kernel.h>
#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ls);

#include "platform.h"
#include "field_manipulate.h"
#include "reg_i2c_type.h"
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include "i2c_bitbang.h"
#include "i2c-priv.h"

#define DT_DRV_COMPAT linkedsemi_ls_i2c

#define MASTER_NACK_RECVIED BIT(0)

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct i2c_ls_config{
	irq_cfg_func_t irq_config_func;
	reg_i2c_t *reg;
	uint32_t clock_frequency;
	struct gpio_dt_spec scl;
	struct gpio_dt_spec sda;
	bool pinctrl_noinit;
	IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct i2c_ls_data {
	struct k_sem device_sync_sem;
	struct k_sem stop_sem;
	struct k_sem bus_mutex;
	uint32_t dev_config;
	struct i2c_target_config *slave_cfg;
	struct i2c_msg *current;
	uint8_t xfer_len;
	uint8_t xfer_remain;
	uint8_t errs;
	bool stop_pending;
	uint8_t pin[2];
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

#if defined(CONFIG_I2C_SHOW_STATE)
const uint8_t *i2c_reg_stat_fsm_str[] = {
    [I2C_STAT_FSM_ST_IDLE] = "ST_IDLE",
    [I2C_STAT_FSM_M_WAIT] = "M_WAIT",
    [I2C_STAT_FSM_M_START] = "M_START",
    [I2C_STAT_FSM_M_ADDR1] = "M_ADDR1",
    [I2C_STAT_FSM_M_ADDR2] = "M_ADDR2",
    [I2C_STAT_FSM_M_DATA] = "M_DATA",
    [I2C_STAT_FSM_M_RESTART] = "M_RESTART",
    [I2C_STAT_FSM_M_HOLD] = "M_HOLD",
    [I2C_STAT_FSM_M_PEC] = "M_PEC",
    [I2C_STAT_FSM_M_STOP] = "M_STOP",
    [I2C_STAT_FSM_S_START] = "S_START",
    [I2C_STAT_FSM_S_ADDR1] = "S_ADDR1",
    [I2C_STAT_FSM_S_ADDR2] = "S_ADDR2",
    [I2C_STAT_FSM_S_DATA] = "S_DATA",
    [I2C_STAT_FSM_S_HOLD] = "S_HOLD",
    [I2C_STAT_FSM_S_PEC] = "S_PEC",
};

static void i2c_ls_show_state(const struct device *dev)
{
    const struct i2c_ls_config *config = dev->config;
    LOG_DBG("%s: FSM: %s\n", dev->name, i2c_reg_stat_fsm_str[REG_FIELD_RD(config->reg->STAT, I2C_STAT_FSM_STAT)]);
    LOG_DBG("%s: SMBA_OE: %x\n", dev->name, REG_FIELD_RD(config->reg->STAT, I2C_STAT_SMBA_OE));
    LOG_DBG("%s: SDA_OE: %x\n", dev->name, REG_FIELD_RD(config->reg->STAT, I2C_STAT_SDA_OE));
    LOG_DBG("%s: SCL_OE: %x\n", dev->name, REG_FIELD_RD(config->reg->STAT, I2C_STAT_SCL_OE));
}
#endif

#if defined(CONFIG_PINCTRL)
static void i2c_ls_bitbang_set_scl(void *io_context, int state)
{
	const struct i2c_ls_config *config = io_context;

	gpio_pin_set_dt(&config->scl, state);
}

static void i2c_ls_bitbang_set_sda(void *io_context, int state)
{
	const struct i2c_ls_config *config = io_context;

	gpio_pin_set_dt(&config->sda, state);
}

static int i2c_ls_bitbang_get_sda(void *io_context)
{
	const struct i2c_ls_config *config = io_context;

	return gpio_pin_get_dt(&config->sda) == 0 ? 0 : 1;
}

static int i2c_ls_recover_bus(const struct device *dev)
{
	const struct i2c_ls_config *config = dev->config;
	struct i2c_bitbang bitbang_ctx;
	struct i2c_bitbang_io bitbang_io = {
		.set_scl = i2c_ls_bitbang_set_scl,
		.set_sda = i2c_ls_bitbang_set_sda,
		.get_sda = i2c_ls_bitbang_get_sda,
	};
	uint32_t bitrate_cfg;
	int error = 0;

	LOG_ERR("%s: attempting to recover bus", dev->name);

	if (!gpio_is_ready_dt(&config->scl)) {
		LOG_ERR("%s: SCL GPIO device not ready", dev->name);
		return -EIO;
	}

	if (!gpio_is_ready_dt(&config->sda)) {
		LOG_ERR("%s: SDA GPIO device not ready", dev->name);
		return -EIO;
	}

	pinctrl_apply_state(config->pcfg, PINCTRL_STATE_PRIV_START);

	error = gpio_pin_configure_dt(&config->scl, GPIO_OUTPUT_HIGH | GPIO_PULL_UP | GPIO_LINE_OPEN_DRAIN);
	if (error != 0) {
		LOG_ERR("%s: failed to configure SCL GPIO (err %d)", dev->name, error);
		goto restore;
	}

	error = gpio_pin_configure_dt(&config->sda, GPIO_OUTPUT_HIGH | GPIO_PULL_UP | GPIO_LINE_OPEN_DRAIN);
	if (error != 0) {
		LOG_ERR("%s: failed to configure SDA GPIO (err %d)", dev->name, error);
		goto restore;
	}

	i2c_bitbang_init(&bitbang_ctx, &bitbang_io, (void *)config);

	bitrate_cfg = i2c_map_dt_bitrate(I2C_BITRATE_STANDARD) | I2C_MODE_CONTROLLER;
	error = i2c_bitbang_configure(&bitbang_ctx, bitrate_cfg);
	if (error != 0) {
		LOG_ERR("%s: failed to configure I2C bitbang (err %d)", dev->name, error);
		goto restore;
	}

	error = i2c_bitbang_recover_bus(&bitbang_ctx);
	if (error != 0) {
		LOG_ERR("%s: failed to recover bus (err %d)", dev->name, error);
	}

restore:
	(void)pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	return error;
}

int i2c_idle_check_prepare(const struct device *dev, const struct pinctrl_dev_config *pcfg, uint8_t pinctrl_state)
{
    struct i2c_ls_data *data = dev->data;
    const struct pinctrl_state *state;
    int ret = -EINVAL;

    ret = pinctrl_lookup_state(pcfg, pinctrl_state, &state);
    if (!ret) {
        data->pin[0] = pinctrl_pin2code(&state->pins[0]);
        data->pin[1] = pinctrl_pin2code(&state->pins[1]);
        __ASSERT(data->pin[0] != data->pin[1], "scl pin and sda pin can not be duplicated");
        ret = 0;
    } else {
        data->pin[0] = 0;
        data->pin[1] = 0;
    }

    return ret;
}
#endif

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
	if(irq&I2C_INT_TXE_MASK)
	{
		cfg->reg->ICR = I2C_INT_TXE_MASK;
		#ifdef CONFIG_I2C_TARGET
		if(!data->current)
		{
			uint8_t val;
			data->slave_cfg->callbacks->read_processed(data->slave_cfg,&val);
			cfg->reg->TXDR = val;
		}else
		#endif
		{
			if(data->xfer_remain)
			{
				cfg->reg->TXDR = *data->current->buf++;
				--data->xfer_remain;
			}else
			{
				cfg->reg->IDR = I2C_INT_TXE_MASK;
				cfg->reg->IER = I2C_INT_TCR_MASK|I2C_INT_TC_MASK;
			}
		}
	}
	if(irq&I2C_INT_RXNE_MASK)
	{
		i2c_slave_addr_reenable(cfg->reg);
		do
		{
			#ifdef CONFIG_I2C_TARGET
			if(!data->current)
			{
				if(data->slave_cfg->callbacks->write_received(data->slave_cfg,cfg->reg->RXDR))
				{
					k_busy_wait(10);
					cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
				}
				cfg->reg->CR2_2 = 1;
			}else
			#endif
			{
				*data->current->buf++ = cfg->reg->RXDR;
				if(--data->xfer_remain == 0)
				{
					cfg->reg->IDR = I2C_INT_RXNE_MASK;
					cfg->reg->IER = I2C_INT_TCR_MASK|I2C_INT_TC_MASK;
					break;
				}
			}
		}while(cfg->reg->SR&I2C_SR_RXNE_MASK);
		cfg->reg->ICR = I2C_INT_RXNE_MASK;
	}
	if(irq&I2C_INT_ADDR_MASK)
	{
		#ifdef CONFIG_I2C_TARGET
		cfg->reg->ICR = I2C_INT_ADDR_MASK;
		if(data->current == NULL)
		{
			k_sem_take(&data->bus_mutex,K_NO_WAIT);
			uint32_t status = cfg->reg->SR;
			if(status&I2C_SR_DIR_MASK)
			{
				uint8_t val;
				data->slave_cfg->callbacks->read_requested(data->slave_cfg,&val);
				cfg->reg->TXDR = val;
				cfg->reg->IER = I2C_INT_TXE_MASK;
			}else
			{
				if(data->slave_cfg->callbacks->write_requested(data->slave_cfg))
				{
					cfg->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
				}
				cfg->reg->IER = I2C_INT_RXNE_MASK;
			}
		}
		#endif
	}
	if(irq&I2C_INT_NACK_MASK)
	{
		cfg->reg->ICR = I2C_INT_NACK_MASK;
		if(data->current)
		{
			data->errs |= MASTER_NACK_RECVIED;
			// if(data->xfer_remain)
			// {
			// 	k_sem_give(&data->device_sync_sem);
			// }
		}
	}
	if(irq&I2C_INT_STOP_MASK)
	{
    	cfg->reg->ICR = I2C_INT_STOP_MASK;
		cfg->reg->IDR = I2C_INT_TXE_MASK | I2C_INT_RXNE_MASK;
		if(data->current)
		{
			if(data->stop_pending)
			{
				data->stop_pending = false;
				k_sem_give(&data->stop_sem);
			}else if(data->xfer_remain||(cfg->reg->SR&I2C_SR_TXFLV_MASK)||(data->current->len == 0))
			{
				k_sem_give(&data->device_sync_sem);
			}
		}
		#ifdef CONFIG_I2C_TARGET
		else if(data->slave_cfg)
		{
			/* workaound to clear i2c slave internal counter after stop */
			cfg->reg->CR1 &= ~I2C_CR1_PE_MASK;
			cfg->reg->CR1 |= I2C_CR1_PE_MASK;
			/* --------------------------------------------- */

			cfg->reg->SR = 1;
			while(cfg->reg->SR&I2C_SR_RXNE_MASK)
			{
				cfg->reg->RXDR;
			}
			data->slave_cfg->callbacks->stop(data->slave_cfg);
			k_sem_give(&data->bus_mutex);
		}
		#endif
	}
	if(irq&I2C_INT_TC_MASK)
	{
		cfg->reg->ICR = I2C_INT_TC_MASK;
		cfg->reg->IDR = I2C_INT_TCR_MASK|I2C_INT_TC_MASK;
		__ASSERT(k_sem_count_get(&data->device_sync_sem) == 0, "TC: %d\n", __LINE__);
		k_sem_give(&data->device_sync_sem);
	}
	if(irq&I2C_INT_TCR_MASK)
	{
		cfg->reg->ICR = I2C_INT_TCR_MASK;
		cfg->reg->IDR = I2C_INT_TCR_MASK|I2C_INT_TC_MASK;
		__ASSERT(k_sem_count_get(&data->device_sync_sem) == 0, "TCR: %d\n", __LINE__);
		k_sem_give(&data->device_sync_sem);
	}
	if(irq&I2C_INT_BERR_MASK)
	{
		__ASSERT(0,"i2c@%08x bus err\n",(uint32_t)cfg->reg);
	}
	if(irq&I2C_INT_ARLO_MASK)
	{
		__ASSERT(0,"i2c@%08x arb loss\n",(uint32_t)cfg->reg);
	}
	if(irq&I2C_INT_OVR_MASK)
	{
		__ASSERT(0,"i2c@%08x overrun err\n",(uint32_t)cfg->reg);
	}
	if(irq&I2C_INT_PECE_MASK)
	{
		__ASSERT(0,"i2c@%08x pec err\n",(uint32_t)cfg->reg);
	}
	if(irq&I2C_INT_TOUT_MASK)
	{
		__ASSERT(0,"i2c@%08x timeout err\n",(uint32_t)cfg->reg);
	}
	if(irq&I2C_INT_ALERT_MASK)
	{
		__ASSERT(0,"i2c@%08x smbus alert\n",(uint32_t)cfg->reg);
	}
}


static inline void data_xfer_len_set(const struct device *dev)
{
	struct i2c_ls_data *data = dev->data;
	const struct i2c_ls_config *config = dev->config;
	data->xfer_len = data->current->len > 255 ? 255 : data->current->len;
	data->xfer_remain = data->xfer_len;
	config->reg->CR2_2 = data->xfer_len;
}

static inline void slv_single_byte(const struct i2c_ls_config *config)
{
	config->reg->CR2_2 = 1;
	config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
}

static int i2c_ls_transfer(const struct device *dev, struct i2c_msg *msg,
			      uint8_t num_msgs, uint16_t slave)
{
	struct i2c_ls_data *data = dev->data;
	const struct i2c_ls_config *config = dev->config;
	uint8_t scl_val;
	uint8_t sda_val;
	int ret = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);
#if defined(CONFIG_PINCTRL)
	if ((NULL != config->scl.port) && (NULL != config->sda.port)) {
		scl_val = gpio_pin_get_dt(&config->scl);
		sda_val = gpio_pin_get_dt(&config->sda);
		if (!((1 == scl_val) && (1 == sda_val))) {
			LOG_DBG("%s: bus busy\n", dev->name);
			IF_ENABLED(CONFIG_I2C_SHOW_STATE, (i2c_ls_show_state(dev)));
			 if ((1 == scl_val) && (0 == sda_val)) {
				LOG_DBG("%s: try recovery\n", dev->name);
				ret = i2c_ls_recover_bus(dev);
				if (ret) {
					goto err;
				}
			} else {
				LOG_DBG("%s: scl: %d.  sda: %d.\n", dev->name, scl_val, sda_val);
				goto err;
			}
		}
	}
#endif
	data->errs = 0;
	config->reg->SR = I2C_SR_TXE_MASK;//clear tx fifo
	uint32_t cr2_0_1 = msg->flags&I2C_MSG_ADDR_10_BITS? I2C_CR2_SADD10_MASK|slave<<I2C_CR2_SADD0_POS :slave<<I2C_CR2_SADD1_7_POS;
	for(data->current = msg;data->current<&msg[num_msgs];data->current++)
	{
		bool read = (data->current->flags&I2C_MSG_RW_MASK)==I2C_MSG_READ;
		if(data->current->len == 0)
		{
			if(read)
			{
				config->reg->CR2_3 |= 0x30;
			}else
			{
				config->reg->CR2_3 &= ~0x30;
			}
			config->reg->CR2_0_1 = cr2_0_1|I2C_CR2_START_MASK|I2C_CR2_STOP_MASK;
			k_sem_take(&data->device_sync_sem, K_FOREVER);
			config->reg->CR2_3 &= ~0x30;
			if(data->errs)
			{
				ret = -EIO;
			}
			break;
		}
		if(read)
		{
			cr2_0_1 |= I2C_CR2_RD_WEN_MASK;
		}else
		{
			cr2_0_1 &= ~I2C_CR2_RD_WEN_MASK;
		}
		data->xfer_len = 0;
		do{
			__ASSERT(k_sem_count_get(&data->device_sync_sem) == 0, "S: %d\n", __LINE__);
			bool start = (data->current->flags&I2C_MSG_RESTART||data->current==msg) && data->xfer_len == 0;
			if(start)
			{
				config->reg->CR2_3 &= ~I2C_CR2_RELOAD_MASK;
			}else
			{
				config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
			}
			data_xfer_len_set(dev);
			if(data->current->len<=255)
			{
				config->reg->CR2_3 &= ~I2C_CR2_RELOAD_MASK;
			}else{
				config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
			}
			if(start)
			{
				config->reg->CR2_0_1 = cr2_0_1 | I2C_CR2_START_MASK;
			}
			data->current->len -= data->xfer_len;
			if(read)
			{
				config->reg->IER = I2C_INT_RXNE_MASK;
			}else
			{
				config->reg->IER = I2C_INT_TXE_MASK;
			}
			k_sem_take(&data->device_sync_sem, K_FOREVER);
			if(data->errs)
			{
				ret = -EIO;
				goto err;
			}
		}while(data->current->len);
		if(data->current->flags&I2C_MSG_STOP)
		{
			data->stop_pending = true;
			config->reg->CR2_0_1 |= I2C_CR2_STOP_MASK;
			k_sem_take(&data->stop_sem, K_FOREVER);
		}
	}
err:
	data->current = NULL;
	slv_single_byte(config);
	k_sem_give(&data->bus_mutex);
	return ret;
}

static void i2c_timing_param_set(const struct i2c_ls_config *config,uint32_t i2c_clk)
{
	uint16_t cycle_count = 0;
	uint8_t prescalar = 0;
	int16_t scll = 0;
	int16_t sclh = 0;
	int16_t scldel = 0;
	int16_t sdadel = 0;

	uint8_t __prescalar = 1;
	uint16_t __cycle_count;
	int16_t __scll;

	while (1) {
		__prescalar++;
		__cycle_count = config->clock_frequency / i2c_clk / __prescalar;
		__scll = __cycle_count >> 1;

		if (((__cycle_count > 256) || (__scll > 16)) && (__cycle_count >= 16) && (__prescalar<=16)) {
			prescalar = __prescalar;
			cycle_count = __cycle_count;
			scll = __scll;
		} else {
			break;
		}
	}
	__ASSERT((cycle_count>=16) && (prescalar<=16) && (scll<48),"Invalid i2c timing");

	scldel = (scll >> 1) > 16 ? 15 : (scll >> 1);
	sclh = scll;
	sdadel = 0;
    MODIFY_REG(config->reg->TIMINGR, (I2C_TIMINGR_PRESC_MASK |I2C_TIMINGR_SCLH_MASK | I2C_TIMINGR_SCLL_MASK | I2C_TIMINGR_SDADEL_MASK | I2C_TIMINGR_SCLDEL_MASK),
        (prescalar - 1)<<I2C_TIMINGR_PRESC_POS|sclh<<I2C_TIMINGR_SCLH_POS|scll<<I2C_TIMINGR_SCLL_POS|sdadel<<I2C_TIMINGR_SDADEL_POS|scldel<<I2C_TIMINGR_SCLDEL_POS);
}

static void i2c_reenable(const struct i2c_ls_config *config ,uint32_t i2c_clk)
{
	config->reg->CR1 &= ~I2C_CR1_PE_MASK;
	i2c_timing_param_set(config,i2c_clk);
    config->reg->CFR = 0xffff;
	config->reg->CR1 |= I2C_CR1_SBC_MASK|I2C_CR1_PE_MASK;
	slv_single_byte(config);
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

int i2c_ls_pinctrl(const struct device *dev, uint32_t pinctrl_state)
{
	const struct i2c_ls_config *dev_config = dev->config;
	int ret = 0;

	__ASSERT_NO_MSG(dev);

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(dev_config->pcfg, pinctrl_state);
	if (ret < 0) {
		LOG_DBG("%s: Could not configure pins", dev->name);
	}
	i2c_idle_check_prepare(dev, dev_config->pcfg, pinctrl_state);

	return ret;
}

static int i2c_ls_init(const struct device *dev)
{
	const struct i2c_ls_config *dev_config = dev->config;
	struct i2c_ls_data *data = dev->data;
    __maybe_unused int ret;

	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->stop_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->bus_mutex, 1, 1);
	dev_config->irq_config_func(dev);

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s: %s device not ready", dev->name, clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            LOG_ERR("%s: Reset controller device is not ready", dev->name);
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERR("%s: toggle reset line failed", dev->name);
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_PINCTRL)
	if (!dev_config->pinctrl_noinit) {
		i2c_ls_pinctrl(dev, PINCTRL_STATE_DEFAULT);
	}
#endif

	i2c_reenable(dev_config,100000);
	dev_config->reg->CR2_3 |= 1<<3; // slv nbytes upd hw workaround
	dev_config->reg->ICR = 0xffff;
	dev_config->reg->IER = I2C_INT_STOP_MASK|I2C_INT_NACK_MASK|I2C_INT_BERR_MASK
		|I2C_INT_ARLO_MASK|I2C_INT_OVR_MASK|I2C_INT_PECE_MASK
		|I2C_INT_TOUT_MASK|I2C_INT_ALERT_MASK;

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
	cfg->reg->IER = I2C_INT_ADDR_MASK;
	return 0;
}

static int i2c_ls_target_unregister(const struct device *dev,struct i2c_target_config *target_cfg)
{
	struct i2c_ls_data *data = dev->data;
	const struct i2c_ls_config *cfg = dev->config;
	cfg->reg->IDR = I2C_INT_ADDR_MASK;
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

#define LS_I2C_INIT(index)\
	IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(index)));	\
	LS_I2C_IRQ_HANDLER(index)\
	static const struct i2c_ls_config i2c_ls_cfg_##index = {\
		.reg = (reg_i2c_t *)DT_INST_REG_ADDR(index),\
		.irq_config_func = i2c_ls_irq_config_func_##index,\
		.clock_frequency = COND_CODE_1(\
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(index, clocks), clock_frequency),\
			(DT_INST_PROP_BY_PHANDLE(index, clocks, clock_frequency)),\
			(DT_INST_PROP(index, clock_frequency))),\
		.scl =	GPIO_DT_SPEC_INST_GET_OR(index, scl_gpios, {0}),\
		.sda = GPIO_DT_SPEC_INST_GET_OR(index, sda_gpios, {0}),\
		.pinctrl_noinit = DT_INST_PROP_OR(index, pinctrl_noinit, 0),\
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), )) \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), )) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), )) \
	};\
	static struct i2c_ls_data i2c_ls_dev_data_##index;\
	I2C_DEVICE_DT_INST_DEFINE(index, i2c_ls_init,\
				NULL, &i2c_ls_dev_data_##index,\
				&i2c_ls_cfg_##index,\
				POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,\
				&api_funcs);
DT_INST_FOREACH_STATUS_OKAY(LS_I2C_INIT)
