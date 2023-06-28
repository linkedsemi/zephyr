#include <soc.h>
#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/kernel.h>

#include "platform.h"
// #include <linkedsemi/soc/field_manipulate.h>
#include <ls_msp_i2c.h>
#include <ls_ll_i2c.h>
#include <log.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <ls_msp_i2c.h>
#include <i2c_misc.h>
#include <field_manipulate.h>
#include <reg_i2c_type.h>
#include <reg_rcc.h>


#define DT_DRV_COMPAT le501x_i2c

#define LE501X_I2C_TRANSFER_TIMEOUT_MSEC  500

#define LE501X_I2C_TIMEOUT_USEC  1000
#define I2C_REQUEST_WRITE       0x00
#define I2C_REQUEST_READ        0x01
#define HEADER                  0xF0
#define LE_I2C_FIFO_DEPTH			8

/* Bus error */
#define I2C_LE501X_ERR_BERR 	BIT(0)
/* I2C bus busy */
#define I2C_LE501X_ERR_BUSY		BIT(1)
/* I2C transfer timeout */
#define I2C_LE501X_ERR_TIMEOUT 	BIT(2)

#define I2C_LE501X_ERR_STOPED 	BIT(3)


#define I2C_GET_TXFLV(__HANDLE__) (REG_FIELD_RD((__HANDLE__)->SR, I2C_SR_TXFLV))
#define I2C_GET_RXFLV(__HANDLE__) (REG_FIELD_RD((__HANDLE__)->SR, I2C_SR_RXFLV))
#define I2C_CLR_TXDR(__HANDLE__) (((__HANDLE__)->SR) |= (1 << 0))
#define I2C_CLEAR_IF(__HANDLE__, __FLAG__)      SET_BIT((__HANDLE__)->ICR, (__FLAG__))

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct i2c_le501x_config{
#ifdef CONFIG_I2C_LE501X_INTERRUPT
	irq_cfg_func_t irq_config_func;   //函数指针类型要重新定义
#endif
    reg_i2c_t *reg;    
    uint32_t bitrate;
    const struct pinctrl_dev_config *pcfg;
    const uint32_t default_config;
};

struct i2c_le501x_data {
#ifdef CONFIG_I2C_LE501X_INTERRUPT
	struct k_sem device_sync_sem;
#endif

	struct k_sem bus_mutex;

	uint32_t dev_config;
	struct i2c_msg *current_msg;
	uint32_t xfer_len;
	uint8_t *buf;
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
static struct i2c_speed_config_t speed_config;

static inline uint32_t i2c_map_dt_bitrate(uint32_t bitrate)
{
	switch (bitrate) {
	case I2C_BITRATE_STANDARD:
		return I2C_SPEED_STANDARD << I2C_SPEED_SHIFT;
	case I2C_BITRATE_FAST:
		return I2C_SPEED_FAST << I2C_SPEED_SHIFT;
	case I2C_BITRATE_FAST_PLUS:
		return I2C_SPEED_FAST_PLUS << I2C_SPEED_SHIFT;
	case I2C_BITRATE_HIGH:
		return I2C_SPEED_HIGH << I2C_SPEED_SHIFT;
	case I2C_BITRATE_ULTRA:
		return I2C_SPEED_ULTRA << I2C_SPEED_SHIFT;
	}
	return 0;
}


static int le501x_i2c_wait_timeout(uint16_t *timeout)
{
	if (*timeout == 0) {
		return 1;
	} else {
		k_busy_wait(1);
		(*timeout)--;
		return 0;
	}
}


#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)

int i2c_msg_init_restart(const struct device *dev, struct i2c_msg *msg,uint16_t saddr)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;
	// reg_i2c_t *i2c = cfg->reg;
	// uint32_t len = msg->len;
	uint16_t timeout;
	// uint8_t *pData = msg->buf;

	if (msg->flags & I2C_MSG_RESTART) {
		timeout = LE501X_I2C_TIMEOUT_USEC;
		while(LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_BUSY))
		{
			if (le501x_i2c_wait_timeout(&timeout)) {
				LOG_I("i2c timeout");
				LL_I2C_Disable(cfg->reg);
				return -EBUSY;
			}
		}
		/* Check if the I2C is already enabled */
		if (!LL_I2C_IsEnabled(cfg->reg))
		{
			/* Enable I2C peripheral */
			LL_I2C_Enable(cfg->reg);
		}
		/* Clear SR Flag */
		I2C_CLR_TXDR(cfg->reg);
		for(uint8_t i =I2C_GET_RXFLV(cfg->reg); i > 0; i--)
		{
			LL_I2C_ReceiveData8(cfg->reg);
		}
		LL_I2C_ClearSR(cfg->reg);
		LL_I2C_DisableAutoEnd(cfg->reg);
		LL_I2C_DisableRELOAD(cfg->reg);
		LL_I2C_SetNumberOfByte(cfg->reg, 0);
  
	}

	if (data->xfer_len > 0xFF)
	{
		LL_I2C_EnableRELOAD(cfg->reg);
		LL_I2C_SetNumberOfByte(cfg->reg, 0xff);
		LL_I2C_DisableAutoEnd(cfg->reg);
	}else
	{
		LL_I2C_DisableRELOAD(cfg->reg);
		LL_I2C_EnableAutoEnd(cfg->reg);
		LL_I2C_SetNumberOfByte(cfg->reg, data->xfer_len);
		LOG_I("msg->len = %d", data->xfer_len);
	}

	LOG_I("saddr = %d", saddr);
	/* Master Generate Start condition */
	if (msg->flags & I2C_MSG_RESTART) {
	/* Send slave address */
		if (I2C_ADDR_10_BITS & data->dev_config) {
			LL_I2C_SetAddressToSlave(cfg->reg, saddr, msg->flags&I2C_MSG_READ, LL_I2C_OWNADDRESS1_10BIT);
		} else {
			LL_I2C_SetAddressToSlave(cfg->reg, saddr, msg->flags&I2C_MSG_READ, LL_I2C_OWNADDRESS1_7BIT);
		}
		LL_I2C_GenerateStartCondition(cfg->reg);
	}

	timeout = LE501X_I2C_TIMEOUT_USEC;
	while(!LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_BUSY))
	{
		if (le501x_i2c_wait_timeout(&timeout)) {
			LOG_I("i2c start timeout");
			LL_I2C_Disable(cfg->reg);
			return	-EBUSY;
		}
	}
	return 0;
}

#ifdef CONFIG_I2C_LE501X_INTERRUPT

void I2C_MasterTXECpltCallback(struct device *dev);
void I2C_MasterRXNECpltCallback(struct device *dev);


int check_error(uint8_t error)
{
	if(error & I2C_LE501X_ERR_BERR)
	{
		LOG_I("Error EIO");
		return -EIO;
	}
	if(error & I2C_LE501X_ERR_TIMEOUT)
	{
		LOG_I("Error EBUSY");
		return	-EBUSY;
	}

	if(error & I2C_LE501X_ERR_STOPED)
	{
		LOG_I("Error STOPED");
		return	-EBUSY;
	}
	return 0;
}

void le501x_i2c_combined_isr(void *arg)
{
	struct device *dev = (struct device *) arg;
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;

    /* Check NACK flag -----------------------------------------------*/
    if ((LL_I2C_IsActiveFlagIT(cfg->reg, I2C_ITRI_NACK) != RESET) && (LL_I2C_IsEnableIT(cfg->reg, I2C_ITVS_NACK) != RESET))
    {
        LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_NACK);
        LL_I2C_Clearflag(cfg->reg, I2C_CFR_NACKCF);
    }

	if (LL_I2C_IsActiveFlagIT(cfg->reg, I2C_ITRI_STOP) != RESET)
    { 
        LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_STOP);
		if(data->current_msg->flags &  I2C_MSG_READ)
		{
			I2C_MasterRXNECpltCallback(dev);
		}else
		{
			I2C_MasterTXECpltCallback(dev);
		}
    }
    else if((LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_TXE)!= RESET) && (LL_I2C_IsEnableIT(cfg->reg, I2C_ITVS_TXE) != RESET))
    {
        LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_TXE);
        I2C_MasterTXECpltCallback(dev);
    }
    else if((LL_I2C_IsActiveFlagIT(cfg->reg, I2C_ITRI_RXNE) != RESET) && (LL_I2C_IsEnableIT(cfg->reg, I2C_ITVS_RXNE) != RESET))
    {
        LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_RXNE);
        I2C_MasterRXNECpltCallback(dev);
    }
    else if(LL_I2C_IsActiveFlagIT(cfg->reg, I2C_ITRI_ERR) != RESET)
    {
        LOG_I("I2C beer error !\n");
		data->errs |= I2C_LE501X_ERR_BERR;
        LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_ERR);
        LL_I2C_Clearflag(cfg->reg, I2C_SR_BERR);
		LL_I2C_DisableIT(cfg->reg, I2C_ITDEN_ADDR | I2C_ITDEN_NACK | I2C_ITDEN_STOP | I2C_ITDEN_ERR | I2C_ITDEN_TC | I2C_ITDEN_TCR);
		k_sem_give(&data->device_sync_sem);
    }
}

    
/**
 * @brief Send  empty callback function
 * 
 * @param struct device *dev 
 */
void I2C_MasterTXECpltCallback(struct device *dev)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;

	I2C_CLEAR_IF(cfg->reg, I2C_ICR_TXEIC_MASK);

	if (data->xfer_len == 0U)
	{
		/* Clear interrupt Flag */
		LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_TXE | I2C_ITIC_TC | I2C_ITIC_TCR);

		/* Disable EVT, TXE and ERR interrupt */
		LL_I2C_DisableIT(cfg->reg, I2C_ITDEN_ADDR | I2C_ITDEN_STOP | I2C_ITDEN_TXE | I2C_ITDEN_ERR | I2C_ITDEN_TC);
		k_sem_give(&data->device_sync_sem);
	}
	else
	{

		// I2C_FIFO_TX(hi2c);
		while(I2C_GET_TXFLV(cfg->reg) < LE_I2C_FIFO_DEPTH && data->xfer_len >> 0)
		{
			LL_I2C_TransmitData8(cfg->reg, (*data->current_msg->buf));
			data->current_msg->buf++;
			data->current_msg->len--;
			data->xfer_len--;
			if ((data->xfer_len > 0U) && (data->current_msg->len == 0U)){
				data->current_msg++;
			}
		}
		// LOG_I("data->xfer_len = %d",data->xfer_len);
	}
}

/**
 * @brief Receive non-empty callback function
 * 
 * @param struct device *dev 
 */
void I2C_MasterRXNECpltCallback(struct device *dev)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;

	I2C_CLEAR_IF(cfg->reg, I2C_ICR_RXNEIC_MASK);

	while(I2C_GET_RXFLV(cfg->reg) > 0 && data->xfer_len > 0)
	{
		*data->current_msg->buf = LL_I2C_ReceiveData8(cfg->reg);
		// LOG_I("len = %d, data = 0x%x", data->current_msg->len,*data->current_msg->buf);
		data->current_msg->buf++;
		data->current_msg->len--;
		data->xfer_len--;
		if ((data->xfer_len > 0U) && (data->current_msg->len == 0U)){
			data->current_msg++;
		}
	}
	if (data->xfer_len == 0)
	{
		/* Clear interrupt Flag */
		LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_RXNE);

		/* Disable EVT, II2C_ITDEN_RXNE and ERR interrupt */
		LL_I2C_DisableIT(cfg->reg, I2C_ITDEN_ADDR | I2C_ITDEN_NACK | I2C_ITDEN_STOP  | I2C_ITDEN_RXNE | I2C_ITDEN_ERR | I2C_ITDEN_TC | I2C_ITDEN_TCR);
		k_sem_give(&data->device_sync_sem);
	}

}

int le501x_i2c_msg_write(const struct device *dev, 
				struct i2c_msg *msg,uint16_t saddr)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;
	// uint32_t TxCount = 0;
	int32_t res;
	/* Check the I2C handle allocation */
	if (cfg->reg == NULL)
	{
		return ENXIO;
	}

	res = i2c_msg_init_restart(dev,msg,saddr);

	if(res < 0)
	{
		return res;
	}

	if((data->xfer_len > 0U) && (LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_TXE) == SET))
	{
		/* Write data in Transmit Data register.
		TXE flag is cleared by writing data in TXDR register */
		LL_I2C_TransmitData8(cfg->reg, (*data->current_msg->buf));
		/* Increment Buffer pointer */
		// LOG_I("pData = %x,len = %d", (*data->current_msg->buf),data->current_msg->len);
		/* Update counter */
		data->current_msg->buf++;
		data->current_msg->len--;
		data->xfer_len--;
	}

	/* Clear interrupt Flag */
	LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_TXE | I2C_ITIC_ADDR | I2C_ITIC_NACK | I2C_ITIC_STOP | I2C_ITIC_ERR);

	/* Enable EVT, TXE and ERR interrupt */
	LL_I2C_EnableIT(cfg->reg, I2C_ITEN_ADDR | I2C_ITEN_NACK | I2C_ITEN_STOP | I2C_ITEN_TXE | I2C_ITEN_ERR);
	/*Enable the sending completion interrupt*/
	LL_I2C_EnableIT(cfg->reg, I2C_ITEN_TC);
	
	if (k_sem_take(&data->device_sync_sem,
			K_MSEC(LE501X_I2C_TRANSFER_TIMEOUT_MSEC)) != 0){
		LOG_I("%s: timeout", __func__);
		data->errs |= I2C_LE501X_ERR_TIMEOUT;
		k_sem_give(&data->device_sync_sem);
	}
	return check_error(data->errs);
}
  

int le501x_i2c_msg_read(const struct device *dev, 
					struct i2c_msg *msg, uint16_t slave)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;
	// uint32_t RxCount = 0;
	int32_t res;
	/* Check the I2C handle allocation */
	if (cfg->reg == NULL)
	{
		return ENXIO;
	}

	res = i2c_msg_init_restart(dev,msg,slave);

	if(res < 0)
	{
		return res;
	}

	/*Enable the sending completion interrupt*/
	LL_I2C_EnableIT(cfg->reg, I2C_ITEN_TC);
	/* Clear interrupt Flag */
	LL_I2C_ClearFlagIT(cfg->reg, I2C_ITIC_RXNE | I2C_ITIC_ADDR | I2C_ITIC_NACK | I2C_ITIC_STOP | I2C_ITIC_ERR);

	/* Enable EVT, RXNE and ERR interrupt */
	LL_I2C_EnableIT(cfg->reg, I2C_ITEN_ADDR | I2C_ITEN_NACK | I2C_ITEN_STOP | I2C_ITEN_RXNE | I2C_ITEN_ERR);
	
	if (k_sem_take(&data->device_sync_sem,
			K_MSEC(LE501X_I2C_TRANSFER_TIMEOUT_MSEC)) != 0){
		LOG_I("%s: timeout", __func__);
		data->errs |= I2C_LE501X_ERR_TIMEOUT;
		k_sem_give(&data->device_sync_sem);
	}
	return check_error(data->errs);
}



#else 
int le501x_i2c_msg_write(const struct device *dev, 
					struct i2c_msg *msg,uint16_t saddr)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;
	int32_t res;
	uint16_t timeout;
	/* Check the I2C handle allocation */
	if (cfg->reg == NULL)
	{
		return ENXIO;
	}

	res = i2c_msg_init_restart(dev,msg,saddr);

	if(res < 0)
	{
		return res;
	}

	timeout = LE501X_I2C_TIMEOUT_USEC;
	while (data->xfer_len)
	{
		/* (6.1) Transmit data (TXE flag raised) **********************************/
		/* Check TXE flag value in ISR register */
		if (LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_NACKF))
		{
			LL_I2C_GenerateStopCondition(cfg->reg);
			LL_I2C_Disable(cfg->reg);
			return -EPERM;
		}
		
		while (I2C_GET_TXFLV(cfg->reg) == LE_I2C_FIFO_DEPTH) // && !LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_TXE) 
		{
			if (le501x_i2c_wait_timeout(&timeout)) {
				LOG_I("i2c stop timeout");
				LL_I2C_Disable(cfg->reg);
				return -EBUSY;
			}
		}
		timeout = LE501X_I2C_TIMEOUT_USEC;
		/* Write data in Transmit Data register.
		TXE flag is cleared by writing data in TXDR register */
		LL_I2C_TransmitData8(cfg->reg, (*data->current_msg->buf));
		/* Increment Buffer pointer */
		/* Update counter */
		data->current_msg->buf++;
		data->current_msg->len--;
		data->xfer_len--;

		if ((data->xfer_len > 0U) && (data->current_msg->len == 0U)){
			data->current_msg++;
		}
	}

	timeout = LE501X_I2C_TIMEOUT_USEC;
	if (msg->flags & I2C_MSG_STOP)
	{
		LL_I2C_GenerateStopCondition(cfg->reg);
		while (!LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_STOPF))
		{
			if (le501x_i2c_wait_timeout(&timeout)) {
				LOG_I("i2c stop timeout");
				LL_I2C_Disable(cfg->reg);
				return -EBUSY;
			}
		}
		LL_I2C_ClearSR(cfg->reg);
	}
	return 0;
}

int le501x_i2c_msg_read(const struct device *dev, 
					struct i2c_msg *msg, uint16_t slave)
{
	const struct i2c_le501x_config *cfg = dev->config;
	struct i2c_le501x_data *data = dev->data;
	int32_t res;
	uint16_t timeout;
	/* Check the I2C handle allocation */
	if (cfg->reg == NULL)
	{
		return ENXIO;
	}

	res = i2c_msg_init_restart(dev,msg,slave);

	if(res < 0)
	{
		return res;
	}
	
	while (data->xfer_len) 
	{
		timeout = LE501X_I2C_TIMEOUT_USEC;
		while(I2C_GET_RXFLV(cfg->reg) == 0U)
		{
			if (le501x_i2c_wait_timeout(&timeout)) {
				LOG_I("i2c transfer	timeout");
				LL_I2C_Disable(cfg->reg);
				return -EBUSY;
			}
		}
		*data->current_msg->buf = LL_I2C_ReceiveData8(cfg->reg);
		data->current_msg->buf++;
		data->current_msg->len--;
		data->xfer_len--;

		if ((data->xfer_len > 0U) && (data->current_msg->len == 0U)){
			data->current_msg++;
		}
	}

	timeout = LE501X_I2C_TIMEOUT_USEC;
	if (msg->flags & I2C_MSG_STOP)
	{
		// LL_I2C_GenerateStopCondition(cfg->reg); //auto end
		while (!LL_I2C_IsActiveFlag(cfg->reg, I2C_SR_STOPF))
		{
			if (le501x_i2c_wait_timeout(&timeout)) {
				LOG_I("i2c stop timeout");
				LL_I2C_Disable(cfg->reg);
				return -EBUSY;
			}
		}
		LL_I2C_ClearSR(cfg->reg);
	}
	return 0;
}

#endif


#define OPERATION(msg) (((struct i2c_msg *) msg)->flags & I2C_MSG_RW_MASK)


static int i2c_le501x_transfer(const struct device *dev, struct i2c_msg *msg,
			      uint8_t num_msgs, uint16_t slave)
{
	struct i2c_le501x_data *data = dev->data;
	struct i2c_msg *current, *next;
	int ret = 0;

	/* Check for validity of all messages, to prevent having to abort
	 * in the middle of a transfer
	 */
	current = msg;

	/*
	 * Set I2C_MSG_RESTART flag on first message in order to send start
	 * condition
	 */
	current->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {    

		if (i < num_msgs) {
			next = current + 1;

			/*
			 * Restart condition between messages
			 * of different directions is required
			 */

			if (OPERATION(current) != OPERATION(next)) { 
				if (!(next->flags & I2C_MSG_RESTART)) {
					ret = -EINVAL;
					break;
				}
			}

			/* le501x cannot transmit bytes larger than 255	*/
			if(current->len > 0xFF){
				ret = -EINVAL;
				break;
			}

			/* Stop condition is only allowed on last message */
			if (current->flags & I2C_MSG_STOP) {   
				ret = -EINVAL;
				break;
			}
		} else {
			/* Stop condition is required for the last message */
			current->flags |= I2C_MSG_STOP;
		}

		current++;
	}

	if (ret) {
		return ret;
	}

	/* Send out messages */
	k_sem_take(&data->bus_mutex, K_FOREVER);  

	// current = msg;
	uint8_t itr = 0;
	for (uint8_t i = 0; i < num_msgs; i = itr) {
		data->current_msg = &msg[i];
		data->xfer_len = msg[i].len;

		for (itr = i + 1; itr < num_msgs; itr++) {
			if ((data->current_msg->flags & I2C_MSG_RW_MASK) !=
				(msg[itr].flags & I2C_MSG_RW_MASK)) {
				break;
			}
			data->xfer_len += msg[itr].len;
			/* le501x cannot transmit bytes larger than 255	*/
			if(data->xfer_len > 0xFF) {
				return -EINVAL;
			}
		}
		if (data->current_msg->flags & I2C_MSG_READ) {
			ret = le501x_i2c_msg_read(dev,data->current_msg,slave);
		} else {
			ret = le501x_i2c_msg_write(dev,data->current_msg,slave);
		}
		LOG_I("ret %d", ret);
		if (ret < 0) {
			break;
		}
	}
	k_sem_give(&data->bus_mutex);
	return ret;
}




static bool I2C_speed_config_calc_master_dft(const struct device *dev, struct i2c_speed_config_t *speed_config)
{
	const struct i2c_le501x_config *config = dev->config;
    /* When calculating master speed config, it should be in init state */
	uint16_t cycle_count = I2C_CLOCK / config->bitrate;
	int16_t scll, sclh, scldel, sdadel;
	uint8_t prescaler = 0;
	if (cycle_count > 256)
	{
		for (uint8_t i = 1; prescaler < 16; i++)
		{
			/* For easier calculating, just set prescaler to an odd number. */
			prescaler = 2 * i - 1;
			cycle_count = (I2C_CLOCK / config->bitrate) / (prescaler + 1);
			if (cycle_count <= 256)
			{
				break;
			}
		}
		if (prescaler >= 16)
		{
			return false;
		}
	}
	if (cycle_count < 16)
	{
		return false;
	}
	/* Set SCL dutycycle to about 30% */
	scll = (cycle_count * 2) / 3;
	sclh = cycle_count / 3 - 1 - 4;
	if (scll < 1)
	{
		scll = 1;
	}
	if (sclh < 0)
	{
		sclh = 0;
	}
	scldel = scll / 3;
	if (scldel > 15)
	{
		scldel = 15;
	}
	else if (scldel < 5)
	{
		/* SCLDEL should not bee too small to keep campatible with different resistance */
		scldel = 5;
	}
	sdadel = scldel - 4;
	if (sdadel > 4)
	{
		/* We should set a ceiling for SDADEL. This is unnecessary, and will reduce compatibility if we are IIC slave */
		sdadel = 4;
	}
	else if (sdadel < 1)
	{
		sdadel = 1;
	}
	/* HW design requires that SCLDEL should be no less than SDADEL + 5 */
	if (scldel <= sdadel + 5)
	{
		scldel = sdadel + 5;
	}
	speed_config->presc = prescaler;
	speed_config->scll = scll;
	speed_config->sclh = sclh;
	speed_config->scldel = scldel;
	speed_config->sdadel = sdadel;
	speed_config->role = 1;

    return true;
}

static int i2c_le501x_runtime_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_le501x_config *config = dev->config;
	struct i2c_le501x_data *data = (struct i2c_le501x_data *const)(dev)->data;  

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		// LOG_ERR("Only I2C Master mode supported.");
		return ENOTSUP;
	}
    LL_I2C_Disable(config->reg);
	I2C_speed_config_calc_master_dft(dev,&speed_config);
    MODIFY_REG(config->reg->TIMINGR, I2C_TIMINGR_PRESC_MASK, speed_config.presc << I2C_TIMINGR_PRESC_POS); 
    MODIFY_REG(config->reg->TIMINGR, (I2C_TIMINGR_SCLH_MASK | I2C_TIMINGR_SCLL_MASK | I2C_TIMINGR_SDADEL_MASK | I2C_TIMINGR_SCLDEL_MASK), 
                                        (speed_config.sclh<<I2C_TIMINGR_SCLH_POS\
                                        |speed_config.scll<<I2C_TIMINGR_SCLL_POS\
                                        |speed_config.sdadel<<I2C_TIMINGR_SDADEL_POS\
                                        |speed_config.scldel<<I2C_TIMINGR_SCLDEL_POS));	

    data->dev_config = dev_config;

    LL_I2C_SetDigitalFilter(config->reg, 0);
    LL_I2C_SetClockPeriod(config->reg, config->bitrate); 
    LL_I2C_AcknowledgeNextData(config->reg, LL_I2C_ACK);

	LL_I2C_Enable(config->reg);
    return  0;
}

static int i2c_le501x_init(const struct device *dev)
{
	const struct i2c_le501x_config *cfg = dev->config;
	int ret;
	struct i2c_le501x_data *data = dev->data;
#ifdef CONFIG_I2C_LE501X_INTERRUPT
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
	cfg->irq_config_func(dev);
#endif

	/* Configure dt provided device signals when available */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);   //pin
	if (ret < 0) {
		// LOG_ERR("I2C pinctrl setup failed (%d)", ret);
		return ret;
	}

    if(cfg->reg == I2C1)
    {
		REG_FIELD_WR(RCC->APB1RST, RCC_I2C1, 1);
		REG_FIELD_WR(RCC->APB1RST, RCC_I2C1, 0);
		REG_FIELD_WR(RCC->APB1EN, RCC_I2C1, 1);
    }else if(cfg->reg == I2C2)
    {
		REG_FIELD_WR(RCC->APB1RST, RCC_I2C2, 1);
		REG_FIELD_WR(RCC->APB1RST, RCC_I2C2, 0);
		REG_FIELD_WR(RCC->APB1EN, RCC_I2C2, 1);
    }else
    {
        return ENOTSUP;
    }

	/*
	 * initialize mutex used when multiple transfers
	 * are taking place to guarantee that each one is
	 * atomic and has exclusive access to the I2C bus.
	 */
	k_sem_init(&data->bus_mutex, 1, 1);

	REG_FIELD_WR(cfg->reg->CR1, I2C_CR1_RXFTH, LE_I2C_FIFO_DEPTH-1);


	ret = i2c_le501x_runtime_configure(dev,cfg->default_config); //
	if (ret < 0) {
		// LOG_ERR("i2c: failure initializing");
		return ret;
	}

	return 0;
}


static const struct i2c_driver_api api_funcs = {
	.configure = i2c_le501x_runtime_configure,
	.transfer = i2c_le501x_transfer,
};


#ifdef CONFIG_I2C_LE501X_INTERRUPT

#define LE501X_I2C_IRQ_ENBALE(index)				\
	do {								\
		IRQ_CONNECT(DT_INST_IRQN(index),			\
			    DT_INST_IRQ(index, priority),		\
			    le501x_i2c_combined_isr,			\
			    DEVICE_DT_INST_GET(index), 0);		\
		irq_enable(DT_INST_IRQN(index));			\
	} while (false)

#define LE501X_I2C_IRQ_HANDLER_DECL(index)				\
static void i2c_le501x_irq_config_func_##index(const struct device *dev)
#define LE501X_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.irq_config_func = i2c_le501x_irq_config_func_##index,
#define LE501X_I2C_IRQ_HANDLER(index)					\
static void i2c_le501x_irq_config_func_##index(const struct device *dev)	\
{									\
	LE501X_I2C_IRQ_ENBALE(index);			\
}
#else

#define LE501X_I2C_IRQ_HANDLER_DECL(index)
#define LE501X_I2C_IRQ_HANDLER_FUNCTION(index)
#define LE501X_I2C_IRQ_HANDLER(index)

#endif /* CONFIG_I2C_INTERRUPT */


#define LE501X_I2C_INIT(index)						\
LE501X_I2C_IRQ_HANDLER_DECL(index);					\
									\
PINCTRL_DT_INST_DEFINE(index);						\
									\
static const struct i2c_le501x_config i2c_le501x_cfg_##index = {		\
	.reg = (reg_i2c_t *)DT_INST_REG_ADDR(index),			\
	LE501X_I2C_IRQ_HANDLER_FUNCTION(index)				\
	.bitrate = DT_INST_PROP(index, clock_frequency),		\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
    .default_config = I2C_MODE_CONTROLLER,                  \
};									\
									\
static struct i2c_le501x_data i2c_le501x_dev_data_##index;		\
									\
I2C_DEVICE_DT_INST_DEFINE(index, i2c_le501x_init,			\
			 NULL, &i2c_le501x_dev_data_##index,		\
			 &i2c_le501x_cfg_##index,			\
			 POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,		\
			 &api_funcs);					\
									\
LE501X_I2C_IRQ_HANDLER(index)


DT_INST_FOREACH_STATUS_OKAY(LE501X_I2C_INIT)
