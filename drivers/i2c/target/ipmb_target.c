#define DT_DRV_COMPAT linkedsemi_ls_ipmb_target

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/drivers/ipmb.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ipmb_target);

#define IPMB_REQUEST_LEN_MIN	7
#define MSG_LEN_IDX				0
#define RS_SA_IDX 				1
#define NETFN_RS_LUN_IDX		2
#define CHECKSUM1_IDX			3

struct i2c_ipmb_target_config{
	struct i2c_dt_spec bus;
};

struct i2c_ipmb_target_data{
//	const struct i2c_ipmb_target_config *dev_cfg;
	struct i2c_target_config config;	
	ipmb_rx_cb_t rx_cb;
	void *param;
	uint8_t *rx_buf;
	uint16_t rx_buf_size;
	uint16_t rx_idx;
};

static int ipmb_target_write_requested(struct i2c_target_config *config)
{
	struct i2c_ipmb_target_data *data = CONTAINER_OF(config,struct i2c_ipmb_target_data,config);
	data->rx_idx = 0;
	data->rx_buf[++data->rx_idx] = config->address<<1;
	return 0;
}

static int ipmb_target_write_received(struct i2c_target_config *config,uint8_t val)
{
	struct i2c_ipmb_target_data *data = CONTAINER_OF(config,struct i2c_ipmb_target_data,config);
	if(data->rx_idx < data->rx_buf_size - 1)
	{
		data->rx_buf[++data->rx_idx] = val;
		return 0;
	}else
	{
		return -1;
	}
}

static uint8_t ipmb_verify_checksum1(uint8_t *rx_buf)
{
	/* The 8 lsb of the sum is 0 when the checksum is valid */
	return rx_buf[RS_SA_IDX]+rx_buf[NETFN_RS_LUN_IDX]+rx_buf[CHECKSUM1_IDX];
}

static bool is_ipmb_msg(struct i2c_ipmb_target_data *data)
{
	if ((data->rx_idx >= IPMB_REQUEST_LEN_MIN) &&
	   (!ipmb_verify_checksum1(data->rx_buf)))
		return true;

	return false;
}

static int ipmb_target_stop(struct i2c_target_config *config)
{
	struct i2c_ipmb_target_data *data = CONTAINER_OF(config,struct i2c_ipmb_target_data,config);
	data->rx_buf[MSG_LEN_IDX] = data->rx_idx;
	if(is_ipmb_msg(data))
	{
		data->rx_cb(data->param,&data->rx_buf);
	}
	return 0;
}

static const struct i2c_target_callbacks ipmb_callbacks = {
	.write_requested = ipmb_target_write_requested,
	.write_received = ipmb_target_write_received,
	.stop = ipmb_target_stop,

};

static int i2c_ipmb_target_init(const struct device *dev)
{
	const struct i2c_ipmb_target_config *cfg = dev->config;
	struct i2c_ipmb_target_data *data = dev->data;
	if (!device_is_ready(cfg->bus.bus)) {
		LOG_ERR("I2C controller device not ready");
		return -ENODEV;
	}
	data->config.address = cfg->bus.addr;
	data->config.callbacks = &ipmb_callbacks;

	return 0;
}

static int ipmb_target_register(const struct device *dev)
{
	const struct i2c_ipmb_target_config *cfg = dev->config;
	struct i2c_ipmb_target_data *data = dev->data;
	return i2c_target_register(cfg->bus.bus, &data->config);
}

static int ipmb_target_unregister(const struct device *dev)
{
	const struct i2c_ipmb_target_config *cfg = dev->config;
	struct i2c_ipmb_target_data *data = dev->data;
	return i2c_target_unregister(cfg->bus.bus, &data->config);
}

static int ipmb_ls_write(const struct device *dev,uint8_t *data,uint32_t size)
{
	const struct i2c_ipmb_target_config *cfg = dev->config;
	return i2c_write(cfg->bus.bus,&data[NETFN_RS_LUN_IDX],data[MSG_LEN_IDX]-1,data[RS_SA_IDX]>>1);
}

static int ipmb_ls_set_rx_callback(const struct device *dev,void (*cb)(void *,uint8_t **),void *param,uint8_t *rx_buf,uint16_t buf_size)
{
	struct i2c_ipmb_target_data *data = dev->data;
	data->rx_cb = cb;
	data->param = param;
	data->rx_buf = rx_buf;
	data->rx_buf_size = buf_size;
	return 0;
}

static const struct ipmb_driver_api api_funcs = {
	.drv_register = ipmb_target_register,
	.drv_unregister = ipmb_target_unregister,
	.write = ipmb_ls_write,
	.set_rx_callback = ipmb_ls_set_rx_callback,
};

#define I2C_IPMB_INIT(inst)						\
	static struct i2c_ipmb_target_data i2c_ipmb_target_##inst##_data;\
	static const struct i2c_ipmb_target_config			\
		i2c_ipmb_target_##inst##_cfg = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst),			\
	};\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &i2c_ipmb_target_init,			\
			    NULL,			\
			    &i2c_ipmb_target_##inst##_data,	\
			    &i2c_ipmb_target_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_TARGET_INIT_PRIORITY,		\
			    &api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_IPMB_INIT)