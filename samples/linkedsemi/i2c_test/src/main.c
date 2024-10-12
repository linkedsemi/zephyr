/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>
#include "reg_sysc_per.h"
#include "ls_soc_gpio.h"
#define BUF_SIZE 256
uint8_t wdata_buf[BUF_SIZE];
uint8_t rdata_buf[BUF_SIZE];
static void write_read_compare(const struct device *const i2c,uint16_t dev_addr,const uint8_t *wdata,uint8_t *rdata,uint16_t len)
{
	i2c_burst_write(i2c,dev_addr,0,wdata,len);
#if 0
	io_toggle_pin(PA02);
#endif
	i2c_burst_read(i2c,dev_addr,0,rdata,len);
	if(memcmp(wdata,rdata,len))
	{
		__ASSERT(0,"wdata rdata not match\n");
	}
#if 0
	else
	{
		printf("data: ");
		for(uint32_t i = 0; i < len; i++)
		{
			printf("%x ", wdata[i]);
		}
		printf("\n\n");
	}
#endif
}

static void gen_rand_data(uint8_t *buf,uint16_t len)
{
	while(len--)
	{
		*buf++=rand();
	}
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	const struct device *const i2c1 = DEVICE_DT_GET(DT_ALIAS(testi2c1));
	const struct device *const i2c2 = DEVICE_DT_GET(DT_ALIAS(testi2c2));
	static const struct device *eeprom1 = DEVICE_DT_GET(DT_ALIAS(testeeprom1));
	static const struct device *eeprom2 = DEVICE_DT_GET(DT_ALIAS(testeeprom2));
	static const uint16_t dev_addr1 = DT_REG_ADDR(DT_ALIAS(testeeprom1));
	static const uint16_t dev_addr2 = DT_REG_ADDR(DT_ALIAS(testeeprom2));
	static const uint16_t dev_addr = dev_addr1;

	if (dev_addr1 != dev_addr2) {
		__ASSERT(0,"addr1 should equal to addr2\n");
	}
	if (!device_is_ready(i2c1)) {
		__ASSERT(0,"I2C device is not ready\n");
	}
	if (i2c_configure(i2c1, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER) )
	{
		__ASSERT(0,"I2C device config failed\n");
	}
	if (!device_is_ready(i2c2)) {
		__ASSERT(0,"I2C device is not ready\n");
	}
	if (i2c_configure(i2c2, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER) )
	{
		__ASSERT(0,"I2C device config failed\n");
	}
	if (!device_is_ready(eeprom1)) {
		printk("eeprom device not ready\n");
		return 0;
	}

	if (i2c_target_driver_register(eeprom1) < 0) {
		printk("Failed to register i2c target driver\n");
		return 0;
	}
	if (!device_is_ready(eeprom2)) {
		printk("eeprom device not ready\n");
		return 0;
	}

	if (i2c_target_driver_register(eeprom2) < 0) {
		printk("Failed to register i2c target driver\n");
		return 0;
	}
#if 0
	io_cfg_output(PA02);
#endif

	while(1)
	{
		for(uint16_t len = 1;len<=BUF_SIZE;len++)
		{
			gen_rand_data(wdata_buf,len);
			write_read_compare(i2c2,dev_addr,wdata_buf,rdata_buf,len);
			write_read_compare(i2c1,dev_addr,wdata_buf,rdata_buf,len);
		}
	}

	// io_cfg_output(PF01);
	// struct i2c_msg msg[2];
	// uint8_t data[50];
	// msg[0].buf = data;
	// msg[0].len = 3U;
	// msg[0].flags = I2C_MSG_READ;

	// msg[1].buf = data;
	// msg[1].len = 20;
	// msg[1].flags = I2C_MSG_READ | I2C_MSG_STOP;

	// i2c_transfer(i2c_dev, msg, 2, dev_addr);

	// uint16_t reg_addr=0x56;
	// uint8_t value;
	// i2c_write_read(i2c_dev, dev_addr,
	// 		      &reg_addr, sizeof(reg_addr),
	// 		      &value, sizeof(value));
	// msg[0].buf = (uint8_t *)&msg;
	// msg[0].len = 10U;
	// msg[0].flags = I2C_MSG_WRITE;

	// msg[1].buf = (uint8_t *)&msg;
	// msg[1].len = 5;
	// msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	// i2c_transfer(i2c_dev, msg, 2, dev_addr);


	return 0;
}
