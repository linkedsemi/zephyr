/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/drivers/i3c/ccc.h>

#include <zephyr/drivers/i3c.h>


#include "core_rv32.h"
static const struct device *i3c_dev_controller = DEVICE_DT_GET(DT_NODELABEL(i3c13));
static const struct device *i3c_dev_target1 = DEVICE_DT_GET(DT_NODELABEL(i3c14));

volatile bool ibi_recived = false;
int target1_ibi_callback(struct i3c_device_desc *target,struct i3c_ibi_payload *payload);
int target2_ibi_callback(struct i3c_device_desc *target,struct i3c_ibi_payload *payload);

int target1_ibi_callback(struct i3c_device_desc *target,struct i3c_ibi_payload *payload)
{
    printf(" target1_ibi_callback ");
    return 0;
}

int target2_ibi_callback(struct i3c_device_desc *target,struct i3c_ibi_payload *payload)
{
    printf(" target2_ibi_callback ");
    return 0;
}

typedef int (*i3c_target_ibi_cb_t)(struct i3c_device_desc *target,
				   struct i3c_ibi_payload *payload);
// static const struct device *i3c_controller = DEVICE_DT_GET(DT_NODELABEL(i3ctarget2));
int i3c_target_ibi_cb(struct i3c_device_desc *target,
				   struct i3c_ibi_payload *payload)
{
    printf(" i3c_target_ibi_cb ,payload : ");
    for(uint8_t i=0;i<payload->payload_len;i++)
    {
        printf(" 0x%X ",payload->payload[i]);
    }
    printf("\r\n");
    ibi_recived = true;
    return 0;
}   

enum{
    TARGET_CONTINUE,
    TARGET_END,
};

#define TEST_COUNT 100
uint8_t target_tx_buffer[TEST_COUNT];
uint8_t target_rx_buffer[TEST_COUNT];
uint8_t controller_tx_buf[TEST_COUNT];
uint8_t controller_rx_buf[TEST_COUNT];
uint8_t test_idx;
int target1_write_requested_cb(struct i3c_target_config *config)
{
    return 0;
}
volatile static uint16_t target_rx_count = 0;
volatile static uint16_t tx_count = 0;
int target1_write_received_cb(struct i3c_target_config *config,uint8_t val)
{
    target_rx_buffer[target_rx_count] = val;
    target_rx_count++;
    return 0;
}

int target1_read_requested_cb(struct i3c_target_config *config,uint8_t *val)
{
    *val = target_tx_buffer[tx_count];
    tx_count++;
    if(tx_count == TEST_COUNT)
    {
        return TARGET_END;
    }

    return TARGET_CONTINUE;
}

int target1_read_processed_cb(struct i3c_target_config *config,uint8_t *val)
{
    *val = target_tx_buffer[tx_count];
    tx_count++;

    if(tx_count == TEST_COUNT)
    {
        return TARGET_END;
    }

    return TARGET_CONTINUE;
}

int target1_stop_cb(struct i3c_target_config *config)
{
    return 0;
}

struct i3c_target_callbacks target1_callbacks = 
{
    .write_requested_cb = target1_write_requested_cb,
    .write_received_cb = target1_write_received_cb,
    .read_requested_cb = target1_read_requested_cb,
    .read_processed_cb = target1_read_processed_cb,
    .stop_cb = target1_stop_cb,
};

struct i3c_target_config target1_cfg;

int main(void)
{

while(1)
{
    for(uint16_t i = 0;i<TEST_COUNT;i++)
    {
        target_tx_buffer[i] = i+1;
        controller_tx_buf[i] = i+1;
        // controller_tx_buf[i] = i+101;
    }
    memset(target_rx_buffer,0,TEST_COUNT);
    memset(controller_rx_buf,0,TEST_COUNT);

    uint8_t target1_addr = 0x32;
    target1_cfg.address = target1_addr;
    target1_cfg.callbacks = &target1_callbacks;
    test_idx = 1;
    /* target */
    i3c_target_register(i3c_dev_target1,&target1_cfg);
    /*controller*/
    struct i3c_device_desc *i3c_target1 = NULL;
    i3c_target1 = i3c_dev_list_i3c_addr_find(i3c_dev_controller,target1_addr);
    i3c_write(i3c_target1,controller_tx_buf,sizeof(controller_tx_buf));

    printf("\r\n test 1 end \r\n");
    printf("target recive end ,rx_count = %d \r\n",target_rx_count);
    for (int i = 0; i < target_rx_count; i++) {
        printf("%d ", target_rx_buffer[i]);
    }
    if(memcmp(controller_tx_buf,target_rx_buffer,TEST_COUNT) !=0)
    {
        printf("\r\n i3c test 1 error\r\n");
        while (1);
    }else
    {
        printf("\r\n i3c test 1 succeed\r\n");
    }


    test_idx = 2;
    /*target*/
    uint8_t pre_count = 0;
    pre_count = i3c_target_tx_write(i3c_dev_target1,target_tx_buffer,sizeof(target_tx_buffer),0);
    tx_count += pre_count;
    printf("target tx pre_count = %d\r\n",pre_count);

    /*controller*/
    i3c_read(i3c_target1,controller_rx_buf,sizeof(controller_rx_buf));
    printf("\r\n test 2 end \r\n");
    printf("controller recive end \r\n");
    for (int i = 0; i < TEST_COUNT; i++) {
        printf("%d ", controller_rx_buf[i]);
    }
    if(memcmp(target_tx_buffer,controller_rx_buf,TEST_COUNT) !=0)
    {
        printf("\r\n i3c test 2 error\r\n");
        while (1);
    }else
    {
        printf("\r\n i3c test 2 succeed\r\n");
    }

    target_rx_count = 0;
    tx_count = 0;


#ifdef CONFIG_I3C_USE_IBI
    /*test 3 ： ibi*/
    /*controller enable ibi function*/
    i3c_ibi_enable(i3c_target1);
    i3c_target1->ibi_cb = i3c_target_ibi_cb;
    /*target raise a ibi*/
    static uint8_t ibi_data[4];/*the qsh i3c controller receives a maximum of 4 bytes of ibi payloads*/
    ibi_data[0] = 0xaa;
    ibi_data[1] = 0xbb;
    ibi_data[2] = 0xcc;
    ibi_data[3] = 0xdd;
    
    static struct i3c_ibi ibi_request =
    {
        .ibi_type = I3C_IBI_TARGET_INTR,
        .payload = ibi_data,
        .payload_len = 4,
    };
    ibi_request.payload_len = 4;
    i3c_ibi_raise(i3c_dev_target1,&ibi_request);

    while(ibi_recived == false);
    ibi_recived = false;

    ibi_request.payload_len = 1;
    i3c_ibi_raise(i3c_dev_target1,&ibi_request);

    while(ibi_recived == false);
    ibi_recived = false;


#endif
}
    while(1);

    return 0;
}
