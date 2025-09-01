/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <ls_hal_adcv2.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static enum adc_action ls_completed_callback(const struct device *dev,
                        const struct adc_sequence *sequence,
                        uint16_t sampling_index)
{
    uint16_t *buffer_ptr = (uint16_t *)sequence->buffer;
    printk("it mode conversion result:\n");
    uint8_t i = 0;
    while(buffer_ptr[i] != 0)
    {
        printk("%d, ", buffer_ptr[i]);
        i++;
    }
    printk("\n");

    return ADC_ACTION_FINISH;
}

void ADC_Channel_setCfg(const struct device* adc)
{
    struct adc_channel_cfg channel_config;
#ifdef QSH
    channel_config.channel_id = 8;          //lsqsh
#else
    channel_config.channel_id = 5;       // ls1010 要和设备树中 “pinctrl-0” 节点所对应的通道一致
#endif
    

    channel_config.reference = ADC_REF_INTERNAL;
    channel_config.acquisition_time = ADC_SAMPLETIME_15CYCLES;
    adc_channel_setup(adc, &channel_config);

#ifdef QSH
    channel_config.channel_id = 9;          //lsqsh
#else
    channel_config.channel_id = 6;       // ls1010
#endif

    channel_config.reference = ADC_REF_INTERNAL;
    channel_config.acquisition_time = ADC_SAMPLETIME_15CYCLES;
    adc_channel_setup(adc, &channel_config);    

#ifdef QSH
    channel_config.channel_id = 10;         //lsqsh
#else
    channel_config.channel_id = 7;       // ls1010
#endif

    channel_config.reference = ADC_REF_INTERNAL;
    channel_config.acquisition_time = ADC_SAMPLETIME_15CYCLES;
    adc_channel_setup(adc, &channel_config);
}

int main(void)
{
    printf("start adc multichannel test! %s\n", CONFIG_BOARD_TARGET);

    const struct device * adc = DEVICE_DT_GET(DT_ALIAS(testadc));

    if (!device_is_ready(adc))
    {
    	__ASSERT(0,"adc device is not ready");
    }

    uint16_t *buffer;             // 缓冲区指针
    size_t buf_size = 40 * sizeof(uint16_t);

    buffer = k_malloc(buf_size);
    if (!buffer) {
        printk("内存分配失败\n");
        return -ENOMEM;
    }

    const struct adc_sequence_options options = {
        .extra_samplings	= 0,
        .interval_us 		= 0,
        .callback			= ls_completed_callback,
    };

    struct adc_sequence sequence = {
        .buffer = buffer,
        .buffer_size = buf_size,
        .options = &options
    };


    ADC_Channel_setCfg(adc);

    printk("/*****************************test polling mode******************************/\n");
    for(uint8_t testcount = 0; testcount < 20; testcount++)
    {
        memset(buffer, 0, buf_size);
    	printk("testcount = %d\n", testcount);
        adc_read(adc, &sequence);
        uint16_t *buffer_ptr = (uint16_t *)sequence.buffer;
        printk("polling mode conversion result:\n");
        uint8_t i = 0;
        while(buffer_ptr[i] != 0)
        {
            printk("%d, ", buffer_ptr[i]);
            i++;
        }
        printk("\n");
    }

    printk("/*****************************test it mode******************************/\n");
    int ret;
    for(uint8_t testcount = 0; testcount < 20; testcount++)
    {
        printk("testcount = %d\n", testcount);
        memset(buffer, 0, buf_size);
        struct k_poll_signal async;
        k_poll_signal_init(&async); // 初始化 async 结构体
        struct k_poll_event  async_evt =
	    K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
	    				 K_POLL_MODE_NOTIFY_ONLY,
	    				 &async);
        adc_read_async(adc, &sequence, &async);
	    ret = k_poll(&async_evt, 1, K_FOREVER);
        __ASSERT(ret==0, "k_poll failed!");
    }

    k_free(buffer);

    return 0;
}