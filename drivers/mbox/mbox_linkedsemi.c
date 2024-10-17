/*
 * Copyright (c) 2021 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <reg_sysc_cpu.h>
#include <fifo.h>

LOG_MODULE_REGISTER(mbox_linkedsem_ipc);

#define DT_DRV_COMPAT linkedsemi_mbox

#define MBOX_BASE_ADDRESS    DT_INST_REG_ADDR(0)
#define MBOX_SIZE            DT_INST_REG_SIZE(0)
#define MBOX_FIFO_DEEPTH     DT_INST_PROP(0, fifo_deepth)
#define MBOX_FIFO_WIDTH      DT_INST_PROP(0, fifo_width)
#define MBOX_RX_CHANNEL_ID   DT_MBOX_CHANNEL_BY_NAME(DT_PATH(mbox_consumer), rx)

#define MAX_CHANNELS 2

BUILD_ASSERT(((MBOX_FIFO_DEEPTH * MBOX_FIFO_WIDTH) + sizeof(struct fifo_env)) * 2 <= MBOX_SIZE, "fifo size overflow\n");

enum mbox_channel_number
{
    MBOX_CH0,
    MBOX_CH1,
};

struct mbox_linkedsemi_data {
    mbox_callback_t cb[MAX_CHANNELS];
    void *user_data[MAX_CHANNELS];
    struct fifo_env *fifo[MAX_CHANNELS];
    const struct device *dev;
    uint32_t enabled_mask;
};

static struct mbox_linkedsemi_data linkedsemi_mbox_data;
uint8_t recv_data[MBOX_FIFO_WIDTH];

static void mbox_linkedsemi_isr(const struct device *dev)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    bool ret;

    if (MBOX_RX_CHANNEL_ID == MBOX_CH0) {
        cpu_intr0_clr();
    } else if (MBOX_RX_CHANNEL_ID == MBOX_CH1) {
        cpu_intr1_clr();
    } else {
        LOG_ERR("channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
    }

    do {
        ret = general_fifo_get((struct fifo_env *)dev_data->fifo[MBOX_RX_CHANNEL_ID], recv_data);
        if (ret) {
            struct mbox_msg msg = { (const void *)recv_data, MBOX_FIFO_WIDTH };
            dev_data->cb[MBOX_RX_CHANNEL_ID](dev, MBOX_RX_CHANNEL_ID, dev_data->user_data, &msg);
        }
#if defined(CONFIG_NOTSUP_NULL_MSG)
        else {
            dev_data->cb[MBOX_RX_CHANNEL_ID](dev, MBOX_RX_CHANNEL_ID, dev_data->user_data, NULL);
        }
#endif
    } while(ret);
}

static int mbox_linkedsemi_send(const struct device *dev, uint32_t channel,
             const struct mbox_msg *msg)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    bool ret;

#if 0
    if (msg->size != MBOX_FIFO_WIDTH) {
        /* We can only send this many bytes at a time. */
        return -EMSGSIZE;
    }
#endif

    if(msg) {
        ret = general_fifo_put((struct fifo_env *)dev_data->fifo[channel], (void *)msg->data);
        if (ret == false) {
            return -EBUSY;
        }
    }
#if defined(CONFIG_NOTSUP_NULL_MSG)
    else {
        LOG_ERR("Not supported signalling mode\n");
        return -ENOTSUP;
    }
#endif

    if (MBOX_RX_CHANNEL_ID == MBOX_CH0) {
        cpu_intr1_activate();
    } else if (MBOX_RX_CHANNEL_ID == MBOX_CH1) {
        cpu_intr0_activate();
    } else {
        LOG_ERR("channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
        return -ENOTSUP;
    }

    return 0;
}

static int mbox_linkedsemi_register_callback(const struct device *dev, uint32_t channel,
                      mbox_callback_t cb, void *user_data)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;

    dev_data->cb[channel] = cb;
    dev_data->user_data[channel] = user_data;

    return 0;
}

static int mbox_linkedsemi_mtu_get(const struct device *dev)
{
    ARG_UNUSED(dev);
    return MBOX_FIFO_WIDTH;
}

static uint32_t mbox_linkedsemi_max_channels_get(const struct device *dev)
{
    ARG_UNUSED(dev);
    /* Only two channels supported, one RX and one TX */
    return (MAX_CHANNELS >> 1);
}

static int mbox_linkedsemi_set_enabled(const struct device *dev, uint32_t channel, bool enable)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    uint32_t addr;

    if (enable) {
        if (channel == MBOX_RX_CHANNEL_ID) {
            if (channel == MBOX_CH0) {
                cpu_intr0_clr();
                cpu_intr0_unmask();
            } else if (channel == MBOX_CH1) {
                cpu_intr1_clr();
                cpu_intr1_unmask();
            } else {
                __ASSERT(0, "channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
                return -1;
            }
        }

        if (channel == MBOX_CH0) {
            addr = MBOX_BASE_ADDRESS;
        } else if (channel == MBOX_CH1) {
            addr = MBOX_BASE_ADDRESS + (MBOX_SIZE / 2);
        } else {
            LOG_ERR("channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
            return -1;
        }
        dev_data->fifo[channel] = (struct fifo_env *)addr;
        if(channel != MBOX_RX_CHANNEL_ID) {
            uint32_t buf_offset = (uint32_t)(dev_data->fifo[channel]) + sizeof(struct fifo_env);
            dev_data->fifo[channel]->buf = (void *)buf_offset;
            dev_data->fifo[channel]->rd_idx = 0;
            dev_data->fifo[channel]->wr_idx = 0;
            dev_data->fifo[channel]->length = MBOX_FIFO_DEEPTH;
            dev_data->fifo[channel]->item_size = MBOX_FIFO_WIDTH;
        }
    } else {
        if (channel == MBOX_CH0) {
            cpu_intr0_clr();
            cpu_intr0_mask();
            dev_data->fifo[channel] = NULL;
        } else if (channel == MBOX_CH1) {
            cpu_intr1_clr();
            cpu_intr1_mask();
        } else {
            LOG_ERR("channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
            return -1;
        }
        dev_data->fifo[channel] = NULL;
    }

    return 0;
}

static int mbox_linkedsemi_init(const struct device *dev)
{
    if (MBOX_RX_CHANNEL_ID == MBOX_CH0) {
        mbox_linkedsemi_set_enabled(dev, MBOX_CH1, true);
    } else if (MBOX_RX_CHANNEL_ID == MBOX_CH1) {
        mbox_linkedsemi_set_enabled(dev, MBOX_CH0, true);
    } else {
        LOG_ERR("channel invalid! it must be %d or %d\n", MBOX_CH0, MBOX_CH1);
        return -1;
    }

    cpu_intr0_clr();
    cpu_intr1_clr();

    IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), mbox_linkedsemi_isr, DEVICE_DT_INST_GET(0), 0);

    irq_enable(DT_INST_IRQN(0));

    return 0;
}

static const struct mbox_driver_api mbox_linkedsemi_driver_api = {
    .send = mbox_linkedsemi_send,
    .register_callback = mbox_linkedsemi_register_callback,
    .mtu_get = mbox_linkedsemi_mtu_get,
    .max_channels_get = mbox_linkedsemi_max_channels_get,
    .set_enabled = mbox_linkedsemi_set_enabled,
};

DEVICE_DT_INST_DEFINE(
                0,
                mbox_linkedsemi_init,
                NULL,
                &linkedsemi_mbox_data,
                NULL,
                POST_KERNEL,
                CONFIG_MBOX_INIT_PRIORITY,
                &mbox_linkedsemi_driver_api);
