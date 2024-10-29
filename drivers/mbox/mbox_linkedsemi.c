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

#define MBOX_BASE_ADDRESS  (DT_INST_REG_ADDR(0))
#define MBOX_SIZE          (DT_INST_REG_SIZE(0))
#define MBOX_NCHANNELS     (DT_INST_PROP(0, nchannels))
#define MBOX_FIFO_DEEPTH   (DT_INST_PROP(0, fifo_deepth))
#define MBOX_FIFO_WIDTH    (DT_INST_PROP(0, fifo_width))
#define MBOX_RX_CHANNEL_ID (DT_MBOX_CHANNEL_BY_NAME(DT_NODELABEL(mbox_consumer0), rx))

#define CALC_MBOX_SIZE (((MBOX_FIFO_DEEPTH * MBOX_FIFO_WIDTH) + sizeof(struct fifo_env)) * MBOX_NCHANNELS * 2)
BUILD_ASSERT(CALC_MBOX_SIZE <= MBOX_SIZE, "fifo size overflow\n");

enum mbox_channel_number {
    MBOX_CH0,
    MBOX_CH1,
};

struct mbox_linkedsemi_data {
    mbox_callback_t cb[MBOX_NCHANNELS];
    void *user_data[MBOX_NCHANNELS];
    uint8_t recv_data[MBOX_NCHANNELS][MBOX_FIFO_WIDTH];
    struct fifo_env *fifo[MBOX_NCHANNELS * 2];
    const struct device *dev;
    uint32_t enabled_mask;
};

static struct mbox_linkedsemi_data linkedsemi_mbox_data;

static void mbox_linkedsemi_isr(const struct device *dev)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    bool ret;

    if (MBOX_RX_CHANNEL_ID == MBOX_CH0) {
        cpu_intr0_clr();
    } else if (MBOX_RX_CHANNEL_ID == MBOX_CH1) {
        cpu_intr1_clr();
    } else {
        __ASSERT(0, "channel invalid!\n");
    }

    /* handle events of all rx channels */
    for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
        const uint8_t rx_fifo_idx = MBOX_RX_CHANNEL_ID + i * 2;
        do {
            ret = general_fifo_get(dev_data->fifo[rx_fifo_idx], dev_data->recv_data[i]);
            if (ret) {
                struct mbox_msg msg = { (const void *)(dev_data->recv_data[i]), MBOX_FIFO_WIDTH };
                dev_data->cb[i](dev, rx_fifo_idx, dev_data->user_data, &msg);
            }
#if defined(CONFIG_SIGNALLING_MODE_SUPPORT)
            else {
                dev_data->cb[i](dev, rx_fifo_idx, dev_data->user_data, NULL);
            }
#endif
        } while (ret);
    }
}

static int mbox_linkedsemi_send(const struct device *dev, uint32_t channel, const struct mbox_msg *msg)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    bool ret;

#if 0
    if (msg->size != MBOX_FIFO_WIDTH) {
        /* We can only send this many bytes at a time. */
        return -EMSGSIZE;
    }
#endif

    if (msg) {
        ret = general_fifo_put(dev_data->fifo[channel], (void *)msg->data);
        if (ret == false) {
            return -ENOSPC;
        }
    }
#if !defined(CONFIG_SIGNALLING_MODE_SUPPORT)
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

static int mbox_linkedsemi_register_callback(const struct device *dev, uint32_t channel, mbox_callback_t cb, void *user_data)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;

    dev_data->cb[channel / 2] = cb;
    dev_data->user_data[channel / 2] = user_data;

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
    return MBOX_NCHANNELS;
}

static int mbox_linkedsemi_fifo_init(const struct device *dev)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;
    const uint32_t cell = (MBOX_SIZE / MBOX_NCHANNELS) / 2;

    for (uint8_t i = 0; i < MBOX_NCHANNELS * 2; i++) {
        uint32_t env_addr = MBOX_BASE_ADDRESS + i * cell;
        dev_data->fifo[i] = (struct fifo_env *)env_addr;
        if (i % 2 != MBOX_RX_CHANNEL_ID) {
            dev_data->fifo[i]->buf = (void *)(env_addr + sizeof(struct fifo_env));
            dev_data->fifo[i]->rd_idx = 0;
            dev_data->fifo[i]->wr_idx = 0;
            dev_data->fifo[i]->length = MBOX_FIFO_DEEPTH;
            dev_data->fifo[i]->item_size = MBOX_FIFO_WIDTH;
        }
    }

    return 0;
}

static int mbox_linkedsemi_set_enabled(const struct device *dev, uint32_t channel, bool enable)
{
    uint32_t intr_num = channel % 2;

    if (enable) {
        if (intr_num == MBOX_RX_CHANNEL_ID) {
            if (intr_num == MBOX_CH0) {
                cpu_intr0_unmask();
            } else if (intr_num == MBOX_CH1) {
                cpu_intr1_unmask();
            } else {
                __ASSERT(0, "channel invalid!\n");
                return -1;
            }
        }
    } else {
        if (intr_num == MBOX_CH0) {
            cpu_intr0_clr();
            cpu_intr0_mask();
        } else if (intr_num == MBOX_CH1) {
            cpu_intr1_clr();
            cpu_intr1_mask();
        } else {
            __ASSERT(0, "channel invalid!\n");
            return -1;
        }
    }

    return 0;
}

static int mbox_linkedsemi_init(const struct device *dev)
{
    mbox_linkedsemi_fifo_init(dev);

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

DEVICE_DT_INST_DEFINE(0,
                      mbox_linkedsemi_init,
                      NULL,
                      &linkedsemi_mbox_data,
                      NULL,
                      POST_KERNEL,
                      CONFIG_MBOX_INIT_PRIORITY,
                      &mbox_linkedsemi_driver_api);
