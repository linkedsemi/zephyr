#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <soc.h>
#include <platform.h>
#include <fifo.h>

LOG_MODULE_REGISTER(mbox_linkedsem);

#define DT_DRV_COMPAT linkedsemi_mbox

#if (DT_INST_IRQN(0) == SYSC_SEC_CPU_IRQN)
#define MBOX_RX_CHANNEL_ID    0
#define MBOX_TX_CHANNEL_ID    1
#else
#define MBOX_RX_CHANNEL_ID    1
#define MBOX_TX_CHANNEL_ID    0
#endif
#define MBOX_NCHANNELS        DT_NUM_INST_STATUS_OKAY(vnd_mbox_consumer)

BUILD_ASSERT(MBOX_NCHANNELS > 0, "vnd,mbox-consumer not found");
BUILD_ASSERT(DT_INST_CHILD_NUM(0) > 0, "vnd,mbox-consumer child not found");

enum mbox_channel_number {
    MBOX_CH0,
    MBOX_CH1,
};

#define MBOX_RX_CH_SEC MBOX_CH0
#define MBOX_RX_CH_APP MBOX_CH1

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct mbox_linkedsemi_config {
    irq_cfg_func_t irq_config_func;
    struct {
        struct fifo_env *env;
        size_t width;
        size_t deepth;
    } fifo[2][MBOX_NCHANNELS];
};

struct mbox_linkedsemi_data {
    mbox_callback_t cb[MBOX_NCHANNELS];
    void *user_data[MBOX_NCHANNELS];
};

#define GET_WR_IDX(fifo) ((fifo)->wr_idx>=(fifo)->length?(fifo)->wr_idx - (fifo)->length:(fifo)->wr_idx)
__ramfunc static bool general_fifo_put_size(struct fifo_env *ptr, void *data, size_t size)
{
    if(sw_fifo_full(ptr)) {
        return false;
    } else {
        uint8_t *elem = ptr->buf;
        __ASSERT_NO_MSG(size <= ptr->item_size);
        memcpy((void*)(elem + ptr->item_size * GET_WR_IDX(ptr)), data, size);
        if(ptr->wr_idx + 1 == 2*ptr->length) {
            ptr->wr_idx = 0;
        } else {
            ptr->wr_idx = ptr->wr_idx + 1;
        }
        return true;
    }
}

static void mbox_linkedsemi_rx_callback_handle(const struct device *dev, uint32_t channel)
{
    /* handle events of all rx channels */
    const struct mbox_linkedsemi_config *dev_config = dev->config;
    struct mbox_linkedsemi_data *dev_data = dev->data;
    bool ret;
    do {
        uint8_t recv_data[DT_INST_PROP(0, mtu)];
        ret = general_fifo_get(dev_config->fifo[MBOX_RX_CHANNEL_ID][channel].env, recv_data);
        if (ret) {
            struct mbox_msg msg = {
                (const void *)recv_data,
                dev_config->fifo[MBOX_RX_CHANNEL_ID][channel].width,
            };
            if (dev_data->cb[channel]) {
                dev_data->cb[channel](dev, channel, dev_data->user_data[channel], &msg);
            } else {
                DEV_WRN(dev, "channel: %d callback() is NULL", channel);
                return;
            }
        }
#if defined(CONFIG_SIGNALLING_MODE_SUPPORT)
        else {
            dev_data->cb[channel](dev, channel, dev_data->user_data[channel], NULL);
        }
#endif
    } while (ret);
}

static void mbox_linkedsemi_isr(const struct device *dev)
{
    if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_SEC) {
        cpu_intr_sec_clr();
    } else if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_APP) {
        cpu_intr_app_clr();
    } else {
        __ASSERT(0, "channel invalid!\n");
    }

    for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
        mbox_linkedsemi_rx_callback_handle(dev, i);
    }
}

__ramfunc static int mbox_linkedsemi_send(const struct device *dev, uint32_t channel, const struct mbox_msg *msg)
{
    const struct mbox_linkedsemi_config *dev_config = dev->config;
    bool ret;

    if (channel >= MBOX_NCHANNELS) {
        DEV_ERR(dev, "invalid channel: %d\n", channel);
        return -EINVAL;
    }

    if (msg) {
        size_t width = dev_config->fifo[MBOX_TX_CHANNEL_ID][channel].width;

        if ((msg->size > width) || ((msg->size > 0U) && (msg->data == NULL))) {
            DEV_ERR(dev, "msg size error, channel: %d, msg size: %zu, mtu: %zu\n", channel, msg->size, width);
            return -EMSGSIZE;
        }

        ret = general_fifo_put_size(dev_config->fifo[MBOX_TX_CHANNEL_ID][channel].env, (void *)msg->data, msg->size);
        if (ret == false) {
            DEV_ERR(dev, "fifo full, channel: %d\n", channel);
            return -ENOSPC;
        }
    }
#if !defined(CONFIG_SIGNALLING_MODE_SUPPORT)
    else {
        DEV_ERR(dev, "Not supported signalling mode\n");
        return -ENOTSUP;
    }
#endif

    if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_SEC) {
        cpu_intr_app_activate();
    } else if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_APP) {
        cpu_intr_sec_activate();
    } else {
        DEV_ERR(dev, "channel invalid! it must be %d or %d\n", MBOX_RX_CH_SEC, MBOX_RX_CH_APP);
        return -ENOTSUP;
    }

    return 0;
}

static int mbox_linkedsemi_register_callback(const struct device *dev, uint32_t channel, mbox_callback_t cb, void *user_data)
{
    struct mbox_linkedsemi_data *dev_data = dev->data;

    if (channel >= MBOX_NCHANNELS) {
        DEV_ERR(dev, "invalid channel: %d\n", channel);
        return -EINVAL;
    }

    dev_data->cb[channel] = cb;
    dev_data->user_data[channel] = user_data;

    return 0;
}

int mbox_linkedsemi_tx_mtu_get(const struct device *dev, uint32_t channel)
{
    const struct mbox_linkedsemi_config *dev_config = dev->config;

    if (channel >= MBOX_NCHANNELS) {
        DEV_ERR(dev, "invalid channel: %d\n", channel);
        return -EINVAL;
    }

    return dev_config->fifo[MBOX_TX_CHANNEL_ID][channel].width;
}

int mbox_linkedsemi_rx_mtu_get(const struct device *dev, uint32_t channel)
{
    const struct mbox_linkedsemi_config *dev_config = dev->config;

    if (channel >= MBOX_NCHANNELS) {
        DEV_ERR(dev, "invalid channel: %d\n", channel);
        return -EINVAL;
    }

    return dev_config->fifo[MBOX_RX_CHANNEL_ID][channel].width;
}

static uint32_t mbox_linkedsemi_max_channels_get(const struct device *dev)
{
    ARG_UNUSED(dev);
    /* Only two channels supported, one RX and one TX */
    return MBOX_NCHANNELS;
}

static int mbox_linkedsemi_set_enabled(const struct device *dev, uint32_t channel, bool enable)
{
    const struct mbox_linkedsemi_config *dev_config = dev->config;
    struct mbox_linkedsemi_data *dev_data = dev->data;

    if (channel >= MBOX_NCHANNELS) {
        DEV_ERR(dev, "invalid channel: %d\n", channel);
        return -EINVAL;
    }

    if (enable) {
        if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_SEC) {
            cpu_intr_sec_unmask();
            if (dev_data->cb[channel]) {
                mbox_linkedsemi_rx_callback_handle(dev, channel);
            }
        } else if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_APP) {
            cpu_intr_app_unmask();
            if (dev_data->cb[channel]) {
                mbox_linkedsemi_rx_callback_handle(dev, channel);
            }
        } else {
            __ASSERT(0, "channel invalid!\n");
            return -1;
        }
        if (!irq_is_enabled(DT_INST_IRQN(0))) {
            dev_config->irq_config_func(dev);
        }
    } else {
        if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_SEC) {
            irq_disable(DT_INST_IRQN(0));
            cpu_intr_sec_clr();
            cpu_intr_sec_mask();
        } else if (MBOX_RX_CHANNEL_ID == MBOX_RX_CH_APP) {
            irq_disable(DT_INST_IRQN(0));
            cpu_intr_app_clr();
            cpu_intr_app_mask();
        } else {
            __ASSERT(0, "channel invalid!\n");
            return -1;
        }
    }

    return 0;
}

static int mbox_linkedsemi_init(const struct device *dev)
{
    const struct mbox_linkedsemi_config *dev_config = dev->config;

    for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
        struct fifo_env *fifo = dev_config->fifo[MBOX_TX_CHANNEL_ID][i].env;
        fifo->rd_idx = 0;
        fifo->wr_idx = 0;
        fifo->buf = (void *)(((uint8_t *)fifo) + sizeof(struct fifo_env));
        fifo->item_size = dev_config->fifo[MBOX_TX_CHANNEL_ID][i].width;
        fifo->length = dev_config->fifo[MBOX_TX_CHANNEL_ID][i].deepth;
    }

    return 0;
}

static struct mbox_driver_api mbox_linkedsemi_driver_api = {
    .send = mbox_linkedsemi_send,
    .register_callback = mbox_linkedsemi_register_callback,
    .max_channels_get = mbox_linkedsemi_max_channels_get,
    .set_enabled = mbox_linkedsemi_set_enabled,
};

#define MBOX_LINKEDSEMI_IRQ_HANDLER(inst)                                    \
    static void mbox_linkedsemi_config_func_##inst(const struct device *dev) \
    {                                                                        \
        IRQ_CONNECT(DT_INST_IRQN(inst),                                      \
                    DT_INST_IRQ(inst, priority),                             \
                    mbox_linkedsemi_isr,                                     \
                    DEVICE_DT_INST_GET(inst),                                \
                    0);                                                      \
        irq_enable(DT_INST_IRQN(inst));                                      \
    }

#if (MBOX_RX_CHANNEL_ID == 0)
#define MBOX_CONSUMER_CHILD_REGION(node_id)                                                                        \
    uint8_t mbox_linkedsemi_fifo_rx_##node_id[(sizeof(struct fifo_env)                                             \
                                                + (DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_deepth)                \
                                                * DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_width)))] __aligned(4); \
    uint8_t mbox_linkedsemi_fifo_tx_##node_id[(sizeof(struct fifo_env)                                             \
                                                + (DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_deepth)                \
                                                * DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_width)))] __aligned(4);
#else
#define MBOX_CONSUMER_CHILD_REGION(node_id)                                                                        \
    uint8_t mbox_linkedsemi_fifo_tx_##node_id[(sizeof(struct fifo_env)                                             \
                                                + (DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_deepth)                \
                                                * DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_width)))] __aligned(4); \
    uint8_t mbox_linkedsemi_fifo_rx_##node_id[(sizeof(struct fifo_env)                                             \
                                                + (DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_deepth)                \
                                                * DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_width)))] __aligned(4);
#endif

#define MBOX_CONSUMER_CHILD_RX(node_id) {                                  \
    .env = (struct fifo_env *)mbox_fifo.mbox_linkedsemi_fifo_rx_##node_id, \
    .width = DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_width),              \
    .deepth = DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_deepth),            \
},

#define MBOX_CONSUMER_CHILD_TX(node_id) {                                  \
    .env = (struct fifo_env *)mbox_fifo.mbox_linkedsemi_fifo_tx_##node_id, \
    .width = DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_width),              \
    .deepth = DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_deepth),            \
},

#define BUILD_ASSERT_MBOX_CONSUMER_CHILD_RX_CHECK_MTU(node_id, mtu_rx) \
    BUILD_ASSERT(DT_PHA_BY_NAME(node_id, mboxes, rx, fifo_width) <= mtu_rx);
#define BUILD_ASSERT_MBOX_CONSUMER_CHILD_TX_CHECK_MTU(node_id, mtu_tx) \
    BUILD_ASSERT(DT_PHA_BY_NAME(node_id, mboxes, tx, fifo_width) <= mtu_tx);

DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(0, BUILD_ASSERT_MBOX_CONSUMER_CHILD_RX_CHECK_MTU, DT_INST_PROP(0, mtu))
DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(0, BUILD_ASSERT_MBOX_CONSUMER_CHILD_TX_CHECK_MTU, DT_INST_PROP(0, mtu))

MBOX_LINKEDSEMI_IRQ_HANDLER(0)

static struct mbox_linkedsemi_data mbox_linkedsemi_data_0;
typedef struct __packed {
    DT_INST_FOREACH_CHILD_STATUS_OKAY(0, MBOX_CONSUMER_CHILD_REGION)
} mbox_fifo_t;
mbox_fifo_t mbox_fifo __attribute__((section("MBOX")));

static const struct mbox_linkedsemi_config mbox_linkedsemi_config_0 = {
    .irq_config_func = mbox_linkedsemi_config_func_0,
    .fifo = {
        [MBOX_RX_CHANNEL_ID] = { DT_INST_FOREACH_CHILD_STATUS_OKAY(0, MBOX_CONSUMER_CHILD_RX) },
        [MBOX_TX_CHANNEL_ID] = { DT_INST_FOREACH_CHILD_STATUS_OKAY(0, MBOX_CONSUMER_CHILD_TX) },
    },
};

DEVICE_DT_INST_DEFINE(0,
                      mbox_linkedsemi_init,
                      NULL,
                      &mbox_linkedsemi_data_0,
                      &mbox_linkedsemi_config_0,
                      PRE_KERNEL_1,
                      CONFIG_MBOX_INIT_PRIORITY,
                      &mbox_linkedsemi_driver_api);
