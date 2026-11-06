#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <soc.h>
#include <fifo.h>

LOG_MODULE_REGISTER(mbox_linkedsemi);

#define DT_DRV_COMPAT linkedsemi_mbox

#define MBOX_NCHANNELS        DT_INST_CHILD_NUM(0)
#define MBOX_INITIATOR        DT_INST_PROP(0, initiator)
BUILD_ASSERT(MBOX_NCHANNELS > 0, "MBOX channels not found");

#define MBOX_TX_FIFO_FIRST (!MBOX_INITIATOR)
#define MBOX_FIFO_TX_SLOT (!MBOX_TX_FIFO_FIRST)
#define MBOX_FIFO_RX_SLOT (!!MBOX_TX_FIFO_FIRST)

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct mbox_fifo_cfg {
	struct fifo_env *env;
	uint16_t width;
	uint16_t depth;
};

struct mbox_fifo_set {
	struct mbox_fifo_cfg cfg[MBOX_NCHANNELS][2];
};

struct mbox_info_s {
	struct mbox_fifo_set fifo;
	uint8_t mbox_magic[3];
	uint8_t total_chans;
};

struct mbox_linkedsemi_config {
	irq_cfg_func_t irq_config_func;
	/* local interrupt clear register address */
	uintptr_t intr_clr;
	/* local interrupt mask register address */
	uintptr_t intr_mask;
	/* peer interrupt set register address, used to raise peer's irq */
	uintptr_t peer_intr_set;
	struct mbox_fifo_set fifo;
};

struct mbox_linkedsemi_data {
	mbox_callback_t cb[MBOX_NCHANNELS];
	void *user_data[MBOX_NCHANNELS];
};


#define MBOX_BASE_ADDR ((uintptr_t)DT_REG_ADDR(DT_NODELABEL(mbox_memory)))
#define MBOX_SIZE (DT_REG_SIZE(DT_NODELABEL(mbox_memory)))
#define MBOX_INFO_ADDR ((uintptr_t)MBOX_BASE_ADDR + MBOX_SIZE - sizeof(struct mbox_info_s))
struct mbox_info_s * const mbox_info = (struct mbox_info_s *)MBOX_INFO_ADDR;
#define GET_WR_IDX(fifo) ((fifo)->wr_idx>=(fifo)->length?(fifo)->wr_idx - (fifo)->length:(fifo)->wr_idx)
#define GET_RD_IDX(fifo) ((fifo)->rd_idx>=(fifo)->length?(fifo)->rd_idx - (fifo)->length:(fifo)->rd_idx)
static bool general_fifo_put_size(struct fifo_env *ptr, void *data, size_t size)
{
	if(sw_fifo_full(ptr) || (size > ptr->item_size)) {
		return false;
	} else {
		uint8_t *elem = ptr->buf;
		memcpy((void*)(elem + ptr->item_size * GET_WR_IDX(ptr)), data, size);
		if(ptr->wr_idx + 1 == 2*ptr->length) {
			ptr->wr_idx = 0;
		} else {
			ptr->wr_idx = ptr->wr_idx + 1;
		}
		return true;
	}
}

static bool general_fifo_peek_ptr(struct fifo_env *ptr,void **data)
{
	if (sw_fifo_empty(ptr)) {
		return false;
	} else {
		uint8_t *elem = ptr->buf;
		*data = (void*)(elem + ptr->item_size * GET_RD_IDX(ptr));
		return true;
	}
}

static void general_fifo_drop(struct fifo_env *ptr)
{
	if (ptr->rd_idx + 1 == 2 * ptr->length) {
		ptr->rd_idx = 0;
	} else {
		ptr->rd_idx = ptr->rd_idx + 1;
	}
}

/* write a value to the given mbox control register, skip if not configured */
static inline void mbox_linkedsemi_reg_write(uintptr_t reg, uint32_t val)
{
	if (reg != 0) {
		*(volatile uint32_t *)reg = val;
	}
}

/* clear pending interrupt of the local cpu */
static inline void mbox_linkedsemi_intr_clr(const struct mbox_linkedsemi_config *dev_config)
{
	mbox_linkedsemi_reg_write(dev_config->intr_clr, 0);
}

/* mask (disable) local cpu interrupt */
static inline void mbox_linkedsemi_intr_mask(const struct mbox_linkedsemi_config *dev_config)
{
	mbox_linkedsemi_reg_write(dev_config->intr_mask, 0);
}

/* unmask (enable) local cpu interrupt */
static inline void mbox_linkedsemi_intr_unmask(const struct mbox_linkedsemi_config *dev_config)
{
	mbox_linkedsemi_reg_write(dev_config->intr_mask, 1);
}

/* raise the interrupt of the peer cpu */
static inline void mbox_linkedsemi_peer_intr_set(const struct mbox_linkedsemi_config *dev_config)
{
	mbox_linkedsemi_reg_write(dev_config->peer_intr_set, 1);
}

/* return true if the channel is present (status okay) in the devicetree */
static inline bool mbox_linkedsemi_channel_enabled(const struct mbox_linkedsemi_config *dev_config,
						   uint32_t channel)
{
	return (dev_config->fifo.cfg[channel][MBOX_FIFO_TX_SLOT].env != NULL) &&
	       (dev_config->fifo.cfg[channel][MBOX_FIFO_RX_SLOT].env != NULL);
}

static void mbox_linkedsemi_rx_callback_handle(const struct device *dev, uint32_t channel)
{
	/* handle events of all rx channels */
	const struct mbox_linkedsemi_config *dev_config = dev->config;
	struct mbox_linkedsemi_data *dev_data = dev->data;
	bool ret;
	do {
		void *recv_data;
		ret = general_fifo_peek_ptr(dev_config->fifo.cfg[channel][MBOX_FIFO_RX_SLOT].env, &recv_data);
		if (ret) {
			struct mbox_msg msg = {
				(const void *)recv_data,
				dev_config->fifo.cfg[channel][MBOX_FIFO_RX_SLOT].width,
			};
			if (dev_data->cb[channel]) {
				dev_data->cb[channel](dev, channel, dev_data->user_data[channel], &msg);
				general_fifo_drop(dev_config->fifo.cfg[channel][MBOX_FIFO_RX_SLOT].env);
			} else {
				DEV_WRN(dev, "channel: %d callback() is NULL, skip", channel);
				return;
			}
		}
	} while (ret);
}

static void mbox_linkedsemi_isr(const struct device *dev)
{
	/* clear local pending interrupt first so that the peer can raise
	 * a new one after this isr finishes
	 */
	mbox_linkedsemi_intr_clr(dev->config);

	for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
		/* skip channels disabled in the devicetree */
		if (!mbox_linkedsemi_channel_enabled(dev->config, i)) {
			continue;
		}
		mbox_linkedsemi_rx_callback_handle(dev, i);
	}
}

static int mbox_linkedsemi_send(const struct device *dev, uint32_t channel, const struct mbox_msg *msg)
{
	const struct mbox_linkedsemi_config *dev_config = dev->config;
	bool ret;

	if (channel >= MBOX_NCHANNELS) {
		DEV_ERR(dev, "invalid channel: %d\n", channel);
		return -EINVAL;
	}

	if (!mbox_linkedsemi_channel_enabled(dev_config, channel)) {
		DEV_ERR(dev, "channel: %d is disabled in DT\n", channel);
		return -ENODEV;
	}

	if (msg) {
		size_t width = dev_config->fifo.cfg[channel][MBOX_FIFO_TX_SLOT].width;

		if ((msg->size > width) || ((msg->size > 0U) && (msg->data == NULL))) {
			DEV_ERR(dev, "msg size error, channel: %d, msg size: %zu, mtu: %zu\n", channel, msg->size, width);
			return -EMSGSIZE;
		}

		ret = general_fifo_put_size(dev_config->fifo.cfg[channel][MBOX_FIFO_TX_SLOT].env, (void *)msg->data, msg->size);
		if (ret == false) {
			DEV_ERR(dev, "fifo full, channel: %d\n", channel);
			return -ENOSPC;
		}
	} else {
		return -EINVAL;
	}

	/* raise the peer cpu interrupt to notify that new data is ready */
	mbox_linkedsemi_peer_intr_set(dev_config);

	return 0;
}

static int mbox_linkedsemi_register_callback(const struct device *dev, uint32_t channel, mbox_callback_t cb, void *user_data)
{
	struct mbox_linkedsemi_data *dev_data = dev->data;

	if (channel >= MBOX_NCHANNELS) {
		DEV_ERR(dev, "invalid channel: %d\n", channel);
		return -EINVAL;
	}

	if (!mbox_linkedsemi_channel_enabled(dev->config, channel)) {
		DEV_ERR(dev, "channel: %d is disabled in DT\n", channel);
		return -ENODEV;
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

	if (!mbox_linkedsemi_channel_enabled(dev_config, channel)) {
		DEV_ERR(dev, "channel: %d is disabled in DT\n", channel);
		return -ENODEV;
	}

	return dev_config->fifo.cfg[channel][MBOX_FIFO_TX_SLOT].width;
}

int mbox_linkedsemi_rx_mtu_get(const struct device *dev, uint32_t channel)
{
	const struct mbox_linkedsemi_config *dev_config = dev->config;

	if (channel >= MBOX_NCHANNELS) {
		DEV_ERR(dev, "invalid channel: %d\n", channel);
		return -EINVAL;
	}

	if (!mbox_linkedsemi_channel_enabled(dev_config, channel)) {
		DEV_ERR(dev, "channel: %d is disabled in DT\n", channel);
		return -ENODEV;
	}

	return dev_config->fifo.cfg[channel][MBOX_FIFO_RX_SLOT].width;
}

static uint32_t mbox_linkedsemi_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);
	/* One TX and one RX fifo per consumer channel */
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

	if (!mbox_linkedsemi_channel_enabled(dev_config, channel)) {
		DEV_ERR(dev, "channel: %d is disabled in DT\n", channel);
		return -ENODEV;
	}

	if (enable) {
		mbox_linkedsemi_intr_unmask(dev_config);
		if (dev_data->cb[channel]) {
			mbox_linkedsemi_rx_callback_handle(dev, channel);
		}
		if (!irq_is_enabled(DT_INST_IRQN(0))) {
			dev_config->irq_config_func(dev);
		}
	} else {
		irq_disable(DT_INST_IRQN(0));
		mbox_linkedsemi_intr_clr(dev_config);
		mbox_linkedsemi_intr_mask(dev_config);
	}

	return 0;
}

#ifdef CONFIG_MBOX_LINKEDSEMI_LAYOUT_CHECK
static bool mbox_info_check(const struct device *dev)
{
	if (mbox_info->mbox_magic[0] != 'M' || mbox_info->mbox_magic[1] != 'B' || mbox_info->mbox_magic[2] != 'X') {
		DEV_ERR(dev, "MBOX magic mismatch, check image");
		return false;
	}
	if (mbox_info->total_chans != MBOX_NCHANNELS) {
		DEV_ERR(dev, "MBOX channel number mismatch, check image");
		return false;
	}
	return true;
}

static bool mbox_fifo_cfg_check(const struct device *dev,const struct mbox_fifo_cfg *cfg, struct mbox_fifo_cfg *info_cfg)
{
	if (cfg->env != info_cfg->env || cfg->width != info_cfg->width || cfg->depth != info_cfg->depth) {
		DEV_ERR(dev, "MBOX TX/RX fifo env/width/depth mismatch, check image");
		return false;
	}
	return true;
}

static int mbox_check(const struct device *dev)
{
	const struct mbox_linkedsemi_config *dev_config = dev->config;
	if (mbox_info_check(dev) == false) {
		return -ENODEV;
	}
	for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
		const struct mbox_fifo_cfg *cfg;
		cfg = &dev_config->fifo.cfg[i][MBOX_FIFO_TX_SLOT];
		if (!mbox_fifo_cfg_check(dev, cfg, &mbox_info->fifo.cfg[i][MBOX_FIFO_RX_SLOT])) {
			return -ENODEV;
		}
		cfg = &dev_config->fifo.cfg[i][MBOX_FIFO_RX_SLOT];
		if (!mbox_fifo_cfg_check(dev, cfg, &mbox_info->fifo.cfg[i][MBOX_FIFO_TX_SLOT])) {
			DEV_ERR(dev, "MBOX RX/TX fifo env/width/depth mismatch, check image");
			return -ENODEV;
		}
	}
	return 0;
}

static inline void mbox_info_init(const struct device *dev)
{
	const struct mbox_linkedsemi_config *dev_config = dev->config;
	mbox_info->mbox_magic[0] = 'M';
	mbox_info->mbox_magic[1] = 'B';
	mbox_info->mbox_magic[2] = 'X';
	mbox_info->total_chans = MBOX_NCHANNELS;
	mbox_info->fifo = dev_config->fifo;
}
#else

static int mbox_check(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

static inline void mbox_info_init(const struct device *dev)
{
	ARG_UNUSED(dev);
}
#endif

#if MBOX_INITIATOR
static void fifo_env_init(struct fifo_env *fifo, uint16_t width, uint16_t depth)
{
	fifo->rd_idx = 0;
	fifo->wr_idx = 0;
	fifo->buf = (void *)(((uint8_t *)fifo) + sizeof(struct fifo_env));
	fifo->item_size = width;
	fifo->length = depth;
}

static int mbox_linkedsemi_init(const struct device *dev)
{
	const struct mbox_linkedsemi_config *dev_config = dev->config;
	if (mbox_initiator_is_peer_working()) {
		return mbox_check(dev);
	} else {
		mbox_info_init(dev);
		for (uint8_t i = 0; i < MBOX_NCHANNELS; i++) {
			const struct mbox_fifo_cfg *cfg;
			cfg = &dev_config->fifo.cfg[i][MBOX_FIFO_TX_SLOT];
			if (!cfg->env) {continue;}
			fifo_env_init(cfg->env, cfg->width, cfg->depth);
			cfg = &dev_config->fifo.cfg[i][MBOX_FIFO_RX_SLOT];
			__ASSERT_NO_MSG(cfg->env);
			fifo_env_init(cfg->env, cfg->width, cfg->depth);
		}
		return 0;
	}
}
#else
static int mbox_linkedsemi_init(const struct device *dev)
{
	return mbox_check(dev);
}
#endif

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

#if MBOX_TX_FIFO_FIRST
#define MBOX_CONSUMER_CHILD_REGION(node_id)                                                                    \
	uint8_t mbox_linkedsemi_fifo_tx_##node_id[(sizeof(struct fifo_env)                                        \
						+ (DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_width)               \
						* DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_depth)))] __aligned(4); \
	uint8_t mbox_linkedsemi_fifo_rx_##node_id[(sizeof(struct fifo_env)                                        \
						+ (DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_width)               \
						* DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_depth)))] __aligned(4);
#else
#define MBOX_CONSUMER_CHILD_REGION(node_id)                                                                    \
	uint8_t mbox_linkedsemi_fifo_rx_##node_id[(sizeof(struct fifo_env)                                        \
						+ (DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_width)               \
						* DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_depth)))] __aligned(4); \
	uint8_t mbox_linkedsemi_fifo_tx_##node_id[(sizeof(struct fifo_env)                                        \
						+ (DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_width)               \
						* DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_depth)))] __aligned(4);
#endif

#define MBOX_CONSUMER_CHILD_CFG(node_id) 							\
	[DT_PHA(node_id, mboxes, channel)] = {							\
		[MBOX_FIFO_TX_SLOT] = {								\
			.env = (struct fifo_env *)mbox_fifo.mbox_linkedsemi_fifo_tx_##node_id, \
			.width = DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_width),		\
			.depth = DT_PHA_BY_IDX(node_id, mboxes, 0, tx_fifo_depth),		\
		}, \
		[MBOX_FIFO_RX_SLOT] = {								\
			.env = (struct fifo_env *)mbox_fifo.mbox_linkedsemi_fifo_rx_##node_id, \
			.width = DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_width),		\
			.depth = DT_PHA_BY_IDX(node_id, mboxes, 0, rx_fifo_depth),		\
		}, \
	},

MBOX_LINKEDSEMI_IRQ_HANDLER(0)

typedef struct {
	DT_INST_FOREACH_CHILD_STATUS_OKAY(0, MBOX_CONSUMER_CHILD_REGION)
} mbox_fifo_t;
mbox_fifo_t mbox_fifo __attribute__((section("MBOX")));
BUILD_ASSERT(sizeof(mbox_fifo) + (uintptr_t)MBOX_BASE_ADDR <= (uintptr_t)MBOX_INFO_ADDR);

static struct mbox_linkedsemi_data mbox_linkedsemi_data_0;
static const struct mbox_linkedsemi_config mbox_linkedsemi_config_0 = {
	.irq_config_func = mbox_linkedsemi_config_func_0,
	.intr_clr = DT_INST_PROP(0, intr_clr),
	.intr_mask = DT_INST_PROP_OR(0, intr_mask, 0),
	.peer_intr_set = DT_INST_PROP(0, peer_intr_set),
	.fifo.cfg = {
		DT_INST_FOREACH_CHILD_STATUS_OKAY(0, MBOX_CONSUMER_CHILD_CFG)
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
