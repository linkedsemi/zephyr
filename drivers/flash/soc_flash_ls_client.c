#define DT_DRV_COMPAT		linkedsemi_ls_flash_delegation_client

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "platform.h"
#include "soc.h"
#include <zephyr/cache.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(linkedsemi_ls_flash_delegation_client, CONFIG_FLASH_LOG_LEVEL);

struct flash_ls_client_config {
	void *reg;
	const struct mbox_dt_spec mbox_tx;
	const struct mbox_dt_spec mbox_rx;
	struct flash_pages_layout layout;
};

struct flash_ls_client_data {
	struct k_sem sem;
	struct k_sem op_return_sem;
	struct flash_ls_shared_data *shared;
	struct flash_op_return ret;
	uint8_t suspend_count;
};

__ramfunc static bool client_polling(void *param)
{
	struct flash_ls_client_data *priv = param;
	return !priv->shared->busy;
}

__ramfunc static void client_hold_ack_and_polling(struct flash_ls_client_data *priv)
{
	priv->shared->hold_ack = true;
	busy_poll(client_polling,priv,CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT*1000);
}

static void delegation_client_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
	const struct delegate_s2c_params *req = data->data;
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if(cfg->reg != req->shared->reg)
	{
		return;
	}
	memcpy((void *)&priv->ret,&req->ret,sizeof(req->ret));
	switch(req->op)
	{
	case FLASH_DELEGATE_CLIENT_HOLD:
	{
		priv->shared = req->shared;
		client_hold_ack_and_polling(priv);
		priv->shared = NULL;
	}break;
	case FLASH_DELEGATE_CLIENT_OP_RETURN:
		k_sem_give(&priv->op_return_sem);
	break;
	default:
		__ASSERT(0,"delegation_client_mbox_handler invalid op");
	break;
	}
}

#define DELEGATION_CLIENT_MBOX_CALLBACK(idx)\
	delegation_client_mbox_handler(DEVICE_DT_INST_GET(idx),data);

static void delegation_client_mbox_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)
{
	DT_INST_FOREACH_STATUS_OKAY(DELEGATION_CLIENT_MBOX_CALLBACK);
}

static int flash_ls_client_init(const struct device *dev)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	k_sem_init(&priv->sem,1,1);
	k_sem_init(&priv->op_return_sem,0,1);
	mbox_set_enabled_dt(&cfg->mbox_tx,true);
	mbox_register_callback_dt(&cfg->mbox_rx,delegation_client_mbox_callback,NULL);
	mbox_set_enabled_dt(&cfg->mbox_rx,true);
	return 0;
}

static int flash_ls_client_erase(const struct device *dev, off_t offset,
					 size_t size)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.offset = offset,
		.size = size,
		.op = FLASH_DELEGATE_SERVER_ERASE
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static int flash_ls_client_write_op(const struct device *dev, off_t offset,
					 const void *data, size_t size, bool flag_align)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.offset = offset,
		.data = (void *)data,
		.size = size,
		.op = flag_align ? FLASH_DELEGATE_SERVER_WRITE_ALIGN : FLASH_DELEGATE_SERVER_WRITE,
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static int flash_ls_client_read_op(const struct device *dev, off_t offset,
					void *data, size_t size, bool flag_align)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.offset = offset,
		.data = data,
		.size = size,
		.op = flag_align ? FLASH_DELEGATE_SERVER_READ_ALIGN : FLASH_DELEGATE_SERVER_READ,
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

#if 0
static void flash_op_align_debug(struct flash_xfer_buf *buf)
{
	for (uint32_t i = 0; i < FLASH_XFER_BUF_IDX_MAX; i++) {
		if (buf[i].len > 0) {
			printf("idx:%d offset:0x%08lx size:0x%08x data:0x%08lx\n",i,buf[i].offset,buf[i].len,(uintptr_t)buf[i].buf);
#if 0
			int data_addr = (uint32_t)buf[i].buf;
			int data_size = buf[i].len;
			off_t offset = buf[i].offset;
			for (int idx = 0; idx < data_size; idx += 16) {
				for (int jdx = 0; jdx < 16; jdx++) {
					printf("%2.2x ", ((uint8_t *)data_addr)[idx + jdx]);
				}
				printf("\n");
			}
#endif
		}
	}
}
#endif

static int flash_op_align(const struct device *dev, off_t offset, void *data, size_t len, bool is_write)
{
	uint8_t buf_align_head[CONFIG_DCACHE_LINE_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE);
	uint8_t buf_align_tail[CONFIG_DCACHE_LINE_SIZE] __aligned(CONFIG_DCACHE_LINE_SIZE);
	flash_op_align_buf_t flash_op_align_buf __aligned(CONFIG_DCACHE_LINE_SIZE);
	struct flash_xfer_buf *buf = flash_op_align_buf.buf;
	uint8_t *user_ptr = (uint8_t *)data;
	size_t remain = len;
	off_t current_offset = offset;
	int ret;

	buf[FLASH_XFER_BUF_IDX_HEAD].len = 0;
	buf[FLASH_XFER_BUF_IDX_MIDDLE].len = 0;
	buf[FLASH_XFER_BUF_IDX_TAIL].len = 0;

	/* Handle misaligned start portion */
	if (((uintptr_t)user_ptr % CONFIG_DCACHE_LINE_SIZE) != 0) {
		size_t chunk;
		if (len > CONFIG_DCACHE_LINE_SIZE) {
			size_t misaligned = CONFIG_DCACHE_LINE_SIZE - ((uintptr_t)user_ptr % CONFIG_DCACHE_LINE_SIZE);
			chunk = (remain < misaligned) ? remain : misaligned;
		} else {
			chunk = remain;
		}

		buf[FLASH_XFER_BUF_IDX_HEAD].offset = offset;
		buf[FLASH_XFER_BUF_IDX_HEAD].buf = (void *)buf_align_head;
		buf[FLASH_XFER_BUF_IDX_HEAD].len = chunk;
		if (is_write) {
			memcpy(buf_align_head, user_ptr, chunk);
			sys_cache_data_flush_range((void *)buf_align_head, sizeof(buf_align_head));
		} else {
			sys_cache_data_invd_range((void *)buf_align_head, sizeof(buf_align_head));
		}

		user_ptr += chunk;
		current_offset += chunk;
		remain -= chunk;
	}

	/* Handle aligned middle portion */
	size_t aligned_len = remain & (~(CONFIG_DCACHE_LINE_SIZE - 1));
	if (aligned_len > 0) {
		if (is_write) {
			sys_cache_data_flush_range((void *)user_ptr, aligned_len);
		} else {
			sys_cache_data_invd_range((void *)user_ptr, aligned_len);
		}

		buf[FLASH_XFER_BUF_IDX_MIDDLE].offset = offset + ((uintptr_t)user_ptr - (uintptr_t)data);
		buf[FLASH_XFER_BUF_IDX_MIDDLE].buf = (void *)user_ptr;
		buf[FLASH_XFER_BUF_IDX_MIDDLE].len = aligned_len;

		user_ptr += aligned_len;
		current_offset += aligned_len;
		remain -= aligned_len;
	}

	/* Handle remain unaligned end portion */
	if (remain > 0) {
		buf[FLASH_XFER_BUF_IDX_TAIL].offset = offset + ((uintptr_t)user_ptr - (uintptr_t)data);
		buf[FLASH_XFER_BUF_IDX_TAIL].buf = (void *)buf_align_tail;
		buf[FLASH_XFER_BUF_IDX_TAIL].len = remain;
		if (is_write) {
			memcpy(buf_align_tail, user_ptr, remain);
			sys_cache_data_flush_range((void *)buf_align_tail, sizeof(buf_align_tail));
		} else {
			sys_cache_data_invd_range((void *)buf_align_tail, sizeof(buf_align_tail));
		}
	}

#if 0
	printf("%s\n", is_write ? "w" : "r");
	flash_op_align_debug(buf);
#endif

	sys_cache_data_flush_range((void *)&flash_op_align_buf, sizeof(flash_op_align_buf));
	if (is_write) {
		ret = flash_ls_client_write_op(dev, current_offset, buf, (size_t)-1, true);
	} else {
		ret = flash_ls_client_read_op(dev, current_offset, buf, (size_t)-1, true);
	}
	if (ret != 0) {
		printf("flash_ls_client_%s_op align failed ret:%d\n", is_write ? "write" : "read", ret);
		return ret;
	}

	if (!is_write) {
		if (buf[FLASH_XFER_BUF_IDX_HEAD].len > 0) {
			sys_cache_data_invd_range(buf_align_head, sizeof(buf_align_head));
			memcpy(data, buf_align_head, buf[FLASH_XFER_BUF_IDX_HEAD].len);
		}
		if (buf[FLASH_XFER_BUF_IDX_TAIL].len > 0) {
			sys_cache_data_invd_range(buf_align_tail, sizeof(buf_align_tail));
			void *user_ptr_tail = (void *)((uintptr_t)data + (len - buf[FLASH_XFER_BUF_IDX_TAIL].len));
			memcpy(user_ptr_tail, buf_align_tail, buf[FLASH_XFER_BUF_IDX_TAIL].len);
		}
	}

	return 0;
}

static int flash_ls_client_read_align(const struct device *dev, off_t offset, void *data, size_t len)
{
	return flash_op_align(dev, offset, data, len, false);
}

static int flash_ls_client_write_align(const struct device *dev, off_t offset, const void *data, size_t len)
{
	/* Note: const qualifier removed safely for internal operation */
	return flash_op_align(dev, offset, (void *)data, len, true);
}

uint8_t flash_ls_client_read_ear(const struct device *dev)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.op = FLASH_DELEGATE_SERVER_READ_EAR
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static const struct flash_parameters *
flash_ls_client_get_parameters(const struct device *dev)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return NULL;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.op = FLASH_DELEGATE_SERVER_GET_PARAMS
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	const struct flash_parameters *ret;
	if(k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT)))
	{
		ret = NULL;
	}else
	{
		ret = &priv->ret.flash_params;
	}
	k_sem_give(&priv->sem);
	return ret;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_ls_client_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	const struct flash_ls_client_config *cfg = dev->config;
	*layout = &cfg->layout;	
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_ls_client_read_jedec_id(const struct device *dev, uint8_t *id)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	__aligned(CONFIG_DCACHE_LINE_SIZE) uint8_t buf_align[CONFIG_DCACHE_LINE_SIZE];

	if (id == NULL) {
		return -EINVAL;
	}
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.data = buf_align,
		.op = FLASH_DELEGATE_SERVER_READ_JEDEC_ID
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	sys_cache_data_invd_range((void *)buf_align, sizeof(buf_align));
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	if (0 == ret) {
		sys_cache_data_invd_range((void *)buf_align, sizeof(buf_align));
		memcpy(id, buf_align, 3);
	}

	return ret;
}

static int flash_ls_client_sfdp_read(const struct device *dev, off_t offset,
					void *data, size_t len)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
	struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.offset = offset,
		.data = data,
		.size = len,
		.op = FLASH_DELEGATE_SERVER_SFDP_READ
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
	int ret = k_sem_take(&priv->op_return_sem,K_MSEC(CONFIG_FLASH_DELEGATION_SYNC_TIMEOUT));
	if(!ret) ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static int flash_ls_client_sfdp_read_align(const struct device *dev, off_t offset,
					void *data, size_t len)
{
	int ret = 0;

	if ((0 == ((uintptr_t)data % CONFIG_DCACHE_LINE_SIZE)) && (0 == (len % CONFIG_DCACHE_LINE_SIZE))) {
		sys_cache_data_invd_range((void *)data, len);
		ret = flash_ls_client_sfdp_read(dev,offset,data,len);
		sys_cache_data_invd_range((void *)data, len);
	} else {
		__aligned(CONFIG_DCACHE_LINE_SIZE) uint8_t buf_align[CONFIG_DCACHE_LINE_SIZE];
		size_t read_len = 0;
		size_t remain_len = len;
		uint8_t *data_ptr = (uint8_t *)data;
		while (remain_len) {
			read_len = MIN(sizeof(buf_align), remain_len);
			sys_cache_data_invd_range((void *)buf_align, sizeof(buf_align));
			ret = flash_ls_client_sfdp_read(dev,offset,buf_align,read_len);
			if (0 == ret) {
				sys_cache_data_invd_range((void *)buf_align, sizeof(buf_align));
				memcpy(data_ptr, buf_align, read_len);
			}
			data_ptr += read_len;
			offset += read_len;
			remain_len -= read_len;
		}
	}

	return ret;
}
#endif /* CONFIG_FLASH_JESD216_API */

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
#if defined(CONFIG_FLASH_DELEGATION_CLIENT_SUSPEND_REQUEST)

__ramfunc static bool poll_suspend_request_true(void *param)
{
	struct flash_ls_client_data *priv = param;
	return priv->shared->suspend_request;
}

__ramfunc int flash_ls_ex_op(const struct device *dev, uint16_t code,
				const uintptr_t in, void *out)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	switch(code)
	{
	case FLASH_DRIVER_SUSPEND_OPCODE:
		if(priv->shared)
		{
			if(priv->suspend_count==0)
			{
				struct delegate_c2s_params param = {
					.reg = cfg->reg,
					.op = FLASH_DELEGATE_SERVER_SUSPEND
				};
				struct mbox_msg msg = {
					.data = &param,
					.size = sizeof(param),
				};
				mbox_send_dt(&cfg->mbox_tx,&msg);
				busy_poll(poll_suspend_request_true,priv,CONFIG_FLASH_DELEGATION_SUSPEND_TIMEOUT);
			}
			priv->suspend_count++;
		}
	break;
	case FLASH_DRIVER_RESUME_OPCODE:
		if(priv->shared)
		{
			priv->suspend_count--;
			if(priv->suspend_count==0)
			{
				priv->shared->suspend_request = false;
			}
		}
	break;
	}
	return 0;
}
#else
__ramfunc int flash_ls_ex_op(const struct device *dev, uint16_t code,
				const uintptr_t in, void *out)
{
	return 0;
}
#endif
#endif

static struct flash_driver_api flash_ls_client_api = {
	.erase = flash_ls_client_erase,
	.write = flash_ls_client_write_align,
	.read = flash_ls_client_read_align,
	.get_parameters = flash_ls_client_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_ls_client_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = flash_ls_client_read_jedec_id,
	.sfdp_read = flash_ls_client_sfdp_read_align,
#endif
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_ls_ex_op,
#endif
};

#define LS_FLASH_CONTROLLER_CLIENT_CHILD(node_id)\
		IF_ENABLED(DT_NODE_HAS_COMPAT(node_id,soc_nv_flash),(\
		.layout = {\
			.pages_count = DT_REG_SIZE(node_id)/FLASH_SECTOR_SIZE,\
			.pages_size = FLASH_SECTOR_SIZE,\
		},))

#define LS_FLASH_CLIENT_INIT(idx)\
	static const struct flash_ls_client_config flash_ls_client_cfg_##idx ={\
		.reg = (void *)DT_INST_REG_ADDR(idx),\
		.mbox_tx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), tx),\
		.mbox_rx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), rx),\
		DT_INST_FOREACH_CHILD(idx,LS_FLASH_CONTROLLER_CLIENT_CHILD)\
	};\
	static struct flash_ls_client_data flash_ls_client_data_##idx;\
	DEVICE_DT_INST_DEFINE(idx,flash_ls_client_init,NULL,\
		&flash_ls_client_data_##idx,&flash_ls_client_cfg_##idx,POST_KERNEL,\
		CONFIG_FLASH_INIT_PRIORITY,&flash_ls_client_api);

DT_INST_FOREACH_STATUS_OKAY(LS_FLASH_CLIENT_INIT)