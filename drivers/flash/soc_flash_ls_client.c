#define DT_DRV_COMPAT        linkedsemi_ls_flash_delegation_client

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "platform.h"
#include "soc_common.h"
#include <zephyr/cache.h>

struct flash_ls_client_config {
    void *reg;
	const struct mbox_dt_spec mbox_tx;
	const struct mbox_dt_spec mbox_rx;
	struct flash_pages_layout layout;
};

struct flash_ls_client_data {
    struct k_sem sem;
    struct k_sem op_return_sem;
    struct flash_ls_data *server;
	struct flash_op_return ret;
    uint8_t suspend_count;
};

__ramfunc static void client_polling(struct flash_ls_client_data *priv)
{
    while(k_sem_count_get(&priv->server->sem)==0);
}

static void delegation_client_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
	const struct delegate_s2c_params *req = data->data;
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	if(cfg->reg != req->server->env.reg)
	{
		return;
	}
	memcpy((void *)&priv->ret,&req->ret,sizeof(req->ret));
	switch(req->op)
	{
	case FLASH_DELEGATE_CLIENT_HOLD:
	{
        priv->server = req->server;
		struct delegate_c2s_params param = {
			.reg = cfg->reg,
			.op = FLASH_DELEGATE_SERVER_HOLD_ACK,
		};
		struct mbox_msg msg = {
			.data = &param,
			.size = sizeof(param),
		};
		mbox_send_dt(&cfg->mbox_tx,&msg);
		client_polling(priv);
        priv->server = NULL;
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
    k_sem_take(&priv->op_return_sem,K_FOREVER);
	int ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static int flash_ls_client_write(const struct device *dev, off_t offset,
				     const void *data, size_t size)
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
		.op = FLASH_DELEGATE_SERVER_WRITE
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
    sys_cache_data_flush_range((void *)data, size);
	mbox_send_dt(&cfg->mbox_tx,&msg);
    k_sem_take(&priv->op_return_sem,K_FOREVER);
	int ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

static int flash_ls_client_read(const struct device *dev, off_t offset,
				    void *data, size_t size)
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
		.op = FLASH_DELEGATE_SERVER_READ
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
    sys_cache_data_invd_range((void *)data, size);
	mbox_send_dt(&cfg->mbox_tx,&msg);
    k_sem_take(&priv->op_return_sem,K_FOREVER);
	int ret = priv->ret.value;
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
	k_sem_take(&priv->op_return_sem,K_FOREVER);
	const struct flash_parameters *ret = &priv->ret.flash_params;
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
static int flash_ls_client_read_jedec_id(const struct device *dev,
								uint8_t *id)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;

	if (id == NULL) {
		return -EINVAL;
	}
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}
    struct delegate_c2s_params param = {
		.reg = cfg->reg,
		.data = id,
		.op = FLASH_DELEGATE_SERVER_READ_JEDEC_ID
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
    sys_cache_data_invd_range((void *)id, 3);
	mbox_send_dt(&cfg->mbox_tx,&msg);
	k_sem_take(&priv->op_return_sem,K_FOREVER);
	int ret = priv->ret.value;
	k_sem_give(&priv->sem);
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
    sys_cache_data_invd_range((void *)data, len);
	mbox_send_dt(&cfg->mbox_tx,&msg);
    k_sem_take(&priv->op_return_sem,K_FOREVER);
	int ret = priv->ret.value;
	k_sem_give(&priv->sem);
	return ret;
}

#endif /* CONFIG_FLASH_JESD216_API */

#if defined(CONFIG_FLASH_EX_OP_ENABLED)
__ramfunc static int flash_ls_client_ex_op(const struct device *dev, uint16_t code,
			  const uintptr_t in, void *out)
{
	struct flash_ls_client_data *priv = dev->data;
	const struct flash_ls_client_config *cfg = dev->config;
	switch(code)
	{
	case FLASH_DRIVER_SUSPEND_OPCODE:
        if(priv->server)
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
                while(priv->server->requested_suspending==false);
            }
            priv->suspend_count++;
        }
	break;
	case FLASH_DRIVER_RESUME_OPCODE:
        if(priv->server)
        {
            priv->suspend_count--;
            if(priv->suspend_count==0)
            {
                priv->server->requested_suspending = false;
            }
        }
	break;
	}
	return 0;
}
#endif

static struct flash_driver_api flash_ls_client_api = {
	.erase = flash_ls_client_erase,
	.write = flash_ls_client_write,
	.read = flash_ls_client_read,
	.get_parameters = flash_ls_client_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_ls_client_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = flash_ls_client_read_jedec_id,
	.sfdp_read = flash_ls_client_sfdp_read,
#endif
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_ls_client_ex_op,
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