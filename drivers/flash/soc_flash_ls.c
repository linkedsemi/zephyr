/*
 * Copyright (c) 2023 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT        linkedsemi_ls_flash_controller

#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/drivers/flash/soc_flash_ls.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "platform.h"
#include "soc_common.h"
#if defined(CONFIG_FLASH_OP_DELEGATION_SERVER)
#include <zephyr/drivers/mbox.h>
#endif

LOG_MODULE_REGISTER(linkedsemi_ls_flash_controller, CONFIG_FLASH_LOG_LEVEL);

struct flash_partition_attr {
	uint32_t base;
	uint32_t end;
	uint32_t size;
	uint32_t attr;
};

struct flash_ls_config {
	#if defined(CONFIG_FLASH_OP_DELEGATION_SERVER)
	const struct mbox_dt_spec mbox_tx;
	const struct mbox_dt_spec mbox_rx;
	#endif
	struct flash_parameters params;
	void *reg;
	uint32_t mem_base;
	uint32_t mem_size;
	struct flash_pages_layout layout;
    bool dual_mode_only;
    bool continuous_mode_enable;
	bool addr4b;
	struct flash_partition_attr *attr;
	uint8_t attr_num;
};


#if defined(CONFIG_FLASH_OP_DELEGATION_SERVER)
static void flash_delegation_server_operation_sync(const struct device *dev)
{
	struct flash_ls_data *priv = dev->data;
	const struct flash_ls_config *cfg = dev->config;
	struct delegate_s2c_params param = {
		.server = priv,
		.op = FLASH_DELEGATE_CLIENT_HOLD,
	};
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};

	if ((!is_app_cpu_running()) || (!IS_ENABLED(CONFIG_CPU2_XIP))) {
		return;
	}

	mbox_send_dt(&cfg->mbox_tx,&msg);
	k_sem_take(&priv->delegate_sem,K_FOREVER);
}

int flash_ls_get_part_info(const struct device *dev, uint8_t idx, uint32_t *base,
				   uint32_t *size, uint32_t *attr)
{
	const struct flash_ls_config *cfg = dev->config;
	if (idx >= cfg->attr_num) {
		return -EINVAL;
	}

	*base = cfg->attr[idx].base;
	*size = cfg->attr[idx].size;
	*attr = cfg->attr[idx].attr;

	return 0;
}

int flash_ls_get_part_num(const struct device *dev)
{
	const struct flash_ls_config *cfg = dev->config;

	return cfg->attr_num;
}

int flash_ls_set_part_info(const struct device *dev, uint8_t idx, uint32_t base,
				   uint32_t size, uint32_t attr)
{
	const struct flash_ls_config *cfg = dev->config;
	if (idx >= cfg->attr_num) {
		return -EINVAL;
	}

	cfg->attr[idx].base = base;
	cfg->attr[idx].size = size;
	cfg->attr[idx].end = cfg->attr[idx].base + cfg->attr[idx].size;
	cfg->attr[idx].attr = attr;

	return 0;
}

int flash_ls_set_part_attr(const struct device *dev, uint8_t idx, uint32_t attr)
{
	const struct flash_ls_config *cfg = dev->config;
	if (idx >= cfg->attr_num) {
		return -EINVAL;
	}

	cfg->attr[idx].attr = attr;

	return 0;
}

static uint32_t get_guest_permission(const struct device *dev, uint32_t base, uint32_t size)
{
	const struct flash_ls_config *cfg = dev->config;
	uint32_t end = base + size;
	uint32_t attr = 0;

	for (uint8_t start_idx = 0; start_idx < cfg->attr_num; start_idx++) {
		if ((base >= cfg->attr[start_idx].base) && (base < cfg->attr[start_idx].end)) {
			attr = cfg->attr[start_idx].attr;
			for (uint8_t end_idx = start_idx; end_idx < cfg->attr_num; end_idx++) {
				if (start_idx != end_idx) {
					attr &= cfg->attr[end_idx].attr;
				}
				if ((end >= cfg->attr[end_idx].base) && (end <= cfg->attr[end_idx].end)) {
					return attr;
				}
			}
		}
	}

	return 0;
}

static bool is_own_ram(uint32_t addr)
{
    return ((addr >= DT_REG_ADDR(DT_CHOSEN(zephyr_sram)))
            && (addr < (DT_REG_ADDR(DT_CHOSEN(zephyr_sram)) + DT_REG_SIZE(DT_CHOSEN(zephyr_sram)))));
}

static bool is_psram(uint32_t addr)
{
    return ((addr >= DT_REG_ADDR(DT_NODELABEL(psram)))
            && (addr < (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram)))));
}

static void delegation_server_work_handler(struct k_work *work)
{
	struct flash_ls_data *priv = CONTAINER_OF(work,struct flash_ls_data,worker);
	struct delegate_s2c_params param;
	const struct flash_ls_config *cfg = priv->dev->config;
	param.server = priv;
	param.op = FLASH_DELEGATE_CLIENT_OP_RETURN;
	switch(priv->req_param.op)
	{
	case FLASH_DELEGATE_SERVER_READ:
		if ((get_guest_permission(priv->dev,priv->req_param.offset,priv->req_param.size) & PMP_R)
			&& (!is_own_ram((uint32_t)priv->req_param.data))) {
			int data_addr = (uint32_t)priv->req_param.data;
			int data_size = (uint32_t)priv->req_param.size;
			param.ret.value = flash_read(priv->dev,priv->req_param.offset,priv->req_param.data,priv->req_param.size);
			if (is_psram(data_addr)) {
				sys_cache_data_flush_range((void *)data_addr, data_size);
			}
		} else {
			param.ret.value = -EINVAL;
		}
	break;
	case FLASH_DELEGATE_SERVER_WRITE:
		if ((get_guest_permission(priv->dev,priv->req_param.offset,priv->req_param.size) & PMP_W)
			&& (!is_own_ram((uint32_t)priv->req_param.data))) {
			if (is_psram((uint32_t)priv->req_param.data)) {
    			sys_cache_data_invd_range((void *)priv->req_param.data, priv->req_param.size);
			}
			param.ret.value = flash_write(priv->dev,priv->req_param.offset,priv->req_param.data,priv->req_param.size);
		} else {
			param.ret.value = -EINVAL;
		}
	break;
	case FLASH_DELEGATE_SERVER_ERASE:
		if (get_guest_permission(priv->dev, priv->req_param.offset,priv->req_param.size) & PMP_W) {
			param.ret.value = flash_erase(priv->dev,priv->req_param.offset,priv->req_param.size);
		} else {
			param.ret.value = -EINVAL;
		}
	break;
	case FLASH_DELEGATE_SERVER_GET_PARAMS:
	{
		const struct flash_parameters *flash_params = flash_get_parameters(priv->dev);
		memcpy(&param.ret.flash_params,flash_params,sizeof(struct flash_parameters));
	}break;
	case FLASH_DELEGATE_SERVER_READ_JEDEC_ID:
		if (!is_own_ram((uint32_t)priv->req_param.data)) {
			int data_addr = (uint32_t)priv->req_param.data;
			int data_size = (uint32_t)priv->req_param.size;
			param.ret.value = flash_read_jedec_id(priv->dev,priv->req_param.data);
			if (is_psram(data_addr)) {
				sys_cache_data_flush_range((void *)data_addr, data_size);
			}
		} else {
			param.ret.value = -EINVAL;
		}
	break;
	case FLASH_DELEGATE_SERVER_SFDP_READ:
		if (!is_own_ram((uint32_t)priv->req_param.data)) {
			int data_addr = (uint32_t)priv->req_param.data;
			int data_size = (uint32_t)priv->req_param.size;
			param.ret.value = flash_sfdp_read(priv->dev,priv->req_param.offset,priv->req_param.data,priv->req_param.size);
			if (is_psram(data_addr)) {
				sys_cache_data_flush_range((void *)data_addr, data_size);
			}
		} else {
			param.ret.value = -EINVAL;
		}
	break;
	default:
		LOG_ERR("delegation_server_work_handler opcode error");
	break;
	}
	struct mbox_msg msg = {
		.data = &param,
		.size = sizeof(param),
	};
	mbox_send_dt(&cfg->mbox_tx,&msg);
}

static void delegation_server_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
	const struct delegate_c2s_params *req = data->data;
	struct flash_ls_data *priv = dev->data;
	const struct flash_ls_config *cfg = dev->config;
	if(cfg->reg != req->reg)
	{
		return;
	}
	priv->req_param = *req;
	switch(priv->req_param.op)
	{
	case FLASH_DELEGATE_SERVER_SUSPEND:
		priv->requested_suspending = true;
		while(priv->requested_suspending);
	break;
	case FLASH_DELEGATE_SERVER_HOLD_ACK:
		k_sem_give(&priv->delegate_sem);
	break;
	default:
		k_work_submit(&priv->worker);
	break;
	}
}

#define DELEGATION_SERVER_MBOX_CALLBACK(idx)\
	delegation_server_mbox_handler(DEVICE_DT_INST_GET(idx),data);


static void delegation_server_mbox_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)
{
	DT_INST_FOREACH_STATUS_OKAY(DELEGATION_SERVER_MBOX_CALLBACK);
}
#else
static void flash_delegation_server_operation_sync(const struct device *dev){}
#endif

static int flash_ls_init(const struct device *dev)
{
	struct flash_ls_data *priv = dev->data;
	const struct flash_ls_config *cfg = dev->config;
	priv->env.reg = cfg->reg;
	priv->env.dual_mode_only = cfg->dual_mode_only;
	priv->env.continuous_mode_enable = cfg->continuous_mode_enable;
	priv->env.continuous_mode_on = cfg->continuous_mode_enable;
	priv->env.addr4b = cfg->addr4b;
	priv->env.writing = false;
	k_sem_init(&priv->sem, 1, 1);
	#ifdef CONFIG_FLASH_OP_DELEGATION_SERVER
	k_sem_init(&priv->delegate_sem,0,1);
	k_work_init(&priv->worker,delegation_server_work_handler);
	mbox_set_enabled_dt(&cfg->mbox_tx,true);
	mbox_register_callback_dt(&cfg->mbox_rx,delegation_server_mbox_callback,NULL);
	mbox_set_enabled_dt(&cfg->mbox_rx,true);
	priv->dev = dev;
	#endif
    IRQ_CONNECT(FLASH_SWINT_NUM, CONFIG_FLASH_SWINT_PRIORITY, SWINT_Handler_ASM, NULL, 0);
	irq_enable(FLASH_SWINT_NUM);
	return 0;
}

static int flash_ls_erase(const struct device *dev, off_t offset,
				     size_t size)
{
	struct flash_ls_data *priv = dev->data;
    
	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}

	flash_delegation_server_operation_sync(dev);
	/* Erase sector one by one*/
    for (off_t addr = offset; addr < offset + size; addr += FLASH_SECTOR_SIZE) {
        hal_flashx_sector_erase(&priv->env,addr);
    }
    
	k_sem_give(&priv->sem);

	return 0;
}

static int flash_ls_write(const struct device *dev, off_t offset,
				     const void *data, size_t size)
{
	struct flash_ls_data *priv = dev->data;
    size_t len = size;
    uint8_t *write_data = (uint8_t *)data;

	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}

	flash_delegation_server_operation_sync(dev);
    while (size) {
		/* If the offset isn't a multiple of the page size, we first need
		 * to write the remaining part that fits, otherwise the write could
		 * be wrapped around within the same page
		 */
		len = MIN(FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE), size);
        hal_flashx_page_program(&priv->env,offset, write_data, len);

		write_data += len;
		offset += len;
		size -= len;
	}

	k_sem_give(&priv->sem);

	return 0;
}

static int flash_ls_read(const struct device *dev, off_t offset,
				    void *data, size_t size)
{
	struct flash_ls_data *priv = dev->data;

	if (!size) {
		return 0;
	}

	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}

	flash_delegation_server_operation_sync(dev);
    hal_flashx_multi_io_read(&priv->env,offset, (uint8_t *)data, size);

	k_sem_give(&priv->sem);

	return 0;
}

static const struct flash_parameters *
flash_ls_get_parameters(const struct device *dev)
{
	const struct flash_ls_config *cfg = dev->config;
	return &cfg->params;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_ls_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	const struct flash_ls_config *cfg = dev->config;
	*layout = &cfg->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_ls_read_jedec_id(const struct device *dev,
								uint8_t *id)
{
	struct flash_ls_data *priv = dev->data;

	if (id == NULL) {
		return -EINVAL;
	}
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}

	flash_delegation_server_operation_sync(dev);
	hal_flashx_read_id(&priv->env,id);
	k_sem_give(&priv->sem);

	return 0;
}

static int flash_ls_sfdp_read(const struct device *dev, off_t offset,
				   void *data, size_t len)
{
	struct flash_ls_data *priv = dev->data;
	if (k_sem_take(&priv->sem, K_FOREVER)) {
		return -EACCES;
	}

	flash_delegation_server_operation_sync(dev);
	hal_flashx_read_sfdp(&priv->env,offset,data,len);
	k_sem_give(&priv->sem);

	return 0;
}

#endif /* CONFIG_FLASH_JESD216_API */
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
__ramfunc static int flash_ls_ex_op(const struct device *dev, uint16_t code,
			  const uintptr_t in, void *out)
{
	struct flash_ls_data *priv = dev->data;
	switch(code)
	{
	case FLASH_DRIVER_SUSPEND_OPCODE:
		hal_flashx_prog_erase_suspend_isr(&priv->env);
	break;
	case FLASH_DRIVER_RESUME_OPCODE:
		hal_flashx_prog_erase_resume_isr(&priv->env);
	break;
	}
	return 0;
}
#endif

static struct flash_driver_api flash_ls_api = {
	.erase = flash_ls_erase,
	.write = flash_ls_write,
	.read = flash_ls_read,
	.get_parameters = flash_ls_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_ls_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.read_jedec_id = flash_ls_read_jedec_id,
	.sfdp_read = flash_ls_sfdp_read,
#endif
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_ls_ex_op,
#endif
};

#define LS_PARTITION_CHILD(node_id)\
			{\
				.base = DT_REG_ADDR(node_id),\
				.size = DT_REG_SIZE(node_id),\
				.end = (DT_REG_ADDR(node_id) + DT_REG_SIZE(node_id)),\
				.attr = DT_PROP_OR(node_id, attr, 0),\
			},\

#define LS_FLASH_CONTROLLER_CHILD(node_id)\
		IF_ENABLED(DT_NODE_HAS_COMPAT(node_id,soc_nv_flash),(\
		.params = {\
			.write_block_size = DT_PROP(node_id,write_block_size),\
			.caps.no_explicit_erase = false,\
			.erase_value = 0xff,\
		},\
		.mem_base = (uint32_t)DT_REG_ADDR(node_id),\
		.mem_size = DT_REG_SIZE(node_id),\
		.layout = {\
			.pages_count = DT_REG_SIZE(node_id)/FLASH_SECTOR_SIZE,\
			.pages_size = FLASH_SECTOR_SIZE,\
		},))

#define LS_FLASH_INIT(idx) \
	struct flash_partition_attr attr_partition_##idx[] =\
		{DT_FOREACH_CHILD(DT_INST(idx, fixed_partitions), LS_PARTITION_CHILD)};\
	static const struct flash_ls_config flash_ls_cfg_##idx = {\
		.reg = (void *)DT_INST_REG_ADDR(idx),\
		.dual_mode_only = !DT_INST_PROP(idx,quad),\
		.continuous_mode_enable = DT_INST_PROP(idx,continuous_mode),\
		.addr4b = DT_INST_PROP(idx,addr4b),\
		IF_ENABLED(CONFIG_FLASH_OP_DELEGATION_SERVER,(\
		.mbox_tx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), tx),\
		.mbox_rx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), rx),\
		))\
		DT_INST_FOREACH_CHILD(idx,LS_FLASH_CONTROLLER_CHILD)\
		.attr = attr_partition_##idx,\
		.attr_num = DT_CHILD_NUM(DT_INST(idx, fixed_partitions)),\
	};\
	IF_ENABLED(CONFIG_FLASH_OP_DELEGATION_SERVER, (__attribute__((section("SHMEM"))))) \
	static struct flash_ls_data flash_ls_data_##idx;\
	DEVICE_DT_INST_DEFINE(idx,flash_ls_init,NULL,\
		&flash_ls_data_##idx,&flash_ls_cfg_##idx,POST_KERNEL,\
		CONFIG_FLASH_INIT_PRIORITY,&flash_ls_api);

DT_INST_FOREACH_STATUS_OKAY(LS_FLASH_INIT)