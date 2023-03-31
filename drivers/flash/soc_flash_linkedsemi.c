/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <string.h>

#include <ls_hal_flash.h>

#define DT_DRV_COMPAT        vnd_linkedsemi_flash_controller
#define SOC_NV_FLASH_NODE    DT_INST(0, soc_nv_flash)

#define FLASH_ADDR           DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_SIZE           DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ERASE_SIZE     DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_WRITE_SIZE     DT_PROP(SOC_NV_FLASH_NODE, write_block_size)

 struct flash_priv {
	struct k_sem mutex;
};

static const struct flash_parameters flash_linkedsemi_parameters = {
	.write_block_size = FLASH_WRITE_SIZE,
	.erase_value = 0xff,
};

static int flash_linkedsemi_init(const struct device *dev)
{
	struct flash_priv *priv = dev->data;

	k_sem_init(&priv->mutex, 1, 1);

	return 0;
}

static bool flash_linkedsemi_valid_range(off_t offset, size_t size)
{
    if ((offset < 0) || (size < 1)) {
		return false;
	}

    if ( offset > FLASH_SIZE || (offset + size) > FLASH_SIZE) {
		return false;
	}

	return true;
}

static int flash_linkedsemi_erase(const struct device *dev, off_t offset,
				     size_t size)
{
	struct flash_priv *priv = dev->data;
    
	if (!size) {
		return 0;
	}

	/* Offset and length should be multiple of erase size */
	if (((offset % FLASH_ERASE_SIZE) != 0) ||
	    ((size % FLASH_ERASE_SIZE) != 0)) {
		return -EINVAL;
	}

	if (!flash_linkedsemi_valid_range(offset, size)) {
		return -EINVAL;
	}

	if (k_sem_take(&priv->mutex, K_FOREVER)) {
		return -EACCES;
	}

	/* Erase sector one by one*/
    for (off_t addr = offset; addr < offset + size; addr += FLASH_ERASE_SIZE) {
        hal_flash_sector_erase(addr);
    }
    
	k_sem_give(&priv->mutex);

	return 0;
}

static int flash_linkedsemi_write(const struct device *dev, off_t offset,
				     const void *data, size_t size)
{
	struct flash_priv *priv = dev->data;
    size_t len = size;
    uint8_t *write_data = (uint8_t *)data;

	if (!size) {
		return 0;
	}

	if (!flash_linkedsemi_valid_range(offset, size)) {
		return -EINVAL;
	}

	if (k_sem_take(&priv->mutex, K_FOREVER)) {
		return -EACCES;
	}

    while (size) {
		/* If the offset isn't a multiple of the page size, we first need
		 * to write the remaining part that fits, otherwise the write could
		 * be wrapped around within the same page
		 */
		len = MIN(FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE), size);
        hal_flash_page_program(offset, write_data, len);

		write_data += len;
		offset += len;
		size -= len;
	}

	k_sem_give(&priv->mutex);

	return 0;
}

static int flash_linkedsemi_read(const struct device *dev, off_t offset,
				    void *data, size_t size)
{
	ARG_UNUSED(dev);

	if (!size) {
		return 0;
	}

	if (!flash_linkedsemi_valid_range(offset, size)) {
		return -EINVAL;
	}
    
    hal_flash_multi_io_read(offset, (uint8_t *)data, size);

	return 0;
}

static const struct flash_parameters *
flash_linkedsemi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_linkedsemi_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
    .pages_count = FLASH_SIZE / FLASH_ERASE_SIZE,
	.pages_size = FLASH_ERASE_SIZE,
};

static void flash_linkedsemi_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flash_linkedsemi_api = {
	.erase = flash_linkedsemi_erase,
	.write = flash_linkedsemi_write,
	.read = flash_linkedsemi_read,
	.get_parameters = flash_linkedsemi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_linkedsemi_layout,
#endif
};

static struct flash_priv flash_data;

DEVICE_DT_INST_DEFINE(0, flash_linkedsemi_init, NULL, &flash_data, NULL,
		      POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,
		      &flash_linkedsemi_api);