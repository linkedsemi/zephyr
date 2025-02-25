/*
 * Copyright (c) 2025 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/drivers/mbox.h>
#include <cpu.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>

#include <ls_hal_flash.h>

#define DT_DRV_COMPAT     linkedsemi_mbox_cpu1_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_ADDR       DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_SIZE       DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ERASE_SIZE DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_WRITE_SIZE DT_PROP(SOC_NV_FLASH_NODE, write_block_size)

struct flash_ls_data {
    struct k_sem mutex;
};

struct flash_ls_config {
    const struct mbox_dt_spec tx_channel;
};

static const struct flash_parameters flash_ls_parameters = {
    .write_block_size = FLASH_WRITE_SIZE,
    .erase_value = 0xff,
};

static int flash_ls_init(const struct device *dev)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;

    k_sem_init(&dev_data->mutex, 1, 1);

    return 0;
}

static bool flash_ls_valid_range(off_t offset, size_t size)
{
    if ((offset < 0) || (size < 1)) {
        return false;
    }

    if (offset > FLASH_SIZE || (offset + size) > FLASH_SIZE) {
        return false;
    }

    return true;
}

static int flash_ls_erase(const struct device *dev, off_t offset, size_t size)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;

    if (!size) {
        return 0;
    }

    /* Offset and length should be multiple of erase size */
    if (((offset % FLASH_ERASE_SIZE) != 0) || ((size % FLASH_ERASE_SIZE) != 0)) {
        return -EINVAL;
    }

    if (!flash_ls_valid_range(offset, size)) {
        return -EINVAL;
    }

    if (k_sem_take(&dev_data->mutex, K_FOREVER)) {
        return -EACCES;
    }

    /* Erase sector one by one*/
    for (off_t addr = offset; addr < offset + size; addr += FLASH_ERASE_SIZE) {
        mbox_func_call(&dev_config->tx_channel, MBOX_FUNC_CALL_HAL_FLASH_SECTOR_ERASE, 1, (void *)&addr);
    }

    k_sem_give(&dev_data->mutex);

    return 0;
}

static int flash_ls_write(const struct device *dev, off_t offset, const void *data, size_t size)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;
    size_t len = size;
    uint8_t *write_data = (uint8_t *)data;

    if (!size) {
        return 0;
    }

    if (!flash_ls_valid_range(offset, size)) {
        return -EINVAL;
    }

    if (k_sem_take(&dev_data->mutex, K_FOREVER)) {
        return -EACCES;
    }

    while (size) {
        /* If the offset isn't a multiple of the page size, we first need
         * to write the remaining part that fits, otherwise the write could
         * be wrapped around within the same page
         */
        len = MIN(FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE), size);
        mbox_func_call(&dev_config->tx_channel,
                       MBOX_FUNC_CALL_HAL_FLASH_PAGE_PROGRAM,
                       3,
                       (void *)&offset,
                       (void *)&write_data,
                       (void *)&len);

        write_data += len;
        offset += len;
        size -= len;
    }

    k_sem_give(&dev_data->mutex);

    return 0;
}

static int flash_ls_read(const struct device *dev, off_t offset, void *data, size_t size)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;

    if (!size) {
        return 0;
    }

    if (!flash_ls_valid_range(offset, size)) {
        return -EINVAL;
    }

    bool xip_present = is_cpu1_xip();
    if (xip_present) {
        memcpy(data, (void *)(FLASH_ADDR + offset), size);
    } else {
        mbox_func_call(&dev_config->tx_channel,
                    MBOX_FUNC_CALL_HAL_FLASH_MULTI_IO_READ,
                    3,
                    (void *)&offset,
                    (void *)&data,
                    (void *)&size);
    }

    return 0;
}

static const struct flash_parameters *flash_ls_get_parameters(const struct device *dev)
{
    ARG_UNUSED(dev);

    return &flash_ls_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
    .pages_count = FLASH_SIZE / FLASH_ERASE_SIZE,
    .pages_size = FLASH_ERASE_SIZE,
};

static void flash_ls_layout(const struct device *dev,
                            const struct flash_pages_layout **layout,
                            size_t *layout_size)
{
    *layout = &dev_layout;
    *layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_JESD216_API)
static int flash_ls_read_jedec_id(const struct device *dev,
                                  uint8_t *id)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;

    if (id == NULL) {
        return -EINVAL;
    }

    mbox_func_call(&dev_config->tx_channel, MBOX_FUNC_CALL_HAL_FLASH_READ_ID, 1, (void *)&id);

    return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

static const struct flash_driver_api flash_ls_api = {
    .erase = flash_ls_erase,
    .write = flash_ls_write,
    .read = flash_ls_read,
    .get_parameters = flash_ls_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
    .page_layout = flash_ls_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
    .read_jedec_id = flash_ls_read_jedec_id,
#endif
};

static const struct flash_ls_config flash_ls_config_0 = {
    .tx_channel = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(0, mbox), tx),
};

static struct flash_ls_data flash_ls_data_0;

DEVICE_DT_INST_DEFINE(0,
                      flash_ls_init,
                      NULL,
                      &flash_ls_data_0,
                      &flash_ls_config_0,
                      POST_KERNEL,
                      CONFIG_FLASH_INIT_PRIORITY,
                      &flash_ls_api);
