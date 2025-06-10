/*
 * Copyright (c) 2025 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <zephyr/drivers/flash/soc_flash_ls_mbox_cpu1.h>
#include <platform.h>
#include <cpu.h>

#include <ls_hal_flash.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif

#define DT_DRV_COMPAT     linkedsemi_mbox_cpu1_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_ADDR       DT_REG_ADDR(SOC_NV_FLASH_NODE)
#define FLASH_SIZE       DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ERASE_SIZE DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)
#define FLASH_WRITE_SIZE DT_PROP(SOC_NV_FLASH_NODE, write_block_size)

struct flash_ls_data {
    struct k_sem mutex;
    bool is_proxy;
};

struct flash_ls_config {
    const struct mbox_dt_spec tx_channel;
    const struct mbox_dt_spec rx_channel;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
};

static const struct flash_parameters flash_ls_parameters = {
    .write_block_size = FLASH_WRITE_SIZE,
    .erase_value = 0xff,
};

int flash_ls_set_proxy_state(const struct device *dev, bool flag)
{
    __unused struct flash_ls_data *dev_data = dev->data;

    dev_data->is_proxy = flag;

    return 0;
}

static bool flash_ls_get_proxy_state(const struct device *dev)
{
    __unused struct flash_ls_data *dev_data = dev->data;

    return dev_data->is_proxy;
}

static bool is_own_ram(uint32_t addr)
{
    return ((addr >= DT_REG_ADDR(DT_CHOSEN(zephyr_sram)))
            && (addr < (DT_REG_ADDR(DT_CHOSEN(zephyr_sram)) + DT_REG_SIZE(DT_CHOSEN(zephyr_sram)))));
}

static int flash_ls_init(const struct device *dev)
{
    __unused struct flash_ls_data *dev_data = dev->data;
    __unused const struct flash_ls_config *dev_config = dev->config;

    k_sem_init(&dev_data->mutex, 1, 1);

    mbox_func_call_trx_register(&dev_config->tx_channel, &dev_config->rx_channel);

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
    bool xip_present = is_cpu2_xip() & is_cpu2_running();
    bool need_interact = flash_ls_get_proxy_state(dev) || xip_present;
    if (need_interact) {
        int ret_mbox = mbox_acquire_cpu2_idle(&dev_config->tx_channel);
        if (ret_mbox) {
            goto interact_fail;
        }
    }

    for (off_t addr = offset; addr < offset + size; addr += FLASH_ERASE_SIZE) {
        hal_flash_sector_erase(addr);
    }

    if (need_interact) {
        mbox_release_cpu2();
    }

interact_fail:
    flash_ls_set_proxy_state(dev, false);

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

    bool xip_present = is_cpu2_xip() & is_cpu2_running();
    bool need_interact = flash_ls_get_proxy_state(dev) || xip_present;
    if (need_interact) {
        int ret_mbox = mbox_acquire_cpu2_idle(&dev_config->tx_channel);
        if (ret_mbox) {
            goto interact_fail;
        }
    }

    while (size) {
        /* If the offset isn't a multiple of the page size, we first need
        * to write the remaining part that fits, otherwise the write could
        * be wrapped around within the same page
        */
        len = MIN(FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE), size);
        if (need_interact) {
            if (is_own_ram((uint32_t)write_data)) {
                sys_cache_data_invd_range((void *)write_data, len);
            }
        }
        hal_flash_page_program(offset, write_data, len);

        write_data += len;
        offset += len;
        size -= len;
    }

    if (need_interact) {
        mbox_release_cpu2();
    }

interact_fail:
    flash_ls_set_proxy_state(dev, false);

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

    if (k_sem_take(&dev_data->mutex, K_FOREVER)) {
        return -EACCES;
    }

    bool xip_present = is_cpu2_xip() & is_cpu2_running();
    bool need_interact = flash_ls_get_proxy_state(dev) || xip_present;
    if (need_interact) {
        int ret_mbox = mbox_acquire_cpu2_idle(&dev_config->tx_channel);
        if (ret_mbox) {
            goto interact_fail;
        }
    }

    hal_flash_multi_io_read(offset, (uint8_t *)data, size);
    if (need_interact) {
        if (is_own_ram((uint32_t)data)) {
            sys_cache_data_flush_range((void *)data, size);
        }
    } else {
        mbox_release_cpu2();
    }

interact_fail:
    flash_ls_set_proxy_state(dev, false);

    k_sem_give(&dev_data->mutex);

    return 0;
}

static const struct flash_parameters *
flash_ls_get_parameters(const struct device *dev)
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

    if (k_sem_take(&dev_data->mutex, K_FOREVER)) {
        return -EACCES;
    }

    bool xip_present = is_cpu2_xip() & is_cpu2_running();
    bool need_interact = flash_ls_get_proxy_state(dev) || xip_present;
    if (need_interact) {
        int ret_mbox = mbox_acquire_cpu2_idle(&dev_config->tx_channel);
        if (ret_mbox) {
            goto interact_fail;
        }
    }

    hal_flash_read_id(id);

    if (need_interact) {
        if (is_own_ram((uint32_t)id)) {
            sys_cache_data_flush_range((void *)id, 3);
        }
        mbox_release_cpu2();
    }

interact_fail:
    flash_ls_set_proxy_state(dev, false);

    k_sem_give(&dev_data->mutex);

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

IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(0)));
static const struct flash_ls_config flash_ls_config_0 = {
    .tx_channel = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(0, mbox), tx),
    .rx_channel = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(0, mbox), rx),
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0), ))
};

static struct flash_ls_data flash_ls_data_0 = {
    .is_proxy = false,
};

DEVICE_DT_INST_DEFINE(0,
                      flash_ls_init,
                      NULL,
                      &flash_ls_data_0,
                      &flash_ls_config_0,
                      POST_KERNEL,
                      CONFIG_FLASH_INIT_PRIORITY,
                      &flash_ls_api);
