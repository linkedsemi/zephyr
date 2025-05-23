/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <zephyr/drivers/flash/soc_flash_ls_mbox_cpu1.h>
#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(mbox_data_func_call);

#include <hal_flash_int.h>
#include <ls_hal_flash.h>
#include <platform.h>

#define DT_DRV_COMPAT linkedsemi_mbox

#define MBOX_BASE_ADDRESS (DT_INST_REG_ADDR(0))
#define MBOX_SIZE         (DT_INST_REG_SIZE(0))
#define MBOX_NCHANNELS    (DT_INST_PROP(0, nchannels))
#define MBOX_FIFO_DEEPTH  (DT_INST_PROP(0, fifo_deepth))
#define MBOX_FIFO_WIDTH   (DT_INST_PROP(0, fifo_width))

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define FLASH_ADDR        DT_REG_ADDR(SOC_NV_FLASH_NODE)

#define FLASH_OPEN_AREA_ADDR FIXED_PARTITION_OFFSET(open_partition)
#define FLASH_OPEN_AREA_SIZE FIXED_PARTITION_SIZE(open_partition)

static K_SEM_DEFINE(g_mbox_data_rx_sem0, 0, 1);

// static mbox_func_call_data_t g_mbox_received_data0;
uint8_t g_mbox_received_data0[MBOX_FIFO_WIDTH];
static mbox_channel_id_t g_mbox_received_channel0;

static void callback0(const struct device *dev, mbox_channel_id_t channel_id, void *user_data, struct mbox_msg *data)
{
    // memcpy(&g_mbox_received_data0, data->data, data->size);
    memcpy(&g_mbox_received_data0, data->data, sizeof(mbox_func_call_data_t));
    g_mbox_received_channel0 = channel_id;

    k_sem_give(&g_mbox_data_rx_sem0);

    LOG_DBG("Server receive (on channel %d)\n", g_mbox_received_channel0);
}

static bool is_open_partition_area(uint32_t addr)
{
    return ((addr >= FLASH_OPEN_AREA_ADDR) && (addr < (FLASH_OPEN_AREA_ADDR + FLASH_OPEN_AREA_SIZE)));
}

int main(void)
{
    const struct device *const flash_dev = DEVICE_DT_GET(DT_NODELABEL(qspi1));
    const struct mbox_dt_spec tx_channel0 = MBOX_DT_SPEC_GET(DT_PHANDLE(DT_NODELABEL(qspi1), mbox), tx);
    const struct mbox_dt_spec rx_channel0 = MBOX_DT_SPEC_GET(DT_PHANDLE(DT_NODELABEL(qspi1), mbox), rx);

    LOG_DBG("mbox_data Server demo started\n");
    const int max_transfer_size_bytes = mbox_mtu_get_dt(&tx_channel0);
    /* Sample currently supports only transfer size up to 4 bytes */
    if ((max_transfer_size_bytes <= 0) || (max_transfer_size_bytes > 256)) {
        LOG_ERR("mbox_mtu_get() error\n");
        return 0;
    }

    if (mbox_register_callback_dt(&rx_channel0, callback0, NULL)) {
        LOG_ERR("mbox_register_callback() error\n");
        return 0;
    }

    if (mbox_set_enabled_dt(&rx_channel0, 1)) {
        LOG_ERR("mbox_set_enable() error\n");
        return 0;
    }

    while (1) {
        k_sem_take(&g_mbox_data_rx_sem0, K_FOREVER);

        LOG_DBG("Server receive (on channel %d)\n", g_mbox_received_channel0);

        mbox_func_call_data_t *mbox_received_data0 = (mbox_func_call_data_t *)g_mbox_received_data0;
        switch (mbox_received_data0->api_id) {
        case MBOX_FUNC_CALL_FLASH_READ_JEDEC_ID:
            LOG_DBG("MBOX_FUNC_CALL_FLASH_READ_JEDEC_ID\n");
            do {
                uint8_t **id = (uint8_t **)(((int *)(mbox_received_data0->parm))[0]);
                __ASSERT_NO_MSG(id);
                __ASSERT_NO_MSG(*id);
                flash_ls_mult_host(flash_dev, false);
                flash_read_jedec_id(flash_dev, *id);
                flash_ls_mult_host(flash_dev, true);

                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_FLASH_ERASE:
            LOG_DBG("MBOX_FUNC_CALL_FLASH_ERASE\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                off_t *size = (off_t *)(((int *)(mbox_received_data0->parm))[1]);
                LOG_DBG("offset: %#lx\n", *offset);
                __ASSERT_NO_MSG(offset);
                if (is_open_partition_area(*offset)) {
                    flash_ls_mult_host(flash_dev, false);
                    flash_erase(flash_dev, *offset, *size);
                    flash_ls_mult_host(flash_dev, true);
                } else {
                    LOG_DBG("offset: %#lx is invalid\n", *offset);
                }

                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_FLASH_WRITE:
            LOG_DBG("MBOX_FUNC_CALL_FLASH_WRITE\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                uint8_t **data = (uint8_t **)(((int *)(mbox_received_data0->parm))[1]);
                size_t *size = (size_t *)(((int *)(mbox_received_data0->parm))[2]);
                LOG_DBG("&offset: %p\n", offset);
                LOG_DBG("&size: %p\n", size);
                LOG_DBG("&data: %p\n", data);

                LOG_DBG("offset: %#lx\n", *offset);
                LOG_DBG("size: %#x\n", *size);
                LOG_DBG("data: %p\n", *data);
                __ASSERT_NO_MSG(offset);
                __ASSERT_NO_MSG(data);
                __ASSERT_NO_MSG(*data);
                __ASSERT_NO_MSG(size);
                if (is_open_partition_area(*offset)) {
                    flash_ls_mult_host(flash_dev, false);
                    LOG_DBG("offset: %#lx\n", *offset);
                    flash_write(flash_dev, *offset, *data, *size);
                    flash_ls_mult_host(flash_dev, true);
                } else {
                    LOG_DBG("offset: %#lx is invalid\n", *offset);
                }

                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_FLASH_READ:
            LOG_DBG("MBOX_FUNC_CALL_FLASH_READ\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                uint8_t **data = (uint8_t **)(((int *)(mbox_received_data0->parm))[1]);
                size_t *size = (size_t *)(((int *)(mbox_received_data0->parm))[2]);
                LOG_DBG("&offset: %p\n", offset);
                LOG_DBG("&size: %p\n", size);
                LOG_DBG("&data: %p\n", data);

                LOG_DBG("offset: %#lx\n", *offset);
                LOG_DBG("size: %#x\n", *size);
                LOG_DBG("data: %p\n", *data);
                __ASSERT_NO_MSG(offset);
                __ASSERT_NO_MSG(data);
                __ASSERT_NO_MSG(*data);
                __ASSERT_NO_MSG(size);
                if (is_open_partition_area(*offset)) {
                    flash_ls_mult_host(flash_dev, false);
                    LOG_DBG("offset: %#lx\n", *offset);
                    flash_read(flash_dev, *offset, *data, *size);
                    flash_ls_mult_host(flash_dev, true);
                } else {
                    LOG_DBG("offset: %#lx is invalid\n", *offset);
                }
                LOG_DBG("rd- - - - - -------------------\n");
                for (int i = 0; i < *size; i++) {
                    if ((i % 16 == 0) && (i != 0)) {
                        LOG_DBG("\n");
                    }
                    LOG_DBG("%2.2x ", ((char *)(*data))[i]);
                }
                LOG_DBG("\n");

                *mbox_received_data0->done = true;
            } while (0);
            break;
        default:
            LOG_DBG("default\n");
            break;
        }
    }

    LOG_DBG("mbox_data Server demo ended.\n");

    return 0;
}
