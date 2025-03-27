/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <ls_hal_flash.h>

#include <platform.h>

#define DT_DRV_COMPAT linkedsemi_mbox

#define MBOX_BASE_ADDRESS  (DT_INST_REG_ADDR(0))
#define MBOX_SIZE          (DT_INST_REG_SIZE(0))
#define MBOX_NCHANNELS     (DT_INST_PROP(0, nchannels))
#define MBOX_FIFO_DEEPTH   (DT_INST_PROP(0, fifo_deepth))
#define MBOX_FIFO_WIDTH    (DT_INST_PROP(0, fifo_width))

#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)
#define FLASH_ADDR        DT_REG_ADDR(SOC_NV_FLASH_NODE)

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

    // printk("Server receive (on channel %d)\n", g_mbox_received_channel0);
}

bool is_cpu2_flash_area(uint32_t addr)
{
    return addr >= (CONFIG_CPU2_BOOT_ADDR - FLASH_ADDR);
}

int main(void)
{
    const struct mbox_dt_spec tx_channel0 = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer2), tx);
    const struct mbox_dt_spec rx_channel0 = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer2), rx);

    printk("mbox_data Server demo started\n");
    const int max_transfer_size_bytes = mbox_mtu_get_dt(&tx_channel0);
    /* Sample currently supports only transfer size up to 4 bytes */
    if ((max_transfer_size_bytes <= 0) || (max_transfer_size_bytes > 256)) {
        printk("mbox_mtu_get() error\n");
        return 0;
    }

    if (mbox_register_callback_dt(&rx_channel0, callback0, NULL)) {
        printk("mbox_register_callback() error\n");
        return 0;
    }

    if (mbox_set_enabled_dt(&rx_channel0, 1)) {
        printk("mbox_set_enable() error\n");
        return 0;
    }

    while (1) {
        k_sem_take(&g_mbox_data_rx_sem0, K_FOREVER);

        // printk("Server receive (on channel %d)\n", g_mbox_received_channel0);

        mbox_func_call_data_t *mbox_received_data0 = (mbox_func_call_data_t *)g_mbox_received_data0;
        switch (mbox_received_data0->api_id) {
        case MBOX_FUNC_CALL_HAL_FLASH_READ_ID:
            // printk("MBOX_FUNC_CALL_HAL_FLASH_READ_ID\n");
            do {
                uint8_t **id = (uint8_t **)(((int *)(mbox_received_data0->parm))[0]);
                __ASSERT_NO_MSG(id);
                hal_flash_read_id(*id);
                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_HAL_FLASH_SECTOR_ERASE:
            // printk("MBOX_FUNC_CALL_HAL_FLASH_SECTOR_ERASE\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                // printk("offset: %#x\n", *offset);
                __ASSERT_NO_MSG(offset);
                if (is_cpu2_flash_area(*offset)) {
                    hal_flash_sector_erase(*offset);
                } else {
                    printk("offset: %#x is invalid\n", *offset);
                }
                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_HAL_FLASH_PAGE_PROGRAM:
            // printk("MBOX_FUNC_CALL_HAL_FLASH_PAGE_PROGRAM\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                uint8_t **data = (uint8_t **)(((int *)(mbox_received_data0->parm))[1]);
                size_t *size = (size_t *)(((int *)(mbox_received_data0->parm))[2]);
                // printk("offset: %#x\n", *offset);
                // printk("size: %#x\n", *size);
                // printk("data: %#x\n", *data);
                // printk("&offset: %#x\n", offset);
                // printk("&size: %#x\n",   size);
                // printk("&data: %#x\n",   data);
                __ASSERT_NO_MSG(offset);
                __ASSERT_NO_MSG(data);
                __ASSERT_NO_MSG(size);
                if (is_cpu2_flash_area(*offset)) {
                    hal_flash_page_program(*offset, *data, *size);
                } else {
                    printk("offset: %#x is invalid\n", *offset);
                }
                *mbox_received_data0->done = true;
            } while (0);
            break;
        case MBOX_FUNC_CALL_HAL_FLASH_MULTI_IO_READ:
            // printk("MBOX_FUNC_CALL_HAL_FLASH_MULTI_IO_READ\n");
            do {
                off_t *offset = (off_t *)(((int *)(mbox_received_data0->parm))[0]);
                uint8_t **data = (uint8_t **)(((int *)(mbox_received_data0->parm))[1]);
                size_t *size = (size_t *)(((int *)(mbox_received_data0->parm))[2]);
                // printk("offset: %#x\n", *offset);
                // printk("size: %#x\n", *size);
                // printk("data: %#x\n", *data);
                // printk("&offset: %#x\n", offset);
                // printk("&size: %#x\n",   size);
                // printk("&data: %#x\n",   data);
                __ASSERT_NO_MSG(offset);
                __ASSERT_NO_MSG(data);
                __ASSERT_NO_MSG(size);
                if (is_cpu2_flash_area(*offset)) {
                    hal_flash_multi_io_read(*offset, *data, *size);
                } else {
                    printk("offset: %#x is invalid\n", *offset);
                }
                // printk("rd- - - - - -------------------\n");
                // for(int i = 0; i < *size; i++) {
                //     if ((i % 16 == 0) && (i != 0)) {
                //         printk("\n");
                //     }
                //     printk("%2.2x ", ((char *)(*data))[i]);
                // }
                // printk("\n");
                *mbox_received_data0->done = true;
            } while (0);
            break;
        default: 
            printk("default\n");
            break;
        }
    }

    printk("mbox_data Server demo ended.\n");
    return 0;
}
