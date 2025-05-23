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
#include <cpu.h>

static K_SEM_DEFINE(g_mbox_data_rx_sem0, 0, 1);

static mbox_func_call_data_t g_mbox_received_data0;
static mbox_channel_id_t g_mbox_received_channel0;

static void callback0(const struct device *dev, mbox_channel_id_t channel_id, void *user_data, struct mbox_msg *data)
{
    // memcpy(&g_mbox_received_data0, data->data, data->size);
    memcpy(&g_mbox_received_data0, data->data, sizeof(mbox_func_call_data_t));
    g_mbox_received_channel0 = channel_id;

    k_sem_give(&g_mbox_data_rx_sem0);

    // printk("Server receive (on channel %d)\n", g_mbox_received_channel0);
}

__ramfunc void wait_for_done(void)
{
    *g_mbox_received_data0.ack = true;
    while(*g_mbox_received_data0.done == false);
}

int main(void)
{
    const struct mbox_dt_spec tx_channel0 = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer3), tx);
    const struct mbox_dt_spec rx_channel0 = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer3), rx);

    printk("mbox_data Server demo started\n");

    const int max_transfer_size_bytes = mbox_mtu_get_dt(&tx_channel0);
    /* Sample currently supports only transfer size up to 4 bytes */
    if ((max_transfer_size_bytes <= 0) || (max_transfer_size_bytes > 256)) {
        printk("mbox_mtu_get() error: %d\n", max_transfer_size_bytes);
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

        switch (g_mbox_received_data0.api_id) {
        case MBOX_FUNC_CALL_DO_IDLE:
            do {
                printk("done: %p\n", g_mbox_received_data0.done);
                printk("ack: %p\n", g_mbox_received_data0.ack);
                disable_global_irq();
                wait_for_done();
                enable_global_irq();
                printk("release\n");
            } while (0);
            break;
        default: break;
        }
    }

    printk("mbox_data Server demo ended.\n");
    return 0;
}
