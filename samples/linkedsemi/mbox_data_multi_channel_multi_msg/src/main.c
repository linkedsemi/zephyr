/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>

static K_SEM_DEFINE(g_mbox_data_rx_sem0, 0, 1);
static K_SEM_DEFINE(g_mbox_data_rx_sem1, 0, 1);

static mbox_channel_id_t g_mbox_received_data0;
static mbox_channel_id_t g_mbox_received_channel0;
static mbox_channel_id_t g_mbox_received_data1;
static mbox_channel_id_t g_mbox_received_channel1;

static void callback0(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
		     struct mbox_msg *data)
{
	memcpy(&g_mbox_received_data0, data->data, data->size);
	g_mbox_received_channel0 = channel_id;

	k_sem_give(&g_mbox_data_rx_sem0);

	printk("Client received (on channel %d) value: %d\n", g_mbox_received_channel0, g_mbox_received_data0);
}

static void callback1(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
		     struct mbox_msg *data)
{
	memcpy(&g_mbox_received_data1, data->data, data->size);
	g_mbox_received_channel1 = channel_id;

	k_sem_give(&g_mbox_data_rx_sem1);

	printk("Client received (on channel %d) value: %d\n", g_mbox_received_channel1, g_mbox_received_data1);
}

int main(void)
{
	const struct mbox_dt_spec tx_channel0 = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer2), tx);
	const struct mbox_dt_spec rx_channel0 = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer2), rx);
	const struct mbox_dt_spec tx_channel1 = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer3), tx);
	const struct mbox_dt_spec rx_channel1 = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer3), rx);
	struct mbox_msg msg = {0};
	uint32_t message = 0;

	printk("mbox_data Client demo started\n");

	const int max_transfer_size_bytes = mbox_mtu_get_dt(&tx_channel0);
	/* Sample currently supports only transfer size up to 4 bytes */
	if ((max_transfer_size_bytes < 0) || (max_transfer_size_bytes > 4)) {
		printk("mbox_mtu_get() error\n");
		return 0;
	}

	if (mbox_register_callback_dt(&rx_channel0, callback0, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}
	if (mbox_register_callback_dt(&rx_channel1, callback1, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}

	if (mbox_set_enabled_dt(&rx_channel0, 1)) {
		printk("mbox_set_enable() error\n");
		return 0;
	}

	if (mbox_set_enabled_dt(&rx_channel1, 1)) {
		printk("mbox_set_enable() error\n");
		return 0;
	}

	while (message < 100) {
		msg.data = &message;
		msg.size = max_transfer_size_bytes;
		while (1) {
			printk("Client send (on channel %d) value: %d\n", tx_channel0.channel_id, message);
			if (mbox_send_dt(&tx_channel0, &msg) == -ENOSPC) {
				printk("mbox_send() full\n");
				break;
			}

			printk("Client send (on channel %d) value: %d\n", tx_channel1.channel_id, message);
			if (mbox_send_dt(&tx_channel1, &msg) == -ENOSPC) {
				printk("mbox_send() full\n");
				break;
			}
		}

		k_sem_take(&g_mbox_data_rx_sem0, K_FOREVER);
		message = g_mbox_received_data0;

		// printk("Client received (on channel %d) value: %d\n", g_mbox_received_channel0, message);

		k_sem_take(&g_mbox_data_rx_sem1, K_FOREVER);
		message = g_mbox_received_data1;

		// printk("Client received (on channel %d) value: %d\n", g_mbox_received_channel1, message);
		message++;
	}

	printk("mbox_data Client demo ended\n");
	return 0;
}
