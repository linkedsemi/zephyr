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

static K_SEM_DEFINE(g_mbox_data_rx_sem, 0, 1);

#define MAX_BUF_SIZE 256

static uint8_t g_mbox_received_data[MAX_BUF_SIZE] = {0};
static mbox_channel_id_t g_mbox_received_channel;
static uint8_t g_message[MAX_BUF_SIZE] = {0};

static void callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
		     struct mbox_msg *data)
{
	memcpy(g_mbox_received_data, data->data, data->size);
	g_mbox_received_channel = channel_id;

	printk("Server receive (on channel %d)\n", g_mbox_received_channel);
	for(uint16_t i = 0; i < data->size; i++) {
		printk("%d  ", g_mbox_received_data[i]);
	}
	printk("\n");

	k_sem_give(&g_mbox_data_rx_sem);
}

int main(void)
{
	const struct mbox_dt_spec tx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), tx);
	const struct mbox_dt_spec rx_channel = MBOX_DT_SPEC_GET(DT_PATH(mbox_consumer), rx);
	struct mbox_msg msg = {0};

	printk("mbox_data Server demo started\n");

	const int max_transfer_size_bytes = mbox_mtu_get_dt(&tx_channel);
	/* Sample currently supports only transfer size up to MAX_BUF_SIZE bytes */
	if ((max_transfer_size_bytes <= 0) || (max_transfer_size_bytes > MAX_BUF_SIZE)) {
		printk("mbox_mtu_get() error\n");
		return 0;
	}

	if (mbox_register_callback_dt(&rx_channel, callback, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}

	if (mbox_set_enabled_dt(&rx_channel, 1)) {
		printk("mbox_set_enable() error\n");
		return 0;
	}

	while (g_message[0] < 99) {
		k_sem_take(&g_mbox_data_rx_sem, K_FOREVER);
		memcpy(g_message, g_mbox_received_data, max_transfer_size_bytes);

		for(uint16_t i = 0; i < max_transfer_size_bytes; i++) {
			g_message[i]++;
		}

		int ret = 0;
		do {
			msg.data = g_message;
			msg.size = max_transfer_size_bytes;

			printk("Server send (on channel %d)\n", tx_channel.channel_id);
			for(uint16_t i = 0; i < max_transfer_size_bytes; i++) {
				printk("%d  ", g_message[i]);
			}
			printk("\n");
			ret = mbox_send_dt(&tx_channel, &msg);
			if (ret == -EBUSY) {
				printk("mbox_send() busy\n");
				k_msleep(1);
				continue;
			}
			if (ret) {
				printk("mbox_send() error\n");
				return 0;
			}
			for(uint16_t i = 0; i < max_transfer_size_bytes; i++) {
				g_message[i]++;
			}
		} while ((ret != -EBUSY) && (g_message[0] < 99));
	}

	printk("mbox_data Server demo ended.\n");
	return 0;
}
