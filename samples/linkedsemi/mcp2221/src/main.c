/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>
#include "hid_bridge.h"
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/uart.h>
#include <usb_device_init.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#undef HID_USAGE_PAGE
#undef HID_USAGE
#undef HID_COLLECTION
#undef HID_END_COLLECTION
#undef HID_REPORT_SIZE
#undef HID_REPORT_COUNT
#undef HID_INPUT
#undef HID_OUTPUT

#define HID_USAGE_PAGE(page)       0x06, (page) & 0xFF, ((page) >> 8) & 0xFF
#define HID_USAGE(usage)           0x09, (usage)
#define HID_COLLECTION(type)       0xA1, (type)
#define HID_END_COLLECTION         0xC0
#define HID_USAGE_MIN(min)         0x19, (min)
#define HID_USAGE_MAX(max)         0x29, (max)
#define HID_LOGICAL_MIN(min)       0x15, (min)
#define HID_LOGICAL_MAX(max)       0x25, (max)
#define HID_REPORT_SIZE(size)      0x75, (size)
#define HID_REPORT_COUNT(count)    0x95, (count)
#define HID_INPUT(flags)           0x81, (flags)
#define HID_OUTPUT(flags)          0x91, (flags)

#define HID_COLLECTION_APPLICATION 0x01

static const uint8_t hid_report_desc[] = {
    HID_USAGE_PAGE(0xFF00),
    HID_USAGE(0x01),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
    // Input Report
    HID_USAGE_MIN(0x01),
    HID_USAGE_MAX(0x40),
    HID_LOGICAL_MIN(0x01),
    HID_LOGICAL_MAX(0x40),
    HID_REPORT_SIZE(8),
    HID_REPORT_COUNT(64),
    HID_INPUT(0x00),
    // Output Report
    HID_USAGE_MIN(0x01),
    HID_USAGE_MAX(0x40),
    HID_OUTPUT(0x00),
    HID_END_COLLECTION
};

static void usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg);

static int enable_usb_device_next(void)
{
	struct usbd_context *sample_usbd;
	int err;

	sample_usbd = usbd_init_device(usbd_msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	err = usbd_enable(sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}

static struct hid_bridge hid;

static void int_in_ok_cb(const struct device *dev)
{
}

static int hid_ep_write(struct hid_bridge *hid, void *data, uint8_t length)
{
	return hid_int_ep_write(hid->user_data, data, length, NULL);
}

static void int_out_ready_cb(const struct device *dev, const uint8_t *buf, const uint16_t len)
{
	hid_bridge_raw_event(&hid, buf);
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ok_cb,
	.int_out_ready = int_out_ready_cb,
};


#define RING_BUF_SIZE 1024
static uint8_t ring_buffer[RING_BUF_SIZE];
static struct ring_buf ringbuf;

static inline void print_baudrate(const struct device *dev)
{
	uint32_t baudrate;
	int ret;

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		LOG_INF("Baudrate %u", baudrate);
	}
}

static void usbd_msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	LOG_INF("USBD message: %s", usbd_msg_type_string(msg->type));

	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}

	if (msg->type == USBD_MSG_CDC_ACM_LINE_CODING) {
		print_baudrate(msg->dev);
	}
}

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	static bool rx_throttled;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (!rx_throttled && uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t buffer[64];
			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(buffer));

			if (len == 0) {
				/* Throttle because ring buffer is full */
				uart_irq_rx_disable(dev);
				rx_throttled = true;
				continue;
			}

			recv_len = uart_fifo_read(dev, buffer, len);
			if (recv_len < 0) {
				LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			rb_len = ring_buf_put(&ringbuf, buffer, recv_len);
			if (rb_len < recv_len) {
				LOG_ERR("Drop %u bytes", recv_len - rb_len);
			}

			LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			if (rb_len) {
				uart_irq_tx_enable(dev);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[64];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			if (rx_throttled) {
				uart_irq_rx_enable(dev);
				rx_throttled = false;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}

int main(void)
{
	int ret;
	const struct device *hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	const struct device *uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

	if (hid_dev == NULL || uart_dev == NULL) {
		LOG_ERR("Cannot get Device");
		return 0;
	}

	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				&ops);

	usb_hid_init(hid_dev);
	hid_bridge_init(&hid, hid_ep_write, (void *)hid_dev);

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	ret = enable_usb_device_next();
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}
	uart_irq_callback_set(uart_dev, interrupt_handler);
	uart_irq_rx_enable(uart_dev);

	while (true) {
		k_msleep(1000);
	}
	return 0;
}
