/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(main);

static struct usbd_context *sample_usbd;
USBD_DEFINE_MSC_LUN(vm, "vmdisk", "Zephyr", "VmDisk", "0.01");

static int msc_mount(const struct shell *sh, size_t argc, char **argv)
{
	int err = usbd_enable(sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	return 0;
}
SHELL_CMD_REGISTER(msc_mount, NULL, "msc mount", msc_mount);

static int msc_unmount(const struct shell *sh, size_t argc, char **argv)
{
	usbd_disable(sample_usbd);
	return 0;
}
SHELL_CMD_REGISTER(msc_unmount, NULL, "msc unmount", msc_unmount);

static int enable_usb_device_next(void)
{
	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}
	LOG_DBG("USB device support enabled");

	return 0;
}

int main(void)
{
	if (enable_usb_device_next() != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	LOG_INF("The device is put in USB mass storage mode.\n");
	return 0;
}
