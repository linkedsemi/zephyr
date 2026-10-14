/*
 * Copyright (c) 2026 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PCC_H_
#define ZEPHYR_INCLUDE_DRIVERS_PCC_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One BIOS post code captured by the PCC.
 *
 * @a port is the I/O port the post code was written to, so a consumer can
 * tell apart the different post-code windows that are being snooped.
 */
struct pcc_data {
	uint8_t post_code;
	uint8_t port;
};

/**
 * @brief Callback invoked when a new post code is captured.
 *
 * @param data The post code that was just captured.
 */
typedef void (*pcc_callback_t)(struct pcc_data *data);

typedef int (*pcc_read_post_code_t)(const struct device *dev, struct pcc_data *buf,
				    size_t *length);

typedef int (*pcc_set_callback_t)(const struct device *dev, pcc_callback_t callback);

/**
 * @brief PCC driver API.
 */
__subsystem struct pcc_driver_api {
	pcc_read_post_code_t read_post_code;
	pcc_set_callback_t set_callback;
};

/**
 * @brief Read the post codes buffered by the PCC.
 *
 * The captured post codes are drained from the post-code FIFO in
 * first-in-first-out order.
 *
 * @param dev    PCC device instance.
 * @param buf    Buffer that receives the post codes.
 * @param length On input, the number of entries @p buf can hold; on output,
 *               the number of entries that were actually read.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p buf or @p length is NULL.
 * @retval negative errno on other failures.
 */
__syscall int pcc_read_post_code(const struct device *dev, struct pcc_data *buf,
				 size_t *length);

static inline int z_impl_pcc_read_post_code(const struct device *dev, struct pcc_data *buf,
					    size_t *length)
{
	const struct pcc_driver_api *api = (const struct pcc_driver_api *)dev->api;

	if (api->read_post_code == NULL) {
		return -ENOSYS;
	}
	return api->read_post_code(dev, buf, length);
}

/**
 * @brief Register the callback invoked for every captured post code.
 *
 * @param dev      PCC device instance.
 * @param callback Callback to invoke, or NULL to unregister the current one.
 *
 * @retval 0 on success.
 * @retval negative errno on failure.
 */
__syscall int pcc_set_callback(const struct device *dev, pcc_callback_t callback);

static inline int z_impl_pcc_set_callback(const struct device *dev, pcc_callback_t callback)
{
	const struct pcc_driver_api *api = (const struct pcc_driver_api *)dev->api;

	if (api->set_callback == NULL) {
		return -ENOSYS;
	}
	return api->set_callback(dev, callback);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/pcc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_PCC_H_ */
