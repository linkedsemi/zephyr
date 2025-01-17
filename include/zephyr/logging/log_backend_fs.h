/******************************************************************************
 * Copyright 2024 Alibaba Group Corporation
 *
 * This software and the related documents are Alibaba copyrighted materials,
 * and your use of them is governed by an express license which must be granted
 * to you pursuant to a fully executed Agreement with Alibaba ("License").
 * Unless the License provides otherwise, you may not use, modify, copy,
 * publish, distribute, disclose or transmit this software or the related
 * documents without Alibaba's prior written permission.
 ******************************************************************************/

#ifndef ZEPHYR_LOG_BACKEND_FS_H_
#define ZEPHYR_LOG_BACKEND_FS_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern struct k_mutex fs_mutex;
int log_backend_fs_enable(void);
int log_backend_fs_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_LOG_BACKEND_FS_H_ */
