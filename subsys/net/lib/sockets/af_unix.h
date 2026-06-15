/*
 * Copyright (c) 2024 Alibaba Group
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AF_UNIX_INTERNAL_H_
#define _AF_UNIX_INTERNAL_H_

#include <zephyr/kernel.h>
#include <zephyr/net/net_ip.h>

/**
 * @brief Create an AF_UNIX SOCK_STREAM socket
 *
 * @param family Must be AF_UNIX
 * @param type   Must be SOCK_STREAM
 * @param proto  Must be 0
 *
 * @return File descriptor on success, -1 on error (errno set)
 */
int unix_socket_create(int family, int type, int proto);

#endif /* _AF_UNIX_INTERNAL_H_ */
