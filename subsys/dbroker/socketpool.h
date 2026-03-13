
/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __SOCKETPOOL_H__
#define __SOCKETPOOL_H__

#include <zephyr/kernel.h>

/**
 * @brief Initialize the socketpool
 * 
 * @return 0 on success, negative errno on failure
 */
int socketpool_init(void);

/**
 * @brief Allocate a socketpair from the pool
 * 
 * @param broker_fd Pointer to store the broker's fd
 * @param client_fd Pointer to store the client's fd
 * @return 0 on success, negative errno on failure
 */
int socketpool_allocate(int *broker_fd, int *client_fd);

/**
 * @brief Free a socketpair back to the pool
 * 
 * @param broker_fd The broker's fd
 * @param client_fd The client's fd
 * @return 0 on success, negative errno on failure
 */
int socketpool_free(int broker_fd, int client_fd);

/**
 * @brief Get socketpool statistics
 * 
 * @param total Pointer to store total number of socketpairs
 * @param available Pointer to store available number of socketpairs
 */
void socketpool_get_stats(int *total, int *available);

/**
 * @brief Add a peer to the broker using a socketpair from the pool
 * @details This function is called when a client requests a connection
 * @param broker Pointer to the broker instance
 * @param broker_fd The broker's fd from the socketpair
 * @return 0 on success, negative error code on failure
 */
int socketpool_add_peer_to_broker(Broker *broker, int broker_fd);

#endif /* __SOCKETPOOL_H__ */
