/* SPDX-License-Identifier: Apache-2.0 */
#ifndef DBUS_BROKER_H
#define DBUS_BROKER_H

#include <zephyr/kernel.h>

/* Forward declarations */
struct deployment_state;
typedef struct Broker Broker;

/* sd-bus forward declaration */
typedef struct sd_bus sd_bus;

/* Global variables for broker communication */
extern int g_controller_fds[2];
extern struct deployment_state deploy_state;
extern Broker *g_broker;

/* Public API functions */

/**
 * @brief Connect to the D-Bus broker
 * @details This function creates a socket connection to the D-Bus broker
 *          and initializes an sd-bus object for communication
 * @param bus [out] Pointer to store the created sd-bus object
 * @param socket_fd [out] Optional pointer to store the socket file descriptor
 * @return 0 on success, negative error code on failure
 */
int connect_to_dbroker(sd_bus **bus, int *socket_fd);

/**
 * @brief Request a connection from the socketpool
 * @details This function allocates a socketpair from the pool and returns
 *          the client's fd. The broker will use the other end to create a peer.
 * @param client_fd [out] Pointer to store the client's fd
 * @return 0 on success, negative error code on failure
 */
int request_dbroker_connection(int *client_fd);

#endif /* DBUS_BROKER_H */