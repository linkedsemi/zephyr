/* SPDX-License-Identifier: Apache-2.0 */
#ifndef DBUS_BROKER_H
#define DBUS_BROKER_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

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
 * @return 0 on success, negative error code on failure
 */
int connect_to_dbroker(sd_bus **bus);

/**
 * @brief Disconnect from the D-Bus broker
 * @details This function integrates the sd-bus flush, close and unref to reliably
 *          disconnect from the broker. And for socketpair pool solution, it will
 *          release the socketpair and return the socketpair to the pool.
 * @param bus Pointer to the in-use sd-bus object
 * @return 0 on success, negative error code on failure
 */
int disconnect_from_dbroker(sd_bus *bus);

#ifdef __cplusplus
}
#endif

#endif /* DBUS_BROKER_H */
