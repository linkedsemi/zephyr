/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <sys/socket.h>
// #include <sys/un.h>
#include <stdio.h>
#include <soc.h>
#include <errno.h>

/* dbus-broker headers */
// #include <broker/broker.h>
#include "../../../modules/lib/dbus-broker/src/broker/broker.h"
#include "../../../modules/lib/basu/include/systemd/sd-bus.h"
#include "../../../modules/lib/dbus-broker/src/util/string.h"

/* D-Bus broker subsystem */
#include "dbus_broker.h"

LOG_MODULE_REGISTER(DBROKER, LOG_LEVEL_DBG);

/* INADDR_LOOPBACK is not defined in Zephyr */
#ifndef INADDR_LOOPBACK
#define INADDR_LOOPBACK 0x7f000001
#endif

/*
 * Global variables for broker communication
 */
int g_controller_fds[2] = { -1, -1 };
struct deployment_state {
    struct k_mutex lock;
    bool broker_ready;
    int listener_fd;
    bool service_provider_ready;
};
struct deployment_state deploy_state = {
    .lock = Z_MUTEX_INITIALIZER(deploy_state.lock),
    .broker_ready = false,
    .listener_fd = -1,
    .service_provider_ready = false
};
extern Broker *g_broker;

static int create_listener_socket_with_retry(void)
{
    int fd = -1;
    int retries = 50;
    int delay_ms = 100;

    LOG_INF("=== CREATING LISTENER SOCKET WITH RETRY ===");

    while (retries > 0) {
        LOG_DBG("Attempt %d/%d to create listener socket...", 51-retries, 50);

        fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd >= 0) {
            LOG_DBG("✓ Socket created successfully on attempt %d", 51-retries);

             /* Set socket options */
            int reuse = 1;
            LOG_DBG("Setting SO_REUSEADDR...");
            if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
                LOG_WRN("Failed to set SO_REUSEADDR: %s (errno: %d)",
                        strerror(errno), errno);
            } else {
                LOG_DBG("✓ SO_REUSEADDR set successfully");
            }

            /*
             * CRITICAL FIX: Listener socket MUST be set to non-blocking mode for
             * concurrent connection handling. The reason is:
             *
             * 1. When multiple clients connect simultaneously, they all queue in the
             *    listen backlog
             * 2. The broker's listener_dispatch() loops accepting ALL pending connections
             * 3. If the listener is blocking, zsock_accept() will block after processing
             *    the first connection, preventing subsequent connections from being accepted
             * 4. Clients waiting in backlog will timeout (ETIMEDOUT) or return EALREADY
             *
             * With non-blocking mode:
             * - listener_dispatch() can accept all connections in a single dispatch cycle
             * - No blocking allows event loop to continue processing other events
             * - Connection queue is properly drained
             */
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags < 0) {
                LOG_WRN("Failed to get socket flags: %s (errno: %d)",
                        strerror(errno), errno);
            } else if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
                LOG_WRN("Failed to set non-blocking mode: %s (errno: %d)",
                        strerror(errno), errno);
            } else {
                LOG_DBG("✓ Non-blocking mode set successfully");
            }

            /* Quick bind attempt */
            struct sockaddr_in addr = {0};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            addr.sin_port = htons(CONFIG_DBUS_BROKER_PORT);

            if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
                /* Use a larger backlog to handle concurrent connections */
                int backlog = 128;  /* Increase backlog for concurrent connections */
                if (listen(fd, backlog) == 0) {
                    LOG_INF("✓ Listener socket created successfully: fd=%d (backlog=%d)", fd, backlog);
                    return fd;
                } else {
                    LOG_DBG("Listen failed: %s (errno: %d)", strerror(errno), errno);
                }
            } else {
                LOG_DBG("Bind failed: %s (errno: %d)", strerror(errno), errno);
            }

            /* If bind/listen failed, close and retry */
            close(fd);
            fd = -1;
        } else {
            LOG_DBG("zsock_socket failed: %s (errno: %d)", strerror(errno), errno);
        }

        LOG_DBG("Retry %d failed, waiting %d ms...", 51-retries, delay_ms);
        k_msleep(delay_ms);
        delay_ms *= 2;  // Exponential backoff
        retries--;
    }

    LOG_ERR("Failed to create listener socket after all retries");
    return -EIO;
}

static int add_listener_to_broker(int listener_fd)
{
    int r;
    ControllerListener *listener = NULL;

    /* Use controller's direct API to add listener */
    r = controller_add_listener(&g_broker->controller, &listener,
                                "/org/bus1/DBus/Listener/0",
                                listener_fd, NULL);
    if (r < 0) {
        LOG_ERR("Failed to append listener fd: %s (code: %d)", strerror(-r), r);
        return r;
    }

    LOG_INF("✓ Listener added to broker successfully");
    return 0;
}

/* Modified deployment function */
static int standard_broker_deployment(void)
{
    int r;
    int listener_fd = -1;

    LOG_INF("===========================================");
    LOG_INF("STANDARD BROKER DEPLOYMENT (PERMISSIVE MODE)");
    LOG_INF("===========================================");

    /* Step 1: Create controller socketpair */
    const char *machine_id = "0123456789abcdef0123456789abcdef";
    r = socketpair(AF_UNIX, SOCK_STREAM, 0, g_controller_fds);
    if (r < 0) {
        LOG_ERR("socketpair failed: %d", r);
        return -errno;
    }
    LOG_INF("✓ Socketpair created: [%d, %d]", g_controller_fds[0], g_controller_fds[1]);

    /* Step 2: Create broker FIRST (this might help with timing) */
    r = broker_new(&g_broker, NULL, machine_id, g_controller_fds[0],
                   1024*1024, 256, 64, 128);
    if (r < 0) {
        LOG_ERR("broker_new failed: %d", r);
        close(g_controller_fds[0]);
        close(g_controller_fds[1]);
        return r;
    }
    LOG_INF("✓ Broker created: %p", g_broker);

    /* Small delay to let broker initialize */
    k_msleep(100);

    /* Step 4: Create listener socket with retry */
    listener_fd = create_listener_socket_with_retry();
    if (listener_fd < 0) {
        LOG_ERR("Failed to create listener socket: %d", listener_fd);
        // This is critical, so we do fail here
        return listener_fd;
    }

    /* Step 5: Add listener to broker */
    r = add_listener_to_broker(listener_fd);
    if (r < 0) {
        LOG_ERR("Failed to add listener to broker: %d", r);
        close(listener_fd);
        return r;
    }
    LOG_INF("✓ Listener fd=%d added to broker, 127.0.0.1:55555 should be listening now", listener_fd);

    /* Update deployment state */
    k_mutex_lock(&deploy_state.lock, K_FOREVER);
    deploy_state.broker_ready = true;
    deploy_state.listener_fd = listener_fd;
    k_mutex_unlock(&deploy_state.lock);

    LOG_INF("===========================================");
    LOG_INF("DEPLOYMENT COMPLETED SUCCESSFULLY!");
    LOG_INF("Broker: %p, Listener FD: %d", g_broker, listener_fd);
    LOG_INF("===========================================");

    k_msleep(1000);  /* Give broker time to start event loop and begin listening */

    return 0;
}

static int deploy_standard_broker(void)
{
    int r;

    r = standard_broker_deployment();
    if (r < 0) {
        LOG_ERR("Standard deployment failed: %d", r);
        return r;
    }

    /* Run broker (handles all client connections in event loop) */
    LOG_INF("Starting broker event loop (will now accept connections on 127.0.0.1:55555)...");
    LOG_INF("Service providers can now connect!");
    k_msleep(500);  /* Give service providers a moment to see this log */
    r = broker_run(g_broker);

    if (g_broker) {
        broker_free(g_broker);
        g_broker = NULL;
    }

    if (g_controller_fds[0] >= 0) {
        close(g_controller_fds[0]);
        close(g_controller_fds[1]);
        g_controller_fds[0] = g_controller_fds[1] = -1;
    }

    return r;
}

/*
 * Broker Thread - Runs the standard broker with our deployment
 */
static void broker_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    int r;

    LOG_INF("[DBus Broker] Starting D-Bus Broker Deployment...");

    /* Run the broker deployment using standard broker_run() */
    r = deploy_standard_broker();

    if (r == 0) {
        LOG_INF("[DBus Broker] Broker completed successfully");
    } else {
        LOG_ERR("[DBus Broker] Broker exited with error: %d", r);
    }
}

/* Thread stack for broker */
K_THREAD_STACK_DEFINE(broker_stack, CONFIG_DBUS_BROKER_STACK_SIZE);
static struct k_thread broker_thread;

/*
 * D-Bus Broker initialization
 */
static int dbus_broker_init(void)
{
    LOG_INF("[DBus Broker] Initializing D-Bus Broker subsystem...");
    
    /* Create broker thread */
    k_thread_create(&broker_thread, 
                   broker_stack, 
                   K_THREAD_STACK_SIZEOF(broker_stack), 
                   broker_thread_entry, 
                   NULL, NULL, NULL, 
                   CONFIG_DBUS_BROKER_PRIORITY, 0, K_NO_WAIT);
    
    LOG_INF("[DBus Broker] D-Bus Broker subsystem initialized");
    return 0;
}


/* Register initialization function */
SYS_INIT(dbus_broker_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);




/*
 * Public API functions
 */

int connect_to_dbroker(sd_bus **bus, int *socket_fd)
{
    int sock = -1;
    int r = -1;
    int retry_count = 0;

    /* Connect to localhost:CONFIG_DBUS_BROKER_PORT */
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(CONFIG_DBUS_BROKER_PORT);

    /* Retry connection until broker is ready */
    retry_count = 0;
    while (retry_count < 200) {  /* Increase to 20 seconds */
        /* Create socket and connect to listener */
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            LOG_INF("[DBroker API] Failed to create socket, cycle %d", retry_count);
            break;
        }

        r = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
        if (r >= 0) {
            LOG_INF("[DBroker API] Connected on attempt %d", retry_count + 1);
            break;
        }
        /* Log first few attempts */
        if (retry_count < 5 || retry_count % 50 == 0) {
            LOG_INF("[DBroker API] Connection attempt %d failed, errno: %d (%s)",
                    retry_count + 1, errno, strerror(errno));
        }
        retry_count++;
        k_msleep(100);
    }

    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to connect after %d retries, errno: %d (%s)",
                retry_count, errno, strerror(errno));
        close(sock);
        return -errno;
    }

    LOG_INF("[DBroker API] Connected to broker");

    /* Create sd-bus using the connected socket */
    r = sd_bus_new(bus);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to create bus, error: %d", r);
        close(sock);
        return r;
    }

    r = sd_bus_set_fd(*bus, sock, sock);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to set fd, error: %d", r);
        sd_bus_unref(*bus);
        close(sock);
        return r;
    }

    r = sd_bus_set_bus_client(*bus, true);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to set client mode, error: %d", r);
        sd_bus_unref(*bus);
        close(sock);
        return r;
    }

    /* Start the bus (authentication) */
    r = sd_bus_start(*bus);
    if (r < 0) {
        LOG_WRN("[DBroker API] Authentication warning: %d", r);
    }

    retry_count = 0;
    while (!sd_bus_is_ready(*bus) && retry_count < 100) {
        k_msleep(50);
        retry_count++;
    }

    /* Return the socket fd if requested */
    if (socket_fd) {
        *socket_fd = sock;
    }

    return 0;
}