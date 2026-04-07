/* SPDX-License-Identifier: Apache-2.0 */

/* Define basu config macros before including any basu headers */
#ifndef SIZEOF_PID_T
#define SIZEOF_PID_T 4
#endif

#ifndef SIZEOF_UID_T
#define SIZEOF_UID_T 4
#endif

#ifndef SIZEOF_GID_T
#define SIZEOF_GID_T 4
#endif

/* Note: _noreturn_ is NOT defined here to avoid conflicts with macro.h */
/* It will be properly defined when basu's macro.h is included */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <sys/socket.h>
#include <zephyr/net/net_ip.h>
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
#include "socketpool.h"

LOG_MODULE_REGISTER(DBROKER, LOG_LEVEL_DBG);

/* INADDR_LOOPBACK is not defined in Zephyr */
#ifndef INADDR_LOOPBACK
#define INADDR_LOOPBACK 0x7f000001
#endif

/*
 * Wrapper structure to store additional FD information
 * This replaces direct access to sd_bus internal fields
 */
struct bus_wrapper {
    sd_bus *bus;
    int broker_fd;
    int client_fd;
    sys_snode_t node;  /* For linking in global list */
};

/* Global list to track all active bus wrappers */
static sys_slist_t g_bus_wrappers = SYS_SLIST_STATIC_INIT(&g_bus_wrappers);
static struct k_mutex g_wrapper_lock = Z_MUTEX_INITIALIZER(g_wrapper_lock);

/* Helper function to find wrapper by sd_bus pointer */
static struct bus_wrapper *find_wrapper_by_bus(sd_bus *bus)
{
    struct bus_wrapper *wrapper;
    sys_snode_t *node;

    k_mutex_lock(&g_wrapper_lock, K_FOREVER);
    SYS_SLIST_FOR_EACH_NODE(&g_bus_wrappers, node) {
        wrapper = CONTAINER_OF(node, struct bus_wrapper, node);
        if (wrapper->bus == bus) {
            k_mutex_unlock(&g_wrapper_lock);
            return wrapper;
        }
    }
    k_mutex_unlock(&g_wrapper_lock);
    return NULL;
}

/* Helper function to add wrapper to global list */
static void add_wrapper_to_list(struct bus_wrapper *wrapper)
{
    k_mutex_lock(&g_wrapper_lock, K_FOREVER);
    sys_slist_append(&g_bus_wrappers, &wrapper->node);
    k_mutex_unlock(&g_wrapper_lock);
}

/* Helper function to remove wrapper from global list */
static void remove_wrapper_from_list(struct bus_wrapper *wrapper)
{
    k_mutex_lock(&g_wrapper_lock, K_FOREVER);
    sys_slist_find_and_remove(&g_bus_wrappers, &wrapper->node);
    k_mutex_unlock(&g_wrapper_lock);
}

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

#ifndef CONFIG_DBUS_BROKER_SOCKETPOOL
static int create_listener_socket_with_retry(void)
{
    int fd = -1;
    int retries = 50;
    int delay_ms = 100;

    // LOG_INF("=== CREATING LISTENER SOCKET WITH RETRY ===");

    while (retries > 0) {
        // LOG_DBG("Attempt %d/%d to create listener socket...", 51-retries, 50);

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
#endif /* CONFIG_DBUS_BROKER_SOCKETPOOL */

#ifndef CONFIG_DBUS_BROKER_SOCKETPOOL
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
#endif /* CONFIG_DBUS_BROKER_SOCKETPOOL */

/* Modified deployment function */
static int standard_broker_deployment(void)
{
    int r;
    int listener_fd = -1;

    // LOG_INF("===========================================");
    // LOG_INF("STANDARD BROKER DEPLOYMENT (PERMISSIVE MODE)");
    // LOG_INF("===========================================");

    /* Step 1: Create controller socketpair */
    const char *machine_id = "0123456789abcdef0123456789abcdef";
    // sd_id128_t saved_machine_id;
    // r = sd_id128_get_machine(&saved_machine_id);
    // if (r < 0) {
    //     LOG_ERR("Failed to get machine ID: %d", r);
    //     return r;
    // }

    // char midstr[SD_ID128_STRING_MAX];
    // char *machine_id1 = sd_id128_to_string(saved_machine_id, midstr);
    // LOG_INF("Machine ID: %s, %s", machine_id1, midstr);

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
    // LOG_INF("✓ Broker created: %p", g_broker);

    /* Small delay to let broker initialize */
    k_msleep(100);

#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
    /* Step 4: Socketpool mode - no listener needed */
    /* The broker will accept connections from socketpool directly */
    LOG_INF("✓ Socketpool mode: broker ready to accept connections from pool");
    listener_fd = -1;  /* No listener fd in socketpool mode */
#else
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
    // LOG_INF("✓ Listener fd=%d added to broker, 127.0.0.1:55555 should be listening now", listener_fd);
#endif

    /* Update deployment state */
    k_mutex_lock(&deploy_state.lock, K_FOREVER);
    deploy_state.broker_ready = true;
    deploy_state.listener_fd = listener_fd;
    k_mutex_unlock(&deploy_state.lock);

    // LOG_INF("===========================================");
    // LOG_INF("DEPLOYMENT COMPLETED SUCCESSFULLY!");
    LOG_INF("Broker: %p, Listener FD: %d", g_broker, listener_fd);
    // LOG_INF("===========================================");

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
    // LOG_INF("Service providers can now connect!");
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

    // LOG_INF("[DBus Broker] Starting D-Bus Broker Deployment...");

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
    int r;
    // LOG_INF("[DBus Broker] Initializing D-Bus Broker subsystem...");

#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
    /* Initialize socketpool */
    r = socketpool_init();
    if (r < 0) {
        LOG_ERR("[DBus Broker] Failed to initialize socketpool: %d", r);
        return r;
    }
    LOG_INF("[DBus Broker] Socketpool initialized successfully");
#endif
    
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

#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL

/**
 * @brief Request a connection from the socketpool
 * @details This function allocates a socketpair from the pool and returns
 *          the client's fd. The broker will use the other end to create a peer.
 * @param client_fd [out] Pointer to store the client's fd
 * @return 0 on success, negative error code on failure
 */
static int request_dbroker_connection(int *broker_fd, int *client_fd)
{
    int r;

    if (!broker_fd) {
        LOG_ERR("[DBroker API] Invalid parameter: broker_fd is NULL");
        return -EINVAL;
    }

    if (!client_fd) {
        LOG_ERR("[DBroker API] Invalid parameter: client_fd is NULL");
        return -EINVAL;
    }

    /* Check if broker is ready */
    if (!g_broker) {
        LOG_ERR("[DBroker API] Broker not initialized yet");
        return -ENOTCONN;
    }

    /* Allocate a socketpair from the pool */
    r = socketpool_allocate(broker_fd, client_fd);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to allocate socketpair: %d", r);
        return r;
    }

    LOG_DBG("[DBroker API] Allocated connection: client_fd=%d, broker_fd=%d",
            *client_fd, *broker_fd);

    /* Add the broker_fd to the broker to create a peer */
    r = socketpool_add_peer_to_broker(g_broker, *broker_fd);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to add peer to broker: %d", r);
        socketpool_free(*broker_fd, *client_fd);
        return r;
    }

    return 0;
}

#endif /* CONFIG_DBUS_BROKER_SOCKETPOOL */

int connect_to_dbroker(sd_bus **bus, int *socket_fd)
{
    int broker_fd = -1;
    int client_fd = -1;
    int r = -1;
    int retry_count = 0;
    static bool broker_started = false;
    struct bus_wrapper *wrapper = NULL;
    sd_bus *internal_bus = NULL;

    if (!bus) {
        LOG_ERR("[DBroker API] Invalid parameter: bus is NULL");
        return -EINVAL;
    }

    if (!broker_started) { 
        k_msleep(1000);
        broker_started = true;
    }
    
    /* Allocate wrapper structure */
    wrapper = k_malloc(sizeof(struct bus_wrapper));
    if (!wrapper) {
        LOG_ERR("[DBroker API] Failed to allocate wrapper structure");
        return -ENOMEM;
    }
    wrapper->bus = NULL;
    wrapper->broker_fd = -1;
    wrapper->client_fd = -1;

#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
    /* Use socketpool for connection */
    r = request_dbroker_connection(&broker_fd, &client_fd);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to get connection from socketpool: %d", r);
        k_free(wrapper);
        if (broker_fd >= 0 && client_fd >= 0) {
            socketpool_free(broker_fd, client_fd);
        }
        return r;
    }

    wrapper->broker_fd = broker_fd;
    wrapper->client_fd = client_fd;

    LOG_INF("[DBroker API] Got connection from socketpool: broker_fd=%d, client_fd=%d",
            broker_fd, client_fd);
#else
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
        client_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (client_fd < 0) {
            LOG_INF("[DBroker API] Failed to create socket, cycle %d", retry_count);
            break;
        }

        r = connect(client_fd, (struct sockaddr *)&addr, sizeof(addr));
        if (r >= 0) {
            // LOG_INF("[DBroker API] Connected on attempt %d", retry_count + 1);
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
        close(client_fd);
        k_free(wrapper);
        return -errno;
    }

    wrapper->client_fd = client_fd;
    LOG_INF("[DBroker API] Connected to broker");
#endif /* CONFIG_DBUS_BROKER_SOCKETPOOL */

    /* Create sd-bus using the connected socket */
    r = sd_bus_new(&internal_bus);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to create bus, error: %d", r);
        k_free(wrapper);
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
        if (broker_fd >= 0 && client_fd >= 0) {
            socketpool_free(broker_fd, client_fd);
        }
#else
        if (client_fd >= 0) {
            close(client_fd);
        }
#endif
        return r;
    }

    wrapper->bus = internal_bus;

    r = sd_bus_set_fd(internal_bus, client_fd, client_fd);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to set fd, error: %d", r);
        sd_bus_unref(internal_bus);
        k_free(wrapper);
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
        if (broker_fd >= 0 && client_fd >= 0) {
            socketpool_free(broker_fd, client_fd);
        }
#else
        if (client_fd >= 0) {
            close(client_fd);
        }
#endif
        return r;
    }

    r = sd_bus_set_bus_client(internal_bus, true);
    if (r < 0) {
        LOG_ERR("[DBroker API] Failed to set client mode, error: %d", r);
        sd_bus_unref(internal_bus);
        k_free(wrapper);
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
        if (broker_fd >= 0 && client_fd >= 0) {
            socketpool_free(broker_fd, client_fd);
        }
#else
        if (client_fd >= 0) {
            close(client_fd);
        }
#endif
        return r;
    }

    /* Add wrapper to global list for tracking */
    add_wrapper_to_list(wrapper);

    /* Start the bus (sends Hello message) */
    r = sd_bus_start(internal_bus);
    if (r < 0) {
        LOG_WRN("[DBroker API] sd_bus_start returned: %d", r);
    }

    /* Wait for bus to become ready (process Hello response) */
    retry_count = 0;
    while (!sd_bus_is_ready(internal_bus) && retry_count < 200) {
        /* Process incoming messages to handle Hello response */
        r = sd_bus_process(internal_bus, NULL);
        if (r < 0) {
            LOG_ERR("[DBroker API] Failed to process bus messages: %d", r);
            break;
        }
        k_msleep(25);
        retry_count++;
    }

    if (!sd_bus_is_ready(internal_bus)) {
        LOG_ERR("[DBroker API] Bus not ready after %d attempts", retry_count);
    } else {
        LOG_DBG("[DBroker API] Bus ready after %d attempts", retry_count);
    }

    /* Return the bus object to caller */
    *bus = internal_bus;

    /* Return the socket fd if requested */
    if (socket_fd) {
        *socket_fd = client_fd;
    }

    return 0;
}


int disconnect_from_dbroker(sd_bus *bus) {
    struct bus_wrapper *wrapper;
    
    if (!bus) {
        return -EINVAL;
    }
    
    LOG_INF("[sd-bus] Disconnecting from dbus-broker...");
    
    /* Find wrapper associated with this bus */
    wrapper = find_wrapper_by_bus(bus);
    
    if (!wrapper) {
        LOG_WRN("[sd-bus] Warning: Could not find wrapper for bus, attempting basic cleanup");
        /* If no wrapper found, perform basic cleanup */
        sd_bus_flush(bus);
        sd_bus_close(bus);
        k_msleep(50);
        sd_bus_unref(bus);
        return 0;
    }
    
    /* Remove wrapper from global list first */
    remove_wrapper_from_list(wrapper);
    
    // 1. Flush pending messages
    sd_bus_flush(bus);
    
    // 2. Close IO FDs (marks as closed, skips actual close for socketpool)
    sd_bus_close(bus);
    
    // 3. Wait for broker to process EOF and free peer
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
    k_msleep(100);  // Shorter delay for socketpair
#else
    k_msleep(200);
#endif
    
    // 4. Recycle socketpool resources
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
    if (wrapper->broker_fd >= 0 && wrapper->client_fd >= 0) {
        int saved_broker_fd = wrapper->broker_fd;
        int saved_client_fd = wrapper->client_fd;
        
        socketpool_free(wrapper->broker_fd, wrapper->client_fd);
        wrapper->broker_fd = -1;
        wrapper->client_fd = -1;
    }
#endif
    
    // 5. Unref the bus object (frees memory)
    sd_bus_unref(bus);
    
    // 6. Free wrapper structure
    k_free(wrapper);
    
    LOG_INF("[sd-bus] Disconnected successfully");
    return 0;
}