
/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <sys/socket.h>
#include <errno.h>
#include <string.h>

/* dbus-broker headers */
#include "../../../modules/lib/dbus-broker/src/broker/broker.h"
#include "../../../modules/lib/dbus-broker/src/bus/peer.h"

LOG_MODULE_REGISTER(SOCKETPOOL, LOG_LEVEL_INF);

/*
 * Socketpool configuration
 */
#ifdef CONFIG_DBUS_BROKER_SOCKETPOOL
#define SOCKETPOOL_MAX_PAIRS  CONFIG_DBUS_BROKER_SOCKETPOOL_SIZE
#define SOCKETPOOL_BUFFER_SIZE CONFIG_DBUS_BROKER_SOCKETPOOL_BUFFER_SIZE
#else
#define SOCKETPOOL_MAX_PAIRS  1
#define SOCKETPOOL_BUFFER_SIZE 1
#endif

/*
 * Socketpool entry
 */
struct socketpool_entry {
    int broker_fd;
    int client_fd;
    bool in_use;
    struct k_sem sem;
};

/*
 * Socketpool global state
 */
struct socketpool {
    struct k_mutex lock;
    struct socketpool_entry entries[SOCKETPOOL_MAX_PAIRS];
    int total_pairs;
    int available_pairs;
};

static struct socketpool socketpool = {
    .lock = Z_MUTEX_INITIALIZER(socketpool.lock),
    .total_pairs = 0,
    .available_pairs = 0,
};

/*
 * Initialize socketpool
 */
int socketpool_init(void)
{
    int r;
    int i;

    LOG_INF("[Socketpool] Initializing socketpool with %d pairs...", SOCKETPOOL_MAX_PAIRS);

    k_mutex_lock(&socketpool.lock, K_FOREVER);

    for (i = 0; i < SOCKETPOOL_MAX_PAIRS; i++) {
        struct socketpool_entry *entry = &socketpool.entries[i];

        entry->in_use = false;
        k_sem_init(&entry->sem, 1, 1);  /* Binary semaphore for each entry */

        /* Create socketpair */
        int sv[2];
        r = socketpair(AF_UNIX, SOCK_STREAM, 0, sv);
        if (r < 0) {
            LOG_ERR("[Socketpool] Failed to create socketpair %d: %d (errno: %d)",
                    i, r, errno);
            k_mutex_unlock(&socketpool.lock);
            return -errno;
        }

        /* Store the fds */
        entry->broker_fd = sv[0];
        entry->client_fd = sv[1];

        /* Set both ends to non-blocking */
        int flags;
        flags = fcntl(entry->broker_fd, F_GETFL, 0);
        if (flags < 0) {
            LOG_ERR("[Socketpool] Failed to get flags for broker_fd %d: %d (errno: %d)",
                    entry->broker_fd, r, errno);
        } else {
            fcntl(entry->broker_fd, F_SETFL, flags | O_NONBLOCK);
        }

        flags = fcntl(entry->client_fd, F_GETFL, 0);
        if (flags < 0) {
            LOG_ERR("[Socketpool] Failed to get flags for client_fd %d: %d (errno: %d)",
                    entry->client_fd, r, errno);
        } else {
            fcntl(entry->client_fd, F_SETFL, flags | O_NONBLOCK);
        }

        /* Set socket buffer sizes */
        /* Note: SO_RCVBUF and SO_SNDBUF are not supported on AF_UNIX sockets in Zephyr */
        /* The kernel manages buffer sizes internally */

        // LOG_DBG("[Socketpool] Created socketpair %d: broker_fd=%d, client_fd=%d",
        //         i, entry->broker_fd, entry->client_fd);

        socketpool.total_pairs++;
        socketpool.available_pairs++;
    }

    k_mutex_unlock(&socketpool.lock);

    LOG_INF("[Socketpool] Socketpool initialized: %d/%d pairs available",
            socketpool.available_pairs, socketpool.total_pairs);

    return 0;
}

/*
 * Allocate a socketpair from the pool
 * Returns: 0 on success, negative errno on failure
 * On success, broker_fd and client_fd are set to the allocated fds
 */
int socketpool_allocate(int *broker_fd, int *client_fd)
{
    int i;
    struct socketpool_entry *entry = NULL;

    if (!broker_fd || !client_fd) {
        LOG_ERR("[Socketpool] Invalid parameters");
        return -EINVAL;
    }

    /* Find an available entry */
    k_mutex_lock(&socketpool.lock, K_FOREVER);

    if (socketpool.available_pairs == 0) {
        LOG_ERR("[Socketpool] No available socketpairs in pool");
        k_mutex_unlock(&socketpool.lock);
        return -EAGAIN;
    }

    for (i = 0; i < socketpool.total_pairs; i++) {
        if (!socketpool.entries[i].in_use) {
            entry = &socketpool.entries[i];
            break;
        }
    }

    if (!entry) {
        LOG_ERR("[Socketpool] Failed to find available socketpair");
        k_mutex_unlock(&socketpool.lock);
        return -EAGAIN;
    }

    /* Mark as in use */
    entry->in_use = true;
    socketpool.available_pairs--;

    *broker_fd = entry->broker_fd;
    *client_fd = entry->client_fd;

    k_mutex_unlock(&socketpool.lock);

    LOG_DBG("[Socketpool] Allocated socketpair: broker_fd=%d, client_fd=%d (%d/%d available)",
            *broker_fd, *client_fd, socketpool.available_pairs, socketpool.total_pairs);

    return 0;
}

/*
 * Free a socketpair back to the pool
 * Returns: 0 on success, negative errno on failure
 */
int socketpool_free(int broker_fd, int client_fd)
{
    int i;
    struct socketpool_entry *entry = NULL;

    k_mutex_lock(&socketpool.lock, K_FOREVER);

    /* Find the entry */
    for (i = 0; i < socketpool.total_pairs; i++) {
        if (socketpool.entries[i].broker_fd == broker_fd &&
            socketpool.entries[i].client_fd == client_fd) {
            entry = &socketpool.entries[i];
            break;
        }
    }

    if (!entry) {
        LOG_ERR("[Socketpool] Socketpair not found: broker_fd=%d, client_fd=%d",
                broker_fd, client_fd);
        k_mutex_unlock(&socketpool.lock);
        return -ENOENT;
    }

    if (!entry->in_use) {
        LOG_WRN("[Socketpool] Socketpair already free: broker_fd=%d, client_fd=%d",
                broker_fd, client_fd);
        k_mutex_unlock(&socketpool.lock);
        return 0;
    }

    /* Mark as available - DO NOT reset FDs to -1!
     * The FDs remain valid and will be reused on next allocation.
     * This prevents pool capacity from shrinking.
     */
    entry->in_use = false;
    socketpool.available_pairs++;

    k_mutex_unlock(&socketpool.lock);

    LOG_DBG("[Socketpool] Freed socketpair: broker_fd=%d, client_fd=%d (%d/%d available)",
            broker_fd, client_fd, socketpool.available_pairs, socketpool.total_pairs);

    return 0;
}

/*
 * Get socketpool statistics
 */
void socketpool_get_stats(int *total, int *available)
{
    if (total) {
        *total = socketpool.total_pairs;
    }
    if (available) {
        *available = socketpool.available_pairs;
    }
}

/*
 * Add a peer to the broker using a socketpair from the pool
 * This function is called when a client requests a connection
 * Returns: 0 on success, negative errno on failure
 */
int socketpool_add_peer_to_broker(Broker *broker, int broker_fd)
{
    Peer *peer = NULL;
    int r;
    char guid[16] = "0123456789abcdef";

    if (!broker) {
        LOG_ERR("[Socketpool] Invalid broker");
        return -EINVAL;
    }

    LOG_DBG("[Socketpool] Creating peer for broker_fd=%d", broker_fd);

    /* Create a new peer with the broker_fd */
    r = peer_new_with_fd(&peer, &broker->bus, NULL, guid,
                        &broker->dispatcher, broker_fd);
    if (r < 0) {
        LOG_ERR("[Socketpool] Failed to create peer: %d", r);
        return r;
    }

    if (peer->policy != NULL) {
        LOG_WRN("[Socketpool] Unexpected non-NULL policy on Zephyr");
    }

    /* Do NOT register the peer here - let the peer register itself via Hello message */
    /* peer_register(peer); */

    LOG_DBG("[Socketpool] Peer created: fd=%d, peer_id=%d (will register via Hello)", broker_fd, peer->id);

    /* Spawn the peer (open the connection) */
    r = peer_spawn(peer);
    if (r < 0) {
        LOG_ERR("[Socketpool] Failed to spawn peer: %d", r);
        peer_unregister(peer);
        peer_free(peer);
        return r;
    }

    LOG_INF("[Socketpool] Peer spawned successfully: fd=%d, connection socket_file user_mask=0x%x",
            broker_fd, peer->connection.socket_file.user_mask);

    /* Peer is now owned by the broker, don't free it */
    return 0;
}
