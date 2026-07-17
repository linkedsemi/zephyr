/*
 * Copyright (c) 2024 Alibaba Group
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AF_UNIX SOCK_STREAM & SOCK_DGRAM socket implementation for Zephyr.
 *
 * Data transport reuses the k_pipe + k_poll_signal pattern from socketpair.c.
 * Each connected socket endpoint owns a recv pipe; writes go to the peer's
 * pipe. A global registry maps sun_path -> listening socket so that connect()
 * can find the target.
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/logging/log.h>

#include <soc.h>

#include "sockets_internal.h"
#include "af_unix.h"

extern int zvfs_close(int fd);

LOG_MODULE_REGISTER(af_unix, CONFIG_NET_SOCKETS_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* Signals carried on k_poll_signal                                   */
/* ------------------------------------------------------------------ */
enum {
	USOCK_SIG_CANCEL, /* peer closed / operation cancelled */
	USOCK_SIG_DATA,   /* data available / space available  */
};

/* ------------------------------------------------------------------ */
/* Flags for unix_socket.flags                                        */
/* ------------------------------------------------------------------ */
enum {
	USOCK_FLAG_NONBLOCK = BIT(0),
	USOCK_FLAG_SHUT_RD  = BIT(1),
	USOCK_FLAG_SHUT_WR  = BIT(2),
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */
static const struct socket_op_vtable unix_fd_op_vtable;

/* ------------------------------------------------------------------ */
/* Per-endpoint structure                                             */
/* ------------------------------------------------------------------ */
/*
 * Accept-queue node: when a client connect()'s to a listening socket, a
 * pending_conn is queued into the listener's accept_queue so that the
 * next accept() call can harvest it.
 */
struct pending_conn {
	sys_snode_t node;
	int client_fd; /* fd of the client-side connected endpoint */
};

#define MAX_PENDING_CONNS CONFIG_NET_UNIX_MAX_SOCKETS

static struct pending_conn pending_pool[MAX_PENDING_CONNS];
static struct k_mutex pending_pool_lock;

static struct pending_conn *pending_alloc(void)
{
	struct pending_conn *p = NULL;

	k_mutex_lock(&pending_pool_lock, K_FOREVER);
	for (int i = 0; i < MAX_PENDING_CONNS; i++) {
		if (pending_pool[i].client_fd == -1) {
			pending_pool[i].client_fd = 0;
			p = &pending_pool[i];
			break;
		}
	}
	k_mutex_unlock(&pending_pool_lock);
	return p;
}

static void pending_free(struct pending_conn *p)
{
	k_mutex_lock(&pending_pool_lock, K_FOREVER);
	p->client_fd = -1;
	k_mutex_unlock(&pending_pool_lock);
}

/* Main socket endpoint */
__net_socket struct unix_socket {
	struct k_pipe recv_q;
	uint8_t *pipe_buf;              /* dynamically allocated        */
	struct k_poll_signal readable;
	struct k_poll_signal writable;
	struct k_sem accept_sem;        /* signalled when a conn arrives */
	struct sockaddr_un addr;
	uint32_t flags;
	bool bound;
	bool listening;
	bool connected;
	bool closed;
	int peer_fd;                    /* fd of the remote endpoint    */
	sys_slist_t accept_queue;
	struct k_mutex lock;
	int type;                       /* SOCK_STREAM only for now     */
	int backlog;
        /* DGRAM fields */
	sys_slist_t dgram_list;
	size_t dgram_queued_bytes;
	struct sockaddr_un dgram_peer;
	bool dgram_peer_set;
};

/* ------------------------------------------------------------------ */
/* Static socket pool                                                 */
/* ------------------------------------------------------------------ */
#define MAX_UNIX_SOCKETS CONFIG_NET_UNIX_MAX_SOCKETS

static struct unix_socket unix_sockets[MAX_UNIX_SOCKETS];
static struct k_mutex registry_lock;

/* ------------------------------------------------------------------ */
/* Static pipe buffer pool - eliminates heap fragmentation              */
/* Each socket gets a fixed 2KB buffer from this pre-allocated pool.   */
/* No k_malloc needed → no fragmentation under heavy object creation. */
/* ------------------------------------------------------------------ */
static uint8_t unix_pipe_buf_pool[MAX_UNIX_SOCKETS][CONFIG_NET_UNIX_BUFFER_SIZE];
static ATOMIC_DEFINE(pipe_buf_used, MAX_UNIX_SOCKETS);

static uint8_t *pipe_buf_alloc(void)
{
	for (int i = 0; i < MAX_UNIX_SOCKETS; i++) {
		if (!atomic_test_and_set_bit(pipe_buf_used, i)) {
			return unix_pipe_buf_pool[i];
		}
	}
	return NULL;
}

static void pipe_buf_free(uint8_t *buf)
{
	if (!buf) return;
	for (int i = 0; i < MAX_UNIX_SOCKETS; i++) {
		if (buf == unix_pipe_buf_pool[i]) {
			atomic_clear_bit(pipe_buf_used, i);
			return;
		}
	}
}

/* ------------------------------------------------------------------ */
/* Pool helpers                                                       */
/* ------------------------------------------------------------------ */
static struct unix_socket *usock_alloc(void)
{
	struct unix_socket *s = NULL;

	k_mutex_lock(&registry_lock, K_FOREVER);
	for (int i = 0; i < MAX_UNIX_SOCKETS; i++) {
		if (!unix_sockets[i].closed &&
		    !unix_sockets[i].bound &&
		    !unix_sockets[i].listening &&
		    !unix_sockets[i].connected &&
		    unix_sockets[i].type == 0) {
			s = &unix_sockets[i];
			/* mark as in-use immediately */
			s->type = SOCK_STREAM;
			break;
		}
	}
	k_mutex_unlock(&registry_lock);
	return s;
}

/**
 * @brief Initialize a unix_socket and allocate its pipe buffer.
 *
 * @return 0 on success, -ENOMEM if pipe buffer allocation fails.
 */
static int usock_init(struct unix_socket *s)
{
	uint8_t *buf;

	buf = pipe_buf_alloc();
	if (buf == NULL) {
		LOG_ERR("AF_UNIX: pipe buffer pool exhausted (%d sockets)",
			MAX_UNIX_SOCKETS);
		return -ENOMEM;
	}

	s->pipe_buf = buf;
	k_pipe_init(&s->recv_q, s->pipe_buf,
		     CONFIG_NET_UNIX_BUFFER_SIZE);
	k_poll_signal_init(&s->readable);
	k_poll_signal_init(&s->writable);
	k_sem_init(&s->accept_sem, 0, K_SEM_MAX_LIMIT);
	k_mutex_init(&s->lock);
	sys_slist_init(&s->accept_queue);

	s->peer_fd = -1;
	s->flags = 0;
	s->bound = false;
	s->listening = false;
	s->connected = false;
	s->closed = false;
	s->type = SOCK_STREAM;
	s->backlog = 0;
    sys_slist_init(&s->dgram_list);
	s->dgram_queued_bytes = 0;
	s->dgram_peer_set = false;
	memset(&s->addr, 0, sizeof(s->addr));

	/* A fresh socket is always writable */
	k_poll_signal_raise(&s->writable, USOCK_SIG_DATA);

	return 0;
}

/* ------------------------------------------------------------------ */
/* Namespace registry: find a bound+listening socket by sun_path      */
/* ------------------------------------------------------------------ */
static struct unix_socket *find_bound_socket(const char *path, size_t pathlen)
{
	for (int i = 0; i < MAX_UNIX_SOCKETS; i++) {
		struct unix_socket *s = &unix_sockets[i];

		if (!s->bound) {
			continue;
		}
		/*
		 * Abstract sockets: sun_path[0] == '\0', compare
		 * the whole pathlen (including the leading NUL).
		 * Filesystem-style paths: strcmp suffices.
		 */
		if (path[0] == '\0') {
			/* abstract path - compare binary */
			size_t bound_len = sizeof(s->addr.sun_path);

			/* pathlen already excludes sun_family */
			if (memcmp(s->addr.sun_path, path,
				   MIN(pathlen, bound_len)) == 0) {
				return s;
			}
		} else {
			if (strncmp(s->addr.sun_path, path,
				    sizeof(s->addr.sun_path)) == 0) {
				return s;
			}
		}
	}
	return NULL;
}

/* ------------------------------------------------------------------ */
/* Helper: is sock nonblocking?                                       */
/* ------------------------------------------------------------------ */
static inline bool usock_is_nonblock(const struct unix_socket *s)
{
	return !!(s->flags & USOCK_FLAG_NONBLOCK);
}

/* ------------------------------------------------------------------ */
/* Helper: get peer unix_socket from peer_fd                          */
/* ------------------------------------------------------------------ */
static inline struct unix_socket *usock_peer(const struct unix_socket *s)
{
	if (s->peer_fd < 0) {
		return NULL;
	}
	return zvfs_get_fd_obj(s->peer_fd,
			       (const struct fd_op_vtable *)&unix_fd_op_vtable,
			       0);
}

/* ------------------------------------------------------------------ */
/* Helper: bytes available to read / write                            */
/* ------------------------------------------------------------------ */
static inline size_t usock_read_avail(struct unix_socket *s)
{
	return k_pipe_read_avail(&s->recv_q);
}

static inline size_t usock_write_avail(struct unix_socket *s)
{
	struct unix_socket *peer = usock_peer(s);

	if (peer == NULL) {
		return 0;
	}
	return k_pipe_write_avail(&peer->recv_q);
}

/* ------------------------------------------------------------------ */
/* write: send data to peer's recv pipe                               */
/* ------------------------------------------------------------------ */
static ssize_t unix_write(void *obj, const void *buffer, size_t count)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct unix_socket *peer;
	size_t bytes_written;
	int res;

	if (s == NULL || buffer == NULL || count == 0) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	if (!s->connected) {
		k_mutex_unlock(&s->lock);
		errno = ENOTCONN;
		return -1;
	}

	if (s->flags & USOCK_FLAG_SHUT_WR) {
		k_mutex_unlock(&s->lock);
		errno = EPIPE;
		return -1;
	}

	peer = usock_peer(s);
	if (peer == NULL) {
		k_mutex_unlock(&s->lock);
		errno = EPIPE;
		return -1;
	}

	k_mutex_lock(&peer->lock, K_FOREVER);

	if (peer->closed) {
		k_mutex_unlock(&peer->lock);
		k_mutex_unlock(&s->lock);
		errno = EPIPE;
		return -1;
	}

	size_t avail = k_pipe_write_avail(&peer->recv_q);

	if (avail == 0) {
		if (usock_is_nonblock(s)) {
			k_mutex_unlock(&peer->lock);
			k_mutex_unlock(&s->lock);
			errno = EAGAIN;
			return -1;
		}

		/* Block until space appears or peer closes */
		k_mutex_unlock(&peer->lock);
		k_mutex_unlock(&s->lock);

		for (;;) {
			struct k_poll_event events[] = {
				K_POLL_EVENT_INITIALIZER(
					K_POLL_TYPE_SIGNAL,
					K_POLL_MODE_NOTIFY_ONLY,
					&s->writable),
			};

			res = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
			if (res < 0) {
				errno = -res;
				return -1;
			}

			int signaled = 0, result = -1;

			k_poll_signal_check(&s->writable, &signaled,
					    &result);
			if (signaled && result == USOCK_SIG_CANCEL) {
				errno = EPIPE;
				return -1;
			}

			k_mutex_lock(&s->lock, K_FOREVER);
			peer = usock_peer(s);
			if (peer == NULL || peer->closed) {
				k_mutex_unlock(&s->lock);
				errno = EPIPE;
				return -1;
			}
			k_mutex_lock(&peer->lock, K_FOREVER);
			avail = k_pipe_write_avail(&peer->recv_q);
			if (avail > 0) {
				break;
			}
			k_mutex_unlock(&peer->lock);
			k_mutex_unlock(&s->lock);
		}
	}

	res = k_pipe_put(&peer->recv_q, (void *)buffer, count,
			 &bytes_written, 1, K_NO_WAIT);
	__ASSERT(res == 0, "k_pipe_put() failed: %d", res);

	/* If peer pipe is now full, reset our writable signal */
	if (k_pipe_write_avail(&peer->recv_q) == 0) {
		k_poll_signal_reset(&s->writable);
	}

	/* Notify peer that data is available */
	k_poll_signal_raise(&peer->readable, USOCK_SIG_DATA);

	k_mutex_unlock(&peer->lock);
	k_mutex_unlock(&s->lock);

	return bytes_written;
}

/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* DGRAM                                                              */
/* ------------------------------------------------------------------ */
struct unix_dgram {
	sys_snode_t node;
	struct sockaddr_un src_addr;
	size_t len;
	uint8_t data[];  /* flexible array member */
};

#define DGRAM_MAX_SIZE CONFIG_NET_UNIX_BUFFER_SIZE

static int dgram_sendto(struct unix_socket *s, const void *buf,
			size_t len, const struct sockaddr *dest_addr,
			socklen_t addrlen)
{
	struct unix_socket *target = NULL;
	const struct sockaddr_un *sun;
	size_t pathlen;

	if (dest_addr != NULL) {
		sun = (const struct sockaddr_un *)dest_addr;
		pathlen = addrlen - offsetof(struct sockaddr_un, sun_path);
		if (pathlen == 0 || pathlen > sizeof(s->addr.sun_path)) {
			errno = EINVAL; return -1;
		}
		target = find_bound_socket(sun->sun_path, pathlen);
	} else if (s->dgram_peer_set) {
		target = find_bound_socket(s->dgram_peer.sun_path,
			strlen(s->dgram_peer.sun_path));
	}
	if (!target) { errno = ECONNREFUSED; return -1; }
	if (len > DGRAM_MAX_SIZE) { errno = EMSGSIZE; return -1; }

	struct unix_dgram *d = k_malloc(sizeof(*d) + len);
	if (!d) { errno = ENOMEM; return -1; }
	d->len = len;
	memcpy(d->data, buf, len);
	if (s->bound)
		memcpy(&d->src_addr, &s->addr, sizeof(d->src_addr));
	else {
		memset(&d->src_addr, 0, sizeof(d->src_addr));
		d->src_addr.sun_family = AF_UNIX;
	}


	k_mutex_lock(&target->lock, K_FOREVER);
	sys_slist_append(&target->dgram_list, &d->node);
	target->dgram_queued_bytes += len;
	k_poll_signal_raise(&target->readable, USOCK_SIG_DATA);
	k_mutex_unlock(&target->lock);
	return len;
}

static ssize_t dgram_recvfrom(struct unix_socket *s, void *buf,
			      size_t max_len, struct sockaddr *src_addr,
			      socklen_t *addrlen)
{
	k_mutex_lock(&s->lock, K_FOREVER);

	for (;;) {
		struct unix_dgram *d = (struct unix_dgram *)sys_slist_get(&s->dgram_list);
		if (d) {
			size_t copy_len = (max_len < d->len) ? max_len : d->len;
			memcpy(buf, d->data, copy_len);
			if (addrlen) {
				*addrlen = sizeof(struct sockaddr_un);
			}
			if (src_addr && addrlen) {
				memcpy(src_addr, &d->src_addr,
				       MIN(*addrlen, sizeof(d->src_addr)));
			}
			s->dgram_queued_bytes -= d->len;
			k_free(d);
			k_mutex_unlock(&s->lock);
			return copy_len;
		}
		if (usock_is_nonblock(s)) {
			k_mutex_unlock(&s->lock);
			errno = EAGAIN;
			return -1;
		}
		
		/* Poll for datagram using k_poll on readable signal */
		{
			struct k_poll_event events[] = {
				K_POLL_EVENT_INITIALIZER(
					K_POLL_TYPE_SIGNAL,
					K_POLL_MODE_NOTIFY_ONLY,
					&s->readable),
			};

			k_mutex_unlock(&s->lock);
			
			int ret = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
			
			k_mutex_lock(&s->lock, K_FOREVER);
			
			/* If poll failed, continue waiting */
			if (ret < 0 && ret != -EAGAIN) {
				errno = EINTR;
				return -1;
			}
		}
	}
}
/* read: receive data from our own recv pipe                          */
/* ------------------------------------------------------------------ */
static ssize_t unix_read(void *obj, void *buffer, size_t count)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct unix_socket *peer;
	size_t bytes_read;
	int res;

	if (s == NULL || buffer == NULL || count == 0) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	if (s->flags & USOCK_FLAG_SHUT_RD) {
		k_mutex_unlock(&s->lock);
		return 0; /* read shut down, EOF */
	}

	if (!s->connected && usock_read_avail(s) == 0) {
		k_mutex_unlock(&s->lock);
		if (s->peer_fd == -1) {
			/* EOF: peer gone and pipe empty */
			return 0;
		}
		errno = ENOTCONN;
		return -1;
	}

	size_t avail = usock_read_avail(s);

	if (avail == 0) {
		/* Check for EOF (peer closed, pipe empty) */
		peer = usock_peer(s);
		if (peer == NULL) {
			k_mutex_unlock(&s->lock);
			return 0; /* EOF */
		}

		if (usock_is_nonblock(s)) {
			k_mutex_unlock(&s->lock);
			errno = EAGAIN;
			return -1;
		}

		/* Block until data arrives or peer closes */
		k_mutex_unlock(&s->lock);

		for (;;) {
			struct k_poll_event events[] = {
				K_POLL_EVENT_INITIALIZER(
					K_POLL_TYPE_SIGNAL,
					K_POLL_MODE_NOTIFY_ONLY,
					&s->readable),
			};

			res = k_poll(events, ARRAY_SIZE(events), K_FOREVER);
			if (res < 0) {
				errno = -res;
				return -1;
			}

			int signaled = 0, result = -1;

			k_poll_signal_check(&s->readable, &signaled,
					    &result);
			if (signaled && result == USOCK_SIG_CANCEL) {
				/* Peer closed - drain remaining data */
				k_mutex_lock(&s->lock, K_FOREVER);
				avail = usock_read_avail(s);
				if (avail == 0) {
					k_mutex_unlock(&s->lock);
					return 0; /* EOF */
				}
				/* Fall through to read remaining data */
				break;
			}

			k_mutex_lock(&s->lock, K_FOREVER);
			avail = usock_read_avail(s);
			if (avail > 0) {
				break;
			}
			/* Spurious wakeup - check for EOF */
			peer = usock_peer(s);
			if (peer == NULL) {
				k_mutex_unlock(&s->lock);
				return 0; /* EOF */
			}
			k_mutex_unlock(&s->lock);
		}
	}

	res = k_pipe_get(&s->recv_q, buffer, count, &bytes_read,
			 1, K_NO_WAIT);
	__ASSERT(res == 0, "k_pipe_get() failed: %d", res);

	/* Reset readable signal if pipe is now empty and peer is alive */
	if (usock_read_avail(s) == 0 && usock_peer(s) != NULL) {
		k_poll_signal_reset(&s->readable);
	}

	/* Notify peer that we consumed data (space available for write) */
	peer = usock_peer(s);
	if (peer != NULL) {
		k_poll_signal_raise(&peer->writable, USOCK_SIG_DATA);
	}

	k_mutex_unlock(&s->lock);
	return bytes_read;
}

/* ------------------------------------------------------------------ */
/* close                                                              */
/* ------------------------------------------------------------------ */
static int unix_close(void *obj)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct unix_socket *peer;

	if (s == NULL) {
		return 0;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	s->closed = true;
	s->connected = false;

	/* Notify peer so it unblocks from read/write */
	peer = usock_peer(s);
	if (peer != NULL) {
		k_mutex_lock(&peer->lock, K_FOREVER);
		peer->peer_fd = -1;
		peer->connected = false;
		k_poll_signal_raise(&peer->readable, USOCK_SIG_CANCEL);
		k_poll_signal_raise(&peer->writable, USOCK_SIG_CANCEL);
		k_mutex_unlock(&peer->lock);
	}

	/* If listening, wake any blocked accept() */
	if (s->listening) {
		k_sem_give(&s->accept_sem);
	}

	/* Drain accept_queue */
	sys_snode_t *node;

	while ((node = sys_slist_get(&s->accept_queue)) != NULL) {
		struct pending_conn *pc =
			CONTAINER_OF(node, struct pending_conn, node);
		pending_free(pc);
	}

	/* Free any pending datagrams */
	{
		sys_snode_t *node_dgram;
		while ((node_dgram = sys_slist_get(&s->dgram_list)) != NULL) {
			struct unix_dgram *d =
				CONTAINER_OF(node_dgram, struct unix_dgram, node);
			k_free(d);
		}
		s->dgram_queued_bytes = 0;
	}

	/* Clear registration */
	s->bound = false;
	s->listening = false;
	s->peer_fd = -1;

	/* Free the dynamically allocated pipe buffer */
	if (s->pipe_buf != NULL) {
		pipe_buf_free(s->pipe_buf);
		s->pipe_buf = NULL;
	}

	k_mutex_unlock(&s->lock);

	/* Reset the slot so it can be reused */
	k_mutex_lock(&registry_lock, K_FOREVER);
	s->type = 0;
	s->closed = false;
	k_mutex_unlock(&registry_lock);

	return 0;
}

/* ------------------------------------------------------------------ */
/* bind                                                               */
/* ------------------------------------------------------------------ */
static int unix_bind(void *obj, const struct sockaddr *addr,
		     socklen_t addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	const struct sockaddr_un *sun = (const struct sockaddr_un *)addr;
	size_t pathlen;

	if (addr == NULL || addr->sa_family != AF_UNIX) {
		errno = EINVAL;
		return -1;
	}

	pathlen = addrlen - offsetof(struct sockaddr_un, sun_path);
	if (pathlen == 0 || pathlen > sizeof(s->addr.sun_path)) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&registry_lock, K_FOREVER);
	k_mutex_lock(&s->lock, K_FOREVER);

	if (s->bound) {
		k_mutex_unlock(&s->lock);
		k_mutex_unlock(&registry_lock);
		errno = EINVAL;
		return -1;
	}

	/* Check for address conflict */
	if (find_bound_socket(sun->sun_path, pathlen) != NULL) {
		k_mutex_unlock(&s->lock);
		k_mutex_unlock(&registry_lock);
		errno = EADDRINUSE;
		return -1;
	}

	memset(&s->addr, 0, sizeof(s->addr));
	s->addr.sun_family = AF_UNIX;
	memcpy(s->addr.sun_path, sun->sun_path, pathlen);
	s->bound = true;

	k_mutex_unlock(&s->lock);
	k_mutex_unlock(&registry_lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* listen                                                             */
/* ------------------------------------------------------------------ */
static int unix_listen(void *obj, int backlog)
{
	struct unix_socket *s = (struct unix_socket *)obj;

	k_mutex_lock(&s->lock, K_FOREVER);

	if (!s->bound) {
		k_mutex_unlock(&s->lock);
		errno = EINVAL;
		return -1;
	}

	if (s->connected) {
		k_mutex_unlock(&s->lock);
		errno = EINVAL;
		return -1;
	}

	s->listening = true;
	s->backlog = (backlog > 0) ? backlog : 1;

	k_mutex_unlock(&s->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* accept                                                             */
/* ------------------------------------------------------------------ */
static int unix_accept(void *obj, struct sockaddr *addr,
		       socklen_t *addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct pending_conn *pc;
	sys_snode_t *node;
	int client_fd;

	k_mutex_lock(&s->lock, K_FOREVER);

	if (!s->listening) {
		k_mutex_unlock(&s->lock);
		errno = EINVAL;
		return -1;
	}

	/* Try to dequeue a pending connection */
	node = sys_slist_peek_head(&s->accept_queue);
	if (node != NULL) {
		node = sys_slist_get(&s->accept_queue);
		pc = CONTAINER_OF(node, struct pending_conn, node);
		client_fd = pc->client_fd;
		pending_free(pc);

		k_mutex_unlock(&s->lock);
		goto accepted;
	}

	if (usock_is_nonblock(s)) {
		k_mutex_unlock(&s->lock);
		errno = EAGAIN;
		return -1;
	}

	/* Block until a connection arrives */
	k_mutex_unlock(&s->lock);

	for (;;) {
		int ret = k_sem_take(&s->accept_sem, K_FOREVER);

		if (ret < 0) {
			errno = EINTR;
			return -1;
		}

		k_mutex_lock(&s->lock, K_FOREVER);

		if (s->closed) {
			k_mutex_unlock(&s->lock);
			errno = EBADF;
			return -1;
		}

		node = sys_slist_get(&s->accept_queue);
		if (node != NULL) {
			pc = CONTAINER_OF(node, struct pending_conn, node);
			client_fd = pc->client_fd;
			pending_free(pc);
			k_mutex_unlock(&s->lock);
			goto accepted;
		}
		k_mutex_unlock(&s->lock);
		/* Spurious wake, retry */
	}

accepted:
	/*
	 * Create a new server-side endpoint and pair it with the
	 * client endpoint.
	 */
	{
		struct unix_socket *server_ep = usock_alloc();

		if (server_ep == NULL) {
			/* No resources - close the client socket */
			zvfs_close(client_fd);
			errno = ENOMEM;
			return -1;
		}

		if (usock_init(server_ep) != 0) {
			k_mutex_lock(&registry_lock, K_FOREVER);
			server_ep->type = 0;
			k_mutex_unlock(&registry_lock);
			zvfs_close(client_fd);
			errno = ENOMEM;
			return -1;
		}

		int server_fd = zvfs_reserve_fd();

		if (server_fd < 0) {
			/* Free pipe buffer and return the slot */
			pipe_buf_free(server_ep->pipe_buf);
			server_ep->pipe_buf = NULL;
			k_mutex_lock(&registry_lock, K_FOREVER);
			server_ep->type = 0;
			k_mutex_unlock(&registry_lock);
			zvfs_close(client_fd);
			errno = ENFILE;
			return -1;
		}

		zvfs_finalize_typed_fd(server_fd, server_ep,
			(const struct fd_op_vtable *)&unix_fd_op_vtable,
			ZVFS_MODE_IFSOCK);

		/* Wire up the pair */
		struct unix_socket *client_ep =
			zvfs_get_fd_obj(client_fd,
				(const struct fd_op_vtable *)&unix_fd_op_vtable,
				0);

		if (client_ep == NULL) {
			zvfs_close(server_fd);
			zvfs_close(client_fd);
			errno = ECONNABORTED;
			return -1;
		}

		k_mutex_lock(&client_ep->lock, K_FOREVER);
		k_mutex_lock(&server_ep->lock, K_FOREVER);

		server_ep->peer_fd = client_fd;
		server_ep->connected = true;
		client_ep->peer_fd = server_fd;
		client_ep->connected = true;

		/* Both sides are now writable and potentially readable */
		k_poll_signal_raise(&client_ep->readable, USOCK_SIG_DATA);

		k_mutex_unlock(&server_ep->lock);
		k_mutex_unlock(&client_ep->lock);

		/* Fill in caller's addr if requested */
		if (addr != NULL && addrlen != NULL) {
			struct sockaddr_un raddr;

			memset(&raddr, 0, sizeof(raddr));
			raddr.sun_family = AF_UNIX;
			/* client endpoint has no bound name */
			socklen_t len = sizeof(raddr.sun_family);

			if (*addrlen >= len) {
				memcpy(addr, &raddr, len);
			}
			*addrlen = len;
		}

		return server_fd;
	}
}

/* ------------------------------------------------------------------ */
/* connect                                                            */
/* ------------------------------------------------------------------ */
static int unix_connect(void *obj, const struct sockaddr *addr,
			socklen_t addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	const struct sockaddr_un *sun = (const struct sockaddr_un *)addr;
	struct unix_socket *listener;
	struct pending_conn *pc;
	size_t pathlen;

	if (addr == NULL || addr->sa_family != AF_UNIX) {
		errno = EINVAL;
		return -1;
	}

	pathlen = addrlen - offsetof(struct sockaddr_un, sun_path);
	if (pathlen == 0 || pathlen > sizeof(s->addr.sun_path)) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	if (s->connected) {
		k_mutex_unlock(&s->lock);
		errno = EISCONN;
		return -1;
	}

	if (s->listening) {
		k_mutex_unlock(&s->lock);
		errno = EINVAL;
		return -1;
	}

	k_mutex_unlock(&s->lock);

	/* DGRAM connect: just store default target address */
	if (s->type == SOCK_DGRAM) {
		if (s->connected) {
			errno = EISCONN;
			return -1;
		}
		const struct sockaddr_un *sun_d = (const struct sockaddr_un *)addr;
		struct unix_socket *target = find_bound_socket(sun_d->sun_path,
			addrlen - offsetof(struct sockaddr_un, sun_path));
		if (!target) {
			errno = ECONNREFUSED;
			return -1;
		}
		memcpy(&s->dgram_peer, sun_d, sizeof(*sun_d));
		s->dgram_peer_set = true;
		s->connected = true;
		return 0;
	}

	/*
	 * Create a client-side endpoint. This is the fd that will
	 * become connected once accept() picks it up. We reserve an fd
	 * for it and enqueue it on the listener's accept_queue.
	 *
	 * 's' itself becomes the client's local endpoint. We need a
	 * separate endpoint for the server side, which accept() creates.
	 * For now, we register 's' fd into the listener's pending queue,
	 * and accept() will wire up the peer_fd links.
	 *
	 * We need to find the fd for 's' to store in pending_conn.
	 * Unfortunately the obj->fd mapping isn't directly stored on
	 * the obj. Instead, connect is called via VTABLE_CALL which
	 * already knows the fd. But the vtable.connect signature only
	 * gets obj.
	 *
	 * Workaround: we store 's' itself in the pending_conn as fd=-2
	 * (a sentinel), and in accept we identify the client_fd by
	 * scanning the fd table for 's'. OR, simpler: we create a new
	 * endpoint for the client side and pair it back with 's'.
	 *
	 * Actually, the simplest correct approach for SOCK_STREAM:
	 * - The "connecting" socket ('s') will be one end
	 * - accept() creates the other end
	 * - We need to know 's' fd to store in pending_conn
	 *
	 * Since connect is dispatched via VTABLE_CALL, we can search
	 * the fd table for obj 's' to find its fd.
	 */

	/* Find our own fd */
	int my_fd = -1;

	for (int fd = 0; fd < CONFIG_ZVFS_OPEN_MAX; fd++) {
		void *ctx = zvfs_get_fd_obj(fd,
			(const struct fd_op_vtable *)&unix_fd_op_vtable, 0);
		if (ctx == s) {
			my_fd = fd;
			break;
		}
	}

	if (my_fd < 0) {
		errno = EBADF;
		return -1;
	}

	/* Find the listening socket */
	k_mutex_lock(&registry_lock, K_FOREVER);
	listener = find_bound_socket(sun->sun_path, pathlen);
	if (listener == NULL) {
		k_mutex_unlock(&registry_lock);
		errno = ECONNREFUSED;
		return -1;
	}

	k_mutex_lock(&listener->lock, K_FOREVER);

	if (!listener->listening) {
		k_mutex_unlock(&listener->lock);
		k_mutex_unlock(&registry_lock);
		errno = ECONNREFUSED;
		return -1;
	}

	/* Check backlog */
	int queue_len = 0;
	sys_snode_t *n;

	SYS_SLIST_FOR_EACH_NODE(&listener->accept_queue, n) {
		queue_len++;
	}

	if (queue_len >= listener->backlog) {
		k_mutex_unlock(&listener->lock);
		k_mutex_unlock(&registry_lock);
		errno = ECONNREFUSED;
		return -1;
	}

	/* Allocate a pending_conn node */
	pc = pending_alloc();
	if (pc == NULL) {
		k_mutex_unlock(&listener->lock);
		k_mutex_unlock(&registry_lock);
		errno = ENOMEM;
		return -1;
	}

	pc->client_fd = my_fd;
	sys_slist_append(&listener->accept_queue, &pc->node);

	/* Wake the listener's accept() */
	k_sem_give(&listener->accept_sem);
	k_poll_signal_raise(&listener->readable, USOCK_SIG_DATA);

	k_mutex_unlock(&listener->lock);
	k_mutex_unlock(&registry_lock);

	/*
	 * Wait until accept() has wired up our peer_fd.
	 * The accept() handler will set s->connected and s->peer_fd.
	 */
	if (usock_is_nonblock(s)) {
		/*
		 * Non-blocking connect: ideally we'd return EINPROGRESS,
		 * but our accept path is synchronous. By the time
		 * connect() returns, accept() may or may not have run.
		 * For simplicity, spin briefly.
		 */
		for (int tries = 0; tries < 100; tries++) {
			k_mutex_lock(&s->lock, K_FOREVER);
			if (s->connected) {
				k_poll_signal_reset(&s->readable);
				k_mutex_unlock(&s->lock);
				return 0;
			}
			k_mutex_unlock(&s->lock);
			k_yield();
		}
		/* Not yet connected, but queued */
		errno = EINPROGRESS;
		return -1;
	}

	/* Blocking: wait for accept() to wire us up */
	for (;;) {
		struct k_poll_event events[] = {
			K_POLL_EVENT_INITIALIZER(
				K_POLL_TYPE_SIGNAL,
				K_POLL_MODE_NOTIFY_ONLY,
				&s->readable),
		};

		int ret = k_poll(events, ARRAY_SIZE(events),
				 K_MSEC(5000));

		k_mutex_lock(&s->lock, K_FOREVER);
		if (s->connected) {
			k_poll_signal_reset(&s->readable);
			k_mutex_unlock(&s->lock);
			return 0;
		}
		if (s->closed) {
			k_mutex_unlock(&s->lock);
			errno = ECONNREFUSED;
			return -1;
		}
		k_mutex_unlock(&s->lock);

		/* Timeout: listener might be slow, keep waiting */
		if (ret == -EAGAIN) {
			continue;
		}
		if (ret < 0) {
			errno = ECONNREFUSED;
			return -1;
		}
	}
}

/* ------------------------------------------------------------------ */
/* sendto / sendmsg wrappers                                          */
/* ------------------------------------------------------------------ */
static ssize_t unix_sendto(void *obj, const void *buf, size_t len,
			   int flags, const struct sockaddr *dest_addr,
			   socklen_t addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	if (s->type == SOCK_DGRAM) {
		bool nb = usock_is_nonblock(s);
		if (flags & ZSOCK_MSG_DONTWAIT) s->flags |= USOCK_FLAG_NONBLOCK;
		int ret = dgram_sendto(s, buf, len, dest_addr, addrlen);
		if ((flags & ZSOCK_MSG_DONTWAIT) && !nb)
			s->flags &= ~USOCK_FLAG_NONBLOCK;
		return ret;
	}
	/* STREAM path */
	ARG_UNUSED(dest_addr);
	ARG_UNUSED(addrlen);
	bool saved_nonblock;
	ssize_t ret;
	if (flags & ZSOCK_MSG_DONTWAIT) {
		saved_nonblock = usock_is_nonblock(s);
		s->flags |= USOCK_FLAG_NONBLOCK;
		ret = unix_write(obj, buf, len);
		if (!saved_nonblock) {
			s->flags &= ~USOCK_FLAG_NONBLOCK;
		}
		return ret;
	}

	return unix_write(obj, buf, len);
}

static ssize_t unix_sendmsg(void *obj, const struct msghdr *msg,
			    int flags)
{
	ARG_UNUSED(flags);

	struct unix_socket *s = (struct unix_socket *)obj;
	ssize_t total = 0;

	if (s == NULL || msg == NULL) {
		errno = EINVAL;
		return -1;
	}

	for (size_t i = 0; i < msg->msg_iovlen; i++) {
		if (msg->msg_iov[i].iov_len == 0) {
			continue;
		}
		ssize_t ret = unix_write(obj, msg->msg_iov[i].iov_base,
					 msg->msg_iov[i].iov_len);
		if (ret < 0) {
			return (total > 0) ? total : ret;
		}
		total += ret;
	}

	return total;
}

/* ------------------------------------------------------------------ */
/* recvfrom wrapper                                                   */
/* ------------------------------------------------------------------ */
static ssize_t unix_recvfrom(void *obj, void *buf, size_t max_len,
			     int flags, struct sockaddr *src_addr,
			     socklen_t *addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	if (s->type == SOCK_DGRAM) {
		bool nb = usock_is_nonblock(s);
		if (flags & ZSOCK_MSG_DONTWAIT) s->flags |= USOCK_FLAG_NONBLOCK;
		ssize_t ret = dgram_recvfrom(s, buf, max_len, src_addr, addrlen);
		if ((flags & ZSOCK_MSG_DONTWAIT) && !nb)
			s->flags &= ~USOCK_FLAG_NONBLOCK;
		return ret;
	}
	/* STREAM path */
	if (addrlen != NULL) *addrlen = 0;
	bool saved_nonblock;
	ssize_t ret;
	if (flags & ZSOCK_MSG_DONTWAIT) {
		saved_nonblock = usock_is_nonblock(s);
		s->flags |= USOCK_FLAG_NONBLOCK;
		ret = unix_read(obj, buf, max_len);
		if (!saved_nonblock) {
			s->flags &= ~USOCK_FLAG_NONBLOCK;
		}
		return ret;
	}

	return unix_read(obj, buf, max_len);
}

/* ------------------------------------------------------------------ */
/* ioctl (fcntl, poll-prepare, poll-update, fionread, fionbio)        */
/* ------------------------------------------------------------------ */
static int unix_ioctl(void *obj, unsigned int request, va_list args)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	int res;

	if (s == NULL) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	switch (request) {
	case F_GETFL: {
		int fl = 0;

		if (usock_is_nonblock(s)) {
			fl |= O_NONBLOCK;
		}
		res = fl;
		break;
	}

	case F_SETFL: {
		int fl = va_arg(args, int);

		if (fl & O_NONBLOCK) {
			s->flags |= USOCK_FLAG_NONBLOCK;
		} else {
			s->flags &= ~USOCK_FLAG_NONBLOCK;
		}
		res = 0;
		break;
	}

	case ZFD_IOCTL_FIONBIO:
		s->flags |= USOCK_FLAG_NONBLOCK;
		res = 0;
		break;

	case ZFD_IOCTL_FIONREAD: {
		int *nbytes = va_arg(args, int *);

		*nbytes = usock_read_avail(s);
		res = 0;
		break;
	}

	case ZFD_IOCTL_POLL_PREPARE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev =
			va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end =
			va_arg(args, struct k_poll_event *);

		/*
		 * Structural EALREADY: the socket is not connected and
		 * has no peer.  This state is irreversible within a
		 * single fd lifetime, so it is safe to short-circuit
		 * without registering any events.  POLL_UPDATE mirrors
		 * this exact check and skips pev advancement.
		 */
		if (!s->listening && s->peer_fd == -1 && !s->connected) {
			res = -EALREADY;
			break;
		}

		if (pfd->events & ZSOCK_POLLIN) {
			if (*pev == pev_end) {
				res = -ENOMEM;
				break;
			}

			(*pev)->obj = &s->readable;
			(*pev)->type = K_POLL_TYPE_SIGNAL;
			(*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
			(*pev)->state = K_POLL_STATE_NOT_READY;
			(*pev)++;
		}

		if (pfd->events & ZSOCK_POLLOUT) {
			if (*pev == pev_end) {
				res = -ENOMEM;
				break;
			}

			(*pev)->obj = &s->writable;
			(*pev)->type = K_POLL_TYPE_SIGNAL;
			(*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
			(*pev)->state = K_POLL_STATE_NOT_READY;
			(*pev)++;
		}

		res = 0;
		break;
	}

	case ZFD_IOCTL_POLL_UPDATE: {
		struct zsock_pollfd *pfd = va_arg(args, struct zsock_pollfd *);
		struct k_poll_event **pev =
			va_arg(args, struct k_poll_event **);

		/*
		 * Mirror the structural EALREADY check: no events were
		 * registered, so do not advance pev.
		 */
		if (!s->listening && s->peer_fd == -1 &&
		    !s->connected) {
			pfd->revents |= ZSOCK_POLLHUP;
			res = 0;
			break;
		}

		/*
		 * Normal path: advance (*pev) once per event that
		 * POLL_PREPARE registered.
		 */
		if (pfd->events & ZSOCK_POLLIN) {
			if (s->listening) {
				if (!sys_slist_is_empty(&s->accept_queue)) {
					pfd->revents |= ZSOCK_POLLIN;
				}
			} else if (usock_read_avail(s) > 0) {
				pfd->revents |= ZSOCK_POLLIN;
			} else if (s->peer_fd == -1) {
				/* Peer disconnected after prepare */
				pfd->revents |= ZSOCK_POLLHUP;
			}
			(*pev)++;
		}

		if (pfd->events & ZSOCK_POLLOUT) {
			if (!s->connected ||
			    (s->flags & USOCK_FLAG_SHUT_WR)) {
				pfd->revents |= ZSOCK_POLLHUP;
			} else if (usock_write_avail(s) > 0) {
				pfd->revents |= ZSOCK_POLLOUT;
			}
			(*pev)++;
		}

		/* Report peer disconnect as HUP unconditionally */
		if (s->connected && s->peer_fd == -1) {
			pfd->revents |= ZSOCK_POLLHUP;
		}

		res = 0;
		break;
	}

	default:
		errno = EOPNOTSUPP;
		res = -1;
		break;
	}

	k_mutex_unlock(&s->lock);
	return res;
}

/* ------------------------------------------------------------------ */
/* getsockopt / setsockopt stubs                                      */
/* ------------------------------------------------------------------ */
static int unix_getsockopt(void *obj, int level, int optname,
			   void *optval, socklen_t *optlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;

	if (level != SOL_SOCKET) {
		errno = ENOPROTOOPT;
		return -1;
	}

	switch (optname) {
	case SO_TYPE:
		if (optval == NULL || optlen == NULL ||
		    *optlen < sizeof(int)) {
			errno = EINVAL;
			return -1;
		}
		*(int *)optval = s->type;
		*optlen = sizeof(int);
		return 0;

	case SO_ERROR:
		if (optval == NULL || optlen == NULL ||
		    *optlen < sizeof(int)) {
			errno = EINVAL;
			return -1;
		}
		/* AF_UNIX has no async error queue; always 0 */
		*(int *)optval = 0;
		*optlen = sizeof(int);
		return 0;

	case SO_RCVBUF:
		if (optval == NULL || optlen == NULL ||
		    *optlen < sizeof(int)) {
			errno = EINVAL;
			return -1;
		}
		*(int *)optval = CONFIG_NET_UNIX_BUFFER_SIZE;
		*optlen = sizeof(int);
		return 0;

	case SO_SNDBUF:
		if (optval == NULL || optlen == NULL ||
		    *optlen < sizeof(int)) {
			errno = EINVAL;
			return -1;
		}
		/* Send buffer = peer's recv pipe size */
		*(int *)optval = CONFIG_NET_UNIX_BUFFER_SIZE;
		*optlen = sizeof(int);
		return 0;

	default:
		errno = ENOPROTOOPT;
		return -1;
	}
}

static int unix_setsockopt(void *obj, int level, int optname,
			   const void *optval, socklen_t optlen)
{
	ARG_UNUSED(obj);
	ARG_UNUSED(optval);
	ARG_UNUSED(optlen);

	if (level != SOL_SOCKET) {
		errno = ENOPROTOOPT;
		return -1;
	}

	switch (optname) {
	case SO_RCVBUF:
	case SO_SNDBUF:
		/* Accept but ignore; buffer size is fixed at build time */
		return 0;

	default:
		errno = ENOPROTOOPT;
		return -1;
	}
}

/* ------------------------------------------------------------------ */
/* getpeername / getsockname                                          */
/* ------------------------------------------------------------------ */
static int unix_getpeername(void *obj, struct sockaddr *addr,
			    socklen_t *addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct unix_socket *peer;

	if (addr == NULL || addrlen == NULL) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	if (!s->connected) {
		k_mutex_unlock(&s->lock);
		errno = ENOTCONN;
		return -1;
	}

	peer = usock_peer(s);
	if (peer == NULL) {
		k_mutex_unlock(&s->lock);
		errno = ENOTCONN;
		return -1;
	}

	struct sockaddr_un paddr;

	memset(&paddr, 0, sizeof(paddr));
	paddr.sun_family = AF_UNIX;
	if (peer->bound) {
		memcpy(paddr.sun_path, peer->addr.sun_path,
		       sizeof(paddr.sun_path));
	}

	socklen_t len = sizeof(paddr);

	if (*addrlen > len) {
		*addrlen = len;
	}
	memcpy(addr, &paddr, *addrlen);

	k_mutex_unlock(&s->lock);
	return 0;
}

static int unix_getsockname(void *obj, struct sockaddr *addr,
			    socklen_t *addrlen)
{
	struct unix_socket *s = (struct unix_socket *)obj;

	if (addr == NULL || addrlen == NULL) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	struct sockaddr_un saddr;

	memset(&saddr, 0, sizeof(saddr));
	saddr.sun_family = AF_UNIX;
	if (s->bound) {
		memcpy(saddr.sun_path, s->addr.sun_path,
		       sizeof(saddr.sun_path));
	}

	socklen_t len = sizeof(saddr);

	if (*addrlen > len) {
		*addrlen = len;
	}
	memcpy(addr, &saddr, *addrlen);

	k_mutex_unlock(&s->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* shutdown                                                           */
/* ------------------------------------------------------------------ */
static int unix_shutdown(void *obj, int how)
{
	struct unix_socket *s = (struct unix_socket *)obj;
	struct unix_socket *peer;

	if (s == NULL) {
		errno = EINVAL;
		return -1;
	}

	k_mutex_lock(&s->lock, K_FOREVER);

	if (!s->connected && !s->listening) {
		k_mutex_unlock(&s->lock);
		errno = ENOTCONN;
		return -1;
	}

	switch (how) {
	case ZSOCK_SHUT_RD:
		s->flags |= USOCK_FLAG_SHUT_RD;
		/* Wake any blocked reader with EOF */
		k_poll_signal_raise(&s->readable, USOCK_SIG_CANCEL);
		break;

	case ZSOCK_SHUT_WR:
		s->flags |= USOCK_FLAG_SHUT_WR;
		/* Notify peer that no more data is coming */
		peer = usock_peer(s);
		if (peer != NULL) {
			k_poll_signal_raise(&peer->readable,
					    USOCK_SIG_CANCEL);
		}
		break;

	case ZSOCK_SHUT_RDWR:
		s->flags |= USOCK_FLAG_SHUT_RD | USOCK_FLAG_SHUT_WR;
		k_poll_signal_raise(&s->readable, USOCK_SIG_CANCEL);
		peer = usock_peer(s);
		if (peer != NULL) {
			k_poll_signal_raise(&peer->readable,
					    USOCK_SIG_CANCEL);
		}
		break;

	default:
		k_mutex_unlock(&s->lock);
		errno = EINVAL;
		return -1;
	}

	k_mutex_unlock(&s->lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* vtable                                                             */
/* ------------------------------------------------------------------ */
static const struct socket_op_vtable unix_fd_op_vtable = {
	.fd_vtable = {
		.read = unix_read,
		.write = unix_write,
		.close = unix_close,
		.ioctl = unix_ioctl,
	},
	.shutdown = unix_shutdown,
	.bind = unix_bind,
	.connect = unix_connect,
	.listen = unix_listen,
	.accept = unix_accept,
	.sendto = unix_sendto,
	.sendmsg = unix_sendmsg,
	.recvfrom = unix_recvfrom,
	.getsockopt = unix_getsockopt,
	.setsockopt = unix_setsockopt,
	.getpeername = unix_getpeername,
	.getsockname = unix_getsockname,
};

/* ------------------------------------------------------------------ */
/* socket creation entry point                                        */
/* ------------------------------------------------------------------ */
int unix_socket_create(int family, int type, int proto)
{
	struct unix_socket *s;
	int fd;

	if (family != AF_UNIX) {
		errno = EAFNOSUPPORT;
		return -1;
	}

	if (type != SOCK_STREAM && type != SOCK_DGRAM) {
		errno = EPROTOTYPE;
		return -1;
	}

	if (proto != 0) {
		errno = EPROTONOSUPPORT;
		return -1;
	}

	s = usock_alloc();
	if (s == NULL) {
		errno = ENOMEM;
		return -1;
	}

	if (usock_init(s) != 0) {
		k_mutex_lock(&registry_lock, K_FOREVER);
		s->type = 0;
		k_mutex_unlock(&registry_lock);
		errno = ENOMEM;
		return -1;
	}
	s->type = type;

	fd = zvfs_reserve_fd();
	if (fd < 0) {
		/* Free pipe buffer and return the slot */
		pipe_buf_free(s->pipe_buf);
		s->pipe_buf = NULL;
		k_mutex_lock(&registry_lock, K_FOREVER);
		s->type = 0;
		k_mutex_unlock(&registry_lock);
		errno = ENFILE;
		return -1;
	}

	zvfs_finalize_typed_fd(fd, s,
		(const struct fd_op_vtable *)&unix_fd_op_vtable,
		ZVFS_MODE_IFSOCK);

	LOG_DBG("AF_UNIX socket created: fd=%d", fd);
	return fd;
}

/* ------------------------------------------------------------------ */
/* is_supported callback for NET_SOCKET_REGISTER                      */
/* ------------------------------------------------------------------ */
static bool unix_is_supported(int family, int type, int proto)
{
	return (family == AF_UNIX) && (type == SOCK_STREAM || type == SOCK_DGRAM) &&
	       (proto == 0);
}

/* ------------------------------------------------------------------ */
/* Register AF_UNIX with the zsock_socket() dispatcher                */
/* ------------------------------------------------------------------ */
NET_SOCKET_REGISTER(af_unix, NET_SOCKET_DEFAULT_PRIO, AF_UNIX,
		    unix_is_supported, unix_socket_create);

/* ------------------------------------------------------------------ */
/* Module initialization                                              */
/* ------------------------------------------------------------------ */
static int af_unix_init(void)
{
	k_mutex_init(&registry_lock);
	k_mutex_init(&pending_pool_lock);

	for (int i = 0; i < MAX_UNIX_SOCKETS; i++) {
		memset(&unix_sockets[i], 0, sizeof(unix_sockets[i]));
		unix_sockets[i].pipe_buf = NULL;
	}

	for (int i = 0; i < MAX_PENDING_CONNS; i++) {
		pending_pool[i].client_fd = -1;
	}

	LOG_INF("AF_UNIX socket layer initialized (max=%d, buf=%d, "
		"dynamic alloc)",
		MAX_UNIX_SOCKETS, CONFIG_NET_UNIX_BUFFER_SIZE);

	return 0;
}

SYS_INIT(af_unix_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
