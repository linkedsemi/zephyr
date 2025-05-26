#ifndef ZEPHYR_INCLUDE_SHELL_SSH_H_
#define ZEPHYR_INCLUDE_SHELL_SSH_H_
#include <zephyr/shell/shell.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const struct shell_transport_api shell_ssh_transport_api;

#define SHELL_WOLFSSH_POLLFD_COUNT 1
#define EXAMPLE_BUFFER_SZ 4096

// // /** SSH-based shell transport using wolfSSH library. */
struct shell_ssh {
	/** Handler function registered by shell. */
    shell_transport_handler_t shell_handler;

	/** Context registered by shell. */
    void *shell_context;

	/** Array for sockets used by the wolfssh service. */
	struct zsock_pollfd fds[SHELL_WOLFSSH_POLLFD_COUNT];

	/** Number of data bytes within the input buffer. */
	size_t rx_len;

	/** Input buffer. */
	byte rx_buf[EXAMPLE_BUFFER_SZ];

	/** Mutex protecting the input buffer access. */
	struct k_mutex rx_lock;
};

#define SHELL_SSH_DEFINE(_name)	\
	static struct shell_ssh _name##_shell_ssh;		\
	struct shell_transport _name = {				\
		.api = &shell_ssh_transport_api,		\
		.ctx = (struct shell_ssh *)&_name##_shell_ssh	\
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SHELL_SSH_H_ */