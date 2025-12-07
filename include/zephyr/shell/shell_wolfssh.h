#ifndef ZEPHYR_INCLUDE_SHELL_SSH_H_
#define ZEPHYR_INCLUDE_SHELL_SSH_H_
#include <zephyr/shell/shell.h>
#ifndef WOLFSSL_USER_SETTINGS
#error WOLFSSL_USER_SETTINGS not defined
#endif
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssh/ssh.h>
#ifdef __cplusplus
extern "C" {
#endif

extern const struct shell_transport_api shell_wolfssh_transport_api;

#define SHELL_WOLFSSH_POLLFD_COUNT 1
// // /** SSH-based shell transport using wolfSSH library. */
struct shell_wolfssh {
	/** Handler function registered by shell. */
    shell_transport_handler_t shell_handler;

	/** Context registered by shell. */
    void *shell_context;

	WOLFSSH *ssh;
	
	WOLFSSH_CHANNEL *local_channel;

	struct k_mutex ssh_lock;

	struct k_timer timer;
	// /** Array for sockets used by the wolfssh service. */
	// struct zsock_pollfd fds[SHELL_WOLFSSH_POLLFD_COUNT];

	// /** Number of data bytes within the input buffer. */
	// size_t rx_len;

	/** Input buffer. */
	// byte rx_buf[SHELL_WOLFSSH_RX_BUF_SIZE];

	// /** Mutex protecting the input buffer access. */
	// struct k_mutex rx_lock;
};

#define SHELL_WOLFSSH_DEFINE(_name)	\
	static struct shell_wolfssh _name##_shell_wolfssh;		\
	struct shell_transport _name = {				\
		.api = &shell_wolfssh_transport_api,		\
		.ctx = (struct shell_wolfssh *)&_name##_shell_wolfssh	\
	}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SHELL_SSH_H_ */