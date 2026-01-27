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
};

#define SHELL_WOLFSSH_DEFINE(_name)	\
	static struct shell_wolfssh _name##_shell_wolfssh;		\
	struct shell_transport _name = {				\
		.api = &shell_wolfssh_transport_api,		\
		.ctx = (struct shell_wolfssh *)&_name##_shell_wolfssh	\
	}

void enable_ssh_shell();

void disable_ssh_shell();

bool is_ssh_shell_disabled();

void set_ssh_port(uint16_t port);

uint16_t get_ssh_port();

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SHELL_SSH_H_ */