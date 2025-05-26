#define WOLFSSH_ZEPHYR
#include "../../../samples/linkedsemi/shell_wolfssh/wolfssl_user_settings_nofs.h"
#include "../../../samples/linkedsemi/shell_wolfssh/wolfssh_user_settings_nofs.h"

#include <zephyr/init.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_service.h>

#include <zephyr/logging/log.h>
#define LOG_MODULE_NAME shell_wolfssh
LOG_MODULE_REGISTER(shell_wolfssh);

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <wolfssl/wolfcrypt/settings.h>
#include <zephyr/shell/shell_wolfssh.h>

#include <stdio.h>

#define WOLFSSH_TEST_SERVER
#define WOLFSSH_TEST_ECHOSERVER

#include <wolfssl/wolfcrypt/hash.h>
#include <wolfssl/wolfcrypt/coding.h>

#include <wolfssl/wolfcrypt/wc_port.h>
#include <wolfssl/wolfcrypt/asn.h>
#include <wolfssl/wolfcrypt/asn_public.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include <wolfssh/ssh.h>
#include <wolfssh/internal.h>
#include <wolfssh/wolfsftp.h>
#include <wolfssh/agent.h>
#include <wolfssh/test.h>
#include <wolfssl/wolfcrypt/ecc.h>

#ifdef NO_FILESYSTEM
#include <wolfssh/certs_test.h>
#endif

#include <errno.h>
#define SOCKET_ERRNO        errno
#define SOCKET_ECONNRESET   ECONNRESET
#define SOCKET_ECONNABORTED ECONNABORTED
#define SOCKET_EWOULDBLOCK  EWOULDBLOCK

static const char echoserverBanner[] = "shell wolfssh example\n";

#define MAX_PASSWD_RETRY 3
static int passwdRetry = MAX_PASSWD_RETRY;

#define EXAMPLE_BUFFER_SZ 4096

#define EXAMPLE_KEYLOAD_BUFFER_SZ 1200

static int load_key(byte isEcc, byte *buf, word32 bufSz);

typedef struct {
	WOLFSSH *ssh;
	WS_SOCKET_T fd;
	word32 id;
	char statsBuffer[EXAMPLE_BUFFER_SZ];
} thread_ctx_t;

typedef void ES_HEAP_HINT;

/* returns buffer size on success */
static int load_key(byte isEcc, byte *buf, word32 bufSz)
{
	word32 sz = 0;

	/* using buffers instead */
	if (isEcc) {
		if ((word32)sizeof_ecc_key_der_256 > bufSz) {
			return 0;
		}
		WMEMCPY(buf, ecc_key_der_256, sizeof_ecc_key_der_256);
		sz = sizeof_ecc_key_der_256;
	} else {
		if ((word32)sizeof_rsa_key_der_2048 > bufSz) {
			return 0;
		}
		WMEMCPY(buf, (byte *)rsa_key_der_2048, sizeof_rsa_key_der_2048);
		sz = sizeof_rsa_key_der_2048;
	}

	return sz;
}

/* Map user names to passwords */
/* Use arrays for username and p. The password or public key can
 * be hashed and the hash stored here. Then I won't need the type. */
typedef struct PwMap {
	byte type;
	byte username[32];
	word32 usernameSz;
	byte p[WC_SHA256_DIGEST_SIZE];
	struct PwMap *next;
} PwMap;

typedef struct PwMapList {
	PwMap *head;
} PwMapList;

static PwMap *PwMapNew(PwMapList *list, byte type, const byte *username, word32 usernameSz,
		       const byte *p, word32 pSz)
{
	PwMap *map;
	map = (PwMap *)WMALLOC(sizeof(PwMap), NULL, 0);
	if (map != NULL) {
		map->type = type;
		if (usernameSz >= sizeof(map->username)) {
			usernameSz = sizeof(map->username) - 1;
		}
		WMEMCPY(map->username, username, usernameSz + 1);
		map->username[usernameSz] = 0;
		map->usernameSz = usernameSz;

		if (type != WOLFSSH_USERAUTH_NONE) {
			wc_Sha256Hash(p, pSz, map->p);
		}

		map->next = list->head;
		list->head = map;
	}

	return map;
}

static const char samplePasswordBuffer[] = "jill:upthehill\n"
					   "jack:fetchapail\n"
					   "linkedsemi:linkedsemi1\n";

static int LoadPasswordBuffer(byte *buf, word32 bufSz, PwMapList *list)
{
	char *str = (char *)buf;
	char *delimiter;
	char *username;
	char *password;

	/* Each line of passwd.txt is in the format
	 *     username:password\n
	 * This function modifies the passed-in buffer. */

	if (list == NULL) {
		return -1;
	}

	if (buf == NULL || bufSz == 0) {
		return 0;
	}

	while (*str != 0) {
		delimiter = WSTRCHR(str, ':');
		if (delimiter == NULL) {
			return -1;
		}
		username = str;
		*delimiter = 0;
		password = delimiter + 1;
		str = WSTRCHR(password, '\n');
		if (str == NULL) {
			return -1;
		}
		*str = 0;
		str++;
		if (PwMapNew(list, WOLFSSH_USERAUTH_PASSWORD, (byte *)username,
			     (word32)WSTRLEN(username), (byte *)password,
			     (word32)WSTRLEN(password)) == NULL) {

			return -1;
		}
	}

	return 0;
}

static int userAuthWouldBlock = 0;
static int wsUserAuth(byte authType, WS_UserAuthData *authData, void *ctx)
{
	PwMapList *list;
	PwMap *map;
	byte authHash[WC_SHA256_DIGEST_SIZE];

	if (ctx == NULL) {
		fprintf(stderr, "wsUserAuth: ctx not set");
		return WOLFSSH_USERAUTH_FAILURE;
	}

	if (userAuthWouldBlock > 0) {
		LOG_DBG("User Auth would block ....\n");
		userAuthWouldBlock--;
		return WOLFSSH_USERAUTH_WOULD_BLOCK;
	}

	if (authType != WOLFSSH_USERAUTH_PASSWORD && authType != WOLFSSH_USERAUTH_PUBLICKEY) {
		return WOLFSSH_USERAUTH_FAILURE;
	}

	if (authType == WOLFSSH_USERAUTH_PASSWORD) {
		wc_Sha256Hash(authData->sf.password.password, authData->sf.password.passwordSz,
			      authHash);
	} else if (authType == WOLFSSH_USERAUTH_PUBLICKEY) {
		wc_Sha256Hash(authData->sf.publicKey.publicKey, authData->sf.publicKey.publicKeySz,
			      authHash);
	}

	list = (PwMapList *)ctx;
	map = list->head;

	while (map != NULL) {
		if (authData->usernameSz == map->usernameSz &&
		    WMEMCMP(authData->username, map->username, map->usernameSz) == 0 &&
		    authData->type == map->type) {

			if (authData->type == WOLFSSH_USERAUTH_PUBLICKEY) {
				if (WMEMCMP(map->p, authHash, WC_SHA256_DIGEST_SIZE) == 0) {
					return WOLFSSH_USERAUTH_SUCCESS;
				} else {
					return WOLFSSH_USERAUTH_INVALID_PUBLICKEY;
				}
			} else if (authData->type == WOLFSSH_USERAUTH_PASSWORD) {
				if (WMEMCMP(map->p, authHash, WC_SHA256_DIGEST_SIZE) == 0) {
					return WOLFSSH_USERAUTH_SUCCESS;
				} else {
					passwdRetry--;
					return (passwdRetry > 0) ? WOLFSSH_USERAUTH_INVALID_PASSWORD
								 : WOLFSSH_USERAUTH_REJECTED;
				}
			} else {
				return WOLFSSH_USERAUTH_INVALID_AUTHTYPE;
			}
		}
		map = map->next;
	}

	return WOLFSSH_USERAUTH_INVALID_USER;
}

static byte find_char(const byte *str, const byte *buf, word32 bufSz)
{
	const byte *cur;

	while (bufSz) {
		cur = str;
		while (*cur != '\0') {
			if (*cur == *buf) {
				return *cur;
			}
			cur++;
		}
		buf++;
		bufSz--;
	}

	return 0;
}

static int dump_stats(thread_ctx_t *ctx)
{
	word32 statsSz;
	word32 txCount, rxCount, seq, peerSeq;

	wolfSSH_GetStats(ctx->ssh, &txCount, &rxCount, &seq, &peerSeq);

	WSNPRINTF(ctx->statsBuffer, sizeof ctx->statsBuffer,
		  "Statistics for Thread #%u:\r\n"
		  "  txCount = %u\r\n  rxCount = %u\r\n"
		  "  seq = %u\r\n  peerSeq = %u\r\n",
		  ctx->id, txCount, rxCount, seq, peerSeq);
	statsSz = (word32)WSTRLEN(ctx->statsBuffer);

	fprintf(stderr, "%s", ctx->statsBuffer);
	return wolfSSH_stream_send(ctx->ssh, (byte *)ctx->statsBuffer, statsSz);
}

static int process_bytes(thread_ctx_t *threadCtx, const byte *buffer, word32 bufferSz)
{
	int stop = 0;
	byte c;
	const byte matches[] = {0x03, 0x05, 0x06, 0x00};

	c = find_char(matches, buffer, bufferSz);
	switch (c) {
	case 0x03:
		stop = 1;
		break;
	case 0x05:
		if (dump_stats(threadCtx) <= 0) {
			stop = 1;
		}
		break;
	case 0x06:
		if (wolfSSH_TriggerKeyExchange(threadCtx->ssh) != WS_SUCCESS) {
			stop = 1;
		}
		break;
	}
	return stop;
}

struct shell_ssh *sh_ssh;
thread_ctx_t *threadCtx;

static void ssh_server_cb(struct net_socket_service_event *evt);
NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(ssh_server, ssh_server_cb, SHELL_WOLFSSH_POLLFD_COUNT);

static void ssh_server_cb(struct net_socket_service_event *evt)
{
	WOLFSSH *ssh = threadCtx->ssh;
	int ret, error;
	word32 lastChannel = 0;
	static const word32 shellChannelId = 0;
	ret = wolfSSH_worker(ssh, &lastChannel);
	if (ret < 0) {
		error = wolfSSH_get_error(ssh);
		if (error == WS_CHAN_RXD) {
			if (lastChannel == shellChannelId) {
				int readBytes = wolfSSH_ChannelIdRead(
					ssh, shellChannelId, sh_ssh->rx_buf, sizeof sh_ssh->rx_buf);
				if (readBytes > 0) {
					if (sh_ssh->rx_len + readBytes <= EXAMPLE_BUFFER_SZ) {
						memmove(sh_ssh->rx_buf + sh_ssh->rx_len,
							sh_ssh->rx_buf, readBytes);

						sh_ssh->rx_len += readBytes;

						LOG_DBG(" received %d bytes, total buffer length: "
						       "%d\n",
						       readBytes, sh_ssh->rx_len);

						sh_ssh->shell_handler(SHELL_TRANSPORT_EVT_RX_RDY,
								      sh_ssh->shell_context);
					} else {
						LOG_DBG("Buffer overflow, discarding data\n");
						sh_ssh->rx_len = 0;
					}
				}
			}
		}
	}
}

static void server_worker(thread_ctx_t *thread_ctx, WS_SOCKET_T clientFd)
{
	int ret = 0;

	passwdRetry = MAX_PASSWD_RETRY;

	ret = wolfSSH_accept(thread_ctx->ssh);

	if (wolfSSH_get_error(thread_ctx->ssh) == WS_AUTH_PENDING) {
		LOG_ERR("Auth pending error, use -N for non blocking\n");
		LOG_ERR("Trying to close down the connection\n");
	}

	switch (ret) {
	case WS_SCP_COMPLETE:
		LOG_DBG("scp file transfer completed\n");
		ret = 0;
		break;

	case WS_SUCCESS:
		sh_ssh->fds[0].fd = clientFd;
		sh_ssh->fds[0].events = ZSOCK_POLLIN;
		ret = net_socket_service_register(&ssh_server, sh_ssh->fds, ARRAY_SIZE(sh_ssh->fds),
						  NULL);
		if (ret < 0) {
			LOG_ERR("Failed to register socket service, %d", ret);
		}
		break;
	}
}

static void create_tcp_connection()
{
	WOLFSSH_CTX *ctx = NULL;
	PwMapList pwMapList;

	WS_SOCKET_T listenFd = WOLFSSH_SOCKET_INVALID;
	word32 threadCount = 0;
	ES_HEAP_HINT *heap = NULL;
	int peerEcc = 0;
	word16 port = wolfSshPort;

	if (wolfSSH_Init() != WS_SUCCESS) {
		LOG_ERR("Couldn't initialize wolfSSH.\n");
	}

	ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_SERVER, heap);
	if (ctx == NULL) {
		LOG_ERR("Couldn't allocate SSH CTX data.\n");
	}

	WMEMSET(&pwMapList, 0, sizeof(pwMapList));

	wolfSSH_SetUserAuth(ctx, wsUserAuth);

	wolfSSH_CTX_SetBanner(ctx, echoserverBanner);

	byte buf[EXAMPLE_KEYLOAD_BUFFER_SZ];
	byte *keyLoadBuf;
	word32 bufSz;

	keyLoadBuf = buf;
	peerEcc = !peerEcc;
	bufSz = EXAMPLE_KEYLOAD_BUFFER_SZ;
	bufSz = load_key(peerEcc, keyLoadBuf, bufSz);
	if (bufSz == 0) {
		LOG_ERR("Couldn't load second key file.\n");
	}
	if (wolfSSH_CTX_UsePrivateKey_buffer(ctx, keyLoadBuf, bufSz, WOLFSSH_FORMAT_ASN1) < 0) {
		LOG_ERR("Couldn't use second key buffer.\n");
	}
	bufSz = (word32)WSTRLEN(samplePasswordBuffer);
	WMEMCPY(keyLoadBuf, samplePasswordBuffer, bufSz);
	keyLoadBuf[bufSz] = 0;
	LoadPasswordBuffer(keyLoadBuf, bufSz, &pwMapList);

	tcp_listen(&listenFd, &port, 1);

	WS_SOCKET_T clientFd = WOLFSSH_SOCKET_INVALID;

	SOCKADDR_IN_T clientAddr;
	socklen_t clientAddrSz = sizeof(clientAddr);

	WOLFSSH *ssh;

	threadCtx = (thread_ctx_t *)WMALLOC(sizeof(thread_ctx_t), NULL, 0);
	if (threadCtx == NULL) {
		LOG_ERR("Couldn't allocate thread context data.\n");
	}
	WMEMSET(threadCtx, 0, sizeof *threadCtx);

	ssh = wolfSSH_new(ctx);
	if (ssh == NULL) {
		WFREE(threadCtx, NULL, 0);
		LOG_ERR("Couldn't allocate SSH data.\n");
	}
	wolfSSH_SetUserAuthCtx(ssh, &pwMapList);

	printf("Start accepting client connections\n");
	clientFd = accept(listenFd, (struct sockaddr *)&clientAddr, &clientAddrSz);

	if (clientFd == -1) {
		LOG_ERR("tcp accept failed");
	} else {
		LOG_DBG("tcp accept success\n");
	}

	wolfSSH_set_fd(ssh, (int)clientFd);
	threadCtx->ssh = ssh;
	threadCtx->fd = clientFd;
	threadCtx->id = threadCount++;

	server_worker(threadCtx, clientFd);

	if (listenFd != WOLFSSH_SOCKET_INVALID) {
		WCLOSESOCKET(listenFd);
	}
}

static int init(const struct shell_transport *transport, const void *config,
		shell_transport_handler_t evt_handler, void *context)
{
	sh_ssh = (struct shell_ssh *)transport->ctx;

	memset(sh_ssh, 0, sizeof(struct shell_ssh));

	for (int i = 0; i < ARRAY_SIZE(sh_ssh->fds); i++) {
		sh_ssh->fds[i].fd = -1;
	}

	sh_ssh->shell_handler = evt_handler;
	sh_ssh->shell_context = context;

	return 0;
}

static int uninit(const struct shell_transport *transport)
{
	return 0;
}

static int enable(const struct shell_transport *transport, bool blocking_tx)
{
	create_tcp_connection();
	return 0;
}

static int shell_write(const struct shell_transport *transport, const void *data, size_t length,
		       size_t *cnt)
{
	if (sh_ssh == NULL) {
		*cnt = 0;
		return -ENODEV;
	}

	if (sh_ssh->fds[0].fd < 0) {
		*cnt = length;
		return 0;
	}

	LOG_DBG("shell_write Data to send (len=%zu): ", length);
	for (size_t i = 0; i < length; i++) {
		LOG_DBG("%c", ((char *)data)[i]); // 按字符打印
	}
	LOG_DBG("\n");

	size_t ret;

	ret = wolfSSH_ChannelIdSend(threadCtx->ssh, 0, (byte *)data, (word32)length);

	*cnt = ret;
	sh_ssh->shell_handler(SHELL_TRANSPORT_EVT_TX_RDY, sh_ssh->shell_context);

	return 0;
}

static int shell_read(const struct shell_transport *transport, void *data, size_t length,
		      size_t *cnt)
{
	size_t read_len;

	if (sh_ssh == NULL) {
		return -ENODEV;
	}

	if (sh_ssh->fds[0].fd < 0) {
		goto no_data;
	}

	k_mutex_lock(&sh_ssh->rx_lock, K_FOREVER);

	if (sh_ssh->rx_len == 0) {
		k_mutex_unlock(&sh_ssh->rx_lock);
		goto no_data;
	}

	read_len = sh_ssh->rx_len;
	if (read_len > length) {
		read_len = length;
	}

	memcpy(data, sh_ssh->rx_buf, read_len);
	*cnt = read_len;

	sh_ssh->rx_len -= read_len;
	if (sh_ssh->rx_len) {
		memmove(sh_ssh->rx_buf, sh_ssh->rx_buf + read_len, sh_ssh->rx_len);
	}

	if (process_bytes(threadCtx, sh_ssh->rx_buf, read_len)) {
		for (int i = 0; i < ARRAY_SIZE(sh_ssh->fds); i++) {
			sh_ssh->fds[i].fd = -1;
		}

		net_socket_service_unregister(&ssh_server);

		wolfSSH_shutdown(threadCtx->ssh);

		if (threadCtx->fd != -1) {
			WCLOSESOCKET(threadCtx->fd);
			threadCtx->fd = -1;
		}

		wolfSSH_free(threadCtx->ssh);

		WFREE(threadCtx, NULL, 0);
		LOG_DBG("channel closed\n");
	}

	k_mutex_unlock(&sh_ssh->rx_lock);

	return 0;

no_data:
	*cnt = 0;
	return 0;
}

const struct shell_transport_api shell_ssh_transport_api = {
	.init = init,
	.uninit = uninit,
	.enable = enable,
	.write = shell_write,
	.read = shell_read,
};

SHELL_SSH_DEFINE(shell_transport_ssh);
SHELL_DEFINE(shell_ssh, CONFIG_SHELL_PROMPT_SSH, &shell_transport_ssh,
	     CONFIG_SHELL_BACKEND_WOLFSSH_LOG_MESSAGE_QUEUE_SIZE,
	     CONFIG_SHELL_BACKEND_WOLFSSH_LOG_MESSAGE_QUEUE_TIMEOUT, SHELL_FLAG_OLF_CRLF);

static int enable_shell_ssh(void)
{
	bool log_backend = CONFIG_SHELL_BACKEND_WOLFSSH_LOG_LEVEL > 0;
	uint32_t level = (CONFIG_SHELL_BACKEND_WOLFSSH_LOG_LEVEL > LOG_LEVEL_DBG)
				 ? CONFIG_LOG_MAX_LEVEL
				 : CONFIG_SHELL_BACKEND_WOLFSSH_LOG_LEVEL;

	static const struct shell_backend_config_flags cfg_flags =
		SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;

	return shell_init(&shell_ssh, NULL, cfg_flags, log_backend, level);
}

SYS_INIT(enable_shell_ssh, POST_KERNEL, CONFIG_SHELL_BACKEND_WOLFSSH_INIT_PRIORITY);