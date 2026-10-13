#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_service.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell_wolfssh.h>

#define WFD_SET_TYPE fd_set
#define WFD_SET FD_SET
#define WFD_ZERO FD_ZERO
#define WFD_ISSET FD_ISSET

LOG_MODULE_REGISTER(shell_wolfssh_backend);

static K_KERNEL_STACK_DEFINE(ssh_daemon_stack,CONFIG_SHELL_WOLFSSH_DAEMON_STACK_SIZE);
static struct k_thread ssh_daemon_thread;
#define SHELL_SSH_DAEMON_THREAD_PRIORITY \
	COND_CODE_1(CONFIG_SHELL_THREAD_PRIORITY_OVERRIDE, \
			(CONFIG_SHELL_THREAD_PRIORITY), (K_LOWEST_APPLICATION_THREAD_PRIO))
#define SHELL_CHANNEL_ID 0

void wolfSSL_Debugging_ON(void);

/* Minimal SCP server support (see shell_wolfssh_scp.c). The implementations
 * live in their own translation unit so that they can define WOLFSSH_ZEPHYR
 * themselves: this file is built without it (only the "app" target gets it
 * from the wolfSSH module), so wolfssh/settings.h would not include
 * CONFIG_WOLFSSH_SETTINGS_FILE here and WOLFSSH_SCP would be invisible. */
void shell_wolfssh_scp_register_ctx(WOLFSSH_CTX *ctx);
int shell_wolfssh_scp_accept(WOLFSSH *ssh);

__attribute__((weak)) int wsUserAuth(byte authType,WS_UserAuthData* authData,void* ctx)
{
    return WOLFSSH_USERAUTH_SUCCESS;
}

__attribute__((weak)) void wsGetEccKeyDer(const unsigned char **ecc_key_der, int *size){
    /* ./keys/server-key-ecc.der, ECC */
    static const unsigned char ecc_key_der_256[] =
    {
        0x30, 0x77, 0x02, 0x01, 0x01, 0x04, 0x20, 0x61, 0x09, 0x99,
        0x0B, 0x79, 0xD2, 0x5F, 0x28, 0x5A, 0x0F, 0x5D, 0x15, 0xCC,
        0xA1, 0x56, 0x54, 0xF9, 0x2B, 0x39, 0x87, 0x21, 0x2D, 0xA7,
        0x7D, 0x85, 0x7B, 0xB8, 0x7F, 0x38, 0xC6, 0x6D, 0xD5, 0xA0,
        0x0A, 0x06, 0x08, 0x2A, 0x86, 0x48, 0xCE, 0x3D, 0x03, 0x01,
        0x07, 0xA1, 0x44, 0x03, 0x42, 0x00, 0x04, 0x81, 0x13, 0xFF,
        0xA4, 0x2B, 0xB7, 0x9C, 0x45, 0x74, 0x7A, 0x83, 0x4C, 0x61,
        0xF3, 0x3F, 0xAD, 0x26, 0xCF, 0x22, 0xCD, 0xA9, 0xA3, 0xBC,
        0xA5, 0x61, 0xB4, 0x7C, 0xE6, 0x62, 0xD4, 0xC2, 0xF7, 0x55,
        0x43, 0x9A, 0x31, 0xFB, 0x80, 0x11, 0x20, 0xB5, 0x12, 0x4B,
        0x24, 0xF5, 0x78, 0xD7, 0xFD, 0x22, 0xEF, 0x46, 0x35, 0xF0,
        0x05, 0x58, 0x6B, 0x5F, 0x63, 0xC8, 0xDA, 0x1B, 0xC4, 0xF5,
        0x69
    };
    static const int sizeof_ecc_key_der_256 = sizeof(ecc_key_der_256);    
    *ecc_key_der = ecc_key_der_256;
    *size = sizeof_ecc_key_der_256;
}

static void wolfssh_server_cb(struct net_socket_service_event *evt);

NET_SOCKET_SERVICE_SYNC_DEFINE_STATIC(wolfssh_server, wolfssh_server_cb, 1);

/* Re-register the SSH socket with the socket service.
 *
 * POLLIN is always monitored. POLLOUT is added only while wolfSSH has data
 * left in its output buffer that the (non-blocking) TCP socket could not take
 * (WS_WANT_WRITE): the socket service then wakes this backend up as soon as
 * the peer accepts more data again, which is what pushes the remaining output
 * out. Registering POLLOUT permanently is not an option because the poll would
 * return immediately on every single iteration (the socket is writable most of
 * the time), and waiting for POLLOUT inline (poll(2) inside the callback or in
 * the write path) blocks the *shared* socket service thread for up to
 * CONFIG_SHELL_WOLFSSH_TIMEOUT seconds, stalling every other registered
 * service.
 *
 * Must be called with sh_ssh->ssh_lock held: both callers (callback thread and
 * shell thread) update the same service entry. The socket service lock is
 * taken *inside* ssh_lock here and is never held while calling back into this
 * file, so there is no lock inversion.
 */
static void ssh_tx_watch_set(struct shell_wolfssh *sh_ssh, bool enable)
{
    struct pollfd pfd;

    if (sh_ssh->tx_watch_armed == enable) {
        return;
    }

    pfd.fd = wolfSSH_get_fd(sh_ssh->ssh);
    pfd.events = POLLIN | (enable ? POLLOUT : 0);
    pfd.revents = 0;

    if (net_socket_service_register(&wolfssh_server, &pfd, 1, sh_ssh) == 0) {
        sh_ssh->tx_watch_armed = enable;
    }
}

static void wolfssh_server_cb(struct net_socket_service_event *evt)
{
    struct shell_wolfssh *sh_ssh = evt->user_data;
    word32 lastChannel = 0;
    int ret;
    int err;
    bool gotRx = false;
    bool txReady;

    k_mutex_lock(&sh_ssh->ssh_lock, K_FOREVER);
    /* wolfSSH_worker() is non-blocking. It may return a "fatal looking"
     * WS_FATAL_ERROR/WS_ERROR (=-1001) whose underlying ssh->error is in
     * fact WS_WANT_READ: that happens when the non-blocking socket has no
     * more data mid-packet (GetInputData() surfaces a would-block as
     * WS_FATAL_ERROR with ssh->error = WS_WANT_READ). Per wolfSSH's
     * contract the application must consult wolfSSH_get_error(): a
     * WS_WANT_READ / WS_WANT_WRITE / WS_REKEYING is transient and only
     * means "call again when the socket is readable/writable" -- it must
     * NEVER tear down the session. The only genuinely fatal conditions are
     * a real transport/decrypt error or the channel being closed.
     *
     * One call per socket event is enough: the callback runs on the socket
     * service thread, which polls in a loop, so a socket buffer that still
     * holds complete packets is reported readable again right away and this
     * callback is invoked once more per packet. Blocking here (waiting for
     * more input, or for POLLOUT, as an earlier revision did) would stall
     * every other service that shares that thread. */
    ret = wolfSSH_worker(sh_ssh->ssh, &lastChannel);
    err = wolfSSH_get_error(sh_ssh->ssh);

    if (ret == WS_CHAN_RXD || ret == WS_EXTDATA) {
        gotRx = true;
    }

    /* The worker flushes wolfSSH's output buffer before it reads. WS_WANT_WRITE
     * means it could not: hand the waiting over to the socket service (POLLOUT,
     * see ssh_tx_watch_set()) instead of polling here. Any other outcome means
     * the buffer is drained, so the shell thread may queue more data. */
    txReady = !(ret == WS_WANT_WRITE || err == WS_WANT_WRITE);
    ssh_tx_watch_set(sh_ssh, !txReady);
    k_mutex_unlock(&sh_ssh->ssh_lock);

    if (gotRx) {
        k_timer_start(&sh_ssh->timer, K_SECONDS(CONFIG_SHELL_WOLFSSH_TIMEOUT),
                      K_NO_WAIT);
        sh_ssh->shell_handler(SHELL_TRANSPORT_EVT_RX_RDY, sh_ssh->shell_context);
    }

    if (txReady) {
        /* Released a shell thread that is waiting for the transport to accept
         * the remainder of a buffer (trace: tmp_cnt == 0 in shell_ops.c). */
        sh_ssh->shell_handler(SHELL_TRANSPORT_EVT_TX_RDY, sh_ssh->shell_context);
    }

    /* Only a genuinely negative, non-transient return tears the session
     * down. Note that WS_CHAN_RXD/WS_EXTDATA are negative too (-1057/-1067)
     * and are the *normal* return for received channel data: now that there
     * is exactly one worker call per event they really do reach this check
     * (the old drain loop could only leave on a transient status).
     * ret == WS_SUCCESS (0) is HEALTHY: wolfSSH_worker() returns it
     * after processing a complete non-channel packet -- notably the
     * client's WINDOW_ADJUST (DoReceive/DoPacket overwrite ssh->error with
     * the handler's return, so err is 0 too). The client sends its first
     * WINDOW_ADJUST only once its consumed output crosses ~half of its
     * initial 2 MB channel window, i.e. after roughly 1 MB of cumulative
     * output -- which is why sessions appeared to die "on the third long
     * busctl tree" while short commands never triggered it. */
    if (ret < 0 && !gotRx && ret != WS_CHANNEL_CLOSED &&
        ret != WS_WANT_READ && ret != WS_WANT_WRITE && ret != WS_REKEYING &&
        err != WS_WANT_READ && err != WS_WANT_WRITE && err != WS_REKEYING) {
        struct shell *sh = sh_ssh->shell_context;
        printk("wolfssh: session closed by worker error: ret=%d err=%d (%s)\n",
               ret, err, wolfSSH_ErrorToName(err));
        net_socket_service_unregister(&wolfssh_server);
        k_poll_signal_raise(&sh->ctx->signals[SHELL_SIGNAL_KILL], 0);
    }
}

static void ssh_timeout_handler(struct k_timer *timer)
{
    struct shell_wolfssh *sh_ssh = CONTAINER_OF(timer,struct shell_wolfssh,timer);
    struct shell *sh = sh_ssh->shell_context;
    printk("wolfssh: idle timeout fired, killing session\n");
    if(sh->ctx->tid)
    {
        k_poll_signal_raise(&sh->ctx->signals[SHELL_SIGNAL_KILL], 0);
    }
}

static int shell_ssh_init(const struct shell_transport *transport,
    const void *config,shell_transport_handler_t evt_handler,void *context)
{
    struct shell_wolfssh *sh_ssh = transport->ctx;
    sh_ssh->shell_handler = evt_handler;
    sh_ssh->shell_context = context;
    sh_ssh->ssh = (WOLFSSH *)config;
    sh_ssh->local_channel = wolfSSH_ChannelFind(sh_ssh->ssh,SHELL_CHANNEL_ID,WS_CHANNEL_ID_SELF);
    struct pollfd poll_fds[1] = {
        [0] = {
            .fd = wolfSSH_get_fd(sh_ssh->ssh),
            .events = POLLIN,
        },
    };
    k_mutex_init(&sh_ssh->ssh_lock);
    k_timer_init(&sh_ssh->timer,ssh_timeout_handler,NULL);
    k_timer_start(&sh_ssh->timer,K_SECONDS(CONFIG_SHELL_WOLFSSH_TIMEOUT),K_NO_WAIT);
    /* The service is (re-)registered for POLLIN only, so a session that took
     * over from another one must not start with a stale "POLLOUT armed". */
    sh_ssh->tx_watch_armed = false;
    sh_ssh->read_peek_ok = true;
    net_socket_service_register(&wolfssh_server,poll_fds,ARRAY_SIZE(poll_fds),sh_ssh);
    return 0;
}

static int shell_ssh_enable(const struct shell_transport *transport, bool blocking_tx)
{
    return 0;
}

static int shell_ssh_read(const struct shell_transport *transport,void *data, size_t length, size_t *cnt)
{
    struct shell_wolfssh *sh_ssh = transport->ctx;
    int pending = -1;
    int ret;

    *cnt = 0;

    k_mutex_lock(&sh_ssh->ssh_lock,K_FOREVER);

    if (sh_ssh->local_channel == NULL) {
        k_mutex_unlock(&sh_ssh->ssh_lock);
        return 0;
    }

    /* wolfSSH_ChannelRead() -> _ChannelRead() copies the pending bytes out of
     * the channel buffer and only *then* refills the receive window
     * (SendChannelWindowAdjust() -> wolfSSH_SendPacket()). When that packet
     * cannot be queued (non-blocking TCP send buffer full) the call returns
     * WS_WANT_WRITE *instead of* the byte count -- while the bytes have already
     * been taken out of the buffer, so the caller cannot tell how many they
     * were. Reporting that negative value (the old code assigned it to a
     * size_t) both loses the data and feeds the shell a bogus count.
     *
     * Ask for the count up front instead: wolfSSH_stream_peek() with a NULL
     * buffer only computes it, without consuming anything. peek and read see
     * the same buffer state because the only other writer, wolfSSH_worker() in
     * the socket service callback, needs this same ssh_lock.
     *
     * peek looks at the session's first channel rather than at the handle we
     * pass to ChannelRead(); that is the same channel here (a session has
     * exactly one, id 0). The success path verifies it and falls back to the
     * old behaviour if that ever stops being true. */
    if (sh_ssh->read_peek_ok) {
        pending = wolfSSH_stream_peek(sh_ssh->ssh, NULL, length);
    }

    ret = wolfSSH_ChannelRead(sh_ssh->local_channel, data, length);

    if (ret >= 0) {
        if (sh_ssh->read_peek_ok && pending >= 0 && pending != ret) {
            sh_ssh->read_peek_ok = false;
            printk("wolfssh: stream_peek %d != ChannelRead %d, "
                   "count-only peek disabled\n", pending, ret);
        }
        *cnt = (size_t)ret;
    } else {
        /* The bytes are in `data` already: publish the count peek reported,
         * and make sure the WINDOW_ADJUST that could not be queued is flushed
         * (the socket service calls us back once the socket accepts data). */
        *cnt = (pending > 0) ? (size_t)pending : 0;
        if (wolfSSH_get_error(sh_ssh->ssh) == WS_WANT_WRITE) {
            ssh_tx_watch_set(sh_ssh, true);
        }
    }

    k_mutex_unlock(&sh_ssh->ssh_lock);
    return 0;
}

static int shell_ssh_write(const struct shell_transport *transport,const void *data, size_t length, size_t *cnt)
{
    struct shell_wolfssh *sh_ssh = transport->ctx;
    int ret;

    *cnt = 0;

    /* Any outgoing data means the session is alive: reset the idle timeout so
     * a command whose execution/output takes a while (with no *received*
     * traffic in the meantime) is not killed by the idle timer. */
    k_timer_start(&sh_ssh->timer, K_SECONDS(CONFIG_SHELL_WOLFSSH_TIMEOUT),
		  K_NO_WAIT);

    /* wolfSSH_ChannelSend() is non-blocking and consumes at most one SSH
     * packet worth of data (peer window / max packet size), possibly without
     * pushing it onto the wire yet. The old code ignored the return value and
     * always reported the whole buffer as sent, so any large command output
     * (e.g. `busctl tree`) was silently truncated on the SSH channel.
     *
     * Report what was really accepted instead of looping until everything is
     * flushed: z_shell_write() already loops over the remainder, and when
     * nothing can be accepted (cnt == 0) it waits for TX_RDY, which
     * wolfssh_server_cb() raises once the socket service reports that the
     * socket is writable again. Waiting here with poll(2) would block the
     * shell thread while holding ssh_lock -- i.e. it would block the socket
     * service thread (and with it every other socket service) for up to
     * CONFIG_SHELL_WOLFSSH_TIMEOUT seconds. */
    k_mutex_lock(&sh_ssh->ssh_lock, K_FOREVER);
    ret = wolfSSH_ChannelSend(sh_ssh->local_channel, data, length);

    if (ret > 0) {
        /* Accepted. If wolfSSH could not put it on the wire now it leaves the
         * packet in its output buffer and reports WS_WANT_WRITE in the session
         * error: let the socket service wake us up when there is room. */
        ssh_tx_watch_set(sh_ssh,
                         wolfSSH_get_error(sh_ssh->ssh) == WS_WANT_WRITE);
        *cnt = (size_t)ret;
    } else if (ret == WS_WANT_WRITE) {
        /* Data from a previous call is still pending, nothing was accepted. */
        ssh_tx_watch_set(sh_ssh, true);
    } else if (ret == WS_WINDOW_FULL || ret == WS_REKEYING) {
        /* Not a socket level block: the peer drives the next step (a
         * WINDOW_ADJUST / rekey packet, i.e. a POLLIN event). Leave the TX
         * watch as it is; it clears itself in the callback. */
    } else {
        /* Real protocol/transport error. Return 0: z_shell_write() asserts on
         * a non-zero status, and claiming the whole buffer was taken avoids
         * leaving the shell thread waiting for a TX_RDY that will not come.
         * The session is broken anyway and is torn down by the callback (or by
         * the idle timer). */
        printk("wolfssh: ChannelSend error ret=%d (%s)\n", ret,
               wolfSSH_ErrorToName(wolfSSH_get_error(sh_ssh->ssh)));
        *cnt = length;
    }
    k_mutex_unlock(&sh_ssh->ssh_lock);

    return 0;
}

static int shell_ssh_uninit(const struct shell_transport *transport)
{
    struct shell_wolfssh *sh_ssh = transport->ctx;
    WOLFSSH *ssh = sh_ssh->ssh;
    int client_fd = wolfSSH_get_fd(ssh);
    k_timer_stop(&sh_ssh->timer);
    net_socket_service_unregister(&wolfssh_server);
    wolfSSH_shutdown(ssh);
    wolfSSH_free(ssh);
    close(client_fd);
    return 0;
}

const struct shell_transport_api shell_wolfssh_transport_api = {
    .init = shell_ssh_init,
    .enable = shell_ssh_enable,
    .read = shell_ssh_read,
    .write = shell_ssh_write,
    .uninit = shell_ssh_uninit,
};
SHELL_WOLFSSH_DEFINE(shell_transport_wolfssh);
SHELL_DEFINE(shell_wolfssh,CONFIG_SHELL_PROMPT_WOLFSSH,&shell_transport_wolfssh,
    CONFIG_SHELL_BACKEND_WOLFSSH_LOG_MESSAGE_QUEUE_SIZE,
    CONFIG_SHELL_BACKEND_WOLFSSH_LOG_MESSAGE_QUEUE_TIMEOUT,
    SHELL_FLAG_OLF_CRLF);
static int sockFd;
static uint16_t ssh_server_port;
static bool ssh_shell_disabled;
static struct k_mutex ssh_shell_enabled_mutex;
static struct k_condvar ssh_shell_enabled_condvar;

static void set_password_suite(WOLFSSH_CTX* ctx)
{
    const char* kex_list = 
        "ecdh-sha2-nistp256,"
        "ecdh-sha2-nistp384";
    if(wolfSSH_CTX_SetAlgoListKex(ctx, kex_list) != WS_SUCCESS)
        LOG_WRN("Failed to set Kex list\n");

    /* Only nistp256 is listed because wsGetEccKeyDer() loads a P-256 key and
     * nothing else. Setting algoListKey makes wolfSSH match against this
     * string instead of ctx->publicKeyAlgo (the keys actually loaded, see
     * DoKexInit()), so listing nistp384 here would let a client that prefers
     * P-384 select a host key algorithm we have no private key for. */
    const char* key_list = 
        "ecdsa-sha2-nistp256";
    if(wolfSSH_CTX_SetAlgoListKey(ctx, key_list) != WS_SUCCESS)
        LOG_WRN("Failed to set Key list\n");

    const char* cipher_list = 
        "aes256-gcm@openssh.com,"
        "aes192-gcm@openssh.com,"
        "aes128-gcm@openssh.com,"
        "aes256-ctr,"
        "aes192-ctr,"
        "aes128-ctr,"
        "aes256-cbc,"
        "aes192-cbc,"
        "aes128-cbc";
    if(wolfSSH_CTX_SetAlgoListCipher(ctx, cipher_list) != WS_SUCCESS)
        LOG_WRN("Failed to set cipher list\n");
}

static int tcp_select(SOCKET_T socketfd, int to_sec)
{
    WFD_SET_TYPE recvfds, errfds;
    int nfds = (int)socketfd + 1;
    struct timeval timeout = {(to_sec > 0) ? to_sec : 0, 0};
    int result;

    WFD_ZERO(&recvfds);
    WFD_SET(socketfd, &recvfds);
    WFD_ZERO(&errfds);
    WFD_SET(socketfd, &errfds);

    /* returns 1 or greater when something is ready to be read */
    result = select(nfds, &recvfds, NULL, &errfds, &timeout);

    if (result == 0)
        return WS_CBIO_ERR_TIMEOUT;
    else
        return result;
}

static int NonBlockSSH_accept(WOLFSSH* ssh)
{
    int ret, select_ret;
    WS_SOCKET_T sockfd;
    sockfd = (WS_SOCKET_T)wolfSSH_get_fd(ssh);
    ret = wolfSSH_accept(ssh);
    /* WS_SCP_INIT also means "accepted, but there is an SCP transfer to run":
     * hand that back to the caller instead of retrying accept() here. */
    while (ret != WS_SUCCESS && ret != WS_SCP_INIT) {
        select_ret = tcp_select(sockfd, CONFIG_SHELL_WOLFSSH_LOGIN_TIMEOUT);
        if(select_ret == WS_CBIO_ERR_TIMEOUT)
        {
            ret = WS_CBIO_ERR_TIMEOUT;
            break;
        }else{
            ret = wolfSSH_accept(ssh);
            if(ret == WS_INVALID_STATE_E)
            {
                break;
            }
        }
    }
    return ret;
}

static void ssh_daemon_func(void *p1,void *p2,void *p3)
{
    void *heap = NULL;
    WOLFSSH_CTX* ctx = NULL;
    ssh_server_port = CONFIG_SHELL_WOLFSSH_PORT;
    #ifdef DEBUG_WOLFSSH
        wolfSSL_Debugging_ON();
        wolfSSH_Debugging_ON();
    #endif
    int ret = wolfSSH_Init();
    __ASSERT_NO_MSG(ret == WS_SUCCESS);
    ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_SERVER, heap);
    __ASSERT_NO_MSG(ctx);

    /* SCP receive/send callbacks (upload and download support) */
    shell_wolfssh_scp_register_ctx(ctx);

    set_password_suite(ctx);

    wolfSSH_SetUserAuth(ctx, wsUserAuth);
    wolfSSH_CTX_SetBanner(ctx, CONFIG_SHELL_WOLFSSH_BANNER"\n");
    const unsigned char *ecc_key_der;
    int key_der_size;
    wsGetEccKeyDer(&ecc_key_der,&key_der_size);
    ret = wolfSSH_CTX_UsePrivateKey_buffer(ctx, ecc_key_der, key_der_size,WOLFSSH_FORMAT_ASN1);
    __ASSERT_NO_MSG(ret == WS_SUCCESS);
    k_mutex_init(&ssh_shell_enabled_mutex);
    k_condvar_init(&ssh_shell_enabled_condvar);
    while(1)
    {
        k_mutex_lock(&ssh_shell_enabled_mutex,K_FOREVER);
        if(ssh_shell_disabled)
        {
            k_condvar_wait(&ssh_shell_enabled_condvar, &ssh_shell_enabled_mutex, K_FOREVER);
            k_mutex_unlock(&ssh_shell_enabled_mutex);
            continue;
        }
        k_mutex_unlock(&ssh_shell_enabled_mutex);
        sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        struct sockaddr_in bind_addr4 = {
            .sin_family = AF_INET,
            .sin_port = htons(ssh_server_port),
            .sin_addr = {
                .s_addr = htonl(INADDR_ANY),
            },
        };
        ret = bind(sockFd, (struct sockaddr *)&bind_addr4, sizeof(bind_addr4));
        __ASSERT_NO_MSG(ret == 0);
        ret = listen(sockFd, 2);
        __ASSERT_NO_MSG(ret == 0);
        while(1)
        {
            struct sockaddr_in client_addr;
            socklen_t client_addr_len = sizeof(client_addr);
            int client_fd = accept(sockFd,(struct sockaddr *)&client_addr,&client_addr_len);
            if(client_fd < 0)
            {
                break;
            }
            WOLFSSH *ssh = wolfSSH_new(ctx);
            int error;
            // wolfSSH_SetUserAuthCtx(ssh,&pwMapList);
            // wolfSSH_SetKeyingCompletionCbCtx(ssh, (void*)ssh);
            // /* Use the session object for its own highwater callback ctx */
            // if (defaultHighwater > 0) {
            //     wolfSSH_SetHighwaterCtx(ssh, (void*)ssh);
            //     wolfSSH_SetHighwater(ssh, defaultHighwater);
            // }

            // Save the original socket flag
            int flags = zsock_fcntl(client_fd, F_GETFL, 0);
            if (flags < 0)
                LOG_ERR("fcntl get failed");
            // Set the client fd non-blocking flag
            flags = zsock_fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
            if (flags < 0)
                LOG_ERR("fcntl set failed");

            wolfSSH_set_fd(ssh,client_fd);
            ret = NonBlockSSH_accept(ssh);
            if(ret == WS_SCP_INIT)
            {
                /* SCP session ("scp -t ..."/"scp -f ..."): run the transfer to
                 * completion and drop the session. The shell must NOT be
                 * attached to this channel, so do not fall through to
                 * shell_init(). */
                ret = shell_wolfssh_scp_accept(ssh);
                if (ret != WS_SCP_COMPLETE) {
                    /* A completed transfer logs "<path> saved"/"serving <path>"
                     * from the SCP callbacks, so only report the abnormal
                     * endings. */
                    printk("wolfssh: scp session ended: ret=%d err=%d (%s)\n",
                           ret, wolfSSH_get_error(ssh),
                           wolfSSH_ErrorToName(wolfSSH_get_error(ssh)));
                }
                wolfSSH_shutdown(ssh);
                wolfSSH_free(ssh);
                close(client_fd);
                continue;
            }
            if(ret == WS_SUCCESS)
            {
                k_tid_t tid = shell_wolfssh.ctx->tid;
                if(tid)
                {
                    k_poll_signal_raise(&shell_wolfssh.ctx->signals[SHELL_SIGNAL_KILL], 0);
                    k_thread_join(tid,K_FOREVER);
                    // LOG_INF("old session killed\n");
                }
                // LOG_INF("new session request\n");
                /* CONFIG_SHELL_WOLFSSH_LOG_LEVEL comes from
                 * Kconfig.template.log_config which depends on LOG. Apps
                 * that do not enable CONFIG_LOG (e.g. they log via printk)
                 * must not fail to compile: fall back to no log backend. */
#if defined(CONFIG_SHELL_WOLFSSH_LOG_LEVEL)
                bool log_backend = CONFIG_SHELL_WOLFSSH_LOG_LEVEL > 0;
                uint32_t level =
                    (CONFIG_SHELL_WOLFSSH_LOG_LEVEL > LOG_LEVEL_DBG) ?
                    CONFIG_LOG_MAX_LEVEL : CONFIG_SHELL_WOLFSSH_LOG_LEVEL;
#else
                bool log_backend = false;
                uint32_t level = LOG_LEVEL_NONE;
#endif
                static const struct shell_backend_config_flags cfg_flags =
                                SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;
                shell_init(&shell_wolfssh,ssh,cfg_flags,log_backend,level);
            }else
            {
                error = wolfSSH_get_error(ssh);
                const char *errorStr = wolfSSH_ErrorToName(error);
                LOG_WRN("%s\n",errorStr);
                if(error == WS_USER_AUTH_E || ret == WS_CBIO_ERR_TIMEOUT)
                {
                    wolfSSH_SendDisconnect(ssh,WOLFSSH_DISCONNECT_NO_MORE_AUTH_METHODS_AVAILABLE);
                }
                wolfSSH_shutdown(ssh);
                wolfSSH_free(ssh);
                close(client_fd);
            }
        }
    }

}

static int create_wolfssh_shell_daemon(void)
{
    /* Name must be set AFTER k_thread_create(): create() re-initializes the
     * thread struct and would wipe a name set beforehand. */
    k_thread_create(&ssh_daemon_thread,ssh_daemon_stack,K_KERNEL_STACK_SIZEOF(ssh_daemon_stack),
        ssh_daemon_func,NULL,NULL,NULL,SHELL_SSH_DAEMON_THREAD_PRIORITY,0,K_NO_WAIT);
    k_thread_name_set(&ssh_daemon_thread,"ssh_daemon_thread");
    return 0;
}

SYS_INIT(create_wolfssh_shell_daemon,APPLICATION,CONFIG_APPLICATION_INIT_PRIORITY);


static void ssh_shell_disabled_set(bool disabled)
{
    k_mutex_lock(&ssh_shell_enabled_mutex, K_FOREVER);
    ssh_shell_disabled = disabled;
    k_condvar_signal(&ssh_shell_enabled_condvar);
    k_mutex_unlock(&ssh_shell_enabled_mutex);
}

void enable_ssh_shell()
{
    ssh_shell_disabled_set(false);
}

void disable_ssh_shell()
{
    ssh_shell_disabled_set(true);
    close(sockFd);
}

bool is_ssh_shell_disabled()
{
    return ssh_shell_disabled;
}

void set_ssh_port(uint16_t port)
{
    ssh_server_port = port;
    close(sockFd);
}

uint16_t get_ssh_port()
{
    return ssh_server_port;
}

static int cmd_exit(const struct shell *sh, size_t argc, char **argv)
{
    if(sh == &shell_wolfssh)
    {
        return k_poll_signal_raise(&sh->ctx->signals[SHELL_SIGNAL_KILL], 0);
    }
    return 0;
}

#define SHELL_HELP_EXIT			"Exit SSH"
SHELL_COND_CMD_ARG_REGISTER(CONFIG_SHELL_VT100_COMMANDS, exit, NULL,
			    SHELL_HELP_EXIT, cmd_exit, 1, 0);

static int cmd_ssh_port(const struct shell *sh, size_t argc, char **argv)
{   
    int err = 0;
    int port = CONFIG_SHELL_WOLFSSH_PORT;
    if(argc==2) 
    {
        port = shell_strtol(argv[1],10,&err);
    }
    if(err)
    {
        shell_print(sh,"Invalid Arguments");
    }else
    {   
        if(argc==2)
        {
            set_ssh_port(port);
        }
        shell_print(sh,"SSH Port:%d",get_ssh_port());
    }
    return 0;
}

#define SHELL_HELP_SSH_PORT		"SSH Port"
SHELL_COND_CMD_ARG_REGISTER(CONFIG_SHELL_VT100_COMMANDS, ssh_port, NULL,
			    SHELL_HELP_SSH_PORT, cmd_ssh_port, 1, 1);

static int cmd_ssh_enable(const struct shell *sh, size_t argc, char **argv)
{
    int err = 0;
    int current_disabled = is_ssh_shell_disabled();
    int enabled = !current_disabled;
    if(argc==2) 
    {
        enabled = shell_strtol(argv[1],10,&err);
    }
    if(err)
    {
        shell_print(sh,"Invalid Arguments");
    }else
    {
        if(enabled == current_disabled)
        {
            if(enabled)
            {
                enable_ssh_shell();
            }else
            {
                disable_ssh_shell();
            }
        }
        shell_print(sh,"SSH Enabled:%d",enabled);
    }
    return 0;
}

#define SHELL_HELP_SSH_ENABLE		"SSH Enable"
SHELL_COND_CMD_ARG_REGISTER(CONFIG_SHELL_VT100_COMMANDS, ssh_enable, NULL,
			    SHELL_HELP_SSH_ENABLE, cmd_ssh_enable, 1, 1);