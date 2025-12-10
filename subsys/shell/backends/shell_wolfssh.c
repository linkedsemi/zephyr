#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_service.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell_wolfssh.h>

LOG_MODULE_REGISTER(shell_wolfssh_backend);

static K_KERNEL_STACK_DEFINE(ssh_daemon_stack,CONFIG_SHELL_WOLFSSH_DAEMON_STACK_SIZE);
static struct k_thread ssh_daemon_thread;
#define SHELL_SSH_DAEMON_THREAD_PRIORITY \
	COND_CODE_1(CONFIG_SHELL_THREAD_PRIORITY_OVERRIDE, \
			(CONFIG_SHELL_THREAD_PRIORITY), (K_LOWEST_APPLICATION_THREAD_PRIO))
#define SHELL_CHANNEL_ID 0

void wolfSSL_Debugging_ON(void);

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

static void wolfssh_server_cb(struct net_socket_service_event *evt)
{
    struct shell_wolfssh *sh_ssh = evt->user_data;
    // struct pollfd *pfd = &evt->event;
    word32 lastChannel = 0;
    k_mutex_lock(&sh_ssh->ssh_lock,K_FOREVER);
    int ret = wolfSSH_worker(sh_ssh->ssh,&lastChannel);
    k_mutex_unlock(&sh_ssh->ssh_lock);
    // LOG_DBG("wolfssh worker ret:%d\n",ret);
    if(ret == WS_CHAN_RXD)
    {
        k_timer_start(&sh_ssh->timer,K_SECONDS(CONFIG_SHELL_WOLFSSH_TIMEOUT),K_NO_WAIT);
        sh_ssh->shell_handler(SHELL_TRANSPORT_EVT_RX_RDY, sh_ssh->shell_context);
    }else if(ret != WS_CHANNEL_CLOSED && ret != WS_WANT_READ)
    {
        struct shell *sh = sh_ssh->shell_context;
        net_socket_service_unregister(&wolfssh_server);
        k_poll_signal_raise(&sh->ctx->signals[SHELL_SIGNAL_KILL], 0);
    }
}

static void ssh_timeout_handler(struct k_timer *timer)
{
    struct shell_wolfssh *sh_ssh = CONTAINER_OF(timer,struct shell_wolfssh,timer);
    struct shell *sh = sh_ssh->shell_context;
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
    k_mutex_lock(&sh_ssh->ssh_lock,K_FOREVER);
    *cnt = wolfSSH_ChannelRead(sh_ssh->local_channel,data,length);
    k_mutex_unlock(&sh_ssh->ssh_lock);
    return 0;
}

static int shell_ssh_write(const struct shell_transport *transport,const void *data, size_t length, size_t *cnt)
{
    struct shell_wolfssh *sh_ssh = transport->ctx;
    k_mutex_lock(&sh_ssh->ssh_lock,K_FOREVER);
    wolfSSH_ChannelSend(sh_ssh->local_channel,data,length);
    k_mutex_unlock(&sh_ssh->ssh_lock);
    *cnt = length;
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

static void ssh_daemon_func(void *p1,void *p2,void *p3)
{
    void *heap = NULL;
    WOLFSSH_CTX* ctx = NULL;
    #ifdef DEBUG_WOLFSSH
        wolfSSL_Debugging_ON();
        wolfSSH_Debugging_ON();
    #endif
    int ret = wolfSSH_Init();
    __ASSERT_NO_MSG(ret == WS_SUCCESS);
    ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_SERVER, heap);
    __ASSERT_NO_MSG(ctx);
    wolfSSH_SetUserAuth(ctx, wsUserAuth);
    wolfSSH_CTX_SetBanner(ctx, CONFIG_SHELL_WOLFSSH_BANNER"\n");
    const unsigned char *ecc_key_der;
    int key_der_size;
    wsGetEccKeyDer(&ecc_key_der,&key_der_size);
    ret = wolfSSH_CTX_UsePrivateKey_buffer(ctx, ecc_key_der, key_der_size,WOLFSSH_FORMAT_ASN1);
    __ASSERT_NO_MSG(ret == WS_SUCCESS);
    int sockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	struct sockaddr_in bind_addr4 = {
		.sin_family = AF_INET,
		.sin_port = htons(CONFIG_SHELL_WOLFSSH_PORT),
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
        WOLFSSH *ssh = wolfSSH_new(ctx);
        int error;
        // wolfSSH_SetUserAuthCtx(ssh,&pwMapList);
        // wolfSSH_SetKeyingCompletionCbCtx(ssh, (void*)ssh);
        // /* Use the session object for its own highwater callback ctx */
        // if (defaultHighwater > 0) {
        //     wolfSSH_SetHighwaterCtx(ssh, (void*)ssh);
        //     wolfSSH_SetHighwater(ssh, defaultHighwater);
        // }
        wolfSSH_set_fd(ssh,client_fd);
        ret = wolfSSH_accept(ssh);
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
            bool log_backend = CONFIG_SHELL_WOLFSSH_LOG_LEVEL > 0;
            uint32_t level =
                (CONFIG_SHELL_WOLFSSH_LOG_LEVEL > LOG_LEVEL_DBG) ?
                CONFIG_LOG_MAX_LEVEL : CONFIG_SHELL_WOLFSSH_LOG_LEVEL;
            static const struct shell_backend_config_flags cfg_flags =
                            SHELL_DEFAULT_BACKEND_CONFIG_FLAGS;
            shell_init(&shell_wolfssh,ssh,cfg_flags,log_backend,level);
        }else
        {
            error = wolfSSH_get_error(ssh);
            const char *errorStr = wolfSSH_ErrorToName(error);
            LOG_WRN("%s\n",errorStr);
            if(error == WS_USER_AUTH_E)
            {
                wolfSSH_SendDisconnect(ssh,WOLFSSH_DISCONNECT_NO_MORE_AUTH_METHODS_AVAILABLE);
            }
            wolfSSH_shutdown(ssh);
            wolfSSH_free(ssh);
            close(client_fd);
        }
    }
}

static int enable_shell_wolfssh(void)
{
    k_thread_name_set(&ssh_daemon_thread,"ssh_daemon_thread");
    k_thread_create(&ssh_daemon_thread,ssh_daemon_stack,K_KERNEL_STACK_SIZEOF(ssh_daemon_stack),
        ssh_daemon_func,NULL,NULL,NULL,SHELL_SSH_DAEMON_THREAD_PRIORITY,0,K_NO_WAIT);
    return 0;
}

SYS_INIT(enable_shell_wolfssh,APPLICATION,CONFIG_APPLICATION_INIT_PRIORITY);


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