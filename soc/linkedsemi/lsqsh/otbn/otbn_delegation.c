/*
 * OTBN delegation bus implementation. See otbn_delegation.h for the design
 * rules; this file is only the transport -- it knows nothing about ECDSA or
 * RSA, it dispatches on OTBN_DELEG_OP_* through the table below.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/cache.h>
#include <errno.h>

#include "otbn_delegation.h"
#include "ls_otbn_delegation.h"

#include <zephyr/logging/log.h>
/* Own REGISTER: this file is built for delegation, while the shared
 * 'mbedtls_otbn' module is registered in mbedtls_otbn_msp.c, which is gated on
 * CONFIG_MBEDTLS_LINKEDSEMI_OTBN only -- a delegation-only build would
 * otherwise fail to link log_const_mbedtls_otbn.
 */
LOG_MODULE_REGISTER(mbedtls_otbn_mbox, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

/* Both cores share SRAM, so the mbox carries pointers. Keep every exchange
 * point greppable: families must not call sys_cache_* on delegated buffers
 * directly. */
void otbn_deleg_cache_flush(void *addr, size_t size)
{
	sys_cache_data_flush_range(addr, size);
}

void otbn_deleg_cache_invd(void *addr, size_t size)
{
	sys_cache_data_invd_range(addr, size);
}

/* The bus is crypto-library agnostic, so it is gated on the generic side
 * switches; a library's own delegation switch (mbedtls', wolfssl's) selects
 * them. See Kconfig.default. */
#if defined(CONFIG_LS_OTBN_DELEGATION_CLIENT)

static struct mbox_dt_spec otbn_deleg_client =
	MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto), mbox);

/* One request in flight at a time: the reply buffer and the sequence counter
 * below are single instances, and the peer executes one op at a time anyway. */
static struct k_sem client_sem;
static struct k_sem client_reply_sem;
static struct otbn_delegate_params recive_params;
static uint8_t client_seq;

static void client_mbox_handler(const struct otbn_delegate_params *param)
{
	/* Copy the whole reply: correlation keys on seq alone, so the bus must
	 * not filter by op. An op whitelist here used to drop replies for ops
	 * it did not know *after* recording the status, and -- worse -- the
	 * default branch still signalled the semaphore, waking a caller that
	 * was waiting for a different operation. */
	memcpy((void *)&recive_params, (const void *)param, sizeof(recive_params));
	k_sem_give(&client_reply_sem);
}

static void client_mbox_callback(const struct device *dev,
				 mbox_channel_id_t channel_id, void *user_data,
				 struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);

	client_mbox_handler(data->data);
}

void ls_otbn_delegation_client_init(void)
{
	/* Re-initialised on purpose: applications and tests do repeat this. */
	k_sem_init(&client_sem, 1, 1);
	k_sem_init(&client_reply_sem, 0, 1);

	/* A failed registration would leave every later call timing out with no
	 * other trace, so it must not pass silently. */
	if (mbox_register_callback_dt(&otbn_deleg_client, client_mbox_callback, NULL) != 0) {
		LOG_ERR("delegation: client callback registration failed");
		return;
	}
	(void)mbox_set_enabled_dt(&otbn_deleg_client, true);
}

int otbn_deleg_call(struct otbn_delegate_params *req, uint32_t timeout_ms)
{
	struct mbox_msg msg = {
		.data = req,
		.size = sizeof(*req),
	};
	int64_t deadline;

	if (k_sem_take(&client_sem, K_FOREVER) != 0) {
		return -EACCES;
	}

	/* seq 0 is the "no reply stored" sentinel, so never hand it out: on wrap
	 * the request's seq would equal the sentinel and any stale reply would be
	 * mistaken for this call's. */
	++client_seq;
	if (client_seq == 0) {
		client_seq = 1;
	}
	req->seq = client_seq;
	recive_params.seq = 0;

	if (mbox_send_dt(&otbn_deleg_client, &msg) != 0) {
		LOG_WRN("delegation: mbox send failed");
		k_sem_give(&client_sem);
		return -EIO;
	}

	deadline = k_uptime_get() + timeout_ms;
	for (;;) {
		int64_t left = deadline - k_uptime_get();

		if (left <= 0) {
			break;
		}
		if (k_sem_take(&client_reply_sem, K_MSEC(left)) != 0) {
			break;
		}
		if (recive_params.seq == req->seq) {
			k_sem_give(&client_sem);
			return 0;
		}
		/* Stale reply from an earlier timed-out call -- keep waiting. */
	}

	LOG_WRN("delegation: no reply within %u ms", timeout_ms);
	k_sem_give(&client_sem);
	return -ETIMEDOUT;
}

const struct otbn_delegate_params *otbn_deleg_reply(void)
{
	return &recive_params;
}

#else /* server */

static struct mbox_dt_spec otbn_deleg_server =
	MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto), mbox);

/* The server runs on its own thread, not on the system workqueue, for two
 * reasons:
 *  - An RSA modexp occupies the peer for hundreds of milliseconds; running that
 *    from the system workqueue stalls every other system work item.
 *  - The handlers need real stack: a single mbedtls_ls_otbn_rsa_modexp() frame
 *    holds four MBEDTLS_MPI_MAX_SIZE buffers, and the ECDSA path adds an
 *    mbedtls_ecdsa_context on top. The system workqueue's stack
 *    (CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE, 1024 here) does not cover that, and
 *    running the handlers there ended in a kernel panic on the first delegated
 *    ECDSA signing request. That panic is gone since the handlers moved here;
 *    the stack figure above is what the two observations are consistent with,
 *    not a measured high-water mark.
 */
#define OTBN_DELEG_SERVER_STACK_SIZE 8192
#define OTBN_DELEG_SERVER_PRIORITY   K_PRIO_PREEMPT(2)

/* Statically initialised so the thread below can block on it before
 * ls_otbn_delegation_server_init() has run. Depth 2 matches the single-consumer,
 * one-request-in-flight model of the bus. */
#define OTBN_DELEG_MSGQ_LEN 2
K_MSGQ_DEFINE(otbn_deleg_msgq, sizeof(struct otbn_delegate_params), OTBN_DELEG_MSGQ_LEN, 4);

/* The single place that lists every delegated operation. Adding a family means
 * adding its two handlers to a family file and its ops here -- nothing else in
 * the transport changes.
 *
 * Each family's rows are gated on that family's own switch: a build that only
 * enables wolfssl (or only mbedtls) must not pull in the other library's
 * handlers, which are not compiled into the image at all -- an unconditional
 * reference here would fail to link. */
static const struct otbn_deleg_op deleg_ops[] = {
#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION)
	{ OTBN_DELEG_OP_ECDSA_GENKEY,   otbn_deleg_ecdsa_genkey },
	{ OTBN_DELEG_OP_ECDSA_GENKEY_RNG, otbn_deleg_ecdsa_genkey_rng },
	{ OTBN_DELEG_OP_ECDSA_SIGN,     otbn_deleg_ecdsa_sign },
	{ OTBN_DELEG_OP_ECDSA_SIGN_RNG, otbn_deleg_ecdsa_sign_rng },
	{ OTBN_DELEG_OP_ECDSA_VERIFY,   otbn_deleg_ecdsa_verify },
#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_SLOT_MBOX)
	{ OTBN_DELEG_OP_ECDSA_SIGN_SLOT, otbn_deleg_ecdsa_sign_slot },
	{ OTBN_DELEG_OP_ECDSA_SIGN_SLOT_RNG, otbn_deleg_ecdsa_sign_slot_rng },
#endif
	{ OTBN_DELEG_OP_RSA_MODEXP_PUB, otbn_deleg_rsa_modexp_pub },
	{ OTBN_DELEG_OP_RSA_MODEXP_PRV, otbn_deleg_rsa_modexp_prv },
#endif
#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION)
	{ OTBN_DELEG_OP_WOLFSSL_GENKEY,         otbn_deleg_wolfssl_genkey },
	{ OTBN_DELEG_OP_WOLFSSL_SIGN,           otbn_deleg_wolfssl_sign },
	{ OTBN_DELEG_OP_WOLFSSL_VERIFY,         otbn_deleg_wolfssl_verify },
	{ OTBN_DELEG_OP_WOLFSSL_SHARED_SECRET,  otbn_deleg_wolfssl_shared_secret },
	{ OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PUB, otbn_deleg_wolfssl_rsa_modexp_pub },
	{ OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PRV, otbn_deleg_wolfssl_rsa_modexp_prv },
#endif
};

static const struct otbn_deleg_op *find_op(uint8_t op)
{
	for (size_t i = 0; i < ARRAY_SIZE(deleg_ops); i++) {
		if (deleg_ops[i].op == op) {
			return &deleg_ops[i];
		}
	}
	return NULL;
}

static void server_reply(const struct otbn_delegate_params *req, int status,
			 const struct otbn_delegate_params *payload)
{
	struct otbn_delegate_params reply = {0};
	struct mbox_msg msg = {
		.data = &reply,
		.size = sizeof(reply),
	};

	if (payload != NULL) {
		reply = *payload;
	}
	/* Stamp op/seq/status here rather than in the handlers: a reply must stay
	 * correlatable even when no handler ran (unknown op, queue full), and a
	 * handler that forgot one of the three used to make the client read a
	 * failure as success. */
	reply.status = status;
	reply.op = req->op;
	reply.seq = req->seq;
	if (mbox_send_dt(&otbn_deleg_server, &msg) != 0) {
		/* The peer's rx fifo is full, so this reply is lost. Nothing can be
		 * done about it from here, but it must not be silent: the caller
		 * sees a timeout, and a timeout with no log is what makes this class
		 * of bug expensive to place. */
		LOG_WRN("delegation: reply send failed, op 0x%02x seq %u dropped",
			reply.op, reply.seq);
	}
}

static void server_mbox_handler(const struct otbn_delegate_params *param)
{
	if (k_msgq_put(&otbn_deleg_msgq, param, K_NO_WAIT) == 0) {
		return;
	}

	/* Queue full: answer immediately so the caller fails fast instead of
	 * burning its whole timeout. */
	LOG_WRN("delegation: request queue full");
	server_reply(param, -EBUSY, NULL);
}

static void server_mbox_callback(const struct device *dev,
				 mbox_channel_id_t channel_id, void *user_data,
				 struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);

	server_mbox_handler(data->data);
}

static void server_handle(const struct otbn_delegate_params *req)
{
	struct otbn_delegate_params reply;
	const struct otbn_deleg_op *op;
	int err;

	memset(&reply, 0, sizeof(reply));
	op = find_op(req->op);
	if (op != NULL) {
		err = op->handle(req, &reply);
	} else {
		LOG_WRN("delegation: unknown op 0x%02x", req->op);
		err = OTBN_DELEG_ERR_UNKNOWN_OP;
	}
	server_reply(req, err, &reply);
}

static void server_thread(void *p1, void *p2, void *p3)
{
	struct otbn_delegate_params req;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (k_msgq_get(&otbn_deleg_msgq, &req, K_FOREVER) != 0) {
			continue;
		}
		/* Strictly one request at a time, in arrival order: the peer's OTBN
		 * is a single serial unit and the client serialises its calls. */
		server_handle(&req);
	}
}

K_THREAD_DEFINE(otbn_deleg_thread, OTBN_DELEG_SERVER_STACK_SIZE, server_thread,
		NULL, NULL, NULL, OTBN_DELEG_SERVER_PRIORITY, 0, 0);

void ls_otbn_delegation_server_init(void)
{
	if (mbox_register_callback_dt(&otbn_deleg_server, server_mbox_callback, NULL) != 0) {
		LOG_ERR("delegation: server callback registration failed");
		return;
	}
	(void)mbox_set_enabled_dt(&otbn_deleg_server, true);
}

#endif /* CONFIG_LS_OTBN_DELEGATION_CLIENT */

/*
 * Bring the channel up from the bus itself rather than from a board hook or
 * from each application:
 *
 * - A server has to be able to answer as soon as the peer asks, and a forgotten
 *   application-side init shows up only as every request timing out.
 * - The board's soc_late_init_hook() is weak, and the SMP build defines it
 *   strongly elsewhere (smp/lsqsh_e906_smp.c): a call placed there would be
 *   silently dropped in an SMP image.
 * - Both inits are idempotent, so an application that calls them anyway (the
 *   samples and the 810 tests do) is unaffected.
 *
 * APPLICATION priority: the mbox is a driver and is initialised at DEVICE
 * priority, so this must not run before it.
 */
static int otbn_delegation_bringup(void)
{
#if defined(CONFIG_LS_OTBN_DELEGATION_CLIENT)
	ls_otbn_delegation_client_init();
#endif
#if defined(CONFIG_LS_OTBN_DELEGATION_SERVER)
	ls_otbn_delegation_server_init();
#endif
	return 0;
}

SYS_INIT(otbn_delegation_bringup, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
