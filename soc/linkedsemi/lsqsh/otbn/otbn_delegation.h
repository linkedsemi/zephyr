/*
 * OTBN delegation bus: the wire format and the transport shared by every
 * delegated OTBN operation family (ECDSA, RSA, ...).
 *
 * Why a bus
 * ---------
 * - One mbox channel can carry one consumer only: the linkedsemi mbox driver
 *   keeps a single callback slot per channel (cb[channel]) and re-registration
 *   overwrites it silently. Two families each registering their own callback
 *   on mbox_consumer_otbn_crypto would mean the second one silently orphans
 *   the first, whose requests then never get answered. Every family therefore
 *   multiplexes through this bus on OTBN_DELEG_OP_*.
 * - OTBN is a single serial unit, so one channel costs no throughput; the
 *   per-call timeout below is what keeps a slow family from starving a fast
 *   one.
 * - The payload is an array of pointers: both cores share SRAM, so buffers are
 *   addressed directly. Cache maintenance stays with the family that owns the
 *   buffer, through otbn_deleg_cache_flush() / otbn_deleg_cache_invd().
 * - There is no wire version field: client and server images must come from the
 *   same build tree. A mismatched pair surfaces as a reply the caller cannot
 *   serve (status OTBN_DELEG_ERR_UNKNOWN_OP), never as silent corruption.
 * - The frame size is sizeof(struct otbn_delegate_params) -- 32 bytes today --
 *   and the mbox driver rejects a longer frame with -EMSGSIZE instead of
 *   truncating it. The board's delegation channel must therefore be at least
 *   that wide (see the mbox_consumer_otbn_crypto node: 32 in the EVB dts) and
 *   both cores must use the same value, since they share the fifo area. Rebuild
 *   and flash the two images together: a mixed pair misplaces the shared fifo
 *   area (an old client against a new server fails every send with
 *   "msg size error ..." at the frame size).
 *
 * Usage flow
 * ----------
 * Two images, one channel. The serving core owns the OTBN; the requesting core
 * has no local OTBN work to do.
 *
 *   serving core                             requesting core
 *   ------------                             ---------------
 *   ls_otbn_delegation_server_init()         ls_otbn_delegation_client_init()
 *     mbox callback + own thread               mbox callback
 *   idle until a request arrives             otbn_deleg_call(&req, timeout_ms)
 *   op table -> family handler                 takes the one-call lock,
 *   executes on the local OTBN                 assigns req.seq, sends,
 *   stamps status/op/seq, sends the reply ---> waits for that seq
 *                                            returns 0 / -EIO / -ETIMEDOUT
 *                                            otbn_deleg_reply() -> struct
 *                                            check .status, read own buffers
 *
 * Applications normally do not touch this. Each family wraps the packing and
 * the cache maintenance in entry points that take the same arguments as their
 * local counterparts -- call mbedtls_ls_otbn_ecdsa_genkey()/_sign()/_verify()
 * or mbedtls_ls_otbn_rsa_modexp_encrypt()/_decrypt(). Use the bus directly only
 * when adding a family.
 *
 * What callers must keep:
 *  - Bring the client up once (idempotent) before the first call. A call made
 *    before ls_otbn_delegation_client_init() has no reply path and can only time out.
 *  - One call in flight per core: otbn_deleg_call() returns -EACCES if another
 *    is still running. The request's buffers belong to the caller and must not
 *    be touched until the call returns -- after a timeout the peer may still be
 *    writing into them.
 *  - The reply points at the same buffers the request pointed at (both cores
 *    share SRAM), so invalidate them before reading; the family entry points
 *    already do this.
 *  - Every failure is an ordinary return value, never silence: -EIO (send
 *    failed), -ETIMEDOUT (no matching reply in time), -EBUSY (the peer's queue
 *    is full), OTBN_DELEG_ERR_UNKNOWN_OP (the peer does not serve this op --
 *    normally a mixed pair of images).
 *
 * Adding a family:
 *  1. write two handlers in a family file: one that packs the request on the
 *     client, one that executes it on the server (see
 *     mbedtls/mbedtls_otbn_crypto_mbox.c and mbedtls/mbedtls_otbn_rsa_mbox.c);
 *  2. add its op code to the list below and its rows to the op table in
 *     otbn_delegation.c;
 *  3. keep the frame within the channel width (see the note above).
 */

#ifndef OTBN_DELEGATION_H
#define OTBN_DELEGATION_H

#include <stddef.h>
#include <stdint.h>

/*
 * Op code space. 0x11/0x22/0x33 keep the values the ECDSA proxy has always put
 * on the wire; changing them would break a paired image silently. New families
 * take their own cluster.
 */
#define OTBN_DELEG_OP_ECDSA_GENKEY    		0x11
#define OTBN_DELEG_OP_ECDSA_SIGN      		0x22
#define OTBN_DELEG_OP_ECDSA_VERIFY    		0x33
/* Client-supplied randomness variants. The requesting core fills data[2] with
 * the keygen seed (genkey_rng) or the signing nonce (sign_rng); the serving
 * core feeds it to the local OTBN instead of using its own entropy. */
#define OTBN_DELEG_OP_ECDSA_GENKEY_RNG 		0x12
#define OTBN_DELEG_OP_ECDSA_SIGN_RNG   		0x24
/* Sign with a key that stays in the serving core's slot table: the request
 * carries a slot id instead of the private scalar. */
#define OTBN_DELEG_OP_ECDSA_SIGN_SLOT 		0x23
/* Slot variant with a client-supplied nonce: data[2] is nonce-in / r-out. */
#define OTBN_DELEG_OP_ECDSA_SIGN_SLOT_RNG 	0x25
#define OTBN_DELEG_OP_RSA_MODEXP_PUB  		0x41
#define OTBN_DELEG_OP_RSA_MODEXP_PRV  		0x42

/*
 * wolfSSL families take their own cluster. The wire contract mirrors the
 * wolfssl port's own exchange (arg[0] carries the wolfssl curve id as the port
 * enumerates it, arg[1] its byte size; the RSA ops put the modulus size in
 * bits in arg[0]). It is deliberately NOT the mbedtls layout: the two libraries
 * disagree on the curve enumeration and on how a public key is split, so a
 * shared op would only force a translation layer neither peer needs. Keeping
 * the two clusters apart is also what lets an image serve both.
 */
#define OTBN_DELEG_OP_WOLFSSL_GENKEY         0x51
#define OTBN_DELEG_OP_WOLFSSL_SIGN           0x52
#define OTBN_DELEG_OP_WOLFSSL_VERIFY         0x53
#define OTBN_DELEG_OP_WOLFSSL_SHARED_SECRET  0x54
#define OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PUB 0x55
#define OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PRV 0x56

/* Reply status for an op the receiver does not serve (image mismatch, or a
 * family compiled out). Outside the mbedtls error space on purpose. */
#define OTBN_DELEG_ERR_UNKNOWN_OP     (-38) /* -ENOSYS */

struct otbn_delegate_params {
	int32_t status;
	uint8_t op;
	/* Request/response correlation. Replies cross the mbox asynchronously
	 * and a call may give up on timeout while its reply is still in flight,
	 * so the op alone cannot tell "this reply is mine" from "this is a
	 * stale one". 0 = no reply stored yet. */
	uint8_t seq;
	uint16_t reserved;
	/* Op-specific scalars (curve id, payload length, modulus size). Two are
	 * enough for every family: ECDSA/RSA use arg[0] (curve/gid) and arg[1]
	 * (hash length or modulus size); the wolfssl families only arg[0..1]. */
	uint32_t arg[2];
	/* Op-specific shared-SRAM buffers. Four are enough: genkey needs 2, sign
	 * and verify 4 (a public key travels packed as X||Y in one buffer). */
	uint8_t *data[4];
};

/* ---------------------------------------------------------------- client */

/* Bring one side of the channel up. Public entry points; declared in
 * ls_otbn_delegation.h. */

/* Send req and wait up to timeout_ms for the reply carrying its sequence.
 * Assigns req->seq; on success the reply is readable through
 * otbn_deleg_reply().
 *
 * The bound is per call rather than global: RSA modexp is orders of magnitude
 * slower than ECDSA, so one shared value would either cut RSA short or make an
 * ECDSA failure take far too long to surface.
 *
 * Returns 0 on success, -EACCES if another call is in flight (calls are
 * serialized), -EIO if the send failed, -ETIMEDOUT if no matching reply
 * arrived in time.
 */
int otbn_deleg_call(struct otbn_delegate_params *req, uint32_t timeout_ms);

/* The reply belonging to the call that just returned 0. Valid until the next
 * otbn_deleg_call(). Never NULL. */
const struct otbn_delegate_params *otbn_deleg_reply(void);

/* Cache maintenance for a buffer exchanged with the peer. Use them instead of
 * the sys_cache_* calls directly so every exchange point is greppable. */
void otbn_deleg_cache_flush(void *addr, size_t size);
void otbn_deleg_cache_invd(void *addr, size_t size);

/* ---------------------------------------------------------------- server */

/* Run one request and fill reply's buffers/scalars. The bus stamps op, seq and
 * status itself (status = the handler's return value), so handlers never write
 * those three -- that is what used to let a failed curve load reach the client
 * as success. */
typedef int (*otbn_deleg_handler_t)(const struct otbn_delegate_params *req,
				    struct otbn_delegate_params *reply);

struct otbn_deleg_op {
	uint8_t op;
	otbn_deleg_handler_t handle;
};

/* Bring the server side up. Public entry point; declared in
 * ls_otbn_delegation.h. */

/* ------------------------------------------- family handlers (server side)
 * Exposed so the bus can keep a single op table in otbn_delegation.c: one
 * place lists every delegated operation, which is what makes "how do I add a
 * family" a two-function question.
 */
int otbn_deleg_ecdsa_genkey(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply);
int otbn_deleg_ecdsa_genkey_rng(const struct otbn_delegate_params *req,
				struct otbn_delegate_params *reply);
int otbn_deleg_ecdsa_sign(const struct otbn_delegate_params *req,
			  struct otbn_delegate_params *reply);
int otbn_deleg_ecdsa_sign_rng(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply);
int otbn_deleg_ecdsa_verify(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply);
#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_ECDSA_SLOT_MBOX)
int otbn_deleg_ecdsa_sign_slot(const struct otbn_delegate_params *req,
			       struct otbn_delegate_params *reply);
int otbn_deleg_ecdsa_sign_slot_rng(const struct otbn_delegate_params *req,
				   struct otbn_delegate_params *reply);
#endif
int otbn_deleg_rsa_modexp_pub(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply);
int otbn_deleg_rsa_modexp_prv(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply);

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION)
/* wolfSSL family handlers, defined in wolfssl/wolfssl_otbn_crypto_mbox.c. They
 * run the request through the wolfssl port's own local OTBN sequences
 * (ecc.c / rsa_modexp.c) rather than a reimplementation. */
int otbn_deleg_wolfssl_genkey(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply);
int otbn_deleg_wolfssl_sign(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply);
int otbn_deleg_wolfssl_verify(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply);
int otbn_deleg_wolfssl_shared_secret(const struct otbn_delegate_params *req,
				     struct otbn_delegate_params *reply);
int otbn_deleg_wolfssl_rsa_modexp_pub(const struct otbn_delegate_params *req,
				      struct otbn_delegate_params *reply);
int otbn_deleg_wolfssl_rsa_modexp_prv(const struct otbn_delegate_params *req,
				      struct otbn_delegate_params *reply);
#endif

#endif /* OTBN_DELEGATION_H */
