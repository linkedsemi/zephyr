/*
 * Public interface for bringing the OTBN delegation channel up.
 *
 * The channel is not tied to a crypto library: it moves an operation family's
 * request to the peer core and brings the answer back, and the families on top
 * of it decide what travels (mbedtls_otbn_ecdsa_mbox.h,
 * mbedtls_otbn_rsa_mbox.h today; the same channel can carry wolfssl families
 * later). That is why these two functions carry the ls_otbn_ prefix rather
 * than a library one: like ls_otbn_session_acquire() and friends, they belong
 * to the OTBN layer itself.
 *
 * The delegation has two sides, and the bring-up is per side -- not per family:
 *   - client: the core that asks for the work. It sends the request and blocks
 *     until the peer answers.
 *   - server: the core that owns the OTBN. It executes whatever request
 *     arrives, so it needs this core's OTBN support for every family it is
 *     expected to serve.
 *
 * Both calls are idempotent: a repeated initialisation is harmless.
 *
 * Typical use:
 *
 *   1. once, at start-up --
 *          on the requesting core:
 *              ls_otbn_delegation_client_init();
 *          on the peer core:
 *              ls_otbn_delegation_server_init();
 *
 *   2. then call a family entry point per operation, with the same arguments
 *      as its local counterpart, e.g.
 *              ret = mbedtls_ls_otbn_ecdsa_sign(&grp, &r, &s, &d,
 *                                               hash, sizeof(hash), NULL, NULL);
 *
 * What the caller must keep:
 *
 *   - Bring the client up once before the first call. A call made before init
 *     has no reply path and can only time out.
 *   - One delegated call at a time per core; a second concurrent call is
 *     refused with -EACCES.
 *   - The buffers handed to a call must stay alive and untouched until it
 *     returns. After a timeout the peer may still be writing into them.
 *   - Every outcome is a return value, never silence: 0 on success, the peer's
 *     mbedtls error code when it rejected the operation, -EIO (request could
 *     not be sent), -ETIMEDOUT (no answer in time), -EBUSY (the peer's request
 *     queue is full), -EOPNOTSUPP (the peer does not have this core's OTBN
 *     support for that family switched on).
 *
 * The transport itself -- wire format, operation table, timeouts, and how to
 * add another family -- is documented in otbn_delegation.h next to the bus
 * implementation.
 */

#ifndef LS_OTBN_DELEGATION_H
#define LS_OTBN_DELEGATION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief   Bring up the delegation client on this core: register the mailbox
 *          callback and prepare the request/reply bookkeeping. Call once
 *          before the first delegated operation.
 */
void ls_otbn_delegation_client_init(void);

/**
 * \brief   Bring up the delegation server on this core: register the mailbox
 *          callback and start the thread that executes incoming requests.
 *          Call once, on the peer of the core that will send requests.
 */
void ls_otbn_delegation_server_init(void);

#ifdef __cplusplus
}
#endif

#endif /* LS_OTBN_DELEGATION_H */
