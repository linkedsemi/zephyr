/*
 * Slot-based ECDSA signing over the OTBN delegation bus.
 *
 * Client side (the requesting core): mbedtls_ls_otbn_ecdsa_sign_slot() packs a
 * request whose data[0] points at a slot id instead of a private key.
 *
 * Server side (the core that owns the OTBN): the key table below keeps a
 * borrowed pointer per slot; otbn_deleg_ecdsa_sign_slot() looks the key up and
 * runs the local mbedtls/OTBN signing path. See mbedtls_otbn_ecdsa_slot_mbox.h for
 * the lifetime contract of the borrowed buffer.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>

#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"
#include "mbedtls/platform_util.h"

#include "otbn_delegation.h"
#include "mbedtls_otbn_ecdsa_slot_mbox.h"

#define MAX_OTBN_CURVE_LEN 48 /* P384 -> 384 bit == 48 byte */

/* ECDSA is one round trip of a few ms; keep the same bound as the other
 * ECDSA family entry points (mbedtls_otbn_ecdsa_mbox.c). */
#define OTBN_DELEG_TIMEOUT_ECDSA_MS 1000

/* Occupancy marker in the descriptor's tail. The whole table is cleared at
 * init and unregister zeroes the descriptor, so 0 means "never written";
 * register stores this non-zero value last under the lock. */
#define OTBN_KEY_SLOT_WRITTEN 0x5A5Au

static size_t slot_curve_size(mbedtls_ecp_group_id gid)
{
	if (gid == MBEDTLS_ECP_DP_SECP384R1) {
		return 48;
	}
	if (gid == MBEDTLS_ECP_DP_SECP256R1 || gid == MBEDTLS_ECP_DP_SM2) {
		return 32;
	}
	return 0;
}

#if defined(CONFIG_LS_OTBN_DELEGATION_CLIENT)

int mbedtls_ls_otbn_ecdsa_sign_slot(mbedtls_ecp_group *grp,
				    mbedtls_mpi *r, mbedtls_mpi *s,
				    uint32_t slot_id,
				    const unsigned char *buf, size_t blen,
				    int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
	int err;
	size_t n_size;
	size_t buf_len;
	__attribute__((aligned(32))) uint32_t slot = slot_id;
	__attribute__((aligned(32))) uint8_t r_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t s_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t user_buf[MAX_OTBN_CURVE_LEN] = {0};
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;

	if (grp == NULL || r == NULL || s == NULL || buf == NULL) {
		return -EINVAL;
	}

	/* Derive the size from the whitelist instead of grp->nbits: the buffers
	 * above are MAX_OTBN_CURVE_LEN wide and an unsupported group would make
	 * a larger n_size overflow them. Failing here also saves a round trip. */
	n_size = slot_curve_size(grp->id);
	if (n_size == 0) {
		return -EIO;
	}
	buf_len = blen > n_size ? n_size : blen;

	memcpy(user_buf, buf, buf_len);

	/* With a caller RNG the nonce is generated here; r_buf doubles as the
	 * nonce input and the r output. Without one the peer supplies the nonce. */
	if (f_rng != NULL) {
		err = f_rng(p_rng, r_buf, n_size);
		if (err != 0) {
			return err;
		}
		otbn_deleg_cache_flush(r_buf, MAX_OTBN_CURVE_LEN);
	} else {
		ARG_UNUSED(p_rng);
	}

	/* data[0] carries the slot id, not a key. Both the id and the hash are
	 * written by this core and read by the peer. */
	otbn_deleg_cache_flush(&slot, sizeof(slot));
	otbn_deleg_cache_flush(user_buf, sizeof(user_buf));
	/* r/s are outputs. Clean any dirty stack line before the peer writes them;
	 * otherwise an eviction while this call waits could write stale data back
	 * over the peer's result. */
	otbn_deleg_cache_flush(r_buf, MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(s_buf, MAX_OTBN_CURVE_LEN);

	req.op = (f_rng != NULL) ? OTBN_DELEG_OP_ECDSA_SIGN_SLOT_RNG
				 : OTBN_DELEG_OP_ECDSA_SIGN_SLOT;
	req.arg[0] = (uint32_t)grp->id;
	req.arg[1] = (uint32_t)buf_len;
	req.data[0] = (uint8_t *)&slot;
	req.data[1] = user_buf;
	req.data[2] = r_buf;
	req.data[3] = s_buf;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err != 0) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status != 0) {
		/* Peer reported a failure: its result buffers are not valid. */
		return rpy->status;
	}

	otbn_deleg_cache_invd(rpy->data[2], n_size);
	otbn_deleg_cache_invd(rpy->data[3], n_size);

	err = mbedtls_mpi_read_binary_le(r, rpy->data[2], n_size);
	if (err != 0) {
		return err;
	}
	return mbedtls_mpi_read_binary_le(s, rpy->data[3], n_size);
}

#endif /* CONFIG_LS_OTBN_DELEGATION_CLIENT */

#if defined(CONFIG_LS_OTBN_DELEGATION_SERVER)

/*
 * Key table. The key bytes are borrowed from the registering application; only
 * the descriptor lives here. written is stored last so a partially updated
 * descriptor is never accepted by a concurrent signing request.
 *
 * 32-bit layout, no implicit padding between fields:
 *   d 0..3, gid 4, klen 5, written 6..7 (sizeof 8).
 */
struct otbn_key_slot {
	const uint8_t *d;
	uint8_t gid;
	uint8_t klen;
	uint16_t written;
};

static struct otbn_key_slot key_slots[CONFIG_LS_OTBN_ECDSA_SLOT_MBOX_COUNT];
static K_MUTEX_DEFINE(key_slots_lock);

static const struct device *slot_trng = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

static int slot_get_random(void *unused, unsigned char *buf, size_t size)
{
	ARG_UNUSED(unused);
	return entropy_get_entropy(slot_trng, buf, size);
}

/* One-shot RNG: serves the client-supplied nonce exactly once. The OTBN alt
 * ignores the f_rng return value, so the handler also checks `used`. */
struct otbn_deleg_rng_once {
	const uint8_t *bytes;
	size_t size;
	bool used;
};

static int otbn_deleg_rng_once(void *ctx, unsigned char *buf, size_t size)
{
	struct otbn_deleg_rng_once *rng = ctx;

	if (rng->used || size != rng->size) {
		memset(buf, 0, size);
		return MBEDTLS_ERR_ECP_RANDOM_FAILED;
	}
	memcpy(buf, rng->bytes, size);
	rng->used = true;
	return 0;
}

/* mbedtls_mpi_free() releases the limbs but does not scrub them. The key
 * passes through an MPI during validation and signing, so wipe it first.
 * MBEDTLS_PRIVATE() resolves the member name for this mbedtls version. */
static void slot_mpi_zeroize(mbedtls_mpi *x)
{
	if (x->MBEDTLS_PRIVATE(p) != NULL) {
		mbedtls_platform_zeroize(x->MBEDTLS_PRIVATE(p),
					 x->MBEDTLS_PRIVATE(n) * sizeof(mbedtls_mpi_uint));
	}
	mbedtls_mpi_free(x);
}

static int slot_key_validate(mbedtls_ecp_group_id gid, const uint8_t *d, size_t klen)
{
	mbedtls_ecp_group grp;
	mbedtls_mpi key;
	int err;

	mbedtls_ecp_group_init(&grp);
	mbedtls_mpi_init(&key);

	err = mbedtls_ecp_group_load(&grp, gid);
	if (err == 0) {
		err = mbedtls_mpi_read_binary_le(&key, d, klen);
	}
	if (err == 0) {
		err = mbedtls_ecp_check_privkey(&grp, &key);
	}

	slot_mpi_zeroize(&key);
	mbedtls_ecp_group_free(&grp);
	return err;
}

/* Clear every descriptor before any application init can register a key:
 * .bss already starts zeroed, but doing this explicitly keeps the contract if
 * the table later moves to a retained or custom section, and PRE_KERNEL_1
 * guarantees it cannot wipe a registration made from an application SYS_INIT. */
static int key_slots_init(void)
{
	memset(key_slots, 0, sizeof(key_slots));
	return 0;
}

SYS_INIT(key_slots_init, PRE_KERNEL_1, 0);

int ls_otbn_ecdsa_key_slot_register(uint32_t slot_id,
				    mbedtls_ecp_group_id gid,
				    const uint8_t *d)
{
	size_t klen;
	int err;

	if (slot_id >= ARRAY_SIZE(key_slots) || d == NULL) {
		return -EINVAL;
	}

	klen = slot_curve_size(gid);
	if (klen == 0) {
		return -EOPNOTSUPP;
	}

	/* Validate before publishing the descriptor: a bad scalar must never
	 * reach the signing path, and the caller's buffer must already hold the
	 * fixed-size little-endian value at this point. */
	err = slot_key_validate(gid, d, klen);
	if (err != 0) {
		return err;
	}

	k_mutex_lock(&key_slots_lock, K_FOREVER);
	key_slots[slot_id].written = 0;
	key_slots[slot_id].d = d;
	key_slots[slot_id].gid = (uint8_t)gid;
	key_slots[slot_id].klen = (uint8_t)klen;
	key_slots[slot_id].written = OTBN_KEY_SLOT_WRITTEN;
	k_mutex_unlock(&key_slots_lock);

	return 0;
}

int ls_otbn_ecdsa_key_slot_unregister(uint32_t slot_id)
{
	if (slot_id >= ARRAY_SIZE(key_slots)) {
		return -EINVAL;
	}

	k_mutex_lock(&key_slots_lock, K_FOREVER);
	if (key_slots[slot_id].written == 0) {
		k_mutex_unlock(&key_slots_lock);
		return -ENOENT;
	}
	/* Descriptor only: the key buffer belongs to the caller. */
	mbedtls_platform_zeroize(&key_slots[slot_id], sizeof(key_slots[slot_id]));
	k_mutex_unlock(&key_slots_lock);

	return 0;
}

int otbn_deleg_ecdsa_sign_slot(const struct otbn_delegate_params *req,
			       struct otbn_delegate_params *reply)
{
	mbedtls_ecp_group grp;
	mbedtls_mpi key, r, s;
	uint8_t key_buf[MAX_OTBN_CURVE_LEN];
	mbedtls_ecp_group_id gid;
	size_t n_size;
	uint32_t slot_id;
	uint8_t klen;
	int err;

	if (req->data[0] == NULL || req->data[1] == NULL ||
	    req->data[2] == NULL || req->data[3] == NULL) {
		return -EINVAL;
	}

	/* data[0] points at a uint32_t in the requesting core's memory. */
	otbn_deleg_cache_invd(req->data[0], sizeof(slot_id));
	memcpy(&slot_id, req->data[0], sizeof(slot_id));
	if (slot_id >= ARRAY_SIZE(key_slots)) {
		return -ENOENT;
	}

	mbedtls_ecp_group_init(&grp);
	mbedtls_mpi_init(&key);
	mbedtls_mpi_init(&r);
	mbedtls_mpi_init(&s);
	err = 0;

	k_mutex_lock(&key_slots_lock, K_FOREVER);
	if (key_slots[slot_id].written == 0 ||
	    key_slots[slot_id].d == NULL) {
		k_mutex_unlock(&key_slots_lock);
		err = -ENOENT;
		goto out;
	}

	gid = (mbedtls_ecp_group_id)key_slots[slot_id].gid;
	klen = key_slots[slot_id].klen;
	n_size = slot_curve_size(gid);
	/* Validate everything that does not need the key before copying it: a
	 * corrupted descriptor, a curve mismatch, or a bogus length must not
	 * touch the private scalar at all. */
	if (n_size == 0 || klen != n_size ||
	    (uint32_t)gid != req->arg[0] || req->arg[1] > n_size) {
		k_mutex_unlock(&key_slots_lock);
		err = -EINVAL;
		goto out;
	}
	memcpy(key_buf, key_slots[slot_id].d, klen);
	k_mutex_unlock(&key_slots_lock);

	otbn_deleg_cache_invd(req->data[1], req->arg[1]);

	err = mbedtls_ecp_group_load(&grp, gid);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_mpi_read_binary_le(&key, key_buf, klen);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_ecdsa_sign(&grp, &r, &s, &key, req->data[1], req->arg[1],
				 slot_get_random, NULL);
	if (err != 0) {
		goto out;
	}

	err = mbedtls_mpi_write_binary_le(&r, req->data[2], n_size);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_mpi_write_binary_le(&s, req->data[3], n_size);
	if (err != 0) {
		goto out;
	}

	otbn_deleg_cache_flush(req->data[2], n_size);
	otbn_deleg_cache_flush(req->data[3], n_size);
	reply->data[2] = req->data[2];
	reply->data[3] = req->data[3];

out:
	slot_mpi_zeroize(&key);
	mbedtls_mpi_free(&r);
	mbedtls_mpi_free(&s);
	mbedtls_ecp_group_free(&grp);
	mbedtls_platform_zeroize(key_buf, sizeof(key_buf));
	return err;
}

int otbn_deleg_ecdsa_sign_slot_rng(const struct otbn_delegate_params *req,
				   struct otbn_delegate_params *reply)
{
	mbedtls_ecp_group grp;
	mbedtls_mpi key, r, s;
	uint8_t key_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t nonce_buf[MAX_OTBN_CURVE_LEN];
	struct otbn_deleg_rng_once rng;
	mbedtls_ecp_group_id gid;
	size_t n_size;
	uint32_t slot_id;
	uint8_t klen;
	int err;

	if (req->data[0] == NULL || req->data[1] == NULL ||
	    req->data[2] == NULL || req->data[3] == NULL) {
		return -EINVAL;
	}

	otbn_deleg_cache_invd(req->data[0], sizeof(slot_id));
	memcpy(&slot_id, req->data[0], sizeof(slot_id));
	if (slot_id >= ARRAY_SIZE(key_slots)) {
		return -ENOENT;
	}

	mbedtls_ecp_group_init(&grp);
	mbedtls_mpi_init(&key);
	mbedtls_mpi_init(&r);
	mbedtls_mpi_init(&s);
	err = 0;

	k_mutex_lock(&key_slots_lock, K_FOREVER);
	if (key_slots[slot_id].written == 0 ||
	    key_slots[slot_id].d == NULL) {
		k_mutex_unlock(&key_slots_lock);
		err = -ENOENT;
		goto out;
	}

	gid = (mbedtls_ecp_group_id)key_slots[slot_id].gid;
	klen = key_slots[slot_id].klen;
	n_size = slot_curve_size(gid);
	if (n_size == 0 || klen != n_size ||
	    (uint32_t)gid != req->arg[0] || req->arg[1] > n_size) {
		k_mutex_unlock(&key_slots_lock);
		err = -EINVAL;
		goto out;
	}
	memcpy(key_buf, key_slots[slot_id].d, klen);
	k_mutex_unlock(&key_slots_lock);

	otbn_deleg_cache_invd(req->data[1], req->arg[1]);
	otbn_deleg_cache_invd(req->data[2], n_size);
	memcpy(nonce_buf, req->data[2], n_size);
	rng.bytes = nonce_buf;
	rng.size = n_size;
	rng.used = false;

	err = mbedtls_ecp_group_load(&grp, gid);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_mpi_read_binary_le(&key, key_buf, klen);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_ecdsa_sign(&grp, &r, &s, &key, req->data[1], req->arg[1],
				 otbn_deleg_rng_once, &rng);
	if (err == 0 && !rng.used) {
		err = MBEDTLS_ERR_ECP_RANDOM_FAILED;
	}
	if (err != 0) {
		goto out;
	}

	/* r overwrites the nonce in data[2]; the client reads it back. */
	err = mbedtls_mpi_write_binary_le(&r, req->data[2], n_size);
	if (err != 0) {
		goto out;
	}
	err = mbedtls_mpi_write_binary_le(&s, req->data[3], n_size);
	if (err != 0) {
		goto out;
	}

	otbn_deleg_cache_flush(req->data[2], n_size);
	otbn_deleg_cache_flush(req->data[3], n_size);
	reply->data[2] = req->data[2];
	reply->data[3] = req->data[3];

out:
	slot_mpi_zeroize(&key);
	mbedtls_mpi_free(&r);
	mbedtls_mpi_free(&s);
	mbedtls_ecp_group_free(&grp);
	mbedtls_platform_zeroize(key_buf, sizeof(key_buf));
	mbedtls_platform_zeroize(nonce_buf, sizeof(nonce_buf));
	return err;
}

#endif /* CONFIG_LS_OTBN_DELEGATION_SERVER */
