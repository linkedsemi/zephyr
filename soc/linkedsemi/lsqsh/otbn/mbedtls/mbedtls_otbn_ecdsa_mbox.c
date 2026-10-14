/*
 * ECDSA family of the OTBN delegation bus.
 *
 * Client side: mbedtls_ls_otbn_ecdsa_genkey()/_sign()/_verify() -- the explicit
 * entry points -- pack a request and hand it to the bus. When the build also
 * asks the delegation to BE the ECDSA alternative, the plain mbedtls_ecdsa_*
 * names forward here as well (see the bottom of this file).
 *
 * Server side: otbn_deleg_ecdsa_*() execute the request with the local OTBN
 * through mbedtls_ecdsa_*.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <string.h>

#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdsa_alt.h"
#include "mbedtls/ecp.h"

#include "otbn_delegation.h"
#include "mbedtls_otbn_ecdsa_mbox.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(mbedtls_otbn_mbox, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

#define MAX_OTBN_CURVE_LEN 48 /* P384 -> 384 bit == 48 byte */

/* ECDSA is one round trip of a few ms; RSA is orders of magnitude slower and
 * passes its own bound, which is why the bus takes the timeout per call. */
#define OTBN_DELEG_TIMEOUT_ECDSA_MS 1000

/* mbedtls_mpi_read_binary_le() can only fail when the value does not fit the
 * MPI, which the fixed curve sizes cannot produce. Keep the historical return
 * value so callers see no change if it ever does. */
#define ECDSA_MPI_READ(dst, src, len)                                        \
	do {                                                                 \
		if (mbedtls_mpi_read_binary_le((dst), (src), (len)) != 0) {  \
			return MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL;             \
		}                                                            \
	} while (0)

/* Same, for the server handlers: they report through `err` and the shared
 * cleanup at `out` rather than returning straight away, so a partially
 * initialised context still gets freed. */
#define ECDSA_MPI_READ_S(err_out, dst, src, len)                             \
	do {                                                                 \
		(err_out) = mbedtls_mpi_read_binary_le((dst), (src), (len));  \
		if ((err_out) != 0) {                                        \
			goto out;                                            \
		}                                                            \
	} while (0)

/* Exporting into a fixed-size buffer can fail the same way; do not leave it
 * unchecked -- a silently empty private key or signature would be sent to the
 * peer as if it were real. */
#define ECDSA_MPI_WRITE(x, buf, len)                                         \
	do {                                                                 \
		if (mbedtls_mpi_write_binary_le((x), (buf), (len)) != 0) {   \
			return MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL;             \
		}                                                            \
	} while (0)

#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

static size_t ecdsa_curve_size(mbedtls_ecp_group_id gid)
{
	if (gid == MBEDTLS_ECP_DP_SECP384R1) {
		return 48;
	}
	if (gid == MBEDTLS_ECP_DP_SECP256R1 || gid == MBEDTLS_ECP_DP_SM2) {
		return 32;
	}
	return 0;
}

int mbedtls_ls_otbn_ecdsa_genkey(mbedtls_ecdsa_context *ctx, mbedtls_ecp_group_id gid,
				 int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
	int err;
	size_t n_size = ecdsa_curve_size(gid);
	__attribute__((aligned(32))) uint8_t public_key[MAX_OTBN_CURVE_LEN * 2];
	__attribute__((aligned(32))) uint8_t private_key[MAX_OTBN_CURVE_LEN];
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;

	/* The peer's whitelist is the same set; failing here saves a round trip. */
	if (n_size == 0) {
		return -EIO;
	}
	/* f_rng/p_rng are unused: the peer generates the key pair and supplies its
	 * own entropy. Kept in the signature so this is a drop-in for
	 * mbedtls_ecdsa_genkey(). */
	ARG_UNUSED(f_rng);
	ARG_UNUSED(p_rng);

	/* mbedtls_ecdsa_genkey() loads the group and callers rely on ctx->grp
	 * afterwards (mbedtls_ecp_check_pubkey(), point export). Keep that. */
	err = mbedtls_ecp_group_load(&ctx->private_grp, gid);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(private_key, MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(public_key, MAX_OTBN_CURVE_LEN * 2);

	req.op = OTBN_DELEG_OP_ECDSA_GENKEY;
	req.arg[0] = (uint32_t)gid;
	req.data[0] = private_key;
	req.data[1] = public_key;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status != 0) {
		/* Peer reported a failure: its result buffers are not valid. */
		return rpy->status;
	}

	otbn_deleg_cache_invd(rpy->data[0], n_size);
	otbn_deleg_cache_invd(rpy->data[1], n_size * 2);
	ECDSA_MPI_READ(&ctx->private_d, rpy->data[0], n_size);
	ECDSA_MPI_READ(&ctx->private_Q.private_X, rpy->data[1], n_size);
	ECDSA_MPI_READ(&ctx->private_Q.private_Y, rpy->data[1] + n_size, n_size);
	/* data[1] carries affine X||Y; set Z=1 so this is a real affine point
	 * rather than the Z==0 point at infinity that check_pubkey() rejects. */
	mbedtls_mpi_lset(&ctx->private_Q.private_Z, 1);

	LOG_HEXDUMP_DBG(rpy->data[0], n_size, "private key");
	LOG_HEXDUMP_DBG(rpy->data[1], n_size * 2, "public_x key");
	return 0;
}

int mbedtls_ls_otbn_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
			       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
			       int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
	int err;
	/* Derived from the whitelist rather than from grp->nbits: the buffers
	 * below are MAX_OTBN_CURVE_LEN wide, and a group this proxy does not
	 * support (secp521r1, ...) would yield a larger n_size and overflow
	 * them. Validating here also fails before spending a round trip. */
	size_t n_size = ecdsa_curve_size(grp->id);
	size_t buf_len;
	__attribute__((aligned(32))) uint8_t r_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t s_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t private_key[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t user_buf[MAX_OTBN_CURVE_LEN] = {0};
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;

	if (n_size == 0) {
		return -EIO;
	}
	buf_len = blen > n_size ? n_size : blen;

	/* The peer supplies the per-signature nonce. */
	ARG_UNUSED(f_rng);
	ARG_UNUSED(p_rng);

	memcpy(user_buf, buf, buf_len);
	ECDSA_MPI_WRITE(d, private_key, n_size);

	otbn_deleg_cache_flush(private_key, MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(user_buf, MAX_OTBN_CURVE_LEN);

	req.op = OTBN_DELEG_OP_ECDSA_SIGN;
	req.arg[0] = (uint32_t)grp->id;
	req.arg[1] = (uint32_t)buf_len;
	req.data[0] = private_key;
	req.data[1] = user_buf;
	req.data[2] = r_buf;
	req.data[3] = s_buf;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status != 0) {
		return rpy->status;
	}

	otbn_deleg_cache_invd(rpy->data[2], n_size);
	otbn_deleg_cache_invd(rpy->data[3], n_size);
	ECDSA_MPI_READ(r, rpy->data[2], n_size);
	ECDSA_MPI_READ(s, rpy->data[3], n_size);

	LOG_HEXDUMP_DBG(rpy->data[2], n_size, "sign r_buf");
	LOG_HEXDUMP_DBG(rpy->data[3], n_size, "sign s_buf");
	return 0;
}

int mbedtls_ls_otbn_ecdsa_verify(mbedtls_ecp_group *grp,
				 const unsigned char *buf, size_t blen,
				 const mbedtls_ecp_point *Q,
				 const mbedtls_mpi *r,
				 const mbedtls_mpi *s)
{
	int err;
	size_t n_size = ecdsa_curve_size(grp->id);
	size_t buf_len;
	__attribute__((aligned(32))) uint8_t r_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t s_buf[MAX_OTBN_CURVE_LEN];
	__attribute__((aligned(32))) uint8_t user_buf[MAX_OTBN_CURVE_LEN] = {0};
	__attribute__((aligned(32))) uint8_t public_key[MAX_OTBN_CURVE_LEN * 2];
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;

	/* See the note in _sign(): derive the size from the whitelist so an
	 * unsupported group cannot overflow the fixed buffers. */
	if (n_size == 0) {
		return -EIO;
	}
	buf_len = blen > n_size ? n_size : blen;

	memcpy(user_buf, buf, buf_len);
	ECDSA_MPI_WRITE(&Q->private_X, public_key, n_size);
	ECDSA_MPI_WRITE(&Q->private_Y, public_key + n_size, n_size);
	ECDSA_MPI_WRITE(r, r_buf, n_size);
	ECDSA_MPI_WRITE(s, s_buf, n_size);

	otbn_deleg_cache_flush(public_key, MAX_OTBN_CURVE_LEN * 2);
	otbn_deleg_cache_flush(user_buf, MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(r_buf, MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(s_buf, MAX_OTBN_CURVE_LEN);

	req.op = OTBN_DELEG_OP_ECDSA_VERIFY;
	req.arg[0] = (uint32_t)grp->id;
	req.arg[1] = (uint32_t)buf_len;
	req.data[0] = public_key;
	req.data[1] = user_buf;
	req.data[2] = r_buf;
	req.data[3] = s_buf;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status != 0) {
		LOG_DBG("ecdsa verification failed or input parameters are incorrect");
	}
	return rpy->status;
}

/* Only when the build asks the delegation to BE the ECDSA alternative do we
 * take the mbedtls entry points over, so that anything reaching ECDSA through
 * the mbedtls API (TLS, pk, ...) is delegated as well. Otherwise ecdsa_alt.c
 * (local OTBN) or ecdsa.c (software) owns these symbols and defining them here
 * would collide -- this is what keeps the delegation client independent of the
 * OTBN ECDSA alternative switch.
 * The proxy body itself lives in the mbedtls_ls_otbn_ecdsa_* functions above:
 * those are built and callable by name whenever the delegation client is
 * enabled, regardless of this option. See mbedtls_otbn_crypto_mbox.h.
 */
#if defined(CONFIG_MBEDTLS_ECDSA_LINKEDSEMI_OTBN_ALT_VIA_DELEGATION)
int mbedtls_ecdsa_genkey(mbedtls_ecdsa_context *ctx, mbedtls_ecp_group_id gid,
			 int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
	return mbedtls_ls_otbn_ecdsa_genkey(ctx, gid, f_rng, p_rng);
}

int mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
		       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
		       int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
	return mbedtls_ls_otbn_ecdsa_sign(grp, r, s, d, buf, blen, f_rng, p_rng);
}

int mbedtls_ecdsa_verify(mbedtls_ecp_group *grp,
			 const unsigned char *buf, size_t blen,
			 const mbedtls_ecp_point *Q,
			 const mbedtls_mpi *r,
			 const mbedtls_mpi *s)
{
	return mbedtls_ls_otbn_ecdsa_verify(grp, buf, blen, Q, r, s);
}
#endif /* CONFIG_MBEDTLS_ECDSA_LINKEDSEMI_OTBN_ALT_VIA_DELEGATION */

#else /* server */

static const struct device *otbn_deleg_trng = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));

static int ecdsa_server_get_random(void *unused, unsigned char *buf, size_t size)
{
	ARG_UNUSED(unused);
	return entropy_get_entropy(otbn_deleg_trng, buf, size);
}

static bool ecdsa_curve_supported(mbedtls_ecp_group_id gid)
{
	return gid == MBEDTLS_ECP_DP_SECP384R1 || gid == MBEDTLS_ECP_DP_SECP256R1 ||
	       gid == MBEDTLS_ECP_DP_SM2;
}

static void ecdsa_ctx_init(mbedtls_ecdsa_context *ctx, mbedtls_mpi *r, mbedtls_mpi *s)
{
	mbedtls_ecp_group_init(&ctx->private_grp);
	mbedtls_mpi_init(&ctx->private_Q.private_X);
	mbedtls_mpi_init(&ctx->private_Q.private_Y);
	mbedtls_mpi_init(&ctx->private_Q.private_Z);
	mbedtls_mpi_init(&ctx->private_d);
	mbedtls_mpi_init(r);
	mbedtls_mpi_init(s);
}

static void ecdsa_ctx_free(mbedtls_ecdsa_context *ctx, mbedtls_mpi *r, mbedtls_mpi *s)
{
	mbedtls_ecp_group_free(&ctx->private_grp);
	mbedtls_mpi_free(&ctx->private_Q.private_X);
	mbedtls_mpi_free(&ctx->private_Q.private_Y);
	mbedtls_mpi_free(&ctx->private_Q.private_Z);
	mbedtls_mpi_free(&ctx->private_d);
	mbedtls_mpi_free(r);
	mbedtls_mpi_free(s);
}

int otbn_deleg_ecdsa_genkey(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply)
{
	mbedtls_ecdsa_context ctx;
	mbedtls_mpi r, s;
	mbedtls_ecp_group_id gid = (mbedtls_ecp_group_id)req->arg[0];
	uint8_t curve_size;
	int err;

	if (!ecdsa_curve_supported(gid)) {
		LOG_WRN("delegation: unsupported curve 0x%x", (unsigned int)gid);
		return -EOPNOTSUPP;
	}

	ecdsa_ctx_init(&ctx, &r, &s);
	err = mbedtls_ecp_group_load(&ctx.private_grp, gid);
	if (err) {
		goto out;
	}
	curve_size = (ctx.private_grp.nbits + 7) / 8;

	otbn_deleg_cache_invd(req->data[0], MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_invd(req->data[1], MAX_OTBN_CURVE_LEN * 2);

	err = mbedtls_ecdsa_genkey(&ctx, gid, ecdsa_server_get_random, NULL);
	if (err) {
		goto out;
	}

	mbedtls_mpi_write_binary_le(&ctx.private_d, req->data[0], curve_size);
	mbedtls_mpi_write_binary_le(&ctx.private_Q.private_X, req->data[1], curve_size);
	mbedtls_mpi_write_binary_le(&ctx.private_Q.private_Y, req->data[1] + curve_size, curve_size);
	otbn_deleg_cache_flush(req->data[0], MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(req->data[1], MAX_OTBN_CURVE_LEN * 2);
	reply->data[0] = req->data[0];
	reply->data[1] = req->data[1];

	LOG_HEXDUMP_DBG(req->data[0], curve_size, "private key");
	LOG_HEXDUMP_DBG(req->data[1], curve_size * 2, "public_x key");

out:
	ecdsa_ctx_free(&ctx, &r, &s);
	return err;
}

int otbn_deleg_ecdsa_sign(const struct otbn_delegate_params *req,
			  struct otbn_delegate_params *reply)
{
	mbedtls_ecdsa_context ctx;
	mbedtls_mpi r, s;
	mbedtls_ecp_group_id gid = (mbedtls_ecp_group_id)req->arg[0];
	uint8_t curve_size;
	int err;

	if (!ecdsa_curve_supported(gid)) {
		LOG_WRN("delegation: unsupported curve 0x%x", (unsigned int)gid);
		return -EOPNOTSUPP;
	}

	ecdsa_ctx_init(&ctx, &r, &s);
	err = mbedtls_ecp_group_load(&ctx.private_grp, gid);
	if (err) {
		goto out;
	}
	curve_size = (ctx.private_grp.nbits + 7) / 8;

	otbn_deleg_cache_invd(req->data[0], curve_size);
	otbn_deleg_cache_invd(req->data[1], req->arg[1]);
	ECDSA_MPI_READ_S(err, &ctx.private_d, req->data[0], curve_size);

	err = mbedtls_ecdsa_sign(&ctx.private_grp, &r, &s, &ctx.private_d,
				 req->data[1], req->arg[1], ecdsa_server_get_random, NULL);
	if (err) {
		goto out;
	}

	mbedtls_mpi_write_binary_le(&r, req->data[2], curve_size);
	mbedtls_mpi_write_binary_le(&s, req->data[3], curve_size);
	otbn_deleg_cache_flush(req->data[2], MAX_OTBN_CURVE_LEN);
	otbn_deleg_cache_flush(req->data[3], MAX_OTBN_CURVE_LEN);
	reply->data[2] = req->data[2];
	reply->data[3] = req->data[3];

	LOG_HEXDUMP_DBG(req->data[2], curve_size, "sign r_buf");
	LOG_HEXDUMP_DBG(req->data[3], curve_size, "sign s_buf");

out:
	ecdsa_ctx_free(&ctx, &r, &s);
	return err;
}

int otbn_deleg_ecdsa_verify(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply)
{
	mbedtls_ecdsa_context ctx;
	mbedtls_mpi r, s;
	mbedtls_ecp_group_id gid = (mbedtls_ecp_group_id)req->arg[0];
	uint8_t curve_size;
	int err;

	ARG_UNUSED(reply);

	if (!ecdsa_curve_supported(gid)) {
		LOG_WRN("delegation: unsupported curve 0x%x", (unsigned int)gid);
		return -EOPNOTSUPP;
	}

	ecdsa_ctx_init(&ctx, &r, &s);
	err = mbedtls_ecp_group_load(&ctx.private_grp, gid);
	if (err) {
		goto out;
	}
	curve_size = (ctx.private_grp.nbits + 7) / 8;

	otbn_deleg_cache_invd(req->data[0], curve_size * 2);
	otbn_deleg_cache_invd(req->data[1], req->arg[1]);
	otbn_deleg_cache_invd(req->data[2], curve_size);
	otbn_deleg_cache_invd(req->data[3], curve_size);

	ECDSA_MPI_READ_S(err, &ctx.private_Q.private_X, req->data[0], curve_size);
	ECDSA_MPI_READ_S(err, &ctx.private_Q.private_Y, req->data[0] + curve_size, curve_size);
	/* data[0] carries affine X||Y; set Z=1 (see the client-side genkey note). */
	mbedtls_mpi_lset(&ctx.private_Q.private_Z, 1);
	ECDSA_MPI_READ_S(err, &r, req->data[2], curve_size);
	ECDSA_MPI_READ_S(err, &s, req->data[3], curve_size);

	err = mbedtls_ecdsa_verify(&ctx.private_grp, req->data[1], req->arg[1],
				   &ctx.private_Q, &r, &s);

out:
	ecdsa_ctx_free(&ctx, &r, &s);
	return err;
}

#endif /* CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT */
