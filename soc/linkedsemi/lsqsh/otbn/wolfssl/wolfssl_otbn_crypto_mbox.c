/*
 * wolfSSL family of the OTBN delegation bus.
 *
 * This file used to carry wolfSSL's own delegation channel (its own mbox
 * callback, k_msgq, worker and request struct). That could not coexist with the
 * mbedtls family: the linkedsemi mbox driver keeps one callback slot per
 * channel, so whichever family registered second silently orphaned the first.
 * The channel mechanism is gone; what is left is a family on the shared bus:
 *
 *   client side -- the wolfssl port's ls_otbn_* / ls_rsa_modexp_* entry points
 *   (called from ecc.c / rsa_modexp.c) pack a request and hand it to the bus.
 *
 *   server side -- otbn_deleg_wolfssl_*() execute the request with the wolfssl
 *   port's own local OTBN routines (ls_otbn_* in ecc.c, ls_rsa_modexp_* in
 *   rsa_modexp.c). No OTBN sequence is reimplemented here.
 *
 * Wire contract (see otbn_delegation.h for the op codes):
 *   arg[0] = the wolfssl curve id (ECC_SECP256R1/ECC_SECP384R1/ECC_SM2P256V1),
 *            or the RSA modulus size in bits for the RSA ops;
 *   arg[1] = the curve size in bytes (ECC ops only);
 *   data[] = the shared-SRAM buffers, in the order the handlers below use.
 */

#include <zephyr/kernel.h>

/* The wolfssl public types (ECC_*, WC_HW_E) come from these; keep the same
 * settings preamble the rest of the port uses. */
#ifndef WOLFSSL_USER_SETTINGS
#define WOLFSSL_USER_SETTINGS
#endif
#ifndef WOLFSSL_ZEPHYR
#define WOLFSSL_ZEPHYR
#endif
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/types.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include <wolfssl/wolfcrypt/ecc.h>
#include <wolfssl/wolfcrypt/port/linkedsemi/ls-rsa.h>

#include "otbn_delegation.h"

/* ECDSA is one round trip of a few ms; RSA modexp is orders of magnitude
 * slower. The bus takes the bound per call, so each op passes its own. */
#define OTBN_DELEG_TIMEOUT_ECDSA_MS 1000
#define OTBN_DELEG_TIMEOUT_RSA_MS   10000

/* The curves the wolfssl OTBN firmware implements (get_otbn_fireware_id()). */
static bool wolfssl_curve_supported(uint32_t curve)
{
	return curve == ECC_SECP256R1 || curve == ECC_SECP384R1 ||
	       curve == ECC_SM2P256V1;
}

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

int ls_otbn_get_key_pair(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
			 uint8_t *public_x, uint8_t *public_y)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	int err;

	otbn_deleg_cache_flush(private_key, curve_size);
	otbn_deleg_cache_flush(public_x, curve_size);
	otbn_deleg_cache_flush(public_y, curve_size);

	req.op = OTBN_DELEG_OP_WOLFSSL_GENKEY;
	req.arg[0] = curve;
	req.arg[1] = curve_size;
	req.data[0] = private_key;
	req.data[1] = public_x;
	req.data[2] = public_y;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		otbn_deleg_cache_invd(rpy->data[0], curve_size);
		otbn_deleg_cache_invd(rpy->data[1], curve_size);
		otbn_deleg_cache_invd(rpy->data[2], curve_size);
	}
	return rpy->status;
}

int ls_otbn_sign_hash(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
		      uint8_t *msg, uint8_t *r, uint8_t *s)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	int err;

	if (!wolfssl_curve_supported(curve)) {
		return WC_HW_E;
	}

	otbn_deleg_cache_flush(private_key, curve_size);
	otbn_deleg_cache_flush(msg, curve_size);
	otbn_deleg_cache_flush(r, curve_size);
	otbn_deleg_cache_flush(s, curve_size);

	req.op = OTBN_DELEG_OP_WOLFSSL_SIGN;
	req.arg[0] = curve;
	req.arg[1] = curve_size;
	req.data[0] = private_key;
	req.data[1] = msg;
	req.data[2] = r;
	req.data[3] = s;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		otbn_deleg_cache_invd(rpy->data[2], curve_size);
		otbn_deleg_cache_invd(rpy->data[3], curve_size);
	}
	return rpy->status;
}

int ls_otbn_verify_hash(uint32_t curve, uint32_t curve_size, uint8_t *r, uint8_t *s,
			uint8_t *msg, uint8_t *pub_x, uint8_t *pub_y)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	int err;

	if (!wolfssl_curve_supported(curve)) {
		return WC_HW_E;
	}

	otbn_deleg_cache_flush(r, curve_size);
	otbn_deleg_cache_flush(s, curve_size);
	otbn_deleg_cache_flush(msg, curve_size);
	otbn_deleg_cache_flush(pub_x, curve_size);
	otbn_deleg_cache_flush(pub_y, curve_size);

	req.op = OTBN_DELEG_OP_WOLFSSL_VERIFY;
	req.arg[0] = curve;
	req.arg[1] = curve_size;
	req.data[0] = r;
	req.data[1] = s;
	req.data[2] = msg;
	req.data[3] = pub_x;
	req.data[4] = pub_y;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		/* The local routine stores the OTBN r_x into `s`; that buffer is
		 * the one the caller (ecc.c) reads back to decide the verdict. */
		otbn_deleg_cache_invd(rpy->data[1], curve_size);
	}
	return rpy->status;
}

int ls_otbn_shared_secret(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
			  uint8_t *public_x, uint8_t *public_y)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	int err;

	otbn_deleg_cache_flush(private_key, curve_size);
	otbn_deleg_cache_flush(public_x, curve_size);
	otbn_deleg_cache_flush(public_y, curve_size);

	req.op = OTBN_DELEG_OP_WOLFSSL_SHARED_SECRET;
	req.arg[0] = curve;
	req.arg[1] = curve_size;
	req.data[0] = private_key;
	req.data[1] = public_x;
	req.data[2] = public_y;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_ECDSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		/* The result is written back into public_x/public_y. */
		otbn_deleg_cache_invd(rpy->data[1], curve_size);
		otbn_deleg_cache_invd(rpy->data[2], curve_size);
	}
	return rpy->status;
}

int ls_rsa_modexp_encrypt(const uint8_t *in, uint32_t inLen, uint8_t *out,
			  uint32_t *outLen, uint8_t *exp, const uint8_t *key_n,
			  uint32_t n_size)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	uint32_t num_bytes = n_size / 8;
	int err;

	otbn_deleg_cache_flush((void *)in, num_bytes);
	otbn_deleg_cache_flush(out, num_bytes);
	otbn_deleg_cache_flush(exp, num_bytes);
	otbn_deleg_cache_flush((void *)key_n, num_bytes);

	req.op = OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PUB;
	req.arg[0] = n_size;
	req.data[0] = (uint8_t *)in;
	req.data[1] = exp;
	req.data[2] = (uint8_t *)key_n;
	req.data[3] = out;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_RSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		otbn_deleg_cache_invd(rpy->data[3], num_bytes);
		*outLen = num_bytes;
	}
	return rpy->status;
}

int ls_rsa_modexp_decrypt(const uint8_t *in, uint32_t inLen, uint8_t *out,
			  uint32_t *outLen, uint8_t *key_d, const uint8_t *key_n,
			  uint32_t d_size)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	uint32_t num_bytes = d_size / 8;
	int err;

	otbn_deleg_cache_flush((void *)in, num_bytes);
	otbn_deleg_cache_flush(out, num_bytes);
	otbn_deleg_cache_flush(key_d, num_bytes);
	otbn_deleg_cache_flush((void *)key_n, num_bytes);

	req.op = OTBN_DELEG_OP_WOLFSSL_RSA_MODEXP_PRV;
	req.arg[0] = d_size;
	req.data[0] = (uint8_t *)in;
	req.data[1] = key_d;
	req.data[2] = (uint8_t *)key_n;
	req.data[3] = out;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_RSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status == 0) {
		otbn_deleg_cache_invd(rpy->data[3], num_bytes);
		*outLen = num_bytes;
	}
	return rpy->status;
}

#else /* server */

/* The local OTBN sequences live in the wolfssl port and are built when this
 * image is not the delegation client. Declared here because the port keeps them
 * file-local (ecc.c declares them for its own use). */
int ls_otbn_get_key_pair(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
			 uint8_t *public_x, uint8_t *public_y);
int ls_otbn_sign_hash(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
		      uint8_t *msg, uint8_t *r, uint8_t *s);
int ls_otbn_verify_hash(uint32_t curve, uint32_t curve_size, uint8_t *r, uint8_t *s,
			uint8_t *msg, uint8_t *pub_x, uint8_t *pub_y);
int ls_otbn_shared_secret(uint32_t curve, uint32_t curve_size, uint8_t *private_key,
			  uint8_t *public_x, uint8_t *public_y);

int otbn_deleg_wolfssl_genkey(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply)
{
	uint32_t curve = req->arg[0];
	uint32_t curve_size = req->arg[1];
	int err;

	otbn_deleg_cache_invd(req->data[0], curve_size);
	otbn_deleg_cache_invd(req->data[1], curve_size);
	otbn_deleg_cache_invd(req->data[2], curve_size);

	err = ls_otbn_get_key_pair(curve, curve_size, req->data[0], req->data[1],
				   req->data[2]);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[0], curve_size);
	otbn_deleg_cache_flush(req->data[1], curve_size);
	otbn_deleg_cache_flush(req->data[2], curve_size);
	reply->data[0] = req->data[0];
	reply->data[1] = req->data[1];
	reply->data[2] = req->data[2];
	return 0;
}

int otbn_deleg_wolfssl_sign(const struct otbn_delegate_params *req,
			    struct otbn_delegate_params *reply)
{
	uint32_t curve = req->arg[0];
	uint32_t curve_size = req->arg[1];
	int err;

	otbn_deleg_cache_invd(req->data[0], curve_size);
	otbn_deleg_cache_invd(req->data[1], curve_size);
	otbn_deleg_cache_invd(req->data[2], curve_size);
	otbn_deleg_cache_invd(req->data[3], curve_size);

	err = ls_otbn_sign_hash(curve, curve_size, req->data[0], req->data[1],
				req->data[2], req->data[3]);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[2], curve_size);
	otbn_deleg_cache_flush(req->data[3], curve_size);
	reply->data[2] = req->data[2];
	reply->data[3] = req->data[3];
	return 0;
}

int otbn_deleg_wolfssl_verify(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply)
{
	uint32_t curve = req->arg[0];
	uint32_t curve_size = req->arg[1];
	int err;

	otbn_deleg_cache_invd(req->data[0], curve_size);
	otbn_deleg_cache_invd(req->data[1], curve_size);
	otbn_deleg_cache_invd(req->data[2], curve_size);
	otbn_deleg_cache_invd(req->data[3], curve_size);
	otbn_deleg_cache_invd(req->data[4], curve_size);

	err = ls_otbn_verify_hash(curve, curve_size, req->data[0], req->data[1],
				  req->data[2], req->data[3], req->data[4]);
	if (err) {
		return err;
	}

	/* The local routine leaves the OTBN r_x in `s` (data[1]); the client
	 * reads exactly that buffer back. */
	otbn_deleg_cache_flush(req->data[1], curve_size);
	reply->data[1] = req->data[1];
	return 0;
}

int otbn_deleg_wolfssl_shared_secret(const struct otbn_delegate_params *req,
				     struct otbn_delegate_params *reply)
{
	uint32_t curve = req->arg[0];
	uint32_t curve_size = req->arg[1];
	int err;

	otbn_deleg_cache_invd(req->data[0], curve_size);
	otbn_deleg_cache_invd(req->data[1], curve_size);
	otbn_deleg_cache_invd(req->data[2], curve_size);

	err = ls_otbn_shared_secret(curve, curve_size, req->data[0], req->data[1],
				    req->data[2]);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[1], curve_size);
	otbn_deleg_cache_flush(req->data[2], curve_size);
	reply->data[1] = req->data[1];
	reply->data[2] = req->data[2];
	return 0;
}

int otbn_deleg_wolfssl_rsa_modexp_pub(const struct otbn_delegate_params *req,
				      struct otbn_delegate_params *reply)
{
	uint32_t num_bytes = req->arg[0] / 8;
	uint32_t outLen = 0;
	int err;

	otbn_deleg_cache_invd(req->data[0], num_bytes);
	otbn_deleg_cache_invd(req->data[1], num_bytes);
	otbn_deleg_cache_invd(req->data[2], num_bytes);
	otbn_deleg_cache_invd(req->data[3], num_bytes);

	err = ls_rsa_modexp_encrypt(req->data[0], req->arg[0], req->data[3], &outLen,
				    req->data[1], req->data[2], req->arg[0]);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[3], num_bytes);
	reply->data[3] = req->data[3];
	return 0;
}

int otbn_deleg_wolfssl_rsa_modexp_prv(const struct otbn_delegate_params *req,
				      struct otbn_delegate_params *reply)
{
	uint32_t num_bytes = req->arg[0] / 8;
	uint32_t outLen = 0;
	int err;

	otbn_deleg_cache_invd(req->data[0], num_bytes);
	otbn_deleg_cache_invd(req->data[1], num_bytes);
	otbn_deleg_cache_invd(req->data[2], num_bytes);
	otbn_deleg_cache_invd(req->data[3], num_bytes);

	err = ls_rsa_modexp_decrypt(req->data[0], req->arg[0], req->data[3], &outLen,
				    req->data[1], req->data[2], req->arg[0]);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[3], num_bytes);
	reply->data[3] = req->data[3];
	return 0;
}

#endif /* CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT */
