/*
 * RSA family of the OTBN delegation bus.
 *
 * Client side: mbedtls_ls_otbn_rsa_modexp_encrypt()/_decrypt() pack a request
 * and hand it to the bus. There is no mbedtls RSA ALT override here on purpose
 * -- the local RSA OTBN support is a hook (mbedtls_rsa_public_otbn() and
 * friends in rsa_alt.c), not an ALT, so a delegated RSA call is always explicit.
 *
 * Server side: otbn_deleg_rsa_*() run the request on the local OTBN through
 * the byte-level backend in rsa_alt.c.
 */

#include <zephyr/kernel.h>

#include "otbn_delegation.h"
#include "mbedtls_otbn_rsa_mbox.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(mbedtls_otbn_mbox, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

/* A modexp is a few orders of magnitude slower than an ECDSA operation, which
 * is why the bus takes the bound per call. This is a safety net against a dead
 * or wedged peer, not an expected duration. */
#define OTBN_DELEG_TIMEOUT_RSA_MS 10000

/* The OTBN RSA firmware implements these only; rsa_otbn_select_mode() returns 0
 * for anything else. */
#define RSA_MODULUS_2048_BYTES 256
#define RSA_MODULUS_3072_BYTES 384
#define RSA_MODULUS_4096_BYTES 512

/* Only the client path and the "local OTBN RSA present" server path use this;
 * the stub server path returns before validating, hence __maybe_unused. */
static bool __maybe_unused rsa_modulus_supported(size_t num_bytes)
{
	return num_bytes == RSA_MODULUS_2048_BYTES ||
	       num_bytes == RSA_MODULUS_3072_BYTES ||
	       num_bytes == RSA_MODULUS_4096_BYTES;
}

#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

static int rsa_modexp_call(uint8_t op, const uint8_t *in, const uint8_t *exp,
			   const uint8_t *n, uint32_t n_size_bits, uint8_t *out)
{
	struct otbn_delegate_params req = {0};
	const struct otbn_delegate_params *rpy;
	size_t num_bytes = n_size_bits / 8;
	int err;

	if (!rsa_modulus_supported(num_bytes)) {
		return -EINVAL;
	}

	/* The peer reads these buffers and writes the result into `out`; both
	 * cores share SRAM, so both directions need explicit cache maintenance. */
	otbn_deleg_cache_flush((void *)in, num_bytes);
	otbn_deleg_cache_flush((void *)exp, num_bytes);
	otbn_deleg_cache_flush((void *)n, num_bytes);

	req.op = op;
	req.arg[0] = n_size_bits;
	req.data[0] = (uint8_t *)in;
	req.data[1] = (uint8_t *)exp;
	req.data[2] = (uint8_t *)n;
	req.data[3] = out;

	err = otbn_deleg_call(&req, OTBN_DELEG_TIMEOUT_RSA_MS);
	if (err) {
		return err;
	}
	rpy = otbn_deleg_reply();
	if (rpy->status != 0) {
		return rpy->status;
	}

	otbn_deleg_cache_invd(rpy->data[3], num_bytes);
	return 0;
}

int mbedtls_ls_otbn_rsa_modexp_encrypt(const uint8_t *in, const uint8_t *exp,
				       const uint8_t *n, uint32_t n_size_bits,
				       uint8_t *out)
{
	return rsa_modexp_call(OTBN_DELEG_OP_RSA_MODEXP_PUB, in, exp, n, n_size_bits, out);
}

int mbedtls_ls_otbn_rsa_modexp_decrypt(const uint8_t *in, const uint8_t *d,
				       const uint8_t *n, uint32_t n_size_bits,
				       uint8_t *out)
{
	return rsa_modexp_call(OTBN_DELEG_OP_RSA_MODEXP_PRV, in, d, n, n_size_bits, out);
}

#else /* server */

#if defined(CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT)

static int rsa_modexp_server(const struct otbn_delegate_params *req,
			     struct otbn_delegate_params *reply, bool is_public)
{
	size_t num_bytes = req->arg[0] / 8;
	int err;

	if (!rsa_modulus_supported(num_bytes)) {
		LOG_WRN("delegation: unsupported RSA modulus %u bits", req->arg[0]);
		return -EOPNOTSUPP;
	}

	otbn_deleg_cache_invd(req->data[0], num_bytes);
	otbn_deleg_cache_invd(req->data[1], num_bytes);
	otbn_deleg_cache_invd(req->data[2], num_bytes);

	err = mbedtls_ls_otbn_rsa_modexp(req->data[0], req->data[3], req->data[1],
					 req->data[2], num_bytes, is_public);
	if (err) {
		return err;
	}

	otbn_deleg_cache_flush(req->data[3], num_bytes);
	reply->data[3] = req->data[3];
	return 0;
}

#else /* !CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT */

/* The op stays in the bus' table even when the peer core has no local OTBN RSA:
 * answering with a clean error is better than a link error on a configuration
 * that only looks wrong at the very end of a build. */
static int rsa_modexp_server(const struct otbn_delegate_params *req,
			     struct otbn_delegate_params *reply, bool is_public)
{
	ARG_UNUSED(req);
	ARG_UNUSED(reply);
	ARG_UNUSED(is_public);

	LOG_WRN("delegation: RSA request received but the local OTBN RSA is off");
	return -EOPNOTSUPP;
}

#endif /* CONFIG_MBEDTLS_RSA_LINKEDSEMI_OTBN_ALT */

int otbn_deleg_rsa_modexp_pub(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply)
{
	return rsa_modexp_server(req, reply, true);
}

int otbn_deleg_rsa_modexp_prv(const struct otbn_delegate_params *req,
			      struct otbn_delegate_params *reply)
{
	return rsa_modexp_server(req, reply, false);
}

#endif /* CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT */
