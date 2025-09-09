

#ifndef ZEPHYR_INCLUDE_CRYPTO_LS_OTBN_P256_H_
#define ZEPHYR_INCLUDE_CRYPTO_LS_OTBN_P256_H_

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>


struct ecc_p256_ops;
struct ecc_p256_ctx;
struct ecc_p256_pkt;
struct ecc_p256_key {
    /* public key*/
	char *qx;
	char *qy;
    /* private key*/
	char *d;
};

struct ecc_p256_ops {
	int (*sign)(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key, struct ecc_p256_pkt *pkt); //uint8_t *digest, 
	int (*verify)(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key, struct ecc_p256_pkt *pkt);
	int (*keygen)(struct ecc_p256_ctx *ctx, struct ecc_p256_key *key);
	int (*shared_key)(struct ecc_p256_ctx *ctx, struct ecc_p256_key *pri_key,struct ecc_p256_key *pub_key,struct ecc_p256_key *out);
};

/**
 * Structure encoding session parameters.
 *
 * Refer to comments for individual fields to know the contract
 * in terms of who fills what and when w.r.t begin_session() call.
 */
struct ecc_p256_ctx {

	/** Place for driver to return function pointers to be invoked per
	 * cipher operation. To be populated by crypto driver on return from
	 * begin_session() based on the algo/mode chosen by the app.
	 */
	struct ecc_p256_ops ops;

	/** The device driver instance this crypto context relates to. Will be
	 * populated by the begin_session() API.
	 */
	const struct device *device;
};

/* The API a ecc_p256 driver should implement */
__subsystem struct ecc_p256_driver_api {
	int (*query_hw_caps)(const struct device *dev);

	/* Setup a ecdas session */
	int (*begin_session)(const struct device *dev, struct ecc_p256_ctx *ctx,
				 struct ecc_p256_key *key);

	/* Tear down an established session */
	int (*free_session)(const struct device *dev, struct ecc_p256_ctx *ctx);
};


/**
 * Structure encoding IO parameters of one cryptographic
 * operation like sign/verify.
 *
 * The fields which has not been explicitly called out has to
 * be filled up by the app before making the cipher_xxx_op()
 * call.
 */
struct ecc_p256_pkt {
//  m：输入摘要msg   r,s：输出签名
	uint8_t *m;
	int  m_len;

	uint8_t *r;
	int  r_len;

	uint8_t *s;
	int  s_len;

	/** Context this packet relates to. This can be useful to get the
	 * session details, especially for async ops. Will be populated by the
	 * cipher_xxx_op() API based on the ctx parameter.
	 */
	struct ecc_p256_ctx *ctx;
};




/**
 * @brief Setup a ecc_p256 session
 *
 * Initializes one time parameters, like the session key, algorithm and cipher
 * mode which may remain constant for all operations in the session. The state
 * may be cached in hardware and/or driver data state variables.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  ctx      Pointer to the context structure.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_begin_session(const struct device *dev,
					struct ecc_p256_ctx *ctx, struct ecc_p256_key *key)
{
	struct ecc_p256_driver_api *api;

	api = (struct ecc_p256_driver_api *)dev->api;
	ctx->device = dev;

	return api->begin_session(dev, ctx, key);
}

/**
 * @brief Cleanup a ecc_p256 session
 *
 * Clears the hardware and/or driver state of a previous session.
 *
 * @param  dev      Pointer to the device structure for the driver instance.
 * @param  ctx      Pointer to the ecc_p256 context structure of the session
 *			to be freed.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_free_session(const struct device *dev,
					   struct ecc_p256_ctx *ctx)
{
	struct ecc_p256_driver_api *api;

	api = (struct ecc_p256_driver_api *)dev->api;

	return api->free_session(dev, ctx);
}

/**
 * @brief Perform ecc_p256 sign.
 *
 * @param  ctx   Pointer to the ecc_p256 context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_sign(struct ecc_p256_ctx *ctx,struct ecc_p256_key *key,
				   struct ecc_p256_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.sign(ctx, key, pkt);
}

/**
 * @brief Perform ecc_p256 sign.
 *
 * @param  ctx   Pointer to the ecc_p256 context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_verify(struct ecc_p256_ctx *ctx,struct ecc_p256_key *key,
				 struct ecc_p256_pkt *pkt)
{
	pkt->ctx = ctx;
	return ctx->ops.verify(ctx, key, pkt);
}


/**
 * @brief Perform genarater ecc_p256 private/public key pair.
 *
 * @param  ctx   Pointer to the ecc_p256 context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_keygen(struct ecc_p256_ctx *ctx,
				 struct ecc_p256_key *key)
{
	return ctx->ops.keygen(ctx, key);
}

/**
 * @brief Perform genarater ecc_p256 private/public key pair.
 *
 * @param  ctx   Pointer to the ecc_p256 context of this op.
 * @param  pkt   Structure holding the input/output buffer pointers.
 *
 * @return 0 on success, negative errno code on fail.
 */
static inline int ecc_p256_shared_key(struct ecc_p256_ctx *ctx,
				 struct ecc_p256_key *pri_key,struct ecc_p256_key *pub_key,struct ecc_p256_key *out)
{
	return ctx->ops.shared_key(ctx,pri_key,pub_key,out);
}


#endif