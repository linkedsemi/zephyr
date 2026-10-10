

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdsa_alt.h"
#include "mbedtls/ecp.h"
#include "mbedtls/pk.h"
#include "mbedtls/md.h"
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
/* Own REGISTER: this file is built for delegation, while the shared
 * 'mbedtls_otbn' module is registered in mbedtls_otbn_msp.c, which is gated
 * on CONFIG_MBEDTLS_LINKEDSEMI_OTBN only -- a delegation-only build would
 * otherwise fail to link log_const_mbedtls_otbn (LOG_DBG used to be culled
 * by the log level, which hid this).
 */
LOG_MODULE_REGISTER(mbedtls_otbn_mbox, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

#define ASSERT_MBEDTLS(error) {if(error){printf("mbedtls :stack is too small\n"); err = MBEDTLS_ERR_ECP_BUFFER_TOO_SMALL; goto exit;}}
#define DELEGATE_ECDSA_GENE_KEY 0x11
#define DELEGATE_ECDSA_SIGN     0x22
#define DELEGATE_ECDSA_VERIFY   0x33
/* Upper bound on waiting for a reply. The stubs used to wait K_FOREVER while
 * ignoring mbox_send_dt() errors, so a dead or absent peer blocked the caller
 * (and held client_sem) forever. A blocking API must fail instead. */
#define DELEGATE_REPLY_TIMEOUT_MS  1000

const struct device *otbn_mbox_dev = DEVICE_DT_GET(DT_NODELABEL(mbox_consumer_otbn_crypto));

#define MAX_OTBN_CURVE_LEN 48 //P384->384bit == 48 Byte
struct otbn_delegate_params
{
    int status;
    uint8_t op;
    uint8_t curve_idx;
    uint8_t data_len;
    /* Request/response correlation. Replies cross the mbox asynchronously, and
     * a call may give up on timeout while its reply is still in flight, so the
     * op alone cannot tell "this reply is mine" from "this is a stale one".
     * 0 = no reply stored yet. */
    uint8_t seq;
    uint8_t *data[5];
};

#if defined(CONFIG_MBEDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
struct mbox_dt_spec ls_otbn_client = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),mbox);
struct k_sem client_sem;
struct k_sem client_op_return_sem;
static volatile struct otbn_delegate_params recive_params;
static uint8_t client_seq;

/* Send one request and wait for the reply carrying `seq`.
 * Returns 0 on our reply, -EIO if the send itself failed, -ETIMEDOUT if no
 * matching reply arrived in time. A reply from a call that already timed out
 * may wake us first, so keep waiting until the sequence matches.
 */
static int deleg_request(const struct mbox_msg *msg, uint8_t seq)
{
    int64_t deadline;

    if(mbox_send_dt(&ls_otbn_client, msg) != 0)
    {
        LOG_WRN("delegation: mbox send failed");
        return -EIO;
    }
    deadline = k_uptime_get() + DELEGATE_REPLY_TIMEOUT_MS;
    for(;;)
    {
        int64_t left = deadline - k_uptime_get();

        if(left <= 0)
        {
            break;
        }
        if(k_sem_take(&client_op_return_sem, K_MSEC(left)) != 0)
        {
            break;
        }
        if(recive_params.seq == seq)
        {
            return 0;
        }
        /* Stale reply from an earlier timed-out call -- keep waiting. */
    }
    LOG_WRN("delegation: no reply within %d ms", DELEGATE_REPLY_TIMEOUT_MS);
    return -ETIMEDOUT;
}

static void delegation_client_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
    const struct otbn_delegate_params *param = data->data;

    /* Record seq/status BEFORE the switch: those two are valid on every reply,
     * including error replies whose op never reached the server's switch (the
     * reply would fall into the default branch and copy no data).
     */
    recive_params.seq = param->seq;
    recive_params.status = param->status;

    switch(param->op)
    {
        case DELEGATE_ECDSA_GENE_KEY:
        case DELEGATE_ECDSA_SIGN:
        case DELEGATE_ECDSA_VERIFY:
            // memcpy(&recive_params,param,sizeof(struct otbn_delegate_params));
            recive_params.curve_idx = param->curve_idx;
            recive_params.data_len = param->data_len;
            recive_params.data[0] = param->data[0];
            recive_params.data[1] = param->data[1];
            recive_params.data[2] = param->data[2];
            recive_params.data[3] = param->data[3];
            break;
        default:
            LOG_DBG("ecdsa error operation\n");
            break;
    }
    
    k_sem_give(&client_op_return_sem);
}

#define DELEGATION_CLIENT_MBOX_CALLBACK(idx)\
	delegation_client_mbox_handler(DEVICE_DT_INST_GET(idx),data);

static void delegation_client_mbox_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)
{
    delegation_client_mbox_handler(dev,data);
	// DT_INST_FOREACH_STATUS_OKAY(DELEGATION_CLIENT_MBOX_CALLBACK);
}

/* client : cpu1 secure*/
void mbedtls_ls_otbn_delegation_client_chanels_init(void)
{
    // LOG_DBG("ls_otbn channel_id= 0x%x\n",ls_otbn_client.channel_id);
    k_sem_init(&client_sem,1,1);
    k_sem_init(&client_op_return_sem,0,1);

	mbox_register_callback_dt(&ls_otbn_client,delegation_client_mbox_callback,NULL);
	mbox_set_enabled_dt(&ls_otbn_client,true);
}


int mbedtls_ecdsa_genkey(mbedtls_ecdsa_context *ctx, mbedtls_ecp_group_id gid,
                         int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
    int err = 0;
    size_t n_size = 0;
    __attribute__((aligned(32)))uint8_t public_key[MAX_OTBN_CURVE_LEN*2];
    __attribute__((aligned(32)))uint8_t private_key[MAX_OTBN_CURVE_LEN];

	if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    if(gid == MBEDTLS_ECP_DP_SECP384R1)
    {
        n_size = 48;
    }else if(gid == MBEDTLS_ECP_DP_SECP256R1 || gid == MBEDTLS_ECP_DP_SM2)
    {
        n_size = 32;
    }else
    {
        /* Exit through the shared epilogue so client_sem is always released;
         * a bare return here leaks it and deadlocks every later proxy call.
         * Return value is unchanged (-EIO).
         */
        err = -EIO;
        goto exit;
    }
    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_GENE_KEY,
        .curve_idx = (uint8_t)gid,
        .data[0] = private_key,
        .data[1] = public_key,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };
    sys_cache_data_flush_range((void *)private_key, MAX_OTBN_CURVE_LEN);
    sys_cache_data_flush_range((void *)public_key, MAX_OTBN_CURVE_LEN*2);

    /* seq 0 is the "no reply stored" sentinel, so never hand it out: on
     * wrap the request's seq would equal the sentinel and any stale reply
     * would be mistaken for this call's. */
    ++client_seq;
    if(client_seq == 0)
    {
        client_seq = 1;
    }
    param.seq = client_seq;
    recive_params.seq = 0;
    err = deleg_request(&msg, param.seq);
    if(err)
    {
        goto exit;
    }
    if(recive_params.status != 0)
    {
        /* Server reported a failure: its result buffers are not valid. */
        err = recive_params.status;
        goto exit;
    }
    sys_cache_data_invd_range((void *)recive_params.data[0], n_size);
    sys_cache_data_invd_range((void *)recive_params.data[1], n_size*2);
    ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx->private_d,recive_params.data[0],n_size)); //private key
    ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx->private_Q.private_X,recive_params.data[1],n_size)); //public_x key
    ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx->private_Q.private_Y,recive_params.data[1]+n_size,n_size)); //public_y key
    /* data[1] carries affine X||Y; set Z=1 so this is a real affine point
     * rather than the Z==0 point at infinity that check_pubkey() rejects.
     */
    mbedtls_mpi_lset(&ctx->private_Q.private_Z, 1);

exit:
    if(recive_params.status == 0)
    {
        LOG_HEXDUMP_DBG(recive_params.data[0],n_size,"private key");
        LOG_HEXDUMP_DBG(recive_params.data[1],n_size*2,"public_x key");
    }
    k_sem_give(&client_sem);
    
    return err;
}

int mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
                       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
                       int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
    int err = 0;
    __attribute__((aligned(32)))uint8_t r_buf[MAX_OTBN_CURVE_LEN];
    __attribute__((aligned(32)))uint8_t s_buf[MAX_OTBN_CURVE_LEN];
    __attribute__((aligned(32)))uint8_t private_key[MAX_OTBN_CURVE_LEN];
    __attribute__((aligned(32)))uint8_t user_buf[MAX_OTBN_CURVE_LEN] = {0};

	if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}

    size_t n_size = (grp->nbits + 7) / 8;
    size_t buf_len =  blen > n_size ? n_size : blen;
    memcpy(user_buf,buf,buf_len);

    mbedtls_mpi_write_binary_le(d,private_key,n_size);
    
    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_SIGN,
        .curve_idx = (uint8_t)grp->id,
        .data[0] = private_key,
        .data[1] = user_buf,
        .data[2] = r_buf,
        .data[3] = s_buf,
        .data_len = buf_len,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };
    sys_cache_data_flush_range((void *)private_key, MAX_OTBN_CURVE_LEN);
    sys_cache_data_flush_range((void *)user_buf, MAX_OTBN_CURVE_LEN);

    /* seq 0 is the "no reply stored" sentinel, so never hand it out: on
     * wrap the request's seq would equal the sentinel and any stale reply
     * would be mistaken for this call's. */
    ++client_seq;
    if(client_seq == 0)
    {
        client_seq = 1;
    }
    param.seq = client_seq;
    recive_params.seq = 0;
    err = deleg_request(&msg, param.seq);
    if(err)
    {
        goto exit;
    }
    if(!recive_params.status)
    {
        sys_cache_data_invd_range((void *)recive_params.data[2], n_size);
        sys_cache_data_invd_range((void *)recive_params.data[3], n_size);
        ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(r,recive_params.data[2],n_size)); //r_buf
        ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(s,recive_params.data[3],n_size)); //s_buf
    }
    if(recive_params.status == 0)
    {
        LOG_HEXDUMP_DBG(recive_params.data[2],n_size,"sign r_buf");
        LOG_HEXDUMP_DBG(recive_params.data[3],n_size,"sign s_buf");
    }
exit:
    k_sem_give(&client_sem);
    if(err)
        return err;
    return recive_params.status;
}
/*
 * Verify ECDSA signature of hashed message
 */
int mbedtls_ecdsa_verify(mbedtls_ecp_group *grp,
                         const unsigned char *buf, size_t blen,
                         const mbedtls_ecp_point *Q,
                         const mbedtls_mpi *r,
                         const mbedtls_mpi *s)
{
    int err = 0;
    __attribute__((aligned(32)))uint8_t r_buf[MAX_OTBN_CURVE_LEN];
    __attribute__((aligned(32)))uint8_t s_buf[MAX_OTBN_CURVE_LEN];
    __attribute__((aligned(32)))uint8_t user_buf[MAX_OTBN_CURVE_LEN] = {0};
    __attribute__((aligned(32)))uint8_t public_key[MAX_OTBN_CURVE_LEN*2];

	if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    size_t n_size = (grp->nbits + 7) / 8;
    size_t buf_len =  blen > n_size ? n_size : blen;
    memcpy(user_buf,buf,buf_len);
    mbedtls_mpi_write_binary_le(&Q->private_X,public_key,n_size);
    mbedtls_mpi_write_binary_le(&Q->private_Y,public_key+n_size,n_size);
    mbedtls_mpi_write_binary_le(r,r_buf,n_size);
    mbedtls_mpi_write_binary_le(s,s_buf,n_size);

    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_VERIFY,
        .curve_idx = (uint8_t)grp->id,
        .data[0] = public_key,
        .data[1] = user_buf,
        .data[2] = r_buf,
        .data[3] = s_buf,
        .data_len = buf_len,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };

    sys_cache_data_flush_range((void *)param.data[0], MAX_OTBN_CURVE_LEN*2);
    sys_cache_data_flush_range((void *)param.data[1], MAX_OTBN_CURVE_LEN);
    sys_cache_data_flush_range((void *)param.data[2], MAX_OTBN_CURVE_LEN);
    sys_cache_data_flush_range((void *)param.data[3], MAX_OTBN_CURVE_LEN);

    /* seq 0 is the "no reply stored" sentinel, so never hand it out: on
     * wrap the request's seq would equal the sentinel and any stale reply
     * would be mistaken for this call's. */
    ++client_seq;
    if(client_seq == 0)
    {
        client_seq = 1;
    }
    param.seq = client_seq;
    recive_params.seq = 0;
    err = deleg_request(&msg, param.seq);
    if(err)
    {
        k_sem_give(&client_sem);
        return err;
    }
    if(recive_params.status != 0)
    {
        LOG_DBG("ecdsa verification  failed or input parameters are incorrect");
    }

    k_sem_give(&client_sem);
    return recive_params.status;
}

#else
static void mbedtls_delegation_server_consumer(struct k_work *work);
// K_THREAD_DEFINE(consumer_thread_id, 1024, mbedtls_delegation_server_consumer, NULL, NULL, NULL, 2, 0, 0);
const static struct device *trng = DEVICE_DT_GET(DT_CHOSEN(zephyr_entropy));
static struct mbox_dt_spec ls_otbn_server = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),mbox);
static struct k_sem server_sem;
static struct k_sem server_op_return_sem;
static struct k_work worker;
static struct k_msgq msgq;
#define MBEDTLS_MSGQ_LEN 2
struct otbn_delegate_params msgq_buf[MBEDTLS_MSGQ_LEN];

static void delegation_server_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
    int err = 0;
    const struct otbn_delegate_params *param = data->data;
    struct mbox_msg msg;
    // LOG_DBG("delegation_server_mbox_handler\n");
    switch(param->op)
    {
        case DELEGATE_ECDSA_GENE_KEY:
        case DELEGATE_ECDSA_SIGN:
        case DELEGATE_ECDSA_VERIFY:
        
            err = k_msgq_put(&msgq, param, K_NO_WAIT);
            if(err)
            {
                struct otbn_delegate_params rparam = 
                {
                    .status = MBEDTLS_ERR_ECP_IN_PROGRESS,
                };
                msg.data = &rparam;
                mbox_send_dt(&ls_otbn_server,&msg);
                LOG_DBG("mbedtls mailbox fifo too small\n");
            }else
            {
                k_work_submit(&worker);
            }
            break;
        default:
            LOG_DBG("error mbedtls mailbox operation\n");
            break;
    }
}

#define DELEGATION_SERVER_MBOX_CALLBACK(idx)\
	delegation_server_mbox_handler(DEVICE_DT_INST_GET(idx),data);

static void delegation_server_mbox_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)
{
    delegation_server_mbox_handler(ls_otbn_server.dev,data);
	// DT_INST_FOREACH_STATUS_OKAY(DELEGATION_SERVER_MBOX_CALLBACK);
}

/* server : cpu2 app core*/
void mbedtls_ls_otbn_delegation_server_chanels_init(void)
{
    // LOG_DBG("ls_otbn channel_id= 0x%x\n",ls_otbn_server.channel_id);
    k_sem_init(&server_sem,1,1);
    k_sem_init(&server_op_return_sem,0,1);
    k_msgq_init(&msgq, (char *)msgq_buf, sizeof(struct otbn_delegate_params), MBEDTLS_MSGQ_LEN);

    k_work_init(&worker,mbedtls_delegation_server_consumer);
    // k_thread_start(consumer_thread_id);
	mbox_register_callback_dt(&ls_otbn_server,delegation_server_mbox_callback,NULL);
	mbox_set_enabled_dt(&ls_otbn_server,true);
}

static int ls_wolfssl_get_random(void *null, unsigned char *buf, size_t size)
{
    (void)null;
    return entropy_get_entropy(trng, buf, size);
}

static void mbedtls_delegation_server_consumer(struct k_work *work)
{
    int err = 0;
    struct otbn_delegate_params ecc_param;
    struct otbn_delegate_params param = {0};
    struct mbox_msg msg = {0};
    mbedtls_ecdsa_context ctx;
    mbedtls_mpi r;
    mbedtls_mpi s;
    uint8_t curve_size;
    LOG_DBG("thread :mbedtls_delegation_server_consumer started\n");
    do
    {
        err = k_msgq_get(&msgq, &ecc_param, K_FOREVER);
        if(err)
        {
            LOG_DBG("mbedtls work queue err\n");
        }

        if(!(ecc_param.curve_idx == MBEDTLS_ECP_DP_SECP384R1 || ecc_param.curve_idx == MBEDTLS_ECP_DP_SECP256R1 
                                                                || ecc_param.curve_idx == MBEDTLS_ECP_DP_SM2))
        {
            LOG_DBG("mebdtls work queue:This curve is not supported, curve id: 0x%x\n",ecc_param.curve_idx);
            ecc_param.op = 0xff;
            err = -EOPNOTSUPP;
        }
        mbedtls_ecp_group_init(&ctx.private_grp);
        err = mbedtls_ecp_group_load(&ctx.private_grp, ecc_param.curve_idx);
        if(err)
        {
            ecc_param.op = 0xff;
            __ASSERT_PRINT("mbedtks load curve err\n");
        }
        mbedtls_mpi_init(&ctx.private_Q.private_X);
        mbedtls_mpi_init(&ctx.private_Q.private_Y);
        mbedtls_mpi_init(&ctx.private_Q.private_Z);
        mbedtls_mpi_init(&ctx.private_d);
        mbedtls_mpi_init(&r);
        mbedtls_mpi_init(&s);
        curve_size = (ctx.private_grp.nbits + 7)/8;
        switch(ecc_param.op)
        {
            case DELEGATE_ECDSA_GENE_KEY:
                err = mbedtls_ecdsa_genkey(&ctx,ecc_param.curve_idx,ls_wolfssl_get_random,NULL);
                if(err)
                {
                    LOG_DBG(" ecc keygen failed \r\n");
                }
                mbedtls_mpi_write_binary_le(&ctx.private_d,ecc_param.data[0],curve_size); //private key
                mbedtls_mpi_write_binary_le(&ctx.private_Q.private_X,ecc_param.data[1],curve_size); //public_x key
                mbedtls_mpi_write_binary_le(&ctx.private_Q.private_Y,ecc_param.data[1]+curve_size,curve_size); //public_y key
                
                sys_cache_data_flush_range((void *)ecc_param.data[0], MAX_OTBN_CURVE_LEN);
                sys_cache_data_flush_range((void *)ecc_param.data[1], MAX_OTBN_CURVE_LEN*2);
                LOG_HEXDUMP_DBG(ecc_param.data[0],curve_size,"private key");
                LOG_HEXDUMP_DBG(ecc_param.data[1],curve_size*2,"public_x key");
                param.op = DELEGATE_ECDSA_GENE_KEY;
                param.curve_idx = (uint8_t)ecc_param.curve_idx;
                param.data[0] = ecc_param.data[0];
                param.data[1] = ecc_param.data[1];
                param.status = err;
                break;
            case DELEGATE_ECDSA_SIGN:
                sys_cache_data_invd_range((void *)ecc_param.data[0], curve_size);
                sys_cache_data_invd_range((void *)ecc_param.data[1], ecc_param.data_len);
                ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx.private_d,ecc_param.data[0],curve_size)); //private key
                err = mbedtls_ecdsa_sign(&ctx.private_grp, &r, &s, &ctx.private_d, ecc_param.data[1], ecc_param.data_len, ls_wolfssl_get_random, NULL);
                if(err)
                {
                    LOG_DBG(" ecdsa sign failed\n");
                }
                mbedtls_mpi_write_binary_le(&r,ecc_param.data[2],curve_size); //r_buf
                mbedtls_mpi_write_binary_le(&s,ecc_param.data[3],curve_size); //s_buf

                sys_cache_data_flush_range((void *)ecc_param.data[2], MAX_OTBN_CURVE_LEN);
                sys_cache_data_flush_range((void *)ecc_param.data[3], MAX_OTBN_CURVE_LEN);
                LOG_HEXDUMP_DBG(ecc_param.data[2],curve_size,"sign r_buf");
                LOG_HEXDUMP_DBG(ecc_param.data[3],curve_size,"sign s_buf");

                param.op = DELEGATE_ECDSA_SIGN;
                param.curve_idx = (uint8_t)ecc_param.curve_idx;
                param.data[2] = ecc_param.data[2];
                param.data[3] = ecc_param.data[3];
                param.status = err;
                break;

            case DELEGATE_ECDSA_VERIFY:
                sys_cache_data_invd_range((void *)ecc_param.data[0], curve_size*2);
                sys_cache_data_invd_range((void *)ecc_param.data[1], ecc_param.data_len);
                sys_cache_data_invd_range((void *)ecc_param.data[2], curve_size);
                sys_cache_data_invd_range((void *)ecc_param.data[3], curve_size);
                ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx.private_Q.private_X,ecc_param.data[0],curve_size));
                ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&ctx.private_Q.private_Y,ecc_param.data[0]+curve_size,curve_size));
                /* data[0] carries affine X||Y; set Z=1 (see the client-side
                 * genkey note above).
                 */
                mbedtls_mpi_lset(&ctx.private_Q.private_Z, 1);
                ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&r,ecc_param.data[2],curve_size));
                ASSERT_MBEDTLS(mbedtls_mpi_read_binary_le(&s,ecc_param.data[3],curve_size));
                err = mbedtls_ecdsa_verify(&ctx.private_grp, ecc_param.data[1], ecc_param.data_len, &ctx.private_Q, &r, &s);
                if(err)
                {
                    LOG_DBG(" ecdsa verify failed\n");
                }
                param.op = DELEGATE_ECDSA_VERIFY;
                param.curve_idx = (uint8_t)ecc_param.curve_idx;
                param.status = err;
                break;

            default:
                LOG_DBG("error operation\n");
                break;
        }
exit:
        /* Report the real verdict on EVERY exit path. The old guarded form
         * only covered two codes, and mbedtls_ecp_group_load() above
         * overwrites err, so a failed curve load left status at its
         * zero-initialised value and the client read the failure as success.
         */
        param.status = err;
        /* Echo op and seq so the client can correlate this reply with its
         * request even on an error path (the switch never set them). */
        param.op = ecc_param.op;
        param.seq = ecc_param.seq;
        msg.data = &param;
        msg.size = sizeof(param);
        mbox_send_dt(&ls_otbn_server,&msg);
        mbedtls_ecp_group_free(&ctx.private_grp);
        mbedtls_mpi_free(&ctx.private_Q.private_X);
        mbedtls_mpi_free(&ctx.private_Q.private_Y);
        mbedtls_mpi_free(&ctx.private_Q.private_Z);
        mbedtls_mpi_free(&ctx.private_d);
        mbedtls_mpi_free(&r);
        mbedtls_mpi_free(&s);
    }while(0);
}
#endif