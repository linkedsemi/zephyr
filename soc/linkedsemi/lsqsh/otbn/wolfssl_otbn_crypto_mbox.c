

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <zephyr/kernel.h>

#ifndef WOLFSSL_USER_SETTINGS
    /* Should already be defined in settings.h for #if defined(ARDUINO) */
    #define WOLFSSL_USER_SETTINGS
#endif

#ifndef WOLFSSL_ZEPHYR
    #define WOLFSSL_ZEPHYR
#endif
/* Reminder: user_settings.h is needed and included from settings.h
 * Be sure to define WOLFSSL_USER_SETTINGS, typically in CMakeLists.txt */
#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/types.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include <wolfssl/wolfcrypt/wc_port.h>
#include <wolfssl/wolfcrypt/ecc.h>
#include <wolfssl/wolfcrypt/port/linkedsemi/ls-otbn-ecc.h>
#include <wolfssl/wolfcrypt/port/linkedsemi/ls-rsa.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/cache.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(mbox_linkedsem_ipc);

// #define ASSERT_WOLFSSL(error) {if(error){__ASSERT_//PRINT("wolfssl :stack is too small\n"); err = WOLFSSL_ERR_ECP_BUFFER_TOO_SMALL; goto exit;}}
#define DELEGATE_ECDSA_GENE_KEY     0x11
#define DELEGATE_ECDSA_SIGN         0x22
#define DELEGATE_ECDSA_VERIFY       0x33
#define DELEGATE_ECC_SHARED_KEY     0x44
#define DELEGATE_RSA_MOD_EXP_ENCRY  0x55
#define DELEGATE_RSA_MOD_EXP_DECRY  0x66
#define DELEGATE_ERROR_OPERATION    0Xff
// const struct device *otbn_mbox_dev = DEVICE_DT_GET(DT_NODELABEL(mbox_consumer_otbn_crypto));

#define RSA_MAX_NUMER_SIZE (4096/8)

struct otbn_delegate_params
{
    int status;
    uint8_t op;
    void *data[5];
    uint32_t param[5];
};
int ls_otbn_get_key_pair(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *public_x, uint8_t *public_y);
int ls_otbn_sign_hash(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *msg, uint8_t *r, uint8_t *s);
int ls_otbn_verify_hash(uint32_t curve, uint32_t curve_size, uint8_t *r, uint8_t *s, uint8_t *msg, uint8_t *pub_x, uint8_t *pub_y);
int ls_otbn_shared_secret(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *public_x, uint8_t *public_y);

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)

struct mbox_dt_spec ls_otbn_client_tx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),tx);
struct mbox_dt_spec ls_otbn_client_rx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),rx);
struct k_sem client_sem;
struct k_sem client_op_return_sem;
static volatile struct otbn_delegate_params recive_params;

static void delegation_client_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
    const struct otbn_delegate_params *param = data->data;

    switch(param->op)
    {
        case DELEGATE_ECDSA_GENE_KEY:
        case DELEGATE_ECDSA_SIGN:
        case DELEGATE_ECDSA_VERIFY:
        case DELEGATE_ECC_SHARED_KEY:
        case DELEGATE_RSA_MOD_EXP_DECRY:
        case DELEGATE_RSA_MOD_EXP_ENCRY:
            memcpy((void *)&recive_params,(void *)param,sizeof(struct otbn_delegate_params));
            // recive_params.op = param->op;
            // recive_params.data[0] = param->data[0];
            // recive_params.data[1] = param->data[1];
            // recive_params.data[2] = param->data[2];
            // recive_params.data[3] = param->data[3];
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
void ls_otbn_delegation_client_chanels_init(void)
{
    LOG_DBG("ls_otbn_tx channel_id= 0x%x\n",ls_otbn_client_tx.channel_id);
    LOG_DBG("ls_otbn_rx channel_id= 0x%x\n",ls_otbn_client_rx.channel_id);
    k_sem_init(&client_sem,1,1);
    k_sem_init(&client_op_return_sem,0,1);

	mbox_set_enabled_dt(&ls_otbn_client_tx,true);
	mbox_register_callback_dt(&ls_otbn_client_rx,delegation_client_mbox_callback,NULL);
	mbox_set_enabled_dt(&ls_otbn_client_rx,true);
}

int ls_otbn_sign_hash(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *msg, uint8_t *r, uint8_t *s)
{
    int ret = 0;
 
    if(!(curve == ECC_SM2P256V1 || curve == ECC_SECP256R1 || curve == ECC_SECP384R1))
    {
        return WC_HW_E;
    }
	if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    sys_cache_data_flush_range((void *)private_key, curve_size);
    sys_cache_data_flush_range((void *)msg, curve_size);
    sys_cache_data_flush_range((void *)r, curve_size);
    sys_cache_data_flush_range((void *)s, curve_size);

    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_SIGN,
        .param[0] = curve,
        .param[1] = curve_size,
        .data[0] = private_key,
        .data[1] = msg,
        .data[2] = r,
        .data[3] = s,
    };
    struct mbox_msg mmsg = {
        .data = &param,
        .size = sizeof(param),
    };

    mbox_send_dt(&ls_otbn_client_tx,&mmsg);
    k_sem_take(&client_op_return_sem,K_FOREVER);
    ret = recive_params.status;
    if(recive_params.status == 0)
    {
        sys_cache_data_invd_range((void *)r, curve_size);//r
        sys_cache_data_invd_range((void *)s, curve_size);//s
    }
    k_sem_give(&client_sem);

    return ret;
}

int ls_otbn_verify_hash(uint32_t curve, uint32_t curve_size, uint8_t *r, uint8_t *s, 
                                                uint8_t *msg, uint8_t *pub_x, uint8_t *pub_y)
{
    int ret;
    if(!(curve == ECC_SM2P256V1 || curve == ECC_SECP256R1 || curve == ECC_SECP384R1))
    {
        return WC_HW_E;
    }
	if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    sys_cache_data_flush_range((void *)r, curve_size);
    sys_cache_data_flush_range((void *)s, curve_size);
    sys_cache_data_flush_range((void *)msg, curve_size);
    sys_cache_data_flush_range((void *)pub_x, curve_size);
    sys_cache_data_flush_range((void *)pub_y, curve_size);

    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_VERIFY,
        .data[0] = r,
        .data[1] = s,
        .data[2] = msg,
        .data[3] = pub_x,
        .data[4] = pub_y,
        .param[0] = curve,
        .param[1] = curve_size,
    };
    struct mbox_msg mmsg = {
        .data = &param,
        .size = sizeof(param),
    };

    mbox_send_dt(&ls_otbn_client_tx,&mmsg);
    k_sem_take(&client_op_return_sem,K_FOREVER);

    ret = recive_params.status;
    sys_cache_data_invd_range((void *)recive_params.data[0], curve_size);
    k_sem_give(&client_sem);

    return  ret;
}
// #endif


int ls_otbn_get_key_pair(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *public_x, uint8_t *public_y)
{
    int ret;
    if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    sys_cache_data_flush_range((void *)private_key, curve_size);
    sys_cache_data_flush_range((void *)public_x, curve_size);
    sys_cache_data_flush_range((void *)public_y, curve_size);
    struct otbn_delegate_params param = {
        .op = DELEGATE_ECDSA_GENE_KEY,
        .data[0] = private_key,
        .data[1] = public_x,
        .data[2] = public_y,
        .param[0] = curve,
        .param[1] = curve_size,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };
    mbox_send_dt(&ls_otbn_client_tx,&msg);
    k_sem_take(&client_op_return_sem,K_FOREVER);
    ret = recive_params.status;
    if(ret == 0)
    {
        sys_cache_data_invd_range((void *)private_key, curve_size);
        sys_cache_data_invd_range((void *)public_x, curve_size);
        sys_cache_data_invd_range((void *)public_y, curve_size);
    }
    k_sem_give(&client_sem);

    return ret;
}


int ls_otbn_shared_secret(uint32_t curve, uint32_t curve_size, uint8_t *private_key, uint8_t *public_x, uint8_t *public_y)
{
    int ret;
    if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    sys_cache_data_flush_range((void *)private_key, curve_size);
    sys_cache_data_flush_range((void *)public_x, curve_size);
    sys_cache_data_flush_range((void *)public_y, curve_size);
    struct otbn_delegate_params param = {
        .op = DELEGATE_ECC_SHARED_KEY,
        .data[0] = private_key,
        .data[1] = public_x,
        .data[2] = public_y,
        .param[0] = curve,
        .param[1] = curve_size,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };
    mbox_send_dt(&ls_otbn_client_tx,&msg);
    k_sem_take(&client_op_return_sem,K_FOREVER);
    ret = recive_params.status;
    if(ret == 0)
    {
        // sys_cache_data_invd_range((void *)private_key, curve_size);
        sys_cache_data_invd_range((void *)public_x, curve_size);
        sys_cache_data_invd_range((void *)public_y, curve_size);
    }
    k_sem_give(&client_sem);

    return ret;
}

int ls_rsa_modexp_decrypt(const uint8_t* in, uint32_t inLen, uint8_t* out,
    uint32_t* outLen, uint8_t *key_d, const uint8_t* key_n, uint32_t d_size)
{
    int ret = 0;
    uint32_t num_bytes = d_size / 8;
    if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}

    sys_cache_data_flush_range((void *)in, num_bytes);
    sys_cache_data_flush_range((void *)out, num_bytes);
    sys_cache_data_flush_range((void *)key_d, num_bytes);
    sys_cache_data_flush_range((void *)key_n, num_bytes);

    struct otbn_delegate_params param = {
        .op = DELEGATE_RSA_MOD_EXP_DECRY,
        .data[0] = (uint8_t *)in,
        .data[1] = (uint8_t *)key_d,
        .data[2] = (uint8_t *)key_n,
        .data[3] = (uint8_t *)out,
        .param[0] = inLen,
        .param[1] = d_size,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };

    mbox_send_dt(&ls_otbn_client_tx,&msg);
    k_sem_take(&client_op_return_sem,K_FOREVER);
    ret = recive_params.status;
    if(ret == 0)
    {
        sys_cache_data_invd_range((void *)out, num_bytes);
        *outLen = num_bytes;
    }
    k_sem_give(&client_sem);
    return ret;
}

int ls_rsa_modexp_encrypt(const uint8_t* in, uint32_t inLen, uint8_t* out,
    uint32_t* outLen, uint8_t *exp, const uint8_t* key_n, uint32_t n_size)
{
    int ret = 0;
    uint32_t num_bytes = n_size / 8;
    if (k_sem_take(&client_sem, K_FOREVER)) {
		return -EACCES;
	}
    k_sem_give(&client_sem);

    sys_cache_data_flush_range((void *)in, num_bytes);
    sys_cache_data_flush_range((void *)out, num_bytes);
    sys_cache_data_flush_range((void *)exp, num_bytes);
    sys_cache_data_flush_range((void *)key_n, num_bytes);
    struct otbn_delegate_params param = {
        .op = DELEGATE_RSA_MOD_EXP_ENCRY,
        .data[0] = (uint8_t *)in,
        .data[1] = (uint8_t *)exp,
        .data[2] = (uint8_t *)key_n,
        .data[3] = (uint8_t *)out,
        .param[0] = inLen,
        .param[1] = n_size,
    };
    struct mbox_msg msg = {
        .data = &param,
        .size = sizeof(param),
    };

    mbox_send_dt(&ls_otbn_client_tx,&msg);
    k_sem_take(&client_op_return_sem,K_FOREVER);
    ret = recive_params.status;
    if(ret == 0)
    {
        sys_cache_data_invd_range((void *)out, num_bytes);
        *outLen = num_bytes;
    }
    k_sem_give(&client_sem);
    return ret;
}

#else
static void wolfssl_delegation_server_consumer(struct k_work *work);
// K_THREAD_DEFINE(consumer_thread_id, 1024, wolfssl_delegation_server_consumer, NULL, NULL, NULL, 2, 0, 0);
static struct mbox_dt_spec ls_otbn_server_tx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),tx);
static struct mbox_dt_spec ls_otbn_server_rx = MBOX_DT_SPEC_GET(DT_NODELABEL(mbox_consumer_otbn_crypto),rx);
static struct k_sem server_sem;
static struct k_sem server_op_return_sem;
static struct k_work worker;
static struct k_msgq msgq;
#define WOLFSSL_MSGQ_LEN 2
struct otbn_delegate_params msgq_buf[WOLFSSL_MSGQ_LEN];

static void delegation_server_mbox_handler(const struct device *dev,struct mbox_msg *data)
{
    int err = 0;
    const struct otbn_delegate_params *param = data->data;
    struct mbox_msg msg;
    LOG_DBG("delegation_server_mbox_handler\n");
    switch(param->op)
    {
        case DELEGATE_ECDSA_GENE_KEY:
        case DELEGATE_ECDSA_SIGN:
        case DELEGATE_ECDSA_VERIFY:
        case DELEGATE_ECC_SHARED_KEY:
        case DELEGATE_RSA_MOD_EXP_DECRY:
        case DELEGATE_RSA_MOD_EXP_ENCRY:
            err = k_msgq_put(&msgq, param, K_NO_WAIT);
            if(err)
            {
                struct otbn_delegate_params rparam = 
                {
                    .status = -1,
                };
                msg.data = &rparam;
                mbox_send_dt(&ls_otbn_server_tx,&msg);
                LOG_ERR("wolfssl mailbox fifo too small\n");
            }else
            {
                k_work_submit(&worker);
            }
            break;
        default:
            LOG_ERR("error wolfssl mailbox operation\n");
            break;
    }
}

#define DELEGATION_SERVER_MBOX_CALLBACK(idx)\
	delegation_server_mbox_handler(DEVICE_DT_INST_GET(idx),data);

static void delegation_server_mbox_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)
{
    delegation_server_mbox_handler(ls_otbn_server_rx.dev,data);
	// DT_INST_FOREACH_STATUS_OKAY(DELEGATION_SERVER_MBOX_CALLBACK);
}
/* server : cpu1 app core*/
void ls_otbn_delegation_server_chanels_init(void)
{
    LOG_DBG("ls_otbn_tx channel_id= 0x%x\n",ls_otbn_server_tx.channel_id);
    LOG_DBG("ls_otbn_rx channel_id= 0x%x\n",ls_otbn_server_rx.channel_id);
    k_sem_init(&server_sem,1,1);
    k_sem_init(&server_op_return_sem,0,1);
    k_msgq_init(&msgq, (char *)msgq_buf, sizeof(struct otbn_delegate_params), WOLFSSL_MSGQ_LEN);

    k_work_init(&worker,wolfssl_delegation_server_consumer);
    // k_thread_start(consumer_thread_id);
	mbox_set_enabled_dt(&ls_otbn_server_tx,true);
	mbox_register_callback_dt(&ls_otbn_server_rx,delegation_server_mbox_callback,NULL);
	mbox_set_enabled_dt(&ls_otbn_server_rx,true);
}

// static int ls_wolfssl_get_random(void *null, unsigned char *buf, size_t size)
// {
//     (void)null;
//     return entropy_get_entropy(trng, buf, size);
// }

static void wolfssl_delegation_server_consumer(struct k_work *work)
{
    int err = 0;
    struct otbn_delegate_params mbox_server;
    struct otbn_delegate_params param = {0};
    struct mbox_msg msg = {0};
    uint32_t curve_id;
    uint32_t curve_size;
    uint32_t nbyte;
    uint32_t out_len;
    LOG_DBG("thread :wolfssl_delegation_server_consumer started\n");
    do
    {
        err = k_msgq_get(&msgq, &mbox_server, K_FOREVER);
        if(err)
        {
            LOG_ERR("wolfssl work queue err\n");
        }
        curve_id = mbox_server.param[0];
        curve_size = mbox_server.param[1];
        
        // LOG_DBG("wolfssl get client request\n");
        // if(!(curve_id == ECC_SECP384R1 || curve_id == ECC_SECP256R1 || curve_id == ECC_SM2P256V1))
        // {
        //     LOG_DBG("wolfssl work queue:This curve is not supported.\n");
        //     mbox_server.op = DELEGATE_ERROR_OPERATION;
        // }

        switch(mbox_server.op)
        {
            case DELEGATE_ECDSA_GENE_KEY:
                LOG_DBG("DELEGATE_ECDSA_GENE_KEY\n");
                sys_cache_data_invd_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[2], curve_size);
                err = ls_otbn_get_key_pair(curve_id,curve_size,mbox_server.data[0],mbox_server.data[1],mbox_server.data[2]);
                if(err)
                {
                    LOG_ERR(" ecc keygen failed\n");
                }
                param.op = DELEGATE_ECDSA_GENE_KEY;
                param.status = err;
                param.data[0] = mbox_server.data[0];
                param.data[1] = mbox_server.data[1];
                param.data[2] = mbox_server.data[2];
                sys_cache_data_flush_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[2], curve_size);
                break;
            case DELEGATE_ECDSA_SIGN:
                LOG_DBG("DELEGATE_ECDSA_SIGN\n");
                sys_cache_data_invd_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[2], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[3], curve_size);
                err = ls_otbn_sign_hash(curve_id,curve_size,mbox_server.data[0],mbox_server.data[1],mbox_server.data[2],mbox_server.data[3]);
                if(err)
                {
                    LOG_ERR(" ecdsa sign failed\n");
                }
                param.op = DELEGATE_ECDSA_SIGN;
                param.status = err;
                param.data[0] = mbox_server.data[0];
                param.data[1] = mbox_server.data[1];
                param.data[2] = mbox_server.data[2];
                param.data[3] = mbox_server.data[3];
                sys_cache_data_flush_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[2], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[3], curve_size);
                break;
            case DELEGATE_ECDSA_VERIFY:
                LOG_DBG("DELEGATE_ECDSA_VERIFY\n");
                sys_cache_data_invd_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[2], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[3], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[4], curve_size);
                err = ls_otbn_verify_hash(curve_id,curve_size,mbox_server.data[0],mbox_server.data[1],mbox_server.data[2],mbox_server.data[3],mbox_server.data[4]);
                if(err)
                {
                    LOG_ERR(" ecdsa verify failed\n");
                }
                param.op = DELEGATE_ECDSA_VERIFY;
                param.status = err;
                param.data[0] = mbox_server.data[1];//r_x
                sys_cache_data_flush_range((void *)param.data[0], curve_size);
                break;
            case DELEGATE_ECC_SHARED_KEY:
                LOG_DBG("DELEGATE_ECC_SHARED_KEY\n");
                sys_cache_data_invd_range((void *)mbox_server.data[0], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_invd_range((void *)mbox_server.data[2], curve_size);
                err = ls_otbn_shared_secret(curve_id,curve_size,mbox_server.data[0],mbox_server.data[1],mbox_server.data[2]);
                if(err)
                {
                    LOG_ERR(" ecc shared key failed\n");
                }
                param.op = DELEGATE_ECC_SHARED_KEY;
                param.status = err;
                param.data[1] = mbox_server.data[1];
                param.data[2] = mbox_server.data[2];
                sys_cache_data_flush_range((void *)mbox_server.data[1], curve_size);
                sys_cache_data_flush_range((void *)mbox_server.data[2], curve_size);
                break;
            case DELEGATE_RSA_MOD_EXP_ENCRY:
                nbyte = mbox_server.param[1]/8;
                LOG_DBG("DELEGATE_RSA_MOD_EXP_ENCRY\n");
                sys_cache_data_invd_range((void *)mbox_server.data[0], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[1], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[2], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[3], nbyte);
                err = ls_rsa_modexp_encrypt(mbox_server.data[0],mbox_server.param[0],mbox_server.data[3],&out_len,mbox_server.data[1],mbox_server.data[2],mbox_server.param[1]);
                if(err)
                {
                    LOG_ERR(" rsa mod exp failed\n");
                }
                param.op = DELEGATE_RSA_MOD_EXP_ENCRY;
                param.status = err;
                param.data[0] = mbox_server.data[3];
                sys_cache_data_flush_range((void *)mbox_server.data[3], nbyte);
                break;
            case DELEGATE_RSA_MOD_EXP_DECRY:
                LOG_DBG("DELEGATE_RSA_MOD_EXP_DECRY\n");
                nbyte = mbox_server.param[1]/8;
                sys_cache_data_invd_range((void *)mbox_server.data[0], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[1], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[2], nbyte);
                sys_cache_data_invd_range((void *)mbox_server.data[3], nbyte);
                err = ls_rsa_modexp_decrypt(mbox_server.data[0],mbox_server.param[0],mbox_server.data[3],&out_len,mbox_server.data[1],mbox_server.data[2],mbox_server.param[1]);
                if(err)
                {
                    LOG_ERR(" rsa mod exp failed\n");
                }
                param.op = DELEGATE_RSA_MOD_EXP_DECRY;
                param.status = err;
                param.data[0] = mbox_server.data[3];
                sys_cache_data_flush_range((void *)mbox_server.data[3], nbyte);
                break;
            default:
                param.status = -1;
                LOG_ERR("error operation\n");
                break;
        }
// exit:
        msg.data = &param;
        msg.size = sizeof(param);
        mbox_send_dt(&ls_otbn_server_tx,&msg);
    }while(0);
}
#endif