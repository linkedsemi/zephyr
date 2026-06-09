#ifndef _LS_OTBN_CONFIG_
#define _LS_OTBN_CONFIG_


typedef uint32_t (*otbn_rand_cb)(void);

typedef void (*otbn_done_callback_t )(void *param);

void ls_otbn_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb);

void ls_otbn_done_callback_register(otbn_done_callback_t handler, void *param);

void ls_otbn_done_callback_unregister(void);

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_SERVER)
void ls_otbn_delegation_server_chanels_init(void);
#endif

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
void ls_otbn_delegation_client_chanels_init(void);
#endif

#endif