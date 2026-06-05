#ifndef _LS_OTBN_CONFIG_
#define _LS_OTBN_CONFIG_



typedef uint32_t (*otbn_rand_cb)(void);

void ls_otbn_wolfssl_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb);



#endif