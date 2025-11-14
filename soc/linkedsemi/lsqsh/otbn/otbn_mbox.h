#ifndef __LS_OTBN_MBOX__
#define __LS_OTBN_MBOX__



#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_SERVER)
void ls_otbn_delegation_server_chanels_init(void);
#endif

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
void ls_otbn_delegation_client_chanels_init(void);
#endif

#endif