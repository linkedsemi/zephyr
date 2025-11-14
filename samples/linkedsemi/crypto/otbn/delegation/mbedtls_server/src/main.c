#include <string.h>
#include <stdio.h>
#include <zephyr/devicetree.h>
#include "mbedtls/threading.h"


#if defined(CONFIG_MEBDTLS_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
void mbedtls_ls_otbn_delegation_client_chanels_init(void);
#else
void mbedtls_ls_otbn_delegation_server_chanels_init(void);
#endif

int main(void)
{
    mbedtls_ls_otbn_delegation_server_chanels_init();

    return 0;
}

