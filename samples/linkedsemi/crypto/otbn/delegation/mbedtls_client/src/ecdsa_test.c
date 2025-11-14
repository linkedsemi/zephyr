

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <limits.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include "mbedtls/bignum.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecdsa_alt.h"
#include "mbedtls/ecp.h"
#include "mbedtls/pk.h"
#include "mbedtls/md.h"
#include "ls_hal_otbn_sha.h"
#include "ls_msp_otbn.h"
#include "log.h"
#include <zephyr/drivers/entropy.h>
/************************************************************
 *
 * defines
 *
 ************************************************************/
#define DIGEST_SIZE         64
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
const static struct device *trng = DEVICE_DT_GET(DT_NODELABEL(trng0));
#endif


#define STACK_SIZE (1024)
static struct k_thread otbn_thread1;
static struct k_thread otbn_thread2;
K_THREAD_STACK_DEFINE(tstack1, STACK_SIZE);
K_THREAD_STACK_DEFINE(tstack2, STACK_SIZE);

static uint8_t fix_trng_output[DIGEST_SIZE];
int ecdsa_test(void);

int ls_wolfssl_get_random(void *null, unsigned char *buf, size_t size)
{
    (void)null;
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    return entropy_get_entropy(trng, buf, size);
#else
    return 0;
#endif
}

void otbn_task1_func(void *p1, void *p2, void *p3)
{
    while(1)
    {
        ecdsa_test();
    }

}

void otbn_task2_fun(void *p1, void *p2, void *p3)
{
    while(1)
    {
        ecdsa_test();
    }

}

void multiple_otbn_thread_test(void)
{
    k_tid_t tid[2];
	/* the highest-priority thread that has waited the longest */
	tid[0] = k_thread_create(&otbn_thread1, tstack1, STACK_SIZE,
			otbn_task1_func, NULL, NULL, NULL,
			K_PRIO_PREEMPT(0), 0, K_MSEC(10));
	/* the lowest-priority thread that has waited the shorter */
	tid[1] = k_thread_create(&otbn_thread2, tstack2, STACK_SIZE,
			otbn_task2_fun, NULL, NULL, NULL,
			K_PRIO_PREEMPT(1), 0, K_MSEC(20));
}

int runIt_unhexify(unsigned char *obuf, const char *ibuf)
{
    #define assert(a) do{} while(0)
    unsigned char c, c2;
    int len = strlen(ibuf) / 2;
    assert(strlen( ibuf ) % 2 == 0); /* must be even number of bytes */

    while (*ibuf != 0)
    {
        c = *ibuf++;
        if (c >= '0' && c <= '9')
            c -= '0';
        else if (c >= 'a' && c <= 'f')
            c -= 'a' - 10;
        else if (c >= 'A' && c <= 'F')
            c -= 'A' - 10;
        else
            assert(0);

        c2 = *ibuf++;
        if (c2 >= '0' && c2 <= '9')
            c2 -= '0';
        else if (c2 >= 'a' && c2 <= 'f')
            c2 -= 'a' - 10;
        else if (c2 >= 'A' && c2 <= 'F')
            c2 -= 'A' - 10;
        else
            assert(0);

        *obuf++ = (c << 4) | c2;
    }

    return len;
}

int ecdsa_p256_test(void)
{
    int rc = 0;
    static const char *d_str = "DC51D3866A15BACDE33D96F992FCA99DA7E6EF0934E7097559C27F1614C88A7F";
    static const char *xQ_str = "2442A5CC0ECD015FA3CA31DC8E2BBC70BF42D60CBCA20085E0822CB04235E970";
    static const char *yQ_str = "6FC98BD7E50211A4A27102FA3549DF79EBCB4BF246B80945CDDFE7D509BBFD7D";
    static const char *k_str = "9E56F509196784D963D1C0A401510EE7ADA3DCC5DEE04B154BF61AF1D5A6DECE";
    static const char *hash_str = "BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD";
    static const char *r_str = "CB28E0999B9C7715FD0A80D8E47A77079716CBBF917DD72E97566EA1C066957C";
    static const char *s_str = "86FA3BB4E26CAD5BF90B7F81899256CE7594BB1EA0C89212748BFF3B3D5B0315";

    mbedtls_ecp_group pGrp;
    mbedtls_ecp_point pQ;
    mbedtls_mpi d;
    mbedtls_mpi r;
    mbedtls_mpi s;
    mbedtls_mpi rCheck;
    mbedtls_mpi sCheck;

    uint8_t hash_buf[100];
    uint8_t radom_buf[100];
    unsigned char *pHash = hash_buf;
    unsigned char *pRndBuf = radom_buf;

    size_t hlen;
    mbedtls_ecp_group_init(&pGrp);
    mbedtls_ecp_point_init(&pQ);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    mbedtls_mpi_init(&rCheck);
    mbedtls_mpi_init(&sCheck);
    memset(pHash, 0, 66);
    memset(pRndBuf, 0, 66);

    mbedtls_ecp_group_load(&pGrp, MBEDTLS_ECP_DP_SECP256R1);
    mbedtls_ecp_point_read_string(&pQ, 16, xQ_str, yQ_str);

    mbedtls_mpi_read_string(&d, 16, d_str);
    mbedtls_mpi_read_string(&rCheck, 16, r_str);
    mbedtls_mpi_read_string(&sCheck, 16, s_str);

    hlen = runIt_unhexify(pHash, hash_str);


    uint16_t rnd_length = runIt_unhexify(pRndBuf, k_str);

    /* Fix pRndBuf by shifting it left if necessary */
    if (pGrp.nbits % 8 != 0)
    {
        unsigned char shift = 8 - (pGrp.nbits % 8);
        size_t i;

        for (i = 0; i < rnd_length - 1; i++)
            pRndBuf[i] = pRndBuf[i] << shift | pRndBuf[i + 1] >> (8 - shift);

        pRndBuf[rnd_length - 1] <<= shift;
    }

    memcpy(fix_trng_output,pRndBuf,DIGEST_SIZE);


    if(mbedtls_ecdsa_sign(&pGrp, &r, &s, &d, pHash, hlen, ls_wolfssl_get_random, NULL) != 0)
    {
        while(1);
    }

    if(mbedtls_mpi_cmp_mpi(&r, &rCheck) != 0)
    {
        printf("error r\r\n");
    }
    if(mbedtls_mpi_cmp_mpi(&s, &sCheck) != 0)
    {
        printf("error s\r\n");
    }
    if(mbedtls_ecdsa_verify(&pGrp, pHash, hlen, &pQ, &rCheck, &sCheck) != 0)
    {
        while(1);
    }

    mbedtls_ecp_group_free(&pGrp);
    mbedtls_ecp_point_free(&pQ);

    mbedtls_mpi_free(&d);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&rCheck);
    mbedtls_mpi_free(&sCheck);

    return rc;
}

int ecdsa_test_curve(mbedtls_ecp_group_id curve)
{
    int err = 0;
    mbedtls_ecdsa_context ctx;
    mbedtls_mpi r;
    mbedtls_mpi s;
    static const char *hash_str = "BA7816BF8F01CFEA414140DE5DAE2223B00361A396177A9CB410FF61F20015AD";

    if(!(curve == MBEDTLS_ECP_DP_SECP384R1 || curve == MBEDTLS_ECP_DP_SECP256R1 || curve == MBEDTLS_ECP_DP_SM2))
    {
        printf("This curve is not supported.\n");
    }

    mbedtls_ecp_group_init(&ctx.private_grp);
    mbedtls_ecp_group_load(&ctx.private_grp, curve);

    mbedtls_mpi_init(&ctx.private_Q.private_X);
    mbedtls_mpi_init(&ctx.private_Q.private_Y);
    mbedtls_mpi_init(&ctx.private_Q.private_Z);
    mbedtls_mpi_init(&ctx.private_d);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    // generator key pairs 
    err = mbedtls_ecdsa_genkey(&ctx,curve,ls_wolfssl_get_random,NULL);
    if(err)
    {
        printf(" ecc keygen failed \r\n");
        goto exit_test;
    }
    uint8_t hash_buf[100];
    unsigned char *pHash = hash_buf;
    size_t hlen;
    hlen = runIt_unhexify(pHash, hash_str);
    // sign
    if(mbedtls_ecdsa_sign(&ctx.private_grp, &r, &s, &ctx.private_d, pHash, hlen, ls_wolfssl_get_random, NULL) != 0)
    {
        err = -1;
        printf(" ecdsa sign failed\n");
        goto exit_test;
    }
    // verify
    if(mbedtls_ecdsa_verify(&ctx.private_grp, pHash, hlen, &ctx.private_Q, &r, &s) != 0)
    {
        err = -1;
        printf(" ecdsa verify failed\n");
        goto exit_test;
    }

exit_test:

    mbedtls_ecp_group_free(&ctx.private_grp);
    mbedtls_mpi_free(&ctx.private_Q.private_X);
    mbedtls_mpi_free(&ctx.private_Q.private_Y);
    mbedtls_mpi_free(&ctx.private_Q.private_Z);
    mbedtls_mpi_free(&ctx.private_d);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
  
    return err;
}

int ecdsa_test(void)
{
    int ret = 0;

    if(ecdsa_test_curve(MBEDTLS_ECP_DP_SECP384R1) != 0)
    {
        ret = -1;
        printf("ecc p384 test failed! \n");
        goto exit;
    }else{
        printf("ecc p384 test passed!\n");
    }
    if(ecdsa_test_curve(MBEDTLS_ECP_DP_SECP256R1) != 0)
    {
        ret = -1;
        printf("ecc p256 test failed! \n");
        goto exit;
    }else{
        printf("ecc p256 test passed!\n");
    }
    if(ecdsa_test_curve(MBEDTLS_ECP_DP_SM2) != 0)
    {
        ret = -1;
        printf("sm2 test failed! \n");
        goto exit;
    }else{
        printf("sm2  test passed!\n");
    }

exit:
    return ret;
}
