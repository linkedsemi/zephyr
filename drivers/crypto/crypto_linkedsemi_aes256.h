#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMI_AES256H_AES_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMI_AES256H_AES_

#include <stdint.h>

#define CRYPT_DATA0 0x0
#define CRYPT_DATA1 0x4
#define CRYPT_DATA2 0x8
#define CRYPT_DATA3 0xc
#define CRYPT_KEY0 0x10
#define CRYPT_KEY1 0x14
#define CRYPT_KEY2 0x18
#define CRYPT_KEY3 0x1c
#define CRYPT_KEY4 0x20
#define CRYPT_KEY5 0x24
#define CRYPT_KEY6 0x28
#define CRYPT_KEY7 0x2c
#define CRYPT_IVR0 0x30
#define CRYPT_IVR1 0x34
#define CRYPT_IVR2 0x38
#define CRYPT_IVR3 0x3c
#define CRYPT_RES0 0x40
#define CRYPT_RES1 0x44
#define CRYPT_RES2 0x48
#define CRYPT_RES3 0x4c
#define CRYPT_CR   0x50
#define CRYPT_SR   0x54
#define CRYPT_ICFR 0x58
#define CRYPT_FIFO 0x5c

typedef union aes_reg_cr {
    uint32_t value;
    struct {
        uint32_t
            GO: 1,         /*[0]    */
            ENCS: 1,       /*[1]    */
            AESKS: 2,      /*[2-3]  */
            MODE: 2,       /*[4-5]  */
            IVREN: 1,      /*[6]    */
            IE: 1,         /*[7]    */
            TYPE: 2,       /*[8-9]  */
            TDES: 1,       /*[10]   */
            DESKS: 1,      /*[11]   */
            FIFOEN: 1,     /*[12]   */
            FIFOODR: 1,    /*[13]   */
            DMAEN: 1,      /*[14]   */
            RESERVED0: 16, /*[15-30]*/
            CRYSEL: 1;     /*[31]   */
    };
} aes_reg_cr_t;

typedef union aes_reg_sr {
    uint32_t value;
    struct {
        uint32_t
            AESRIF: 1,       /*[0]*/
            DESRIF: 1,       /*[1]*/
            RESERVED0 : 6,   /*[2-7]*/
            DONE: 1,         /*[8]*/
            RESERVED1 : 23;  /*[9-31]*/
    };
} aes_reg_sr_t;

typedef union aes_reg_icfr {
    uint32_t value;
    struct {
        uint32_t
            AESIF: 1,       /*[0]*/
            DESIF: 1,       /*[1]*/
            MULIF: 1,       /*[2]*/
            RESERVED0 : 29; /*[3-31]*/
    };
} aes_reg_icfr_t;

#define CRYPTO_LINKEDSEMI_AES256_CIPHER_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

#define CRYPTO_LINKEDSEMI_AES256_AES_MAX_KEY_LEN_BIT  512
#define CRYPTO_LINKEDSEMI_AES256_AES_MAX_KEY_LEN_BYTE 32
#define AES_BLOCK_LEN_BYTE                     16
#define IV_LEN_BYTE                            AES_BLOCK_LEN_BYTE

/* max additional authenticated size in bytes: 2^16 - 2^8 = 65280 */
#define CCM_AAD_MAX_BYTES 0xff00

/* max message size in bytes: 2^(8L) = 2^16 = 65536 */
#define CCM_PAYLOAD_MAX_BYTES 0x10000

struct crypto_linkedsemi_aes256_data {
    void *user_data;
    const struct device *dev;
    uint32_t data;

/*  cipher engine */
    struct k_mutex cipher_mutex;
    struct k_sem cipher_device_sync_sem;
    enum cipher_algo cipher_algo;
    enum cipher_mode cipher_mode;
};

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct crypto_linkedsemi_aes256_config {
    mem_addr_t reg_calc_crc;
    mem_addr_t reg_calc;
    mem_addr_t reg_crypt;
    uint32_t data;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    irq_cfg_func_t irq_config_func;
};

void linkedsemi_crypto_isr(const struct device *dev);
int crypto_linkedsemi_aes256_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt);
int crypto_linkedsemi_aes256_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt);
int crypto_linkedsemi_aes256_cbc_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv);
int crypto_linkedsemi_aes256_cbc_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv);
int crypto_linkedsemi_aes256_ctr(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *ctr);
int crypto_linkedsemi_aes256_ccm_encrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);
int crypto_linkedsemi_aes256_ccm_decrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);
int crypto_linkedsemi_aes256_gcm_encrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);
int crypto_linkedsemi_aes256_gcm_decrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMI_AES256H_AES_ */
