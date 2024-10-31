#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_

#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>

#define CRYPTO_LINKEDSEMI_CIPHER_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)
#define CRYPTO_LINKEDSEMI_HASH_CAPS   (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#define CRYPTO_LINKEDSEMI_AES_MAX_KEY_LEN_BIT  512
#define CRYPTO_LINKEDSEMI_AES_MAX_KEY_LEN_BYTE 32
#define AES_BLOCK_LEN_BYTE                     16
#define IV_LEN_BYTE                            AES_BLOCK_LEN_BYTE

#define CRYPTO_LINKEDSEMI_SHA_MAX_BLOCK_NUM 0x4000
#define SHA_BLOCK_LEN_BYTE                  64
#define SHA_PADDING_MOD_LEN_BYTE            56

struct crypto_linkedsemi_data {
    void *user_data;
    const struct device *dev;
    uint32_t data;

/*  cipher engine */
    struct k_mutex cipher_mutex;
    struct k_sem cipher_device_sync_sem;
    enum cipher_algo cipher_algo;
    enum cipher_mode cipher_mode;

/*  hash engine */
    struct k_mutex hash_mutex;
    struct k_sem hash_device_sync_sem;
    enum hash_algo hash_algo;
    struct hash_ctx *hash_ctx;
    struct hash_pkt *hash_pkt;
    uint32_t sha_fifo;
    uint8_t sha_fifo_index;
    uint16_t sha_current_block_index;
    uint64_t sha_pkt_in_buf_index;
    uint64_t sha_total_len;
    bool sha_is_final;
};

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct crypto_linkedsemi_config {
    mem_addr_t reg_calc_sha;
    mem_addr_t reg_calc_crc;
    mem_addr_t reg_calc;
    mem_addr_t reg_crypt;
    mem_addr_t reg_calc_sm4;
    uint32_t data;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    irq_cfg_func_t irq_config_func;
};

void linkedsemi_crypto_isr(const struct device *dev);
int crypto_linkedsemi_ecb_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt);
int crypto_linkedsemi_ecb_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt);
int crypto_linkedsemi_cbc_encrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv);
int crypto_linkedsemi_cbc_decrypt(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv);
int crypto_linkedsemi_ctr(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *ctr);
int crypto_linkedsemi_gcm_encrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);
int crypto_linkedsemi_gcm_decrypt_auth(struct cipher_ctx *ctx, struct cipher_aead_pkt *apkt, uint8_t *nonce);

void linkedsemi_sha_isr(const struct device *dev);
int crypto_linkedsemi_sha(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish);

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_ */
