#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SM4_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SM4_

#include <stdint.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/drivers/clock_control.h>
#include "soc_clock.h"
#include "reg_sm4_type.h"

#define LOG_LEVEL CONFIG_SM4_LOG_LEVEL

#define SM4_LINKEDSEMI_CIPHER_CAPS (CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_NO_IV_PREFIX)

#define SM4_CCM_NONCE_LENTH         (0x12)
/* max additional authenticated size in bytes: 2^16 - 2^8 = 65280 */
#define SM4_CCM_AAD_MAX_BYTES           (0xff00)
/* max message size in bytes: 2^(8L) = 2^16 = 65536 */
#define SM4_CCM_PAYLOAD_MAX_BYTES       (0x10000)

#define SM4_GCM_MIN_AUTH_TAG_SIZE   (0xc)

#define SM4_KEY_LENGTH              (0x10)

#define SM4_BLOCK_LENGTH            (0x10)

struct sm4_linkedsemi_data {
    struct k_mutex sm4_engine_mutex;
    struct k_sem key_ex_sem;
    struct k_sem cal_end_sem;
    struct k_sem wait_data_sem;

	uint32_t buffer[SM4_BLOCK_LENGTH / 4];
};

struct sm4_linkedsemi_config {
	reg_sm4_t *reg; /* SM4 engine base address */
	void (*irq_config_func)(const struct device *);
    struct ls_clk_cfg cctl_cfg;
};

extern struct cipher_ops sm4_encrypt_ops;
extern struct cipher_ops sm4_decrypt_ops;

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SM4_ */
