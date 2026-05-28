#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA256_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA256_

#include <stdint.h>
#include <zephyr/crypto/crypto.h>
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include "reg_sha_type.h"

#define LOG_LEVEL CONFIG_SHA256_LOG_LEVEL

#define SHA256_LINKEDSEMI_HASH_CAPS   (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#define SHA256_TOTAL_LEN_BYTE       (0x08)
#define SHA256_PADDING_ZERO         (0x00)
#define SHA256_PADDING_BYTE         (0x80)
#define SHA256_BLOCK_BYTE_SIZE      (0x40)
#define SHA256_BLOCK_WORD_SIZE      (SHA256_BLOCK_BYTE_SIZE / sizeof(uint32_t))
#define SHA_PADDING_MOD             (SHA256_BLOCK_BYTE_SIZE - SHA256_TOTAL_LEN_BYTE)
#define SHA224_WORDS_NUM 7
#define SHA256_SM3_WORDS_NUM 8

struct sha256_linkedsemi_data {
    struct k_mutex sha256_engine_mutex;
    uint32_t current_word;
    uint8_t current_block_bytes;
    uint64_t total_length;
    bool first_update;
    enum hash_algo algo;
};

struct sha256_linkedsemi_config {
	reg_sha_t *reg; /* SHA256 engine base address */
	void (*irq_config_func)(const struct device *);
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA256_ */
