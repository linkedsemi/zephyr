#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA512_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA512_

#include <stdint.h>
#include <zephyr/crypto/crypto.h>
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include "reg_sha512_type.h"

#define LOG_LEVEL CONFIG_SHA512_LOG_LEVEL

#define SHA512_LINKEDSEMI_HASH_CAPS   (CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS)

#define SHA512_MODE_SHA384          (0x2)
#define SHA512_MODE_SHA512          (0x3)

#define SHA384_RESULT_WORD_NUM      (0xC)
#define SHA512_RESULT_WORD_NUM      (0x10)

#define SHA512_TOTAL_LEN_BYTE       (0x10)
#define SHA512_PADDING_ZERO         (0x00)
#define SHA512_PADDING_BYTE         (0x80)
#define SHA512_BLOCK_BYTE_SIZE      (0x80)
#define SHA512_BLOCK_WORD_SIZE      (SHA512_BLOCK_BYTE_SIZE / sizeof(uint32_t))
#define SHA512_FIANL_LENGTH         (SHA512_BLOCK_BYTE_SIZE - SHA512_TOTAL_LEN_BYTE)

__attribute__((aligned(32))) struct sha512_linkedsemi_data {
	uint32_t buffer[SHA512_BLOCK_WORD_SIZE];
	struct k_mutex sha512_engine_mutex;
	struct k_sem calc_end_sem;
	uint32_t buf_idx;
	uint32_t total_len;
	uint8_t result_word_num;
	uint8_t algo;
	bool is_first;
};

struct sha512_linkedsemi_config {
	reg_sha512_t *reg; /* SHA512 engine base address */
	void (*irq_config_func)(const struct device *);
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA512_ */
