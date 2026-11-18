/*
 * Copyright (c) 2026 LINKEDSEMI Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LS_CRYPTO_CLOCK_H_
#define LS_CRYPTO_CLOCK_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware crypto engine clock domains.
 *
 * All of them live in the SEC_SYSC_CPU register block, which only the
 * security core (CPU1) can access. Enable the clocks from CPU1 before
 * releasing CPU2 when the CPU2 crypto drivers are built with
 * <MODULE>_LINKEDSEMI_NO_CLOCK_RESET.
 */
enum ls_crypto_clk {
	LS_CRYPTO_CLK_SHA256 = 0,
	LS_CRYPTO_CLK_SHA512,
	LS_CRYPTO_CLK_AES256,
	LS_CRYPTO_CLK_SM4,
};

/**
 * @brief Enable the clock of a crypto engine.
 *
 * @return 0 on success, -EINVAL for an unknown id, -ENOTSUP on CPU2.
 */
int ls_crypto_clock_on(enum ls_crypto_clk clk);

/** @brief Disable the clock of a crypto engine. See ls_crypto_clock_on(). */
int ls_crypto_clock_off(enum ls_crypto_clk clk);

/** @brief Toggle the reset line without changing the clock state. */
int ls_crypto_clock_reset(enum ls_crypto_clk clk);

/**
 * @brief Bring a crypto engine clock up if it is not already enabled.
 *
 * When the clock is already on this does nothing: the engine and its
 * reset line are left untouched. Use ls_crypto_clock_reset() for a
 * forced reset.
 */
int ls_crypto_clock_prepare(enum ls_crypto_clk clk);

/** @brief Report whether the clock of a crypto engine is enabled. */
bool ls_crypto_clock_is_on(enum ls_crypto_clk clk);

#ifdef __cplusplus
}
#endif

#endif /* LS_CRYPTO_CLOCK_H_ */
