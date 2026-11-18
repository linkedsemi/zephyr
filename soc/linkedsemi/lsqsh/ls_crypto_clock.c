/*
 * Copyright (c) 2026 LINKEDSEMI Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * The crypto engine clocks live in SEC_SYSC_CPU. CPU1 (security core)
 * owns that block; CPU2 has no IOPMP access to it. This file compiles
 * the real implementation on CPU1 and stubs on CPU2, so a stray call
 * returns -ENOTSUP instead of faulting the bus.
 */

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include "ls_crypto_clock.h"

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)

#include <platform.h>

#define LS_CRYPTO_CLK_TABLE(X)                                              \
	X(LS_CRYPTO_CLK_SHA256, CALC_SHA_CLOCK, CALC_SHA_RESET)             \
	X(LS_CRYPTO_CLK_SHA512, SHA512_CLOCK, SHA512_RESET)                 \
	X(LS_CRYPTO_CLK_AES256, CRYPT_CLOCK, CRYPT_RESET)                   \
	X(LS_CRYPTO_CLK_SM4, CALC_SM4_CLOCK, CALC_SM4_RESET)

int ls_crypto_clock_on(enum ls_crypto_clk clk)
{
	switch (clk) {
#define X(_id, _clk, _rst)                                                  \
	case _id:                                                           \
		ls_clock_control_on(_clk);                                  \
		return 0;
		LS_CRYPTO_CLK_TABLE(X)
#undef X
	default:
		return -EINVAL;
	}
}

int ls_crypto_clock_off(enum ls_crypto_clk clk)
{
	switch (clk) {
#define X(_id, _clk, _rst)                                                  \
	case _id:                                                           \
		ls_clock_control_off(_clk);                                 \
		return 0;
		LS_CRYPTO_CLK_TABLE(X)
#undef X
	default:
		return -EINVAL;
	}
}

int ls_crypto_clock_reset(enum ls_crypto_clk clk)
{
	switch (clk) {
#define X(_id, _clk, _rst)                                                  \
	case _id:                                                           \
		ls_reset_line_toggle(_rst);                                 \
		return 0;
		LS_CRYPTO_CLK_TABLE(X)
#undef X
	default:
		return -EINVAL;
	}
}

int ls_crypto_clock_prepare(enum ls_crypto_clk clk)
{
	switch (clk) {
#define X(_id, _clk, _rst)                                                  \
	case _id:                                                           \
		ls_clock_control_on_reset_line_toggle_once(_clk, _rst);     \
		return 0;
		LS_CRYPTO_CLK_TABLE(X)
#undef X
	default:
		return -EINVAL;
	}
}

bool ls_crypto_clock_is_on(enum ls_crypto_clk clk)
{
	switch (clk) {
#define X(_id, _clk, _rst)                                                  \
	case _id:                                                           \
		return ls_clock_control_is_on(_clk);
		LS_CRYPTO_CLK_TABLE(X)
#undef X
	default:
		return false;
	}
}

#else /* CPU2: the SEC_SYSC_CPU block is not accessible. */

int ls_crypto_clock_on(enum ls_crypto_clk clk)
{
	ARG_UNUSED(clk);
	return -ENOTSUP;
}

int ls_crypto_clock_off(enum ls_crypto_clk clk)
{
	ARG_UNUSED(clk);
	return -ENOTSUP;
}

int ls_crypto_clock_reset(enum ls_crypto_clk clk)
{
	ARG_UNUSED(clk);
	return -ENOTSUP;
}

int ls_crypto_clock_prepare(enum ls_crypto_clk clk)
{
	ARG_UNUSED(clk);
	return -ENOTSUP;
}

bool ls_crypto_clock_is_on(enum ls_crypto_clk clk)
{
	ARG_UNUSED(clk);
	return false;
}

#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay) */
