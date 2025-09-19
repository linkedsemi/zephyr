/*
 * Copyright (c) 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/crypto/crypto.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "stdio.h"

#include <zephyr/crypto/ls_otbn_sm2.h>
#include <zephyr/crypto/ls_otbn_ecc_p384.h>
#include <zephyr/crypto/crypto_linkedsemi_otbn.h>


LOG_MODULE_REGISTER(main);
const struct device *otbn_ecc_p384_dev = DEVICE_DT_GET(DT_NODELABEL(ecc_p384));

uint8_t re_sign[65] = {0};

uint8_t private_k[64] = {0};
uint8_t public_x[64] = {0};
uint8_t public_y[64] = {0};

uint8_t private_k2[64] = {0};
uint8_t public_x2[64] = {0};
uint8_t public_y2[64] = {0};

uint8_t private_k3[64] = {0};
uint8_t public_x3[64] = {0};
uint8_t public_y3[64] = {0};

static struct ecc_p384_ctx ctx_p384;
static struct ecc_p384_key key_p384 =
{
	.d = private_k,
	.qx = public_x,
	.qy = public_y,
};


uint8_t msg_e[64] = 
{
	0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,
	0x8,0x9,0xa,0xb,0xc,0xd,0xe,0xf,
	0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,
	0x8,0x9,0xa,0xb,0xc,0xd,0xe,0xf,
};

uint8_t msg_r[64] = {0};
uint8_t msg_s[64]= {0}; 
struct ecc_p384_pkt pkt_p384 =
{
	.m = msg_e,
	.m_len = 48,
	.r = msg_r,
	.s = msg_s
};

int main(void)
{

	LOG_INF("___________________start p384 ecdsa test_______________");
	ecc_p384_begin_session(otbn_ecc_p384_dev,&ctx_p384,&key_p384);
	LOG_INF("start generater keys");
	ecc_p384_keygen(&ctx_p384,&key_p384);
	LOG_HEXDUMP_INF(private_k,32,"private_k");
	LOG_HEXDUMP_INF(public_x,32,"public_x");
	LOG_HEXDUMP_INF(public_y,32,"public_y");

	// ecc_p384_keygen(&ctx_p384,&key2_p384);
	// LOG_HEXDUMP_INF(private_k2,32,"private_k2");
	// LOG_HEXDUMP_INF(public_x2,32,"public_x2");
	// LOG_HEXDUMP_INF(public_y2,32,"public_y2");

	LOG_INF("start sign");
	ecc_p384_sign(&ctx_p384,&key_p384,&pkt_p384);
	LOG_HEXDUMP_INF(msg_e,32,"msg_e");
	LOG_HEXDUMP_INF(msg_r,32,"msg_r");
	LOG_HEXDUMP_INF(msg_s,32,"msg_s");

	LOG_INF("start verify");
	uint32_t verify_result;
	verify_result = ecc_p384_verify(&ctx_p384,&key_p384,&pkt_p384);
	if(verify_result == 0)
	{
		LOG_INF("ecc_p384_verify, verify_result :succeed");
	}else
	{
		LOG_INF("ecc_p384_verify, verify_result :fail");
	}
	return 0;
}