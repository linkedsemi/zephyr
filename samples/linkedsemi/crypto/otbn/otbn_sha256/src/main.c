/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ls_hal_otbn_sha.h>

#define MAX_STR_BUF (32 * 2 + 1)
#define MAX_BUF 32
extern const char sha256_text[540];

static const uint8_t *result_check = \
    "81c03b03b75b7779ed13fb36038795a81a1f7f642f5c4a1fc8e391e1930bfd08";
static uint8_t result_str[MAX_STR_BUF];
static uint8_t result[MAX_BUF];

int main(void)
{
    int len = 0;
    int ret = 0;

    HAL_OTBN_SHA256_Init();
    HAL_OTBN_SHA256_Update((uint8_t *)sha256_text, 540);
    HAL_OTBN_SHA256_Final(result);

    for (uint8_t i = 0; i < MAX_BUF; i++) {
        len += snprintf(result_str + len, MAX_STR_BUF - len, "%2.2x", result[i]);
    }

    ret = strcmp(result_check, result_str);
    __ASSERT(ret == 0, "verify fail");

    printf("verify pass\n");
    printf("expect: %s\n", result_check);
    printf("result: %s\n", result_str);

    return 0;
}
