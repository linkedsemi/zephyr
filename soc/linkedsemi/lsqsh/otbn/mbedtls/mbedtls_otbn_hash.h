#include <stdio.h>
#include <string.h>
#include "ls_hal_otbn_sha.h"

void ls_otbn_sha256_init_for_rtos(void);
void ls_otbn_sha256_update_for_rtos(uint8_t *msg, uint32_t length);
void ls_otbn_sha256_final_for_rtos(uint8_t result[0x20]);


void ls_otbn_sha384_init_for_rtos(void);
void ls_otbn_sha384_update_for_rtos(uint8_t *msg, uint32_t length);
void ls_otbn_sha384_final_for_rtos(uint8_t result[SHA384_RESULT_SIZE]);

void ls_otbn_sha512_init_for_rtos(void);
void ls_otbn_sha512_update_for_rtos(uint8_t *msg, uint32_t length);
void ls_otbn_sha512_final_for_rtos(uint8_t result[SHA512_RESULT_SIZE]);

void ls_otbn_sm3_init_for_rtos(void);
void ls_otbn_sm3_update_for_rtos(uint8_t *msg, uint32_t length);
void ls_otbn_sm3_final_for_rtos(uint8_t result[0x20]);