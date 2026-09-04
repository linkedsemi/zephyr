
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include "ls_hal_otbn.h"
#include "ls_msp_otbn.h"
#include "field_manipulate.h"
#include "reg_sysc_sec_cpu.h"
#include "ls_otbn_config.h"

/* Single REGISTER for all mbedtls OTBN alt sources (rsa_alt/ecdsa_alt/mbox). */
LOG_MODULE_REGISTER(mbedtls_otbn, CONFIG_LINKEDSEMI_OTBN_LOG_LEVEL);

int mbedtls_ls_otbn_operation_init(otbn_firmware_t firmware_id)
{
    int err;
    err = ls_otbn_session_acquire(firmware_id, 10);
    if (err) {
        return err;
    }
    return err;
}