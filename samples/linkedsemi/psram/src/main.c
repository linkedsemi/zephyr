/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include "reg_sysc_app_cpu.h"
#include "per_func_mux.h"
#include "ls_soc_gpio.h"
#include "DWC_ssi_v2_header.h"

#define DW_FIELD_BUILD(field,val) \
    (((unsigned int)(val)<<(field##_BitAddressOffset))&(((1 << field##_RegisterSize) - 1)<<(field##_BitAddressOffset)))

#define PSRAM_CACHE_SIZE 2048
uint8_t test_arr[PSRAM_CACHE_SIZE + 4] = {};

int main(void)
{
    uint32_t val = 0;
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    printf("SSIV2_SSIC_VERSION_ID: %#x\n", sys_read32(APP_PSRAM_CFG_ADDR + SSIV2_SSIC_VERSION_ID));

    pinmux_cfg_pin_func_alt(PSRAM_CSN0_FUNC4_PE12_PIN, PSRAM_CSN0_FUNC4_PE12_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_CLK_FUNC4_PE02_PIN, PSRAM_CLK_FUNC4_PE02_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT0_FUNC4_PE13_PIN, PSRAM_DAT0_FUNC4_PE13_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT1_FUNC4_PE14_PIN, PSRAM_DAT1_FUNC4_PE14_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT2_FUNC4_PE15_PIN, PSRAM_DAT2_FUNC4_PE15_FUNC, 0);
    pinmux_cfg_pin_func_alt(PSRAM_DAT3_FUNC4_PE03_PIN, PSRAM_DAT3_FUNC4_PE03_FUNC, 0);

    val = DW_FIELD_BUILD(SSIV2_CTRLR0_SSI_IS_MST, 0x1)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_SPI_FRF, 0x2)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_CFS, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_TMOD, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_FRF, 0x0)
        | DW_FIELD_BUILD(SSIV2_CTRLR0_DFS, 0x1f);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_CTRLR0);

    val = DW_FIELD_BUILD(SSIV2_BAUDR_SCKDV, 0x20);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_BAUDR);

    val = DW_FIELD_BUILD(SSIV2_TXFTLR_TFT, 0x8);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_TXFTLR);

    val = DW_FIELD_BUILD(SSIV2_RXFTLR_RFT, 0x8);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_RXFTLR);

    val = 0;
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_IMR);
 
    val = DW_FIELD_BUILD(SSIV2_XIP_MODE_BITS_XIP_MD_BITS, 0x0);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_MODE_BITS);
 
    val = DW_FIELD_BUILD(SSIV2_XIP_INCR_INST_INCR_INST, 0xeb);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_INCR_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRAP_INST_WRAP_INST, 0xeb);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRAP_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES, 0x0)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_INST_L, 0x2)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_ADDR_L, 0x6)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_TRANS_TYPE, 0x1)
         |DW_FIELD_BUILD(SSIV2_XIP_WRITE_CTRL_WR_FRF, 0x2);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_CTRL);

    val = DW_FIELD_BUILD(SSIV2_XIP_CTRL_XIP_PREFETCH_EN, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_XIP_MBL, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_INST_EN, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_DFS_HC, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_WAIT_CYCLES, 0x6)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_MD_BITS_EN, 0x0)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_INST_L, 0x2)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_ADDR_L, 0x6)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_TRANS_TYPE, 0x1)
        |DW_FIELD_BUILD(SSIV2_XIP_CTRL_FRF, 0x2);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_CTRL);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_INCR_INST_INCR_WRITE_INST, 0x38);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_INCR_INST);

    val = DW_FIELD_BUILD(SSIV2_XIP_WRITE_WRAP_INST_WRAP_WRITE_INST, 0x38);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_XIP_WRITE_WRAP_INST);

    val = DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_PREFETCH_EN, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_MBL, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_WAIT_CYCLES, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_INST_L, 0x2)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_XIP_MD_BIT_EN, 0x0)
         |DW_FIELD_BUILD(SSIV2_SPI_CTRLR0_TRANS_TYPE, 0x1);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_SPI_CTRLR0);

    val = DW_FIELD_BUILD(SSIV2_SSIENR_SSIC_EN, 1);
    sys_write32(val, APP_PSRAM_CFG_ADDR + SSIV2_SSIENR);

    /* cache enable */
    val = 0x00070001;
    sys_write32(val, APP_PSRAM_CCH_ADDR + 0x10);
    while(sys_read32(APP_PSRAM_CCH_ADDR + 0x28) != 0x101);

    /* ahb config */
    val = 0x4c2;
    sys_write32(val, APP_SYSC_CPU_APP_ADDR + 0x84);

    /* xip enable */
    val = BIT(0) | BIT(2);
    sys_write32(val, APP_SYSC_CPU_APP_ADDR + 0x80);

    printf("PSRAM_ADDR: %#x\n", sys_read32(PSRAM_ADDR + 0x123454));
    printf("PSRAM_ADDR: %#x\n", sys_read16(PSRAM_ADDR + 0x123454));
    printf("PSRAM_ADDR: %#x\n", sys_read8(PSRAM_ADDR + 0x123454));

    printf("PSRAM_ADDR: %#x\n", sys_read32(PSRAM_ADDR + 0x123453));

    sys_write32(0x125a346b, PSRAM_ADDR + 0x123454);

    printf("PSRAM_ADDR: %#x\n", sys_read32(PSRAM_ADDR + 0x123454));
    printf("PSRAM_ADDR: %#x\n", sys_read16(PSRAM_ADDR + 0x123454));
    printf("PSRAM_ADDR: %#x\n", sys_read8(PSRAM_ADDR + 0x123454));

    printf("PSRAM_ADDR: %#x\n", sys_read32(PSRAM_ADDR + 0x123453));

    memcpy((void *)PSRAM_ADDR, (void *)test_arr, sizeof(test_arr));

    return 0;
}
