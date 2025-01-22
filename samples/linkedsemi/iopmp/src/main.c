/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <reg_base_addr.h>
#include <platform.h>
#include <iopmp.h>

#define TEST_NAPOT
#if 0
    #define TEST_TOR
#endif

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    app_cpu_reset();

    for (uint32_t idx = 0; idx < 5; idx++) {
        uint32_t dev = SEC_IOPMP1_ADDR + (idx * 0x400);
#if defined(TEST_NAPOT)
/*

| N | addr                             | mode  | RWX  |       desc               |
| - | - | - | - | - |
| 0 | 0x8000000--(0x8000000+2MB)       | NAPOT | ---  | sec flash xip mem        |
| 1 | 0x8000000--(0x8000000+64MB)      | NAPOT | RWX  | app flash xip mem        |
| 2 | 0x10000000--(0x10000000+512KB)   | NAPOT | ---  | sec sram                 |
| 3 | 0x10000000--(0x10000000+128MB)   | NAPOT | RWX  | app sram                 |
| 4 | 0x40000000--(0x40000000+12KB)    | NAPOT | ---  | sec peripheral region 1  |
| 5 | 0x400A0000--(0x400A0000+42KB)    | NAPOT | ---  | sec peripheral region 2  |
| 6 | 0x40000000--(0x40000000+128MB)   | NAPOT | RWX  | app peripheral           |
| 7 |                                  |       |      |                          |
|

*/
        iopmp_config_region_napot4(dev, 0, 0x8000000, MB(2), false, false, false, false);
        iopmp_config_region_napot4(dev, 1, 0x8000000, MB(64), true, true, true, false);
        iopmp_config_region_napot4(dev, 2, 0x10000000, KB(512), false, false, false, false);
        iopmp_config_region_napot4(dev, 3, 0x10000000, MB(128), true, true, true, false);
        iopmp_config_region_napot4(dev, 4, 0x40000000, KB(12), false, false, false, false);
        iopmp_config_region_napot4(dev, 5, 0x400A0000, KB(42), false, false, false, false);
        iopmp_config_region_napot4(dev, 6, 0x40000000, MB(128), true, true, true, false);
#elif defined(TEST_TOR)
/*

| N | addr                             | mode  | RWX  |       desc            |        
| - | - | - | - | - | 
| 0 | 0x8000000+2MB                    | TOR   | ---  |                       |
| 1 | 0x8000000+64MB                   | TOR   | RWX  | app flash xip mem     |
| 2 | 0x10000000+512KB                 | TOR   | ---  |                       |
| 3 | 0x10000000+128MB                 | TOR   | RWX  | app sram              |
| 4 | 0x40000000+12KB                  | TOR   | ---  |                       |
| 5 | 0x400A0000                       | TOR   | RWX  | app peripheral        |
| 6 | 0xffffffff                       | TOR   | ---  |                       |
| 7 |                                  |       |      |                       |
|

*/
        iopmp_config_region_tor(dev, 0, (0x8000000 + MB(2)), false, false, false, false);
        iopmp_config_region_tor(dev, 1, (0x8000000 + MB(64)), true, true, true, false);
        iopmp_config_region_tor(dev, 2, (0x10000000 + KB(512)), false, false, false, false);
        iopmp_config_region_tor(dev, 3, (0x10000000 + MB(128)), true, true, true, false);
        iopmp_config_region_tor(dev, 4, (0x40000000 + KB(12)), false, false, false, false);
        iopmp_config_region_tor(dev, 5, (0x400A0000), true, true, true, false);
        iopmp_config_region_tor(dev, 6, (0xffffffff), false, false, false, false);
#endif
        iopmp_config_enable(dev, true);
    }

    app_cpu_dereset();

    return 0;
}
