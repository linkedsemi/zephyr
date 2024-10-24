/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <peci.h>
#include <zephyr/kernel.h>

#ifndef ABS
#define ABS(_v_) (((_v_) > 0) ? (_v_) : -(_v_))
#endif


/* PECI Target address */
#define PECI_TARGET_ADDR          0x30u

int main(void)
{
    EPECIStatus ret = PECI_CC_SUCCESS;
    printf("Hello World from APP! %s\n", CONFIG_BOARD);
    uint64_t dib;
    short temperature;
    int n = 0;

    while(1){
        printf("Debug: In true loop!\n");
        ret = peci_Ping(PECI_TARGET_ADDR);
        printf("PECI Ping: %d\n", ret);

        ret = peci_GetDIB(PECI_TARGET_ADDR, &dib);
        printf("   0x%" PRIx64 "\n", dib);

        ret = peci_GetTemp(PECI_TARGET_ADDR, &temperature);
        printf("   %04xh (%c%d.%02dC)\n",
                           (int)(unsigned int)(unsigned short)temperature,
                           (0 > temperature) ? '-' : '+',
                           (int)((unsigned int)ABS(temperature) / 64),
                           (int)(((unsigned int)ABS(temperature) % 64) * 100) /
                               64);
            
        printf("Debug in %s: Times = %d\n", __func__, n++);

        k_sleep(K_SECONDS(0.5));    
    }

    return 0;
}
