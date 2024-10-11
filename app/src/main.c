/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <peci.h>


/* PECI Target address */
#define PECI_TARGET_ADDR          0x30u

int main(void)
{
    EPECIStatus ret = PECI_CC_SUCCESS;
    printf("Hello World from APP! %s\n", CONFIG_BOARD);

    ret = peci_Ping(PECI_TARGET_ADDR);
    printf("PECI Ping: %d\n", ret);
    return 0;
}
