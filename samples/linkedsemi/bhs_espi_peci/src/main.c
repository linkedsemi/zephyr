/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "bhs.c"
#include "peci.c"

int main(void)
{
    printf("boot...");
    bhs_bmc_ready();
    printf("done");

    printf("peci start");
    peci_main();

    return 0;
}
