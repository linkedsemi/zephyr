/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "bhs.c"

int main(void)
{
    printf("boot...");
    bhs_bmc_ready();
    printf("done");

    return 0;
}
