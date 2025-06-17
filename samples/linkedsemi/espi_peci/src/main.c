/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "peci.c"

int main(void)
{
    printf("boot...");
    printf("espi done");

    printf("peci start");
    peci_main();

    return 0;
}
