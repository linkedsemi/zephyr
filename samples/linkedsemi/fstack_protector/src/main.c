/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>

void test_func(void)
{
	printf("\n%s\n", __func__);

	int test[32] = {0};
	for (int i = 0; i < 100; i++) {
		printf("test[%d]: %d\n", i, test[i]);
	}

	for (int i = 0; i < 100; i++) {
		test[i] = i;
	}

	for (int i = 0; i < 100; i++) {
		printf("test[%d]: %d\n", i, test[i]);
	}
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	test_func();
	test_func();

	return 0;
}
