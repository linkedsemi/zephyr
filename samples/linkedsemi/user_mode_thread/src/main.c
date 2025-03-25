/*
 * Copyright (c) 2020 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#define USER_STACKSIZE	4096

#ifndef CONFIG_USERSPACE
#error This sample requires CONFIG_USERSPACE.
#endif

#include <zephyr/app_memory/app_memdomain.h>

struct k_thread user_thread1;
struct k_thread user_thread2;

K_THREAD_STACK_DEFINE(user_stack1, USER_STACKSIZE);
K_THREAD_STACK_DEFINE(user_stack2, USER_STACKSIZE);



K_APPMEM_PARTITION_DEFINE(part_common);
K_APP_BMEM(part_common) static uint8_t t1_count = 0;

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *pEsf)
{
	// while(1);
	printf("-thread user_function1 is aborted !!!!!!!!!!\r\n");
}

static void user_function1(void *p1, void *p2, void *p3)
{
	static uint32_t Z_THREAD_LOCAL pr_cnt = 0;
	static uint32_t Z_THREAD_LOCAL thread_data32 = 0x11111111;
	while(1)
	{
		k_msleep(1);
		t1_count ++;
		printf("+user_function1---- 0x%x\r\n",thread_data32);
		t1_count++;
		if(pr_cnt++ > 10)
		{
			printf("+thread user_function1 can be abort !!!!!!!!!!\r\n");
			uint32_t *pcc = NULL;
			*pcc = thread_data32;
		}
	}
}

static void user_function2(void *p1, void *p2, void *p3)
{
	static uint32_t Z_THREAD_LOCAL thread_data32 = 0x22222222;
	while(1)
	{
		k_msleep(1);
		printf("-user_function2---- 0x%x\r\n",thread_data32);
	}
}

int main(void)
{
	/*
		用户模式函数中局部静态变量，定义是需要加Z_THREAD_LOCAL关键字，
		如果需要变量在多个用户模式线程之间传递，可查阅domain相关说明。
	*/
	k_mem_domain_add_partition(&k_mem_domain_default, &part_common);
	k_thread_create(&user_thread1, user_stack1, USER_STACKSIZE,
			user_function1, NULL, NULL, NULL,
			2, K_USER|K_INHERIT_PERMS, K_MSEC(0));
	k_thread_create(&user_thread2, user_stack2, USER_STACKSIZE,
		user_function2, NULL, NULL, NULL,
		2, K_USER|K_INHERIT_PERMS, K_MSEC(0));
	k_mem_domain_add_thread(&k_mem_domain_default, &user_thread1);
	return 0;
}


