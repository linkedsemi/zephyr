/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "kernel_shell.h"

#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef CONFIG_THREAD_RUNTIME_STATS
static void rt_stats_dump(const struct shell *sh, struct k_thread *thread)
{
	k_thread_runtime_stats_t rt_stats_thread;
	k_thread_runtime_stats_t rt_stats_all;
	int ret = 0;
	unsigned int pcnt;

	if (k_thread_runtime_stats_get(thread, &rt_stats_thread) != 0) {
		ret++;
	}

	if (k_thread_runtime_stats_all_get(&rt_stats_all) != 0) {
		ret++;
	}

	if (ret == 0) {
		pcnt = (rt_stats_thread.execution_cycles * 100U) /
		       rt_stats_all.execution_cycles;

		/*
		 * z_prf() does not support %llu by default unless
		 * CONFIG_MINIMAL_LIBC_LL_PRINTF=y. So do conditional
		 * compilation to avoid blindly enabling this kconfig
		 * so it won't increase RAM/ROM usage too much on 32-bit
		 * targets.
		 */
		shell_print(sh, "\tTotal execution cycles: %u (%u %%)",
			    (uint32_t)rt_stats_thread.execution_cycles,
			    pcnt);
#ifdef CONFIG_SCHED_THREAD_USAGE_ANALYSIS
		shell_print(sh, "\tCurrent execution cycles: %u",
			    (uint32_t)rt_stats_thread.current_cycles);
		shell_print(sh, "\tPeak execution cycles: %u",
			    (uint32_t)rt_stats_thread.peak_cycles);
		shell_print(sh, "\tAverage execution cycles: %u",
			    (uint32_t)rt_stats_thread.average_cycles);
#endif /* CONFIG_SCHED_THREAD_USAGE_ANALYSIS */
	} else {
		shell_print(sh, "\tTotal execution cycles: ? (? %%)");
#ifdef CONFIG_SCHED_THREAD_USAGE_ANALYSIS
		shell_print(sh, "\tCurrent execution cycles: ?");
		shell_print(sh, "\tPeak execution cycles: ?");
		shell_print(sh, "\tAverage execution cycles: ?");
#endif /* CONFIG_SCHED_THREAD_USAGE_ANALYSIS */
	}
}
#endif /* CONFIG_THREAD_RUNTIME_STATS */

static void shell_tdata_dump(const struct k_thread *cthread, void *user_data)
{
	struct k_thread *thread = (struct k_thread *)cthread;
	const struct shell *sh = (const struct shell *)user_data;
	unsigned int pcnt;
	size_t unused;
	size_t size = thread->stack_info.size;
	const char *tname;
	int ret;
	char state_str[32];

	tname = k_thread_name_get(thread);

	shell_print(sh, "%s%p %-10s",
		      (thread == k_current_get()) ? "*" : " ",
		      thread,
		      tname ? tname : "NA");
	/* Cannot use lld as it's less portable. */
	shell_print(sh, "\toptions: 0x%x, priority: %d timeout: %" PRId64,
		      thread->base.user_options,
		      thread->base.prio,
		      (int64_t)thread->base.timeout.dticks);
	shell_print(sh, "\tstate: %s, entry: %p",
		    k_thread_state_str(thread, state_str, sizeof(state_str)),
		    thread->entry.pEntry);

#ifdef CONFIG_SCHED_CPU_MASK
	shell_print(sh, "\tcpu_mask: 0x%x", thread->base.cpu_mask);
#endif /* CONFIG_SCHED_CPU_MASK */

	IF_ENABLED(CONFIG_THREAD_RUNTIME_STATS, (rt_stats_dump(sh, thread)));

	ret = k_thread_stack_space_get(thread, &unused);
	if (ret) {
		shell_print(sh,
			    "Unable to determine unused stack size (%d)\n",
			    ret);
	} else {
		/* Calculate the real size reserved for the stack */
		pcnt = ((size - unused) * 100U) / size;

		shell_print(sh,
			    "\tstack size %zu, unused %zu, usage %zu / %zu (%u %%)\n",
			    size, unused, size - unused, size, pcnt);
	}

}

static int cmd_kernel_thread_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Scheduler: %u since last call", sys_clock_elapsed());
	shell_print(sh, "Threads:");

	/*
	 * Use the unlocked version as the callback itself might call
	 * arch_irq_unlock.
	 */
	k_thread_foreach_unlocked(shell_tdata_dump, (void *)sh);

	return 0;
}

KERNEL_THREAD_CMD_ADD(list, NULL, "List kernel threads.", cmd_kernel_thread_list);

#ifdef CONFIG_THREAD_RUNTIME_STATS
struct kernel_thread_list_result {
	uint32_t count;
	struct {
		char *name;
		uint64_t total;
	} log[CONFIG_KERNEL_THREAD_MAX];
};

struct kernel_thread_list_result kernel_thread_list_result_start;
struct kernel_thread_list_result kernel_thread_list_result_end;
struct kernel_thread_list_result kernel_thread_list_result_delta;
static uint64_t delta_total;
static K_SEM_DEFINE(kernel_thread_list_sem, 1, 1);

static void rt_stats_dump_cycle(struct kernel_thread_list_result *kernel_thread_list_result_p, struct k_thread *thread)
{
	k_thread_runtime_stats_t rt_stats_thread;
	int ret = 0;

	if (k_thread_runtime_stats_get(thread, &rt_stats_thread) != 0) {
		ret++;
	}

	if (ret == 0) {
		kernel_thread_list_result_p->log[kernel_thread_list_result_p->count].total = rt_stats_thread.execution_cycles;
	} else {
		printf("NA\n");
	}
}

static void shell_tdata_dump_cycle(const struct k_thread *cthread, void *user_data)
{
	struct k_thread *thread = (struct k_thread *)cthread;
	struct kernel_thread_list_result *kernel_thread_list_result_p = (struct kernel_thread_list_result *)user_data;
	const char *tname;

	tname = k_thread_name_get(thread);
	kernel_thread_list_result_p->log[kernel_thread_list_result_p->count].name = (char *)tname;

	IF_ENABLED(CONFIG_THREAD_RUNTIME_STATS, (rt_stats_dump_cycle(kernel_thread_list_result_p, thread)));
	if ((kernel_thread_list_result_p->count + 1) >= CONFIG_KERNEL_THREAD_MAX) {
		printf("Exceed max thread number %d\n", CONFIG_KERNEL_THREAD_MAX);
		return;
	}
	kernel_thread_list_result_p->count++;
}

int kernel_thread_list_start(void)
{
	int ret = k_sem_take(&kernel_thread_list_sem, K_NO_WAIT);
	if (ret != 0) {
		return ret;
	}
	memset(&kernel_thread_list_result_start, 0, sizeof(kernel_thread_list_result_start));
	memset(&kernel_thread_list_result_end, 0, sizeof(kernel_thread_list_result_end));
	memset(&kernel_thread_list_result_delta, 0, sizeof(kernel_thread_list_result_delta));
	delta_total = 0;
	/*
	 * Use the unlocked version as the callback itself might call
	 * arch_irq_unlock.
	 */
	k_thread_foreach_unlocked(shell_tdata_dump_cycle, (void *)&kernel_thread_list_result_start);

	return 0;
}

static int kernel_thread_list_show(void)
{
	printk("Scheduler: %u since last call\n"
		"Threads:\n"
		"count: %u\n"
		"delta_total: %llu\n",
		sys_clock_elapsed(),
		kernel_thread_list_result_start.count,
		delta_total);
	for (int i = 0; i < kernel_thread_list_result_start.count; i++) {
		unsigned int pcnt = (kernel_thread_list_result_delta.log[i].total * 100ULL) / delta_total;
		unsigned int pdec = ((kernel_thread_list_result_delta.log[i].total * 10000ULL) / delta_total) % 100;
		printk("[%2d] %-32s | %10llu | %2u.%02u %%\n",
			    i,
			    kernel_thread_list_result_start.log[i].name ? kernel_thread_list_result_start.log[i].name : "NA",
			    kernel_thread_list_result_delta.log[i].total,
			    pcnt, pdec);
	}
	k_sem_give(&kernel_thread_list_sem);

	return 0;
}

static void kernel_thread_list_show_work(struct k_work *work)
{
	ARG_UNUSED(work);

	kernel_thread_list_show();
}

K_WORK_DEFINE(kernel_thread_list_work, kernel_thread_list_show_work);
int kernel_thread_list_stop(void)
{
	/*
	 * Use the unlocked version as the callback itself might call
	 * arch_irq_unlock.
	 */
	k_thread_foreach_unlocked(shell_tdata_dump_cycle, (void *)&kernel_thread_list_result_end);
	for (int i = 0; i < kernel_thread_list_result_start.count; i++) {
		/* kernel_thread_list_result_delta.log[i].name = kernel_thread_list_result_start.log[i].name; */
		kernel_thread_list_result_delta.log[i].total = kernel_thread_list_result_end.log[i].total - kernel_thread_list_result_start.log[i].total;
		delta_total += kernel_thread_list_result_delta.log[i].total;
	}

	k_work_submit(&kernel_thread_list_work);

	return 0;
}

static void kernel_thread_list_timeout_handler(struct k_timer *timer)
{
	kernel_thread_list_stop();
}

static K_TIMER_DEFINE(timer, kernel_thread_list_timeout_handler, NULL);

int kernel_thread_list_ms(uint32_t time_ms)
{
	int ret = kernel_thread_list_start();
	if (ret != 0) {
		printf("Another kernel thread list operation is in progress\n");
		return ret;
	}
	k_timer_start(&timer, K_MSEC(time_ms), K_NO_WAIT);

	return 0;
}

int kernel_thread_list_s(uint32_t time_s)
{
	int ret = kernel_thread_list_start();
	if (ret != 0) {
		printf("Another kernel thread list operation is in progress\n");
		return ret;
	}
	k_timer_start(&timer, K_SECONDS(time_s), K_NO_WAIT);

	return 0;
}

static int cmd_kernel_thread_list_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "%s", __func__);

	kernel_thread_list_start();

	return 0;
}
KERNEL_THREAD_CMD_ADD(list_start, NULL, "List kernel threads.", cmd_kernel_thread_list_start);

static int cmd_kernel_thread_list_stop(const struct shell *sh, size_t argc, char **argv)
{
	kernel_thread_list_stop();

	return 0;
}
KERNEL_THREAD_CMD_ADD(list_stop, NULL, "List kernel threads.", cmd_kernel_thread_list_stop);

static int cmd_kernel_thread_list_ms(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t time_ms = strtoul(argv[1], NULL, 10);

	kernel_thread_list_ms(time_ms);

	return 0;
}
KERNEL_THREAD_CMD_ARG_ADD(list_ms, NULL, "List kernel threads.", cmd_kernel_thread_list_ms, 2, 0);

static int cmd_kernel_thread_list_s(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t time_s = strtoul(argv[1], NULL, 10);

	kernel_thread_list_s(time_s);

	return 0;
}
KERNEL_THREAD_CMD_ARG_ADD(list_s, NULL, "List kernel threads.", cmd_kernel_thread_list_s, 2, 0);
#endif /* CONFIG_THREAD_RUNTIME_STATS */
