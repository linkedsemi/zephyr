#include <zephyr/kernel.h>
#include <kernel_arch_interface.h>

#if defined(CONFIG_ARCH_HAS_RAMFUNC_SUPPORT)
#include <zephyr/arch/riscv/pmp.h>

extern char __ramfunc_start[];
extern char __ramfunc_end[];

/*
 * The .ramfunc section is copied to SRAM and executed from there.
 * With CONFIG_PMP_DATA_EXECUTION_PREVENTION the whole address space is
 * execute-protected by default, so the ramfunc region must be whitelisted
 * explicitly.
 */
PMP_SOC_REGION_DEFINE(linkedsemi_ramfunc,
		      __ramfunc_start, __ramfunc_end,
		      PMP_R | PMP_W | PMP_X);
#endif /* CONFIG_ARCH_HAS_RAMFUNC_SUPPORT */

__ramfunc int busy_poll(bool (*poll_fn)(void *),void *param,uint32_t usec_to_wait)
{
	uint64_t start_cycles = k_cycle_get_64();
	uint64_t cycles_to_wait = k_us_to_cyc_ceil64(usec_to_wait);
	int ret;
	for (;;) {
		uint64_t current_cycles = k_cycle_get_64();

		/* this handles the rollover on an unsigned 64-bit value */
		if ((current_cycles - start_cycles) >= cycles_to_wait) {
			ret = -ETIME;
			break;
		}
		if(poll_fn(param))
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

#if defined(CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT)
void arch_busy_wait(uint32_t usec_to_wait)
{
    uint64_t start_cycles = k_cycle_get_64();
    uint64_t cycles_to_wait = k_us_to_cyc_ceil64(usec_to_wait);

    for (;;) {
        uint64_t current_cycles = k_cycle_get_64();

        if ((current_cycles - start_cycles) >= cycles_to_wait) {
            break;
        }
    }
}
#endif
