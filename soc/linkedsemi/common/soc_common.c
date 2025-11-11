#include <zephyr/kernel.h>
#include <kernel_arch_interface.h>

__ramfunc int busy_poll(bool (*poll_fn)(void *),void *param,uint32_t usec_to_wait)
{
	uint32_t start_cycles = k_cycle_get_32();
	uint32_t cycles_to_wait = k_us_to_cyc_ceil32(usec_to_wait);
	int ret;
	for (;;) {
		uint32_t current_cycles = k_cycle_get_32();

		/* this handles the rollover on an unsigned 32-bit value */
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
