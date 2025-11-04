#include <zephyr/kernel.h>
#include <kernel_arch_interface.h>

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
