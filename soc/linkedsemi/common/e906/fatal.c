#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>
#include "soc_isr_stacking_e906.h"

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

FUNC_NORETURN void arch_system_halt(unsigned int reason)
{
    ARG_UNUSED(reason);

    /* TODO: What's the best way to totally halt the system if SMP
     * is enabled?
     */

    (void)arch_irq_lock();
    for (;;) {
        /* Spin endlessly */
    }
}

void k_sys_fatal_error_handler(unsigned int reason,
                      const struct arch_esf *esf)
{
    ARG_UNUSED(esf);

    LOG_PANIC();
    LOG_ERR("Halting thread");
    if (IS_ENABLED(CONFIG_MULTITHREADING)) {
        discard_current_irq_nested();
        k_thread_abort(_current);
    } else {
        arch_system_halt(reason);
    }
    CODE_UNREACHABLE;
}
