#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>
#include <cpu.h>
#include "soc_isr_stacking_e906.h"

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf)
{
    ARG_UNUSED(esf);

    LOG_PANIC();
    if (irq_nested_level > 1) {
        LOG_ERR("fatal error! Halting system");
        disable_global_irq();
        for (;;) {;}
        CODE_UNREACHABLE;
    } else {
        // irq_nested_level--;
        LOG_ERR("not Halting system");
    }
}
