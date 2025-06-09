#include <zephyr/kernel.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/fatal.h>
#include "soc_isr_stacking_e906.h"

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

void k_sys_fatal_error_handler(unsigned int reason,
                               const struct arch_esf *esf)
{
    ARG_UNUSED(esf);

    discard_current_irq_nested();
    LOG_PANIC();
    LOG_ERROR("not Halting system");
}
