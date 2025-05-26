
#ifndef _SOC_RESET_H_
#define _SOC_RESET_H_

#include <zephyr/kernel.h>

enum reset_reason {
    NO_RESET_REASON,
    COLD_RESET,
    GLOBAL_RESET, /* reset source: 1. write cpu1 system reset reg 2. debugger */
    HART_RESET, /* reset source: write cpu1/cpu2 core reset reg */
    PASSIVE_RESET, /* reset source: 1. wdt 2. debugger */
};

enum reset_reason reset_reason_get(void);
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
void global_reset_reason_clean(void);
#endif
void reset_reason_clean(void);
void reset_reason_set(enum reset_reason reason);
void reset_reason_magic_set();
void reset_reason_flush_cache(void);

#endif /* _SOC_RESET_H_ */
