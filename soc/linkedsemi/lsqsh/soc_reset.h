
#ifndef _SOC_RESET_H_
#define _SOC_RESET_H_

#include <zephyr/kernel.h>

enum reset_reason {
    NO_RESET_REASON,
    GLOBAL_RESET,
    HART_RESET,
    SEC_WDT_RESET,
    APP_WDT_RESET,
};

enum reset_reason reset_reason_get(void);
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
void global_reset_reason_clean(void);
#endif
void reset_reason_clean(void);
void reset_reason_set(enum reset_reason reason);

#endif /* _SOC_RESET_H_ */
