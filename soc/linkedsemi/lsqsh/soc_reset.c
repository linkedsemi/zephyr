#include "soc_reset.h"
#include "reg_sec_pmu_rg.h"
#include "field_manipulate.h"

__nocache volatile uint8_t reset_reason;

enum reset_reason reset_reason_get(void)
{
    enum reset_reason ret = NO_RESET_REASON;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    uint8_t reset_src = REG_FIELD_RD(SEC_PMU->PMU_STATUS, SEC_PMU_RG_RST_SRC);
    if (reset_src) {
        ret = GLOBAL_RESET;
    } else {
        if (HART_RESET == reset_reason) {
            ret = HART_RESET;
        }
        else {
            ret = SEC_WDT_RESET;
        }
    }
#else
    if (HART_RESET == reset_reason) {
        ret = HART_RESET;
    }
    else {
        ret = APP_WDT_RESET;
    }
#endif

    return ret;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
void global_reset_reason_clean(void)
{
    REG_FIELD_WR(SEC_PMU->PMU_STATUS, SEC_PMU_RG_RST_SRC_CLR, 1);
    REG_FIELD_WR(SEC_PMU->PMU_STATUS, SEC_PMU_RG_RST_SRC_CLR, 0);
}
#endif

void reset_reason_clean(void)
{
    reset_reason = NO_RESET_REASON;
}

void reset_reason_set(enum reset_reason reason)
{
    reset_reason = reason;
}
