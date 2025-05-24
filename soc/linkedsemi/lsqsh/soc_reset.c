#include "soc_reset.h"
#include "reg_sec_pmu_rg.h"
#include "field_manipulate.h"

#define MAGIC_VALUE 0xdeadbeef

struct magic_u8 {
    volatile uint32_t magic;
    volatile enum reset_reason u8_val;
};
__noinit struct magic_u8 reset_reason;

enum reset_reason reset_reason_get(void)
{
    enum reset_reason ret = NO_RESET_REASON;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    uint8_t reset_src = REG_FIELD_RD(SEC_PMU->PMU_STATUS, SEC_PMU_RG_RST_SRC);
    if (reset_src) {
        if (MAGIC_VALUE != reset_reason.magic) {
            ret = COLD_RESET;
        } else {
            ret = GLOBAL_RESET;
        }
    } else {
        if (HART_RESET == reset_reason.u8_val) {
            ret = HART_RESET;
        }
        else {
            ret = PASSIVE_RESET;
        }
    }
#else
    if (HART_RESET == reset_reason.u8_val) {
        ret = HART_RESET;
    }
    else {
        ret = PASSIVE_RESET;
    }
#endif

    return ret;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
void global_reset_reason_clean(void)
{
    REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_SRC_CLR, 1);
    REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_SRC_CLR, 0);
}
#endif

void reset_reason_clean(void)
{
    reset_reason.u8_val = NO_RESET_REASON;
}

void reset_reason_set(enum reset_reason reason)
{
    reset_reason.u8_val = reason;
}

void reset_reason_magic_set()
{
    reset_reason.magic = MAGIC_VALUE;
}
