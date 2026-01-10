#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include "platform.h"
#include "reg_sec_pmu_rg.h"

void sys_arch_reboot(int type)
{
    switch(type) {
    case SYS_REBOOT_COLD:
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
        REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_FROM_SFT, 0x1);
#else
        printf("SYS_REBOOT_COLD is not supported\n");
#endif
        break;
    case SYS_REBOOT_WARM:
#if defined(CONFIG_EMUL_SOFT_RESET)
        disable_global_irq();
        reset_reason_magic_set();
        sys_cache_data_flush_all();
        sys_cache_data_disable();
        sys_cache_instr_disable();
        for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
            irq_disable(irq);
        }
        void (* goto_rom_region_start)();
        goto_rom_region_start = (void *)__rom_region_start;
        goto_rom_region_start();
#else
        csi_core_reset();
#endif
        break;
    default:
        printf("type: %d is not supported\n", type);
        csi_core_reset();
        break;
    };
}
