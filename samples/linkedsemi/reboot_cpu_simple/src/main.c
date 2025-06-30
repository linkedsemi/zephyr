#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <stdio.h>
#include <cpu.h>
#include <zephyr/linker/linker-defs.h>

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    k_msleep(500);

    disable_global_irq();
    sys_cache_data_flush_all();
    sys_cache_data_disable();
    sys_cache_instr_disable();
    for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
        irq_disable(irq);
    }
    void (* goto_rom_region_start)();
    goto_rom_region_start = (void *)__rom_region_start;
    goto_rom_region_start();

    return 0;
}
