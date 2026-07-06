#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
#include "smp/lsqsh_smp.h"
#include <platform.h>

uint32_t irq_nested_level[CONFIG_MP_MAX_NUM_CPUS];
static uint32_t irq_nested_mcause[CONFIG_MP_MAX_NUM_CPUS][IRQ_NESTED_MAX];
static struct device zephyr_flash_controller_ram_struct;
const struct device *const zephyr_flash_controller = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

int zephyr_flash_controller_ram_struct_init()
{
    zephyr_flash_controller_ram_struct = *zephyr_flash_controller;
    return 0;
}
SYS_INIT(zephyr_flash_controller_ram_struct_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

extern int flash_ls_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out);

__ramfunc void isr_stacking_mcause(void)
{
	uint32_t _cpu_id = get_cur_cpu_id();
    flash_ls_ex_op(&zephyr_flash_controller_ram_struct,FLASH_DRIVER_SUSPEND_OPCODE,_cpu_id,NULL);
    if(irq_nested_level[_cpu_id] < IRQ_NESTED_MAX)
    {
        irq_nested_mcause[_cpu_id][irq_nested_level[_cpu_id]] = csr_read(mcause);
        irq_nested_level[_cpu_id]++;
    }
    else
    {
        while(1);
    }
}

#define MCAUSE_MPP_MASK (3UL << 28)
#define MCAUSE_MPIE_MASK (1UL << 27)

__ramfunc void isr_unstacking_mcause(void)
{
    uint32_t current_mcause;
    uint32_t restore_mcause;
	/* get current cpu number */
	uint32_t _cpu_id = get_cur_cpu_id();
    if(irq_nested_level[_cpu_id] > 0 && irq_nested_level[_cpu_id] <= IRQ_NESTED_MAX)
    {
        irq_nested_level[_cpu_id]--;
        restore_mcause = irq_nested_mcause[_cpu_id][irq_nested_level[_cpu_id]];
        current_mcause = csr_read(mcause);

        current_mcause &=  (MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK);
        MODIFY_REG(restore_mcause,(MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK),current_mcause);

        csr_write(mcause,restore_mcause);
    }
    else
    {
        while(1);
    }
    flash_ls_ex_op(&zephyr_flash_controller_ram_struct,FLASH_DRIVER_RESUME_OPCODE,_cpu_id,NULL);
}

void Swint_Handler_C(struct arch_esf *args)
{
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args->a4;
    args->a0 = func(args->a0, args->a1, args->a2, args->a3);
}

#if defined(CONFIG_RISCV_SOC_HAS_CUSTOM_IRQ_HANDLING)
#ifdef CONFIG_TRACING_ISR
#include <zephyr/tracing/tracing.h>
#include <ctf_top.h>
static inline uint32_t mnxti_get_no_set_mie(void)
{
    uint32_t mnxti;

    __asm__ volatile (
        "csrrs %0, mnxti, x0"
        : "=r"(mnxti)
        :
        : "memory"
    );

    return mnxti;
}

__attribute__((optimize("-O2")))
void __soc_handle_all_irqs(void)
{
	while (1) {
		uint32_t mnxti = mnxti_get_no_set_mie();
		uint32_t irq_num = mnxti >> 2;
		struct _isr_table_entry *entry;

		if (0 == mnxti) {
			break;
		}
		entry = &_sw_isr_table[irq_num];
		ctf_top_isr_enter_id(get_cur_cpu_id(), irq_num);
		csr_set(mstatus, MSTATUS_MIE);
		(entry->isr)(entry->arg);
		__disable_irq();
		ctf_top_isr_exit_id(get_cur_cpu_id());
	}

	__disable_irq();
}
#else
static inline uint32_t mnxti_get_and_set_mie(void)
{
    uint32_t mnxti;

    __asm__ volatile (
        "csrrsi %0, mnxti, 8"
        : "=r"(mnxti)
        :
        : "memory"
    );

    return mnxti;
}

__attribute__((optimize("-O2")))
void __soc_handle_all_irqs(void)
{
	while (1) {
		uint32_t mnxti = mnxti_get_and_set_mie();
		uint32_t irq_num = mnxti >> 2;
		struct _isr_table_entry *entry;

		if (0 == mnxti) {
			break;
		}
		entry = &_sw_isr_table[irq_num];
		(entry->isr)(entry->arg);
		__disable_irq();
	}

	__disable_irq();
}
#endif
#endif
