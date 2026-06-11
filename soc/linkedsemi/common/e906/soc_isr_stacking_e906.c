#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
#include "field_manipulate.h"
#include "platform.h"
#include "ls_soc_gpio.h"
#include "smp/lsqsh_smp.h"
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

uint32_t irq_nested_level[CONFIG_MP_MAX_NUM_CPUS] = {0,0};
static uint32_t irq_nested_mcause[CONFIG_MP_MAX_NUM_CPUS][IRQ_NESTED_MAX] = {{0},{0}};
static const struct device *const zephyr_flash_controller =
    DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

extern int flash_ls_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out);

__ramfunc void isr_stacking_mcause(void)
{
	uint32_t _cpu_id = get_cur_cpu_id();
    flash_ls_ex_op(zephyr_flash_controller,FLASH_DRIVER_SUSPEND_OPCODE,_cpu_id,NULL);
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
    flash_ls_ex_op(zephyr_flash_controller,FLASH_DRIVER_RESUME_OPCODE,_cpu_id,NULL);
}

void Swint_Handler_C(struct arch_esf *args)
{
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args->a4;
    args->a0 = func(args->a0, args->a1, args->a2, args->a3);
}
