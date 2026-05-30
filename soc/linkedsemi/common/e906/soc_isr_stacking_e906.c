#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
#include "field_manipulate.h"

uint32_t irq_nested_level = 0;
static uint32_t irq_nested_mcause[IRQ_NESTED_MAX] = {0,0,0,0,0,0,0,0,0,0};


static const struct device *const zephyr_flash_controller =
    DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

extern int flash_ls_ex_op(const struct device *dev, uint16_t code, const uintptr_t in, void *out);

__ramfunc void isr_stacking_mcause(void)
{
    flash_ls_ex_op(zephyr_flash_controller,FLASH_DRIVER_SUSPEND_OPCODE,0,NULL);
    if(irq_nested_level < IRQ_NESTED_MAX)
    {
        irq_nested_mcause[irq_nested_level] = csr_read(mcause);
        irq_nested_level++;
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
    if(irq_nested_level > 0 && irq_nested_level <= IRQ_NESTED_MAX)
    {
        irq_nested_level--;
        restore_mcause = irq_nested_mcause[irq_nested_level];
        current_mcause = csr_read(mcause);

        current_mcause &=  (MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK);
        MODIFY_REG(restore_mcause,(MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK),current_mcause);

        csr_write(mcause,restore_mcause);
    }
    else
    {
        while(1);
    }
    flash_ls_ex_op(zephyr_flash_controller,FLASH_DRIVER_RESUME_OPCODE,0,NULL);
}

void discard_current_irq_nested(void)
{
    irq_nested_level--;
}

void Swint_Handler_C(struct arch_esf *args)
{
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args->a4;
    args->a0 = func(args->a0, args->a1, args->a2, args->a3);
}
