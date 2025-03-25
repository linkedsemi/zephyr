#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
#include "field_manipulate.h"

static uint32_t irq_nested_level = 0;
static uint32_t irq_nested_mcause[IRQ_NESTED_MAX] = {0,0,0,0,0,0,0,0,0,0};


void isr_stacking_mcause(void)
{
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

#define MCAUSE_MPP_MASK (3UL << 27)
#define MCAUSE_MPIE_MASK (1UL << 26)

void isr_unstacking_mcause(void)
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
}