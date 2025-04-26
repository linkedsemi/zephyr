#include <stdint.h>
#include <string.h>
#include "core_rv32.h"
#include <zephyr/irq.h>
#include "cpu.h"
#include "field_manipulate.h"


/**
 * @brief Enable interrupt
 */
void riscv_clic_irq_enable(uint32_t irq)
{
    enable_irq(irq);
}

/**
 * @brief Disable interrupt
 */
void riscv_clic_irq_disable(uint32_t irq)
{
    disable_irq(irq);
}

/**
 * @brief Get enable status of interrupt
 */
int riscv_clic_irq_is_enabled(uint32_t irq)
{
    return (uint32_t)csi_vic_get_enabled_irq(irq);
}

/**
 * @brief Set priority and level of interrupt
 */
void riscv_clic_irq_priority_set(uint32_t irq, uint32_t pri, uint32_t flags)
{
    csi_vic_set_prio(irq,pri);
}


void riscv_clic_irq_set_pending(uint32_t irq)
{
    MODIFY_REG(CLIC->CLICINT[irq].ATTR,CLIC_INTATTR_TRIG_Msk,1<<CLIC_INTATTR_TRIG_Pos);
	csi_vic_set_pending_irq(irq);
}



