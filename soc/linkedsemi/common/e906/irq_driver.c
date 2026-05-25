#include <stdint.h>
#include <string.h>
#include <zephyr/irq.h>
#include <soc.h>
#include <platform.h>


/**
 * @brief Enable interrupt
 */
void riscv_clic_irq_enable(uint32_t irq)
{
    csi_vic_enable_irq(irq);
}

/**
 * @brief Disable interrupt
 */
void riscv_clic_irq_disable(uint32_t irq)
{
    csi_vic_disable_irq(irq);
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
    switch (flags) {
    case IRQ_TYPE_LEVEL_HIGH:
        CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_POSITIVE_LEVEL << CLIC_INTATTR_TRIG_Pos;
        break;
    case IRQ_TYPE_EDGE_RISING:
        CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_POSITIVE_EDGE << CLIC_INTATTR_TRIG_Pos;
        break;
    case IRQ_TYPE_EDGE_FALLING:
        CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_NEGATIVE_EDGE << CLIC_INTATTR_TRIG_Pos;
        break;
    default:
        break;
    };
}


void riscv_clic_irq_set_pending(uint32_t irq)
{
    MODIFY_REG(CLIC->CLICINT[irq].ATTR,CLIC_INTATTR_TRIG_Msk,1<<CLIC_INTATTR_TRIG_Pos);
	csi_vic_set_pending_irq(irq);
}

void riscv_clic_irq_disable_trigger_mode(uint32_t irq)
{
    MODIFY_REG(CLIC->CLICINT[irq].ATTR,CLIC_INTATTR_TRIG_Msk,0<<CLIC_INTATTR_TRIG_Pos);
}
