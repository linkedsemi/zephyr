
#include <zephyr/kernel_structs.h>
#include <offsets.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <soc.h>

#define REGBYTES                4

GTEXT(__soc_handle_irq)
SECTION_FUNC(exception.other, __soc_handle_irq) 
	ret

#ifdef CONFIG_RISCV_SOC_HAS_CUSTOM_IRQ_HANDLING

GTEXT(__soc_handle_all_irqs)
SECTION_FUNC(exception.other, __soc_handle_all_irqs)

	addi    sp, sp, -REGBYTES
	sw      ra, 0(sp)

	addi 	sp, sp, -REGBYTES
	csrr    a0, mcause
	sw 		a0, 0(sp)

	/* In the interrupt stack, open mie to implement interrupt nesting  */
	csrrwi  t0, mnxti, 8

	li 		t0, CONFIG_RISCV_MCAUSE_EXCEPTION_MASK
	and 	a0, a0, t0

	/*
	 * Call corresponding registered function in _sw_isr_table.
	 * (table is 2-word wide, we should shift index accordingly)
	 */
	la 		t0, _sw_isr_table
	slli 	a0, a0, (RV_REGSHIFT + 1)
	add 	t0, t0, a0

	/* Load argument in a0 register */
	lw 		a0, 0(t0)

	/* Load ISR function address in register t1 */
	lw 		t1, RV_REGSIZE(t0)

	/* Call ISR function */
	jalr 	ra, t1, 0

	/* Disable global irq */
	csrc    mstatus, (1 << 3) 

	lw 		t0, 0(sp)
	csrw 	mcause,t0
	addi 	sp, sp, REGBYTES

	lw      ra, 0(sp)
	addi    sp, sp, REGBYTES
	ret
#endif

.section .text
.global SWINT_Handler_Asm
SWINT_Handler_Asm:
    lw a0, 0(sp)
    j Swint_Handler_C
	ret