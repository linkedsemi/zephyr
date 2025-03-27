
#include <zephyr/kernel_structs.h>
#include <offsets.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <soc.h>


GTEXT(__soc_handle_irq)
SECTION_FUNC(exception.other, __soc_handle_irq) 
	ret

#ifdef CONFIG_RISCV_SOC_HAS_CUSTOM_IRQ_HANDLING

GTEXT(__soc_handle_all_irqs)
SECTION_FUNC(exception.other, __soc_handle_all_irqs)

	addi    sp, sp, -REGBYTES
	sw      ra, 0(sp)

	csrr    a0, mcause
	/* In the interrupt stack, open mie to implement interrupt nesting  */
	// csrs    mstatus, (1 << 3)
	csrrwi  t0, mnxti, 8

	csrrwi  t0, 0x345, 8   //0x345:mnxti

	/* Return */
	ret


/* Exports */
#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
GTEXT(__soc_save_context)
GTEXT(__soc_restore_context)
#endif

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
#define MEXSTATUS_EX_MASK 0x20


	lw      ra, 0(sp)
	addi    sp, sp, REGBYTES
	ret
#endif


.section .text
.global SWINT_Handler_ASM
SWINT_Handler_ASM:
    lw a0, 4(sp)
    j Swint_Handler_C
	ret
