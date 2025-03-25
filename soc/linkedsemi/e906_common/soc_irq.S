
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
	csrs    mstatus, (1 << 3)

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


<<<<<<< HEAD:soc/linkedsemi/ls101x/soc_irq.S
SECTION_FUNC(exception.other, __soc_save_context)
	csrr 	t0, mcause
	sw 		t0, 0(a0)
	ret

SECTION_FUNC(exception.other, __soc_restore_context)
	# csrr 	t0, mexstatus
	# li 		t1, MEXSTATUS_EX_MASK
	# and 	t0, t0, t1
	# bnez  	t0, 2f 

	lw t0, 0(a0)
	li t1, 0x80000000
	or t0, t0, t1
	csrw mcause,t0
2:
=======
	lw      ra, 0(sp)
	addi    sp, sp, REGBYTES
>>>>>>> 2002c5fd3ff... e906 ISR: mcause stacking and unstacking:soc/linkedsemi/e906_common/soc_irq.S
	ret

#endif