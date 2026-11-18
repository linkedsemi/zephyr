
#ifndef __SOC_ISR_STACKING__
#define __SOC_ISR_STACKING__
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/riscv/csr.h>

#ifndef _ASMLANGUAGE
#ifdef CONFIG_RISCV_SOC_HAS_ISR_STACKING
#include <zephyr/types.h>
#include <zephyr/toolchain.h>
#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
#include <soc_context.h>
#endif

struct arch_esf {
	unsigned long ra;		/* return address */

	unsigned long t0;		/* Caller-saved temporary register */
	unsigned long t1;		/* Caller-saved temporary register */
	unsigned long t2;		/* Caller-saved temporary register */
#if !defined(CONFIG_RISCV_ISA_RV32E)
	unsigned long t3;		/* Caller-saved temporary register */
	unsigned long t4;		/* Caller-saved temporary register */
	unsigned long t5;		/* Caller-saved temporary register */
	unsigned long t6;		/* Caller-saved temporary register */
#endif /* !CONFIG_RISCV_ISA_RV32E */

	unsigned long a0;		/* function argument/return value */
	unsigned long a1;		/* function argument */
	unsigned long a2;		/* function argument */
	unsigned long a3;		/* function argument */
	unsigned long a4;		/* function argument */
	unsigned long a5;		/* function argument */
#if !defined(CONFIG_RISCV_ISA_RV32E)
	unsigned long a6;		/* function argument */
	unsigned long a7;		/* function argument */
#endif /* !CONFIG_RISCV_ISA_RV32E */

#ifdef CONFIG_CLIC_SUPPORT_INTERRUPT_LEVEL
	unsigned long mcause;		/* machine cause register */
#endif /* CONFIG_CLIC_SUPPORT_INTERRUPT_LEVEL */

	unsigned long mepc;		/* machine exception program counter */
	unsigned long mstatus;	/* machine status register */

	unsigned long s0;		/* callee-saved s0 */

#ifdef CONFIG_USERSPACE
	unsigned long sp;		/* preserved (user or kernel) stack pointer */
#endif

#ifdef CONFIG_EXCEPTION_DEBUG
	struct _callee_saved *csf;	/* pointer to callee-saved-registers */
#endif /* CONFIG_EXCEPTION_DEBUG */

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	struct soc_esf soc_context;
#endif
} __aligned(16);

#define SOC_ISR_STACKING_ESF_DECLARE
#define SOC_ISR_STACKING_ESR_INIT
#endif /* RISCV_SOC_HAS_ISR_STACKING*/

#else /* _ASMLANGUAGE */

#ifndef DO_CALLER_SAVED
/* Convenience macro for loading/storing register states. */
#define DO_CALLER_SAVED(op) \
	RV_E(	op t0, __struct_arch_esf_t0_OFFSET(sp)	);\
	RV_E(	op t1, __struct_arch_esf_t1_OFFSET(sp)	);\
	RV_E(	op t2, __struct_arch_esf_t2_OFFSET(sp)	);\
	RV_I(	op t3, __struct_arch_esf_t3_OFFSET(sp)	);\
	RV_I(	op t4, __struct_arch_esf_t4_OFFSET(sp)	);\
	RV_I(	op t5, __struct_arch_esf_t5_OFFSET(sp)	);\
	RV_I(	op t6, __struct_arch_esf_t6_OFFSET(sp)	);\
	RV_E(	op a0, __struct_arch_esf_a0_OFFSET(sp)	);\
	RV_E(	op a1, __struct_arch_esf_a1_OFFSET(sp)	);\
	RV_E(	op a2, __struct_arch_esf_a2_OFFSET(sp)	);\
	RV_E(	op a3, __struct_arch_esf_a3_OFFSET(sp)	);\
	RV_E(	op a4, __struct_arch_esf_a4_OFFSET(sp)	);\
	RV_E(	op a5, __struct_arch_esf_a5_OFFSET(sp)	);\
	RV_I(	op a6, __struct_arch_esf_a6_OFFSET(sp)	);\
	RV_I(	op a7, __struct_arch_esf_a7_OFFSET(sp)	);\
	RV_E(	op ra, __struct_arch_esf_ra_OFFSET(sp)	)
#endif /*DO_CALLER_SAVED*/

GDATA(irq_nested_level)
GDATA(irq_nested_mcause)
GTEXT(isr_stacking_mcause)
GTEXT(isr_unstacking_mcause)

#define SOC_ISR_SW_STACKING \
	addi sp, sp, -__struct_arch_esf_SIZEOF;\
	DO_CALLER_SAVED(sr)		;\
	call isr_stacking_mcause;

#ifdef CONFIG_USERSPACE
#define SOC_ISR_SW_UNSTACKING\
	call isr_unstacking_mcause;\
	DO_CALLER_SAVED(lr);\
	lr sp, __struct_arch_esf_sp_OFFSET(sp);
#else

#define SOC_ISR_SW_UNSTACKING\
	call isr_unstacking_mcause;\
	DO_CALLER_SAVED(lr);\
	addi sp, sp, __struct_arch_esf_SIZEOF;
#endif /*CONFIG_USERSPACE*/

#endif


#endif /**/
