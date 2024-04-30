
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
// #include "exp.h"
// #define RV_E(op...) op
// #define RV_I(op...) /* unavailable */

#ifndef __ASSEMBLER__ 
struct __esf {
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

	unsigned long mepc;		/* machine exception program counter */
	unsigned long mstatus;	/* machine status register */
	unsigned long mcause;	/* machine status register */

	unsigned long s0;		/* callee-saved s0 */

#ifdef CONFIG_USERSPACE
	unsigned long sp;		/* preserved (user or kernel) stack pointer */
#endif

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
	struct soc_esf soc_context;
#endif
} __aligned(16);

#define SOC_ISR_STACKING_ESF_DECLARE 

#else
#define MCAUSE_MPIL_OFFSET_MASK 0xff
#define MCAUSE_MPIL_OFFSET 0x16
#define MEXSTATUS_EX_MASK 0x20
#define MINTSTATUS_INT_MAKE 0xff000000


#define CALLER_SAVED(op) \
	RV_E(	op t0, __z_arch_esf_t_t0_OFFSET(sp)	);\
	RV_E(	op t1, __z_arch_esf_t_t1_OFFSET(sp)	);\
	RV_E(	op t2, __z_arch_esf_t_t2_OFFSET(sp)	);\
	RV_I(	op t3, __z_arch_esf_t_t3_OFFSET(sp)	);\
	RV_I(	op t4, __z_arch_esf_t_t4_OFFSET(sp)	);\
	RV_I(	op t5, __z_arch_esf_t_t5_OFFSET(sp)	);\
	RV_I(	op t6, __z_arch_esf_t_t6_OFFSET(sp)	);\
	RV_E(	op a0, __z_arch_esf_t_a0_OFFSET(sp)	);\
	RV_E(	op a1, __z_arch_esf_t_a1_OFFSET(sp)	);\
	RV_E(	op a2, __z_arch_esf_t_a2_OFFSET(sp)	);\
	RV_E(	op a3, __z_arch_esf_t_a3_OFFSET(sp)	);\
	RV_E(	op a4, __z_arch_esf_t_a4_OFFSET(sp)	);\
	RV_E(	op a5, __z_arch_esf_t_a5_OFFSET(sp)	);\
	RV_I(	op a6, __z_arch_esf_t_a6_OFFSET(sp)	);\
	RV_I(	op a7, __z_arch_esf_t_a7_OFFSET(sp)	);\
	RV_E(	op ra, __z_arch_esf_t_ra_OFFSET(sp)	)


GTEXT(clr_irq_flag)
.global noint

#define SOC_ISR_SW_STACKING \
    addi sp, sp, -__z_arch_esf_t_SIZEOF;  \
    CALLER_SAVED(sr);\
    csrr t0, mcause;\
    sr t0, 0x48(sp);\
	li t1, SOC_MCAUSE_IRQ_MASK;\
	and t0, t0, t1;\
	beqz t0, is_exca;\
	csrr t0,mcause;\
	csrw mscratch, t0;\
is_exca:

#define SOC_ISR_SW_UNSTACKING\
	csrr t0, mexstatus;\
	li t1, MEXSTATUS_EX_MASK;\
	and t0, t0, t1;\
	bnez  t0, noint;\
	csrr t0, mcause;\
	li t1, SOC_MCAUSE_IRQ_MASK;\
	and t0, t0, t1;\
	beqz t0, set_mcause;\
	call noint;\
set_mcause:\
	csrr t0,mscratch;\
	csrw mcause,t0;\
	RESET_MPIE;\
noint:\
	nop;\
	lw	t0,64(sp);\
	lw	t2,68(sp);\
	csrw	mepc,t0;\
	csrw	mstatus,t2;\
    CALLER_SAVED(lr);\
	addi sp, sp, __z_arch_esf_t_SIZEOF;

#define RESET_MPIE\
	csrr t0, mstatus;\
	li t1, 0x80;\
	or t0, t0, t1;\
	csrw mstatus, t0;
#endif