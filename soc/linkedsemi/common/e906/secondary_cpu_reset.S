#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/syscall.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/arch/riscv/irq.h>
#include <zephyr/arch/riscv/syscall.h>

#ifdef CONFIG_64BIT
	/* register-wide load/store based on ld/sd (XLEN = 64) */

	.macro lr, rd, mem
	ld \rd, \mem
	.endm

	.macro sr, rs, mem
	sd \rs, \mem
	.endm

#else
	/* register-wide load/store based on lw/sw (XLEN = 32) */

	.macro lr, rd, mem
	lw \rd, \mem
	.endm

	.macro sr, rs, mem
	sw \rs, \mem
	.endm

#endif

/* exports */
GTEXT(__scondary_cpu_reset)

/* imports */
GTEXT(riscv_cpu_wake_flag)
GTEXT(riscv_cpu_sp)
GTEXT(secondary_cpu_init)
GTEXT(arch_secondary_cpu_init)
#if defined(CONFIG_GEN_SW_ISR_TABLE)
GTEXT(_isr_wrapper)
#endif

__scondary_cpu_reset:
#if defined(CONFIG_RISCV_GP)
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
	.option pop
#endif

	.option norvc;

#if defined(CONFIG_RISCV_HAS_CLIC) && !defined(CONFIG_LEGACY_CLIC)

	la t0, _isr_wrapper
	addi t0, t0, 0x03 /* Set mode bits to 3, signifying CLIC. Everything else is reserved. */
	csrw mtvec, t0

#else /* !CONFIG_RISCV_HAS_CLIC || CONFIG_LEGACY_CLIC */

	/*
	 * CLINT direct mode
	 *
	 * Set mtvec (Machine Trap-Vector Base-Address Register)
	 * to _isr_wrapper.
	 */
	la t0, _isr_wrapper
	csrw mtvec, t0

#endif /* CONFIG_RISCV_HAS_CLIC&& !CONFIG_LEGACY_CLIC */

    /*the cpu2 riscv_cpu_wake_flag is the fixed value : 1*/
    la t0, riscv_cpu_wake_flag
    li t1, 1
    sr t1, 0(t0)

    la t0, riscv_cpu_boot_flag
    li t1, 1
    sr t1, 0(t0)

	la t0, riscv_cpu_sp
	lr sp, 0(t0)

	call secondary_cpu_init

    li a0, 1
    j arch_secondary_cpu_init

cpu2_loop_unconfigured_cores:
	wfi
	j cpu2_loop_unconfigured_cores