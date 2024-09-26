
#ifndef ZEPHYR_LSQSH_RISCV_OFFSETS_H_
#define ZEPHYR_LSQSH_RISCV_OFFSETS_H_

#ifdef CONFIG_RISCV_SOC_CONTEXT_SAVE
/*
 * Ensure offset macros are available in <offsets.h>.
 *
 * Also create a macro which contains the value of &EVENT0->INTPTPENDCLEAR,
 * for use in assembly.
 */
#define GEN_SOC_OFFSET_SYMS()					\
	GEN_OFFSET_SYM(soc_esf_t, mcause);
#endif

#endif /* ZEPHYR_LSQSH_RISCV_OFFSETS_H_ */
