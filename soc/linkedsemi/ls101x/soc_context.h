#ifndef ZEPHYR_LS101X_RISCV_EXCEPTION_H_
#define ZEPHYR_LS101X_RISCV_EXCEPTION_H_

#include <zephyr/arch/arch_inlines.h>

#define INITIAL_MSTATUS    (MSTATUS_MIE | MSTATUS_MPP | MSTATUS_MPIE | MSTATUS_FS_INIT)
#define INITIAL_MCAUSE     0x30000000U
#define SOC_ESF_MEMBERS\
	unsigned long mcause;	/* machine status register */


#define SOC_ESF_INIT\
	INITIAL_MCAUSE	/* Initializes the value of mcause */



#endif //ZEPHYR_LS101X_RISCV_EXCEPTION_H_