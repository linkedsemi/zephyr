#ifndef _LSQSH_SMP_H_
#define _LSQSH_SMP_H_

#include <zephyr/sys/atomic.h>

uint32_t get_cur_cpu_id(void);
void lsqsh_ipi_intr_set(uint32_t cpu_id);
void lsqsh_ipi_intr_clr(uint32_t cpu_id);

int xip_lock(void);
int xip_lock_relesae(void);
void poll_wait_xip_unlock(void);
bool get_xip_lock_owner(void);


void secondary_cpu_init(void);
void lsqsh_xip_lcok_broadcast_ipi(void);
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak);

void lsqsh_secondary_cpu_init(void);

#define IPI_SCHED	0
#define IPI_FPU_FLUSH	1
#define IPI_XIP_LOCK    2

#endif