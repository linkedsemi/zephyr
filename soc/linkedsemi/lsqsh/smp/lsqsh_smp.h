#ifndef _LSQSH_SMP_H_
#define _LSQSH_SMP_H_

#include <zephyr/sys/atomic.h>

#if defined(CONFIG_SMP)
uint32_t get_cur_cpu_id(void);
#else
static inline uint32_t get_cur_cpu_id(void)
{
    return 0;
}
#endif
void lsqsh_ipi_intr_set(uint32_t cpu_id);
void lsqsh_ipi_intr_clr(uint32_t cpu_id);

void poll_wait_xip_unlock(void);

void flash_xip_lock_clear(void);
void flash_xip_lock_sync(void);
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak,void (*ipi_handler)(const void *));
void smp_mode_cache_region_init(void);
void lsqsh_secondary_cpu_init(void);

#define IPI_SCHED	0
#define IPI_FPU_FLUSH	1
#define IPI_XIP_LOCK    2

#endif