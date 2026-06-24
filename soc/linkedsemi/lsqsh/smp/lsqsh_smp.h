#ifndef _LSQSH_SMP_H_
#define _LSQSH_SMP_H_

#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
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

void poll_wait_xip_unlock(bool is_write);
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak);
void smp_mode_cache_region_init(void);
void lsqsh_secondary_cpu_init(void);

void smp_mode_cache_config(void);

k_spinlock_key_t e906_smp_spin_lock(struct k_spinlock *l);

void e906_smp_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key);

void lsqsh_xip_lock_broadcast_ipi(bool is_write);

void flash_critical_exit_sync();

void flash_critical_enter_sync();

#define IPI_SCHED	0
#define IPI_FPU_FLUSH	1
#define IPI_XIP_LOCK_WRITE    2
#define IPI_XIP_LOCK_READ    3
#endif