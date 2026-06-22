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

void poll_wait_xip_unlock(void);
void flash_xip_lock_clear(void);
void flash_xip_lock_sync(void);
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak);
void smp_mode_cache_region_init(void);
void lsqsh_secondary_cpu_init(void);

void smp_mode_cache_config(void);

k_spinlock_key_t e906_smp_spin_lock(struct k_spinlock *l);

void e906_smp_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key);

void flash_critical_sync_ack();

static ALWAYS_INLINE bool e906_smp_spin_lock_is_locked(struct k_spinlock *l)
{
	return atomic_get(&l->owner) != atomic_get(&l->tail);
}

#define IPI_SCHED	0
#define IPI_FPU_FLUSH	1
#define IPI_XIP_LOCK    2

#endif