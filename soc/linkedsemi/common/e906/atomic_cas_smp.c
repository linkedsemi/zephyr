#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/toolchain.h>

static struct k_spinlock lock;

__ramfunc k_spinlock_key_t e906_smp_spin_lock(struct k_spinlock *l)
{
	ARG_UNUSED(l);
	k_spinlock_key_t k;

	/* Note that we need to use the underlying arch-specific lock
	 * implementation.  The "irq_lock()" API in SMP context is
	 * actually a wrapper for a global spinlock!
	 */
	k.key = arch_irq_lock();

	/*
	 * Enqueue ourselves to the end of a spinlock waiters queue
	 * receiving a ticket
	 */
	atomic_val_t ticket = atomic_inc(&l->tail);
	/* Spin until our ticket is served */
	while (atomic_get(&l->owner) != ticket) {
	}

	return k;
}

__ramfunc void e906_smp_spin_unlock(struct k_spinlock *l,
					k_spinlock_key_t key)
{
	ARG_UNUSED(l);

	/* Give the spinlock to the next CPU in a FIFO */
	(void)atomic_inc(&l->owner);
	arch_irq_unlock(key.key);
}

__ramfunc bool atomic_cas_ram(atomic_t *target, atomic_val_t old_value,
			  atomic_val_t new_value)
{
	k_spinlock_key_t key;
	int ret = false;

	key = e906_smp_spin_lock(&lock);

	if (*target == old_value) {
		*target = new_value;
		ret = true;
	}

	e906_smp_spin_unlock(&lock, key);

	return ret;

}

bool atomic_cas(atomic_t *target, atomic_val_t old_value,
			  atomic_val_t new_value)
{
	k_spinlock_key_t key;
	int ret = false;

	key = k_spin_lock(&lock);

	if (*target == old_value) {
		*target = new_value;
		ret = true;
	}

	k_spin_unlock(&lock, key);

	return ret;
}

bool atomic_ptr_cas(atomic_ptr_t *target, atomic_ptr_val_t old_value,
				  atomic_ptr_val_t new_value)
{
	k_spinlock_key_t key;
	int ret = false;

	key = k_spin_lock(&lock);

	if (*target == old_value) {
		*target = new_value;
		ret = true;
	}

	k_spin_unlock(&lock, key);

	return ret;
}