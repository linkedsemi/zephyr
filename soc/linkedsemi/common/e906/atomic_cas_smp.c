#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/toolchain.h>

static struct k_spinlock lock;

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