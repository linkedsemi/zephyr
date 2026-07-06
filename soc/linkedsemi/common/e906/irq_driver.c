#include <stdint.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <soc.h>
#include <platform.h>
#include "smp/lsqsh_smp.h"


static void clic_irq_set_trigger(uint32_t irq, uint32_t flags)
{
	switch (flags) {
	case IRQ_TYPE_LEVEL_HIGH:
		CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_POSITIVE_LEVEL << CLIC_INTATTR_TRIG_Pos;
		break;
	case IRQ_TYPE_EDGE_RISING:
		CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_POSITIVE_EDGE << CLIC_INTATTR_TRIG_Pos;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		CLIC->CLICINT[irq].ATTR = CLIC_ATTR_TRIG_NEGATIVE_EDGE << CLIC_INTATTR_TRIG_Pos;
		break;
	default:
		break;
	}
}

#ifdef CONFIG_SMP
extern atomic_val_t *p_cpu_pending_ipi;

struct irq_affinity_cfg {
	atomic_val_t cpumask;	/* target CPU bitmask; 0 means local-only / default behavior */
	uint8_t      prio;	/* cached priority */
	uint8_t      flags;	/* cached trigger flags (IRQ_TYPE_*) */
};

static struct irq_affinity_cfg irq_affinity[CONFIG_NUM_IRQS];

#define AFFINITY_BITMAP_WORDS   ((CONFIG_NUM_IRQS + ATOMIC_BITS - 1) / ATOMIC_BITS)

struct irq_affinity_cpu_pending {
	atomic_val_t apply[AFFINITY_BITMAP_WORDS];
	atomic_val_t disable[AFFINITY_BITMAP_WORDS];
};

static struct irq_affinity_cpu_pending affinity_pending[CONFIG_MP_MAX_NUM_CPUS];
__nocache volatile bool affinity_ack[CONFIG_MP_MAX_NUM_CPUS];

/**
 * @brief Apply affinity configuration on the calling CPU for a single IRQ.
 */
static void clic_irq_apply_local(uint32_t irq, uint32_t cpu, uint32_t mask)
{
	if (mask & BIT(cpu)) {
		csi_vic_set_prio(irq, irq_affinity[irq].prio);
		clic_irq_set_trigger(irq, irq_affinity[irq].flags);
		csi_vic_enable_irq(irq);
	} else {
		csi_vic_disable_irq(irq);
		csi_vic_clear_pending_irq(irq);
	}
}

/**
 * @brief Notify target CPU(s) that they need to re-apply affinity for an IRQ.
 *
 * Only online CPUs are notified. Use this for runtime enable/disable/priority
 * changes where an offline CPU does not need to be pre-configured.
 */
static void clic_irq_notify_online(uint32_t irq, uint32_t mask)
{
	uint32_t cur_cpu = get_cur_cpu_id();

	for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
		if (cpu == cur_cpu || (mask & BIT(cpu)) == 0) {
			continue;
		}
		if (!_kernel.cpus[cpu].arch.online) {
			continue;
		}

		atomic_set_bit(&affinity_pending[cpu].apply[irq / ATOMIC_BITS],
			       irq % ATOMIC_BITS);
		atomic_set_bit(&p_cpu_pending_ipi[cpu], IPI_IRQ_AFFINITY);
		lsqsh_ipi_intr_set(cpu);
	}
}

#endif /* CONFIG_SMP */


/**
 * @brief Enable interrupt
 */
void riscv_clic_irq_enable(uint32_t irq)
{
#ifdef CONFIG_SMP
	uint32_t mask = (uint32_t)atomic_get(&irq_affinity[irq].cpumask);

	if (mask == 0) {
		/* Local-only interrupt (e.g. per-CPU IPI line): keep legacy behavior. */
		csi_vic_enable_irq(irq);
		return;
	}

	uint32_t cur_cpu = get_cur_cpu_id();

	/* Enable locally if this CPU is in the affinity mask. */
	if (mask & BIT(cur_cpu)) {
		csi_vic_enable_irq(irq);
	}

	/* Ask every other target CPU to enable it as well. */
	unsigned int key = arch_irq_lock();

	clic_irq_notify_online(irq, mask);
	arch_irq_unlock(key);
#else
	csi_vic_enable_irq(irq);
#endif
}

/**
 * @brief Disable interrupt
 */
void riscv_clic_irq_disable(uint32_t irq)
{
#ifdef CONFIG_SMP
	uint32_t mask = (uint32_t)atomic_get(&irq_affinity[irq].cpumask);

	if (mask == 0) {
		/* Local-only interrupt: keep legacy behavior. */
		csi_vic_disable_irq(irq);
		return;
	}

	uint32_t cur_cpu = get_cur_cpu_id();

	/* Disable locally if this CPU is in the affinity mask. */
	if (mask & BIT(cur_cpu)) {
		csi_vic_disable_irq(irq);
		csi_vic_clear_pending_irq(irq);
	}

	/* Ask every other target CPU to disable it as well.  We also clear any
	 * pending affinity-reapply request for the same IRQ so that a concurrent
	 * or in-flight IPI cannot re-enable it after we have disabled it.
	 */
	unsigned int key = arch_irq_lock();

	for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
		if (cpu == cur_cpu || (mask & BIT(cpu)) == 0) {
			continue;
		}

		atomic_clear_bit(&affinity_pending[cpu].apply[irq / ATOMIC_BITS],
				 irq % ATOMIC_BITS);
		atomic_set_bit(&affinity_pending[cpu].disable[irq / ATOMIC_BITS],
			       irq % ATOMIC_BITS);
		if (_kernel.cpus[cpu].arch.online) {
			atomic_set_bit(&p_cpu_pending_ipi[cpu], IPI_IRQ_AFFINITY);
			lsqsh_ipi_intr_set(cpu);
		}
	}
	arch_irq_unlock(key);
#else
	csi_vic_disable_irq(irq);
#endif
}

/**
 * @brief Get enable status of interrupt
 */
int riscv_clic_irq_is_enabled(uint32_t irq)
{
    return (uint32_t)csi_vic_get_enabled_irq(irq);
}

/**
 * @brief Set priority and level of interrupt
 */
void riscv_clic_irq_priority_set(uint32_t irq, uint32_t pri, uint32_t flags)
{
#ifdef CONFIG_SMP
    uint32_t mask = (uint32_t)atomic_get(&irq_affinity[irq].cpumask);
    uint32_t cur_cpu = get_cur_cpu_id();

    /* Cache the configuration so migration can replay it on another CPU. */
    irq_affinity[irq].prio = (uint8_t)pri;
    irq_affinity[irq].flags = (uint8_t)flags;

    /* Apply locally only if this CPU is supposed to own the interrupt. */
    if (mask == 0 || (mask & BIT(cur_cpu))) {
	    csi_vic_set_prio(irq, pri);
	    clic_irq_set_trigger(irq, flags);
    }

    /* Propagate to other target CPUs. */
    if (mask != 0) {
	    unsigned int key = arch_irq_lock();

	    clic_irq_notify_online(irq, mask);
	    arch_irq_unlock(key);
    }
#else
    csi_vic_set_prio(irq, pri);
    clic_irq_set_trigger(irq, flags);
#endif
}


void lsqsh_clic_irq_set_pending(uint32_t irq)
{
    MODIFY_REG(CLIC->CLICINT[irq].ATTR,CLIC_INTATTR_TRIG_Msk,1<<CLIC_INTATTR_TRIG_Pos);
	csi_vic_set_pending_irq(irq);
}

void lsqsh_clic_irq_disable_trigger_mode(uint32_t irq)
{
    MODIFY_REG(CLIC->CLICINT[irq].ATTR,CLIC_INTATTR_TRIG_Msk,0<<CLIC_INTATTR_TRIG_Pos);
}

#ifdef CONFIG_SMP
int lsqsh_clic_irq_set_affinity(uint32_t irq, uint32_t cpumask)
{
	if (irq >= CONFIG_NUM_IRQS) {
		return -EINVAL;
	}
	if (cpumask == 0 || (cpumask & ~BIT_MASK(arch_num_cpus())) != 0) {
		return -EINVAL;
	}

	unsigned int key = arch_irq_lock();
	uint32_t cur_cpu = get_cur_cpu_id();

	atomic_set(&irq_affinity[irq].cpumask, (atomic_val_t)cpumask);
	/* Ensure the updated affinity/priority/flags are visible to the
	 * target CPU before the IPI arrives.
	 */
	__sync_synchronize();

	/* Apply on the calling CPU immediately so that local enable/disable
	 * does not depend on the self-IPI being handled.
	 */
	clic_irq_apply_local(irq, cur_cpu, cpumask);

	/* Notify every other CPU.  Pending bits are set even for offline CPUs
	 * so that a secondary core which boots later will apply the new mask.
	 */
	for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
		if (cpu == cur_cpu) {
			continue;
		}
		atomic_set_bit(&affinity_pending[cpu].apply[irq / ATOMIC_BITS],
			       irq % ATOMIC_BITS);
		if (_kernel.cpus[cpu].arch.online) {
			atomic_set_bit(&p_cpu_pending_ipi[cpu], IPI_IRQ_AFFINITY);
			lsqsh_ipi_intr_set(cpu);
		}
	}

	arch_irq_unlock(key);
	return 0;
}

int lsqsh_clic_irq_set_affinity_sync(uint32_t irq, uint32_t cpumask)
{
	uint32_t cur_cpu = get_cur_cpu_id();
	int ret;

	/* Clear acks for every other online CPU that will be notified. */
	for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
		if (cpu != cur_cpu && _kernel.cpus[cpu].arch.online) {
			affinity_ack[cpu] = false;
		}
	}

	ret = lsqsh_clic_irq_set_affinity(irq, cpumask);
	if (ret != 0) {
		return ret;
	}

	/* Wait for each notified CPU to finish applying the affinity in its
	 * IPI handler.  The local CPU has already been applied synchronously
	 * inside riscv_clic_irq_set_affinity().
	 */
	for (uint32_t cpu = 0; cpu < arch_num_cpus(); cpu++) {
		if (cpu != cur_cpu && _kernel.cpus[cpu].arch.online) {
			while (!affinity_ack[cpu]) {
				/* spin; IPI handler runs with interrupts enabled */
			}
		}
	}

	return 0;
}

void lsqsh_clic_apply_affinity(void)
{
	uint32_t cpu = get_cur_cpu_id();

	/* First handle explicit disable requests.  These take precedence over a
	 * concurrent affinity re-apply so that irq_disable() really turns the
	 * interrupt off on the target CPU.
	 */
	for (uint32_t irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		if (!atomic_test_and_clear_bit(
			    &affinity_pending[cpu].disable[irq / ATOMIC_BITS],
			    irq % ATOMIC_BITS)) {
			continue;
		}

		csi_vic_disable_irq(irq);
		csi_vic_clear_pending_irq(irq);

		/* Make sure a stale affinity-reapply request does not re-enable this
		 * IRQ after we have just disabled it.
		 */
		atomic_clear_bit(&affinity_pending[cpu].apply[irq / ATOMIC_BITS],
				 irq % ATOMIC_BITS);
	}

	for (uint32_t irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		if (!atomic_test_and_clear_bit(&affinity_pending[cpu].apply[irq / ATOMIC_BITS],
					       irq % ATOMIC_BITS)) {
			continue;
		}

		uint32_t mask = (uint32_t)atomic_get(&irq_affinity[irq].cpumask);

		/* Mask == 0 should not normally appear as a pending affinity request,
		 * but skip it to avoid touching local-only interrupts such as IPIs.
		 */
		if (mask == 0) {
			continue;
		}

		clic_irq_apply_local(irq, cpu, mask);
	}

	affinity_ack[cpu] = true;
}

static int lsqsh_irq_affinity_init(void)
{
	memset(irq_affinity, 0, sizeof(irq_affinity));
	memset(affinity_pending, 0, sizeof(affinity_pending));
	memset((void *)affinity_ack, 0, sizeof(affinity_ack));

	/* Default routing: all interrupts handled on CPU0 unless explicitly
	 * rebound via riscv_clic_irq_set_affinity().
	 */
	for (uint32_t irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
		atomic_set(&irq_affinity[irq].cpumask, BIT(0));
	}

	/* IPI lines are per-CPU local and must not be routed by the affinity
	 * manager.  Keep mask == 0 so enable/disable stay local.
	 */
	atomic_set(&irq_affinity[SYSC_SEC_CPU_IRQN].cpumask, 0);
	atomic_set(&irq_affinity[SYSC_APP_CPU_IRQN].cpumask, 0);

	/* RISC-V local interrupts are per-CPU as well.  In particular the
	 * machine timer must be enabled on every core by smp_timer_init().
	 */
	atomic_set(&irq_affinity[RV_SOFT_IRQN].cpumask, 0);
	atomic_set(&irq_affinity[RV_TIME_IRQN].cpumask, 0);
	// atomic_set(&irq_affinity[RV_EXT_IRQN].cpumask, 0);

	/* Flash software interrupt is used by both cores for XIP sync. */
	atomic_set(&irq_affinity[FLASH_SWINT_NUM].cpumask, BIT(0) | BIT(1));

	return 0;
}
SYS_INIT(lsqsh_irq_affinity_init, PRE_KERNEL_1, 0);
#endif /* CONFIG_SMP */
