#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/arch/riscv/csr.h>
#include <zephyr/sys/atomic_types.h>
#include <zephyr/arch/riscv/arch_inlines.h>
#include "soc.h"
#include "platform.h"
#include "smp/lsqsh_smp.h"
#include "ipi.h"
#include "ls_soc_gpio.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);
#define XIP_LOCK_COUNT                          (CONFIG_MP_MAX_NUM_CPUS-1)

void __scondary_cpu_reset(void);
atomic_val_t *p_cpu_pending_ipi;;

/* the cpu1 boot address is fixed */
#define LSQSH_CPU0_BOOT_ADDR                    (uint32_t)0x1000000
#define LSQSH_CPU1_BOOT_ADDR                    (uint32_t)__scondary_cpu_reset
#define LSQSH_CPU0     0
#define LSQSH_CPU1     1

typedef struct {
    volatile bool release_flag;
    volatile uint32_t make_lock_cpu_id;
    volatile uint32_t slave_count;
    uint32_t total_slaves;
} sync_control_t;

sync_control_t xip_sync;

uint32_t get_cur_cpu_id(void)
{
    return arch_curr_cpu()->id;
}

// uint32_t arch_irq_is_locked(void)
// {
//     uint32_t mstatus = csr_read(mstatus);
//     if((mstatus & BIT(3)) == 0)
//     {
//         while(1);
//     }
// }

__ramfunc void poll_wait_xip_unlock(void) 
{
    unsigned int key = arch_irq_lock();
    sync_control_t *ctrl = &xip_sync;

    __atomic_add_fetch(&ctrl->slave_count, 1, __ATOMIC_RELEASE);

    while (__atomic_load_n(&ctrl->release_flag, __ATOMIC_ACQUIRE) == false) 
    {
        // __asm__ volatile("wfi");
    }

    arch_irq_unlock(key);
}

int xip_lock(void) 
{
    unsigned int key = arch_irq_lock();
    sync_control_t *ctrl = &xip_sync;
    uint32_t _cpu_id = get_cur_cpu_id();
    if(ctrl->make_lock_cpu_id != 0xFF)
    {
        LOG_DBG("ctrl->make_lock_cpu_id != 0xFF\n");
        return -1;
    }
    ctrl->make_lock_cpu_id = _cpu_id;
    ctrl->slave_count = 0;
    __atomic_store_n(&ctrl->release_flag, false, __ATOMIC_RELEASE);

    lsqsh_xip_lcok_broadcast_ipi();
    // LOG_DBG("_cpu_id ipi : %d\n",_cpu_id);
    while (__atomic_load_n(&ctrl->slave_count, __ATOMIC_ACQUIRE) != ctrl->total_slaves);
    arch_irq_unlock(key);

    return 0;
}

int xip_lock_relesae(void)
{
    sync_control_t *ctrl = &xip_sync;
    uint32_t _cpu_id = get_cur_cpu_id();
    if(ctrl->make_lock_cpu_id == _cpu_id)
    {
        __atomic_store_n(&xip_sync.release_flag, true, __ATOMIC_RELEASE);
    }
    else
    {
        LOG_DBG(" xip_lock_relesae: unexception state \n");
        return -1;
    }
    ctrl->make_lock_cpu_id = 0xff;
    return 0;
}

bool get_xip_lock_owner(void)
{
    sync_control_t *ctrl = &xip_sync;
    uint32_t _cpu_id = get_cur_cpu_id();
    if(ctrl->make_lock_cpu_id == _cpu_id)
    {
        return true;
    }
    else
    {
        return false;
    }

}

void soc_late_init_hook(void)
{
    pinmux_hal_flash_quad_init();
}

void lsqsh_ipi_intr_clr(uint32_t cpu_id)
{
    if(cpu_id == LSQSH_CPU0)
    {
        cpu_intr_sec_clr();
    }
    else if(cpu_id == LSQSH_CPU1)
    {
        cpu_intr_app_clr();
    }else
    {
        while(1);
    }
}

void lsqsh_ipi_intr_set(uint32_t cpu_id)
{
    if(cpu_id == LSQSH_CPU0)
    {
        /* set cpu0 irq */
        cpu_intr_sec_activate();
    }
    else if(cpu_id == LSQSH_CPU1)
    {
        /* set cpu1 irq */
        cpu_intr_app_activate();
    }
    else
    {
        while(1);
    }
}

void cpu_early_common_config(void);
void cpu_sleep_mode_config(uint8_t deep);
__no_optimization void smp_mode_cache_region_init(void);

void lsqsh_xip_lcok_broadcast_ipi(void)
{
    unsigned int key = arch_irq_lock();
    unsigned int id = _current_cpu->id;
    unsigned int num_cpus = arch_num_cpus();
    uint32_t cpu_bitmap = IPI_ALL_CPUS_MASK;

	for (unsigned int i = 0; i < num_cpus; i++) {
		if ((i != id) && _kernel.cpus[i].arch.online &&
		 ((cpu_bitmap & BIT(i)) != 0)) {
			atomic_set_bit(&p_cpu_pending_ipi[i], IPI_XIP_LOCK);
			// MSIP(_kernel.cpus[i].arch.hartid) = 1;
            lsqsh_ipi_intr_set(i);
		}
	}

    arch_irq_unlock(key);
}

void sched_ipi_handler(const void *unused);
/* cpu1 */
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak)
{
    p_cpu_pending_ipi = p_ipi_msak;
    xip_sync.release_flag = false;
    xip_sync.slave_count = 0;
    xip_sync.total_slaves = XIP_LOCK_COUNT;
    xip_sync.make_lock_cpu_id = 0xff;
    /* premary processors init ipi isr*/
    IRQ_CONNECT(SYSC_SEC_CPU_IRQN, 0, sched_ipi_handler, NULL, 0);
	irq_enable(SYSC_SEC_CPU_IRQN);
    // /*enable on other processors*/
    IRQ_CONNECT(SYSC_APP_CPU_IRQN, 0, sched_ipi_handler, NULL, 0);
	irq_disable(SYSC_APP_CPU_IRQN);
}

void lsqsh_secondary_cpu_init(void)
{
    if(get_cur_cpu_id() == LSQSH_CPU1)
    {
        cpu_early_common_config();
        cpu_sleep_mode_config(0);
        smp_mode_cache_region_init();
        cpu_intr_sec_unmask();
        cpu_intr_app_unmask();
        irq_enable(SYSC_APP_CPU_IRQN);
	    irq_disable(SYSC_SEC_CPU_IRQN);
        // 当前cpu的flash中断要打开
        z_riscv_irq_priority_set(FLASH_SWINT_NUM, CONFIG_FLASH_SWINT_PRIORITY, IRQ_TYPE_EDGE_RISING);
        irq_enable(FLASH_SWINT_NUM);
    }
    else
    {
        while(1);
    }
}

#ifdef CONFIG_PM_CPU_OPS
int pm_cpu_on(unsigned long cpuid, uintptr_t entry_point)
{
    if(cpuid == LSQSH_CPU1)
    {
        app_cpu_dereset_by_addr((int)__scondary_cpu_reset);
        app_cpu_reset_hold_clr();
        return 0;
    }
    else
    {
        return -1;
    }
}
#endif