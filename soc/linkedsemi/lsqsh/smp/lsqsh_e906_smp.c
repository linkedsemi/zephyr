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
atomic_val_t *p_cpu_pending_ipi;

/* the cpu1 boot address is fixed */
#define LSQSH_CPU0_BOOT_ADDR                    (uint32_t)0x1000000
#define LSQSH_CPU1_BOOT_ADDR                    (uint32_t)__scondary_cpu_reset
#define LSQSH_CPU0     0
#define LSQSH_CPU1     1

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

void soc_late_init_hook(void)
{
    // pinmux_hal_flash_quad_init();
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

static void lsqsh_xip_lock_broadcast_ipi(void)
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

struct xip_sync_control{
    volatile bool flash_op_ongoing;
    volatile bool sync_ack[CONFIG_MP_MAX_NUM_CPUS]; 
};
__nocache struct xip_sync_control xip_sync;

__ramfunc void poll_wait_xip_unlock(void) 
{
    xip_sync.sync_ack[get_cur_cpu_id()] = true;
    while(xip_sync.flash_op_ongoing);
}

static void wait_xip_sync_ack()
{
    uint8_t i;
    for(i=0;i<CONFIG_MP_MAX_NUM_CPUS;i++)
    {
        if(i != get_cur_cpu_id())
        {
            while(!xip_sync.sync_ack[i]);
        }
    }
    for(i=0;i<CONFIG_MP_MAX_NUM_CPUS;i++)
    {
        xip_sync.sync_ack[i] = false;
    }
}

void flash_xip_lock_clear(void)
{
    xip_sync.flash_op_ongoing = false;
}

void flash_xip_lock_sync(void)
{
	xip_sync.flash_op_ongoing = true;
	lsqsh_xip_lock_broadcast_ipi();
	wait_xip_sync_ack();
}
void sched_ipi_handler(const void *unused);
/* cpu1 */
void lsqsh_primary_cpu_smp_init(atomic_val_t *p_ipi_msak)
{
    p_cpu_pending_ipi = p_ipi_msak;
    /* premary processors init ipi isr*/
    IRQ_CONNECT(SYSC_SEC_CPU_IRQN, 0, sched_ipi_handler, NULL, 0);
	irq_enable(SYSC_SEC_CPU_IRQN);
    // /*enable on other processors*/
    IRQ_CONNECT(SYSC_APP_CPU_IRQN, 0, sched_ipi_handler, NULL, 0);
	irq_disable(SYSC_APP_CPU_IRQN);

    
}

void smp_mode_cache_config(void)
{
#if defined(CONFIG_XIP)
    csi_icache_enable();
#else
    csi_icache_enable();
    // csi_dcache_enable();
#endif
}

void lsqsh_secondary_cpu_init(void)
{
    if(get_cur_cpu_id() == LSQSH_CPU1)
    {
        cpu_early_common_config();
        smp_mode_cache_config();
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
