#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/arch/riscv/csr.h>
#include "soc.h"
#include "field_manipulate.h"
#include "platform.h"
#include "ls_soc_gpio.h"
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);
#ifdef CONFIG_SMP
#include "smp/lsqsh_smp.h"
bool flash_ls_suspend_state_writing(const struct device *dev);
#endif

uint32_t irq_nested_level[CONFIG_MP_MAX_NUM_CPUS] = {0,0};
static uint32_t irq_nested_mcause[CONFIG_MP_MAX_NUM_CPUS][IRQ_NESTED_MAX] = {{0},{0}};
static const struct device *const zephyr_flash_controller =
    DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));

__ramfunc void isr_stacking_mcause(void)
{
#if defined(CONFIG_SMP)
	uint32_t _cpu_id = get_cur_cpu_id();
#else
    uint32_t _cpu_id = 0;
#endif
    uint32_t mcause = csr_read(mcause);
    
    flash_ex_op(zephyr_flash_controller,FLASH_DRIVER_SUSPEND_OPCODE,0,NULL);
#if defined(CONFIG_SMP)
    if((mcause & CONFIG_RISCV_MCAUSE_EXCEPTION_MASK) == FLASH_SWINT_NUM)
    {   
        LOG_DBG("cpu%d:xip_lock\n",_cpu_id);
        if(xip_lock() != 0)
        {
            while(1);
        }
    }
#endif
    if(irq_nested_level[_cpu_id] < IRQ_NESTED_MAX)
    {
        irq_nested_mcause[_cpu_id][irq_nested_level[_cpu_id]] = mcause;
        irq_nested_level[_cpu_id]++;
    }
    else
    {
        while(1);
    }
}

#define MCAUSE_MPP_MASK (3UL << 27)
#define MCAUSE_MPIE_MASK (1UL << 26)

__ramfunc void isr_unstacking_mcause(void)
{
    uint32_t current_mcause;
    uint32_t restore_mcause;
	/* get current cpu number */
#if defined(CONFIG_SMP)
	uint32_t _cpu_id = get_cur_cpu_id();
#else
    uint32_t _cpu_id = 0;
#endif
    if(irq_nested_level[_cpu_id] > 0 && irq_nested_level[_cpu_id] <= IRQ_NESTED_MAX)
    {
        irq_nested_level[_cpu_id]--;
        restore_mcause = irq_nested_mcause[_cpu_id][irq_nested_level[_cpu_id]];
        current_mcause = csr_read(mcause);

        current_mcause &=  (MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK);
        MODIFY_REG(restore_mcause,(MCAUSE_MPP_MASK | MCAUSE_MPIE_MASK),current_mcause);

        csr_write(mcause,restore_mcause);
        
    }
    else
    {
        while(1);
    }

    flash_ex_op(zephyr_flash_controller,FLASH_DRIVER_RESUME_OPCODE,0,NULL);
#if defined(CONFIG_SMP)
    if((restore_mcause & CONFIG_RISCV_MCAUSE_EXCEPTION_MASK) == FLASH_SWINT_NUM)
    {
        if(get_xip_lock_owner())
        {
            if(xip_lock_relesae() != 0)
            {
                while(1);
            }
            LOG_DBG("cpu%d:lock_relesae\n",_cpu_id);
        }
    }
#endif
}

void Swint_Handler_C(struct arch_esf *args)
{
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args->a4;
    args->a0 = func(args->a0, args->a1, args->a2, args->a3);
}
