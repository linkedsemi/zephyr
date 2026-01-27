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
#define LSQSH_BOOT_ADDR (0x1000000)
uint32_t irq_nested_level[CONFIG_MP_MAX_NUM_CPUS] = {0,0};
static uint32_t irq_nested_mcause[CONFIG_MP_MAX_NUM_CPUS][IRQ_NESTED_MAX] = {{0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0}};
static volatile bool cpu1_flash_xip_banned = false;
static volatile bool cpu2_flash_xip_banned = false;
static volatile bool cpu1_pendding = false;
static volatile bool cpu2_pendding = false;
static const struct device *const zephyr_flash_controller =
	DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_flash_controller));
extern bool flash_ls_suspend_state_writing(const struct device *dev);

#define LSQSH_CPU1_IDX  0
#define LSQSG_CPU2_IDX  1

static uint8_t get_cur_cpu_id(void)
{
    if(csr_read(mraddr) == LSQSH_BOOT_ADDR)
    {
        return LSQSH_CPU1_IDX;
    }else
    {
        return LSQSG_CPU2_IDX;
    }
}

__ramfunc void isr_stacking_mcause(void)
{
    io_set_pin(PF13);
	/* get current cpu number */
	uint32_t _cpu_id = get_cur_cpu_id();//这里要加自旋锁吗~  不能用这个接口，接口里边会重新打开中断，要用baddress寄存器
    uint32_t mcause = csr_read(mcause);
#if defined(CONFIG_SMP)
    if(_cpu_id == LSQSH_CPU1_IDX && cpu1_flash_xip_banned)
    {
        cpu1_pendding = true;
        while(cpu1_flash_xip_banned);
        cpu1_pendding = false;
    }
    else if(_cpu_id == LSQSG_CPU2_IDX && cpu2_flash_xip_banned)
    {
        cpu2_pendding = true;
        while(cpu2_flash_xip_banned);
        cpu2_pendding = false;
    }
    /* cpu1 */
    if(mcause == FLASH_SWINT_NUM)
    {
        arch_sched_broadcast_ipi();
        if(_cpu_id == LSQSH_CPU1_IDX)
        {
            cpu2_flash_xip_banned = true;
            while(cpu2_pendding == false);
        }else
        {
            cpu1_flash_xip_banned = true;
            while(cpu1_pendding == false);
        }
    }
    else if(flash_ls_suspend_state_writing(zephyr_flash_controller))
    {
        arch_sched_broadcast_ipi();
        if(_cpu_id == LSQSH_CPU1_IDX)
        {
            cpu2_flash_xip_banned = true;
            while(cpu2_pendding == false);
        }else
        {
            cpu1_flash_xip_banned = true;
            while(cpu1_pendding == false);
        }
        flash_ex_op(zephyr_flash_controller,FLASH_DRIVER_SUSPEND_OPCODE,0,NULL);
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
    io_clr_pin(PF13);
}

#define MCAUSE_MPP_MASK (3UL << 27)
#define MCAUSE_MPIE_MASK (1UL << 26)

__ramfunc void isr_unstacking_mcause(void)
{
    uint32_t current_mcause;
    uint32_t restore_mcause;

	/* get current cpu number */
	uint32_t _cpu_id = get_cur_cpu_id();//这里要加自旋锁吗~
#if defined(CONFIG_SMP)
    if(csr_read(mraddr) != LSQSH_BOOT_ADDR)
    {
        // return;
    }
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
#if defined(CONFIG_SMP)
    if(restore_mcause == FLASH_SWINT_NUM)
    {
        if(_cpu_id == LSQSH_CPU1_IDX)
        {
            cpu2_flash_xip_banned = false;
        }
        else
        {
            cpu1_flash_xip_banned = false;
        }
    }
#endif
    flash_ex_op(zephyr_flash_controller,FLASH_DRIVER_RESUME_OPCODE,0,NULL);
}

void discard_current_irq_nested(void)
{
    uint32_t _cpu_id = get_cur_cpu_id();
    irq_nested_level[_cpu_id]--;
}

//flash_ex_op 需要加自旋锁 ， 其他flash接口加 sched_lock
// cpu2 把对应中断开起来就行，不需要管~ 通过setpending的 方式实现的 只会在本地cpu执行~   
// 想办法让当先线程执行flash操作的时候，将当前线程锁在当前cpu执行   sched_lock 可以保证当前正在工作的线程不被切换，只i执行在当前cpu上
void Swint_Handler_C(struct arch_esf *args)
{
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args->a4;
    args->a0 = func(args->a0, args->a1, args->a2, args->a3);
}
