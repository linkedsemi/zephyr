#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/state.h>
#include "platform.h"
#include "core_rv32.h"
#include "exception_isr.h"
#include "systick.h"
#include "cpu.h"
#include <stdint.h>
#include <string.h>
#include "qsh.h"
#include <zephyr/irq.h>
#include "reg_sysc_per.h"
#if !defined(CONFIG_PINCTRL)
    #include "ls_soc_gpio.h"
#endif

#define RV_SOFT_IRQ_IDX 23
extern void noint(void);
uint32_t *pTaskStack = NULL;

static void cpu_sleep_mode_config(uint8_t deep)
{
    uint32_t mextstaus = __get_MEXSTATUS();
    MODIFY_REG(mextstaus,MEXSTATUS_SLEEP_Msk,(!deep)<<MEXSTATUS_SLEEP_Pos);
    __set_MEXSTATUS(mextstaus);
}

void systick_start(void){};
void sw_timer_module_init(void){};

static void driver_init(void)
{
}

void sys_arch_reboot(int type)
{
	// platform_reset(0);
}

void Swint_Handler_C(uint32_t *args)
{
    csi_vic_clear_pending_irq(RV_SOFT_IRQn);
    uint32_t *task_sp = pTaskStack;
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)task_sp[12];
    task_sp[8] = func(task_sp[8],task_sp[9],task_sp[10],task_sp[11]);
}

extern void SystemInit();
static int lsqsh_init(void)
{
    SystemInit();
    sys_init_none();

#if defined(CONFIG_CACHE)
    SYSMAP->SYSMAPADDR0 = 0x10001000 >> 12;
    SYSMAP->SYSMAPCFG0 = 0x10;
    SYSMAP->SYSMAPADDR1 = 0x1013f000 >> 12;
    SYSMAP->SYSMAPCFG1 = 0xc;
    SYSMAP->SYSMAPADDR2 = 0xffffffff >> 12;
    SYSMAP->SYSMAPCFG2 = 0x10;
    SYSMAP->SYSMAPADDR3 = 0;
    SYSMAP->SYSMAPCFG3 = 0;
    SYSMAP->SYSMAPADDR4 = 0;
    SYSMAP->SYSMAPCFG4 = 0;
    SYSMAP->SYSMAPADDR5 = 0;
    SYSMAP->SYSMAPCFG5 = 0;
    SYSMAP->SYSMAPADDR6 = 0;
    SYSMAP->SYSMAPCFG6 = 0;
    SYSMAP->SYSMAPADDR7 = 0;
    SYSMAP->SYSMAPCFG7 = 0;

    csi_dcache_enable();
    csi_icache_enable();

    csi_dcache_invalid();
    csi_icache_invalid();
#endif

#if !defined(CONFIG_PINCTRL)
    /* RMII */
    io_cfg_input(PT01);
    io_cfg_input(PT08);
    io_cfg_input(PT09);
    io_cfg_input(PT10);
    io_cfg_input(PT11);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xbc) = 0x2f3b;
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x58) = 0x206C80;

    /* PECI */
    io_cfg_input(PK00);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xb4) = 0x1;

    /* UART */
    pinmux_dwuart1_init(PC03, PC04);
    pinmux_dwuart2_init(PD09, PD10);

    /* I2C */
    pinmux_iic2_init(PB13, PB14);
    pinmux_iic7_init(PG15, PG14);
#endif

	IRQ_CONNECT(RV_SOFT_IRQn, 0, Swint_Handler_C, NULL, 0);
    cpu_sleep_mode_config(0);
    driver_init();
    arch_irq_lock();
    return 0;
}

SYS_INIT(lsqsh_init, PRE_KERNEL_1, 0);

/**
 * @brief Enable interrupt
 */
void riscv_clic_irq_enable(uint32_t irq)
{
    enable_irq(irq);
}

/**
 * @brief Disable interrupt
 */
void riscv_clic_irq_disable(uint32_t irq)
{
    disable_irq(irq);
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
    csi_vic_set_prio(irq,pri);
}
