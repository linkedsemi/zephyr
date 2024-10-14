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
#include "reg_sysc_cpu.h"
#include "reg_sysc_per.h"
#include "ls_soc_gpio.h"

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
    uint32_t addr = 0;

    SystemInit();
    sys_init_none();

    addr = 0x10000000;
    SYSMAP->SYSMAPADDR0 = addr >> 12;
    SYSMAP->SYSMAPCFG0 = 0x10;
    addr = 0x10000000 + (508 * 1024);
    SYSMAP->SYSMAPADDR1 = addr >> 12;
    SYSMAP->SYSMAPCFG1 = 0xc;
    addr = 0x10000000 + (512 * 1024);
    SYSMAP->SYSMAPADDR2 = addr >> 12;
    SYSMAP->SYSMAPCFG2 = 0x10;
    addr = 0x10000000 + ((512 + 760) * 1024);
    SYSMAP->SYSMAPADDR3 = addr >> 12;
    SYSMAP->SYSMAPCFG3 = 0xc;
    addr = 0xffffffff;
    SYSMAP->SYSMAPADDR4 = addr >> 12;
    SYSMAP->SYSMAPCFG4 = 0x10;
    SYSMAP->SYSMAPADDR5 = 0;
    SYSMAP->SYSMAPCFG5 = 0;
    SYSMAP->SYSMAPADDR6 = 0;
    SYSMAP->SYSMAPCFG6 = 0;
    SYSMAP->SYSMAPADDR7 = 0;
    SYSMAP->SYSMAPCFG7 = 0;

#if defined(CONFIG_CACHE)
    csi_dcache_enable();
    csi_icache_enable();

    csi_dcache_invalid();
    csi_icache_invalid();
#endif

#if defined(CONFIG_ETH_DRIVER)
#if !defined(CONFIG_PINCTRL)
    /* RMII */
    io_cfg_input(PT01);
    io_cfg_input(PT08);
    io_cfg_input(PT09);
    io_cfg_input(PT10);
    io_cfg_input(PT11);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xbc) = 0x2f3b;
#endif
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x58) = 0x206C80;
#endif

#if defined(CONFIG_SDHC)
#if !defined(CONFIG_PINCTRL)
    /* SDHC */
    io_cfg_input(PH04);
    io_cfg_input(PH05);
    io_cfg_input(PH06);
    io_cfg_input(PH07);
    io_cfg_input(PH08);
    io_cfg_input(PH09);
    io_cfg_input(PH10);
    io_cfg_input(PH11);
    io_cfg_input(PH13);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xac) = 0x3FF00000;
#endif

    *(volatile uint32_t *)(QSH_SYSC_CPU_ADDR + 0x10) = 0x10000000;
    *(volatile uint32_t *)(QSH_SYSC_CPU_ADDR + 0x18) = 0x10000000;

    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x60) = 0xf0;
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x64) = 0xa0a00268;
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x68) = 0x1a0;
#endif

#if defined(CONFIG_PECI)
#if !defined(CONFIG_PINCTRL)
    /* PECI */
    io_cfg_input(PK00);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xb4) = 0x1;
#endif
#endif

#if defined(CONFIG_SERIAL)
#if !defined(CONFIG_PINCTRL)
    /* UART */
    pinmux_dwuart1_init(PC03, PC04);
    pinmux_dwuart2_init(PD09, PD10);
#endif
#endif

#if defined(CONFIG_I2C)
#if !defined(CONFIG_PINCTRL)
    /* I2C */
    pinmux_iic2_init(PB03, PB04);
    pinmux_iic3_init(PC13, PC11);
    pinmux_iic4_init(PD00, PD01);
    pinmux_iic5_init(PE07, PE08);
    pinmux_iic6_init(PF07, PF08);
    pinmux_iic7_init(PK15, PK14);
    pinmux_iic9_init(PI03, PI02);
    pinmux_iic10_init(PJ03, PJ02);
    pinmux_iic11_init(PN09, PN08);
    pinmux_iic12_init(PN13, PN14);
    pinmux_iic13_init(PQ00, PQ01);
    pinmux_iic14_init(PQ03, PQ02);
#endif
#endif

#if defined(CONFIG_CRYPTO_LINKEDSEMI)
    SYSC_CPU->PD_CPU_CLKG[1] = SYSC_CPU_CLKG_CLR_OTBN_MASK;
    SYSC_CPU->PD_CPU_SRST[1] = SYSC_CPU_SRST_CLR_OTBN_MASK;
    SYSC_CPU->PD_CPU_SRST[1] = SYSC_CPU_SRST_SET_OTBN_MASK;
    SYSC_CPU->PD_CPU_CLKG[1] = SYSC_CPU_CLKG_SET_OTBN_MASK;

    SYSC_CPU->OTBN_CTRL1 |= SYSC_CPU_EDN_URND_FIPS_MASK;
    for (uint8_t i = 0; i < 8; i++)
    {
        SYSC_CPU->EDN_URND_BUS = i;
        SYSC_CPU->OTBN_CTRL1 |= SYSC_CPU_EDN_URND_ACK_MASK;
        SYSC_CPU->OTBN_CTRL1 &= ~SYSC_CPU_EDN_URND_ACK_MASK;
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        SYSC_CPU->EDN_URND_BUS = i;
        SYSC_CPU->OTBN_CTRL1 |= SYSC_CPU_EDN_URND_ACK_MASK;
        SYSC_CPU->OTBN_CTRL1 &= ~SYSC_CPU_EDN_URND_ACK_MASK;
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    }

    SYSC_CPU->INTR_CTRL_CLR = 0x7ff;
    SYSC_CPU->INTR_CTRL_MSK = 0x700;
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
