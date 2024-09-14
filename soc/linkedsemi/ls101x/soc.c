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
#include "leo.h"
#include <zephyr/irq.h>
#include "reg_sysc_per.h"
#include "reg_v33_rg.h"
#include "field_manipulate.h"
#include "ls_msp_peci.h"

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

void dwuart_init()
{
	if (DT_PROP(DT_NODELABEL(dwuart1), status)) {

		SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_CLR_DWUART1_N_MASK;
		SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_SET_DWUART1_N_MASK;
		SYSC_PER->PD_PER_CLKG1 = SYSC_PER_CLKG_SET_DWUART1_MASK;
	}

	if (DT_PROP(DT_NODELABEL(dwuart2), status)) {
		SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_CLR_DWUART2_N_MASK;
		SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_SET_DWUART2_N_MASK;
		SYSC_PER->PD_PER_CLKG1 = SYSC_PER_CLKG_SET_DWUART2_MASK;
	}
}

void peci_init()
{
    SYSC_PER->PD_PER_CLKG3 = SYSC_PER_CLKG_SET_PECI_MASK;
    REG_FIELD_WR(V33_RG->TRIM0, V33_RG_LDO_PECI_VSEL, PECI_LDO_VOLT_1100);
    MODIFY_REG(V33_RG->TRIM0,0,1<<23);  
    SYSC_AWO->IO[2].DS |= 0x00100010;
}

static void driver_init(void)
{
    SYSC_PER->PD_PER_CLKG0 = SYSC_PER_CLKG_SET_I2C1_MASK;
    SYSC_PER->PD_PER_CLKG0 = SYSC_PER_CLKG_SET_I2C2_MASK;
    SYSC_PER->PD_PER_CLKG0 = SYSC_PER_CLKG_SET_I2C3_MASK;

#if CONFIG_SPI == 1
	SYSC_PER->PD_PER_CLKG1 = SYSC_PER_CLKG_CLR_SPI1_MASK;
	SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_CLR_SPI1_N_MASK;
	SYSC_PER->PD_PER_SRST1 = SYSC_PER_SRST_SET_SPI1_N_MASK;
	SYSC_PER->PD_PER_CLKG1 = SYSC_PER_CLKG_SET_SPI1_MASK;
#endif
    dwuart_init();
    
    peci_init();
}

void sys_arch_reboot(int type)
{
	platform_reset(0);
}

void Swint_Handler_C(uint32_t *args)
{
    csi_vic_clear_pending_irq(RV_SOFT_IRQn);
    uint32_t *task_sp = pTaskStack;
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)task_sp[12];
    task_sp[8] = func(task_sp[8],task_sp[9],task_sp[10],task_sp[11]);
}

extern void SystemInit();
static int ls101x_init(void)
{
    SystemInit();
    sys_init_none();
	IRQ_CONNECT(RV_SOFT_IRQn, 0, Swint_Handler_C, NULL, 0);
    cpu_sleep_mode_config(0);
    driver_init();
    arch_irq_lock();
    return 0;
}

SYS_INIT(ls101x_init, PRE_KERNEL_1, 0);

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
