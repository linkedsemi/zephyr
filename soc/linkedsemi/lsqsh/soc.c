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
#include "reg_sysc_sec_cpu.h"
#include "ls_soc_gpio.h"
#include "ls_hal_flash.h"
#include "ls_hal_cache.h"
#include "ls_msp_qspiv2.h"

BUILD_ASSERT(CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU, "CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU");

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
    uint32_t (*func)(uint32_t,uint32_t,uint32_t,uint32_t) = (void *)args[12];
    args[8] = func(args[8],args[9],args[10],args[11]);
}

/* strong order | cacheable | bufferable */
/*       2      |     1     |     0      */
#define BUFFERABLE BIT(0)
#define CACHEABLE BIT(1)
#define STRONG_ORDER BIT(2)
void cache_region_init(void)
{
    csi_sysmap_config_region(0, 0x8000000, STRONG_ORDER);
    csi_sysmap_config_region(1, 0x8000000 + (16 << 20), CACHEABLE); /* 16MB PSRAM */
    csi_sysmap_config_region(2, 0x10000000, STRONG_ORDER);
    csi_sysmap_config_region(3, 0x10000000 + ((512 + 760) << 10), CACHEABLE | BUFFERABLE); /* 512KB + 768KB SRAM */
    csi_sysmap_config_region(4, 0x18000000, STRONG_ORDER);
    csi_sysmap_config_region(5, 0x18000000 + (16 << 20), CACHEABLE | BUFFERABLE);
    csi_sysmap_config_region(6, 0xffffffff, STRONG_ORDER);
}

extern void SWINT_Handler_Asm(void);
extern void SystemInit();
static int lsqsh_init(void)
{
    SystemInit();
    // sys_init_none();
    cache_region_init();

#if defined(CONFIG_CACHE)
    csi_dcache_enable();
    csi_icache_enable();

    csi_dcache_invalid();
    csi_icache_invalid();
#endif

#if defined(CONFIG_ETH_DRIVER)
#if !defined(CONFIG_PINCTRL)
    /* RMII */
    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xbc) = 0x2f3b;
#endif
    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x54) = 0x10;
    // *(volatile uint32_t *)(QSH_SYSC_CPU_ADDR + 0x100) = 0x9;
#endif

#if defined(CONFIG_SDHC)
#if !defined(CONFIG_PINCTRL)
    /* SDHC */
    *(volatile uint32_t *)(APP_SYSC_AWO_APP_ADDR + 0x60) = BIT(14) | BIT(26);
    *(volatile uint32_t *)(APP_SYSC_AWO_APP_ADDR + 0x64) = BIT(7) | BIT(15);
    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xac) = 0x3FF00000;
#endif

    // *(volatile uint32_t *)(QSH_SYSC_CPU_ADDR + 0x10) = 0x10000000;
    // *(volatile uint32_t *)(QSH_SYSC_CPU_ADDR + 0x18) = 0x10000000;

    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x60) = 0xf0;
    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x64) = 0xa0a00268;
    // *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x68) = 0x1a0;
#endif

    cpu_sleep_mode_config(0);
    driver_init();
    arch_irq_lock();

#if (CONFIG_NUM_USE_CPU == 2)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu0), okay)
#if ((CONFIG_CPU1_BOOT_ADDR >= 0x8000000) && (CONFIG_CPU1_BOOT_ADDR <= (0x8000000 + 64*1024*1024)))
    lsqspiv2_msp_init();
    pinmux_hal_flash_init();
    hal_flash_dual_mode_set(true);
    hal_flash_drv_var_init(false, false);
    hal_flash_xip_func_ptr_dummy();
    hal_flash_init();

    hal_flash_xip_mode_reset();
    hal_flash_release_from_deep_power_down();
    DELAY_US(20);
    hal_flash_software_reset();
    DELAY_US(200);

    // pinmux_hal_flash_quad_init();
    hal_flash_xip_start();
    lscache_cache_enable(1);
    hal_flash_xip_func_ptr_init();
#endif
    SYSC_SEC_CPU->APP_CPU_ADDR_CFG = CONFIG_CPU1_BOOT_ADDR; /* set cpu1 pc addr */
    SYSC_SEC_CPU->APP_CPU_SRST = 0x1; /* release reset */
#endif
#endif

#if defined(CONFIG_SOC_FLASH_LS)
#if !defined(CONFIG_CPU1_BOOT_ADDR) && !defined(CONFIG_XIP)
    hal_flash_init();
#else
    qspiv2_global_int_ctrl_fn_init();
#endif

    hal_flash_dual_mode_set(1);
    flash_swint_init();
#if defined(CONFIG_XIP)
    hal_flash_drv_var_init(true,false);
#endif
    hal_flash_xip_func_ptr_init();
    IRQ_CONNECT(RV_SOFT_IRQN, 0, SWINT_Handler_Asm, NULL, 0);

#if !defined(CONFIG_CPU1_BOOT_ADDR) && !defined(CONFIG_XIP)
    hal_flash_xip_mode_reset();
#endif
#endif

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
