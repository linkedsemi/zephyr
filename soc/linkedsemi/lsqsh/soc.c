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
#include "iopmp.h"
#include "qsh.h"
#include <zephyr/irq.h>
#include "reg_sysc_sec_cpu.h"
#include "ls_soc_gpio.h"
#include "ls_hal_flash.h"
#include "ls_hal_cache.h"
#include "ls_msp_qspiv2.h"

BUILD_ASSERT(CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU, "CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU");
BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);

extern void noint(void);

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

extern uint32_t _nocache_ram_start;
extern uint32_t _nocache_ram_end;
extern uint32_t _nocache_ram_size;

#define CPU0_FW_REGION_SIZE MB(2)
#define CPU1_FW_REGION_SIZE MB(14)
/* strong order | cacheable | bufferable */
/*       2      |     1     |     0      */
#define BUFFERABLE BIT(0)
#define CACHEABLE BIT(1)
#define STRONG_ORDER BIT(2)
void cpu0_cache_region_init(void)
{
    uint8_t idx = 0;
    csi_sysmap_config_region(idx++, 0x10000000, 0);
    if ((uint32_t)&_nocache_ram_size > 0) {
        csi_sysmap_config_region(idx++, (uint32_t)&_nocache_ram_start, CACHEABLE | BUFFERABLE);
        csi_sysmap_config_region(idx++, (uint32_t)&_nocache_ram_end, 0);
    }
    csi_sysmap_config_region(idx++, 0x10000000 + KB(512 + 764), CACHEABLE | BUFFERABLE); /* 512KB + 768KB SRAM */

    csi_sysmap_config_region(idx++, 0x18000000, 0);
    csi_sysmap_config_region(idx++, 0x18000000 + MB(16), CACHEABLE | BUFFERABLE); /* 16MB PSRAM */
    csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
}

void cpu1_cache_region_init(void)
{
    uint8_t idx = 0;
    csi_sysmap_config_region(idx++, 0x10000000 + KB(512), 0);
    if ((uint32_t)&_nocache_ram_size > 0) {
        csi_sysmap_config_region(idx++, (uint32_t)&_nocache_ram_start, CACHEABLE | BUFFERABLE);
        csi_sysmap_config_region(idx++, (uint32_t)&_nocache_ram_end, 0);
    }
    csi_sysmap_config_region(idx++, 0x10000000 + KB(512 + 764), CACHEABLE | BUFFERABLE); /* 512KB + 768KB SRAM */

    csi_sysmap_config_region(idx++, 0x18000000, 0);
    csi_sysmap_config_region(idx++, 0x18000000 + MB(16), CACHEABLE | BUFFERABLE); /* 16MB PSRAM */
    csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
}

/*
| N | addr                           | mode  | rwx | desc                    |
|---|--------------------------------|-------|-----|-------------------------|
| 0 | 0x8000000--(0x8000000+2MB)     | NAPOT | --- | sec flash xip mem       |
| 1 | 0x10000000--(0x10000000+512KB) | NAPOT | --- | sec sram                |
| 2 | 0x40000000--(0x40000000+256KB) | NAPOT | --- | sec peripheral region 1 |
| 3 | 0x400a0000--(0x400A0000+32KB)  | NAPOT | --- | sec peripheral region 2 |
| 4 |                                | ----- |     |                         |
| 5 |                                | ----- |     |                         |
| 6 |                                | ----- |     |                         |
| 7 | 0x0 -- 4GB                     | NAPOT | rwx |                         |
|   |                                |       |     |                         |
*/
void iopmp_region_init(void)
{
    for (uint32_t idx = 0; idx < 5; idx++) {
        uint32_t dev = SEC_IOPMP1_ADDR + (idx * 0x400);
        iopmp_config_region_napot4(dev, 0, 0x8000000, MB(2), false, false, false, false);
        iopmp_config_region_napot4(dev, 1, 0x10000000, KB(512), false, false, false, false);
        iopmp_config_region_napot4(dev, 2, 0x40000000, KB(256), false, false, false, false);
        iopmp_config_region_napot4(dev, 3, 0x400A0000, KB(32), false, false, false, false);

        iopmp_config_region_napot4(dev, 7, 0x0, (uint64_t)4 * 1024 * 1024 * 1024, true, true, true, false);
        iopmp_config_enable(dev, true);
    }
}

extern void SWINT_Handler_Asm(void);
extern void SystemInit();
static int lsqsh_init(void)
{
    SystemInit();
    // sys_init_none();
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu0), okay)
    if (!(((uint32_t)&_nocache_ram_start > 0x10000000)
                    && ((uint32_t)&_nocache_ram_end < (0x10000000 + KB(512))))) {
        while(1);
    }
    cpu0_cache_region_init();
#else
    if (!(((uint32_t)&_nocache_ram_start > (0x10000000 + KB(512))
                    && ((uint32_t)&_nocache_ram_end < (0x10000000 + KB(512) + KB(764)))))) {
        while(1);
    }
    cpu1_cache_region_init();
#endif

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu0), okay)) && defined(CONFIG_IOPMP)
    iopmp_region_init();
#endif

#if defined(CONFIG_CACHE)
#if !defined(CONFIG_SMP)
    csi_dcache_enable();
#endif
    csi_icache_enable();

#if !defined(CONFIG_SMP)
    csi_dcache_invalid();
#endif
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
#if defined(CONFIG_BOOT_CPU1)
    SYSC_SEC_CPU->APP_CPU_ADDR_CFG = CONFIG_CPU1_BOOT_ADDR; /* set cpu1 pc addr */
    SYSC_SEC_CPU->APP_CPU_SRST = 0x1; /* release reset */
#endif
#endif
#endif

#if defined(CONFIG_PECI)
    sys_write32(0x0, APP_PMU_RG_APP_ADDR + 0x3e8);
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
