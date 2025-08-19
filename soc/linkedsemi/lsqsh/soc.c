#include <zephyr/init.h>
#include <zephyr/platform/hooks.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/crc.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/state.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
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
#include "reg_sec_pmu_rg.h"
#include "reg_sysc_sec_awo.h"
#include "reg_sysc_app_awo.h"
#include "reg_sysc_sec_cpu.h"
#include "ls_hal_iwdgv2.h"
#include "ls_soc_gpio.h"
#include "ls_hal_flash.h"
#include "ls_hal_cache.h"
#include "ls_msp_qspiv2.h"
#include "soc.h"
#include "soc_reset.h"
#include "soc_boot.h"

#define MHINT_AEE_POS 20
BUILD_ASSERT(CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU, "CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU");
BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);
BUILD_ASSERT(CONFIG_FLASH);
BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(qspi1)));
#if defined(CONFIG_CACHE)
IF_ENABLED(CONFIG_DCACHE, (BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE_DETECT)));
IF_ENABLED(CONFIG_DCACHE, (BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE > 0)));
#endif
BUILD_ASSERT(FIXED_PARTITION_OFFSET(a_app_image_partition) < FIXED_PARTITION_OFFSET(b_app_image_partition));

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
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    disable_global_irq();
    reset_reason_magic_set();
    sys_cache_data_flush_all();
    sys_cache_data_disable();
    sys_cache_instr_disable();
    for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
        irq_disable(irq);
    }
    void (* goto_rom_region_start)();
    goto_rom_region_start = (void *)__rom_region_start;
    goto_rom_region_start();
#else
    sys_cache_data_flush_all();
    csi_core_reset();
#endif
}

#define CPU0_FW_REGION_SIZE MB(2)
#define CPU2_FW_REGION_SIZE MB(14)
/* strong order | cacheable | bufferable */
/*       2      |     1     |     0      */
#define WEAK_ORDER 0
#define BUFFERABLE BIT(0)
#define CACHEABLE BIT(1)
#define STRONG_ORDER BIT(2)

extern char __SHMEM_start[];
extern char __SHMEM_end[];
extern char __SHMEM_size[];

__no_optimization void cpu1_cache_region_init(void)
{
    __maybe_unused const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
    __maybe_unused const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
    __maybe_unused const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
    __maybe_unused const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    __maybe_unused const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    __maybe_unused const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
    __maybe_unused const uint32_t ___SHMEM_start = (uint32_t)__SHMEM_start;
    __maybe_unused const uint32_t ___SHMEM_end = (uint32_t)__SHMEM_end;
    __maybe_unused const uint32_t ___SHMEM_size = (uint32_t)__SHMEM_size;
    uint8_t idx = 0;

    csi_sysmap_config_region(idx++, __image_ram_start, WEAK_ORDER);

#if defined(CONFIG_NOCACHE_MEMORY)
    if ((__nocache_ram_size > 0) && (__nocache_ram_size < __image_ram_size)) {
        // __ASSERT_NO_MSG(__nocache_ram_size % CONFIG_PMP_GRANULARITY == 0);
        while(!(__nocache_ram_size % CONFIG_PMP_GRANULARITY == 0));
        if (__image_ram_start != __nocache_ram_start) {
            csi_sysmap_config_region(idx++, __nocache_ram_start, CACHEABLE | BUFFERABLE);
        }
        csi_sysmap_config_region(idx++, __nocache_ram_end, WEAK_ORDER);
    }
#endif

    csi_sysmap_config_region(idx++, __image_ram_end, CACHEABLE | BUFFERABLE);

#if DT_NODE_EXISTS(DT_NODELABEL(mbox))
    csi_sysmap_config_region(idx++, (___SHMEM_start + DT_REG_SIZE(DT_NODELABEL(share_memory)) + DT_REG_SIZE(DT_NODELABEL(mbox))), WEAK_ORDER);
#endif

    csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram))), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */

    if (idx < 8) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
}

__no_optimization void cpu2_cache_region_init(void)
{
    __maybe_unused const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
    __maybe_unused const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
    __maybe_unused const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
    __maybe_unused const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    __maybe_unused const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    __maybe_unused const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
    __maybe_unused const uint32_t ___SHMEM_start = (uint32_t)__SHMEM_start;
    __maybe_unused const uint32_t ___SHMEM_end = (uint32_t)__SHMEM_end;
    __maybe_unused const uint32_t ___SHMEM_size = (uint32_t)__SHMEM_size;
    uint8_t idx = 0;

#if defined(CONFIG_XIP)
    csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), WEAK_ORDER);
    csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + DT_REG_SIZE(DT_CHOSEN(zephyr_flash))), CACHEABLE);
#endif

    csi_sysmap_config_region(idx++, __image_ram_start, WEAK_ORDER);

#if defined(CONFIG_NOCACHE_MEMORY)
    if ((__nocache_ram_size > 0) && (__nocache_ram_size < __image_ram_size)) {
        // __ASSERT_NO_MSG(__nocache_ram_size % CONFIG_PMP_GRANULARITY == 0);
        while(!(__nocache_ram_size % CONFIG_PMP_GRANULARITY == 0));
        if (__image_ram_start != __nocache_ram_start) {
            csi_sysmap_config_region(idx++, __nocache_ram_start, CACHEABLE | BUFFERABLE);
        }
        csi_sysmap_config_region(idx++, __nocache_ram_end, WEAK_ORDER);
    }
#endif

    csi_sysmap_config_region(idx++, __image_ram_end, CACHEABLE | BUFFERABLE);

#if DT_NODE_EXISTS(DT_NODELABEL(mbox))
    csi_sysmap_config_region(idx++, (___SHMEM_start + DT_REG_SIZE(DT_NODELABEL(share_memory)) + DT_REG_SIZE(DT_NODELABEL(mbox))), WEAK_ORDER);
#endif

    csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram))), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */

    if (idx < 8) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
}


/*
| N | addr                           | mode  | rwx | desc                    |
|---|--------------------------------|-------|-----|-------------------------|
| 0 | 0x1000000--(0x1000000+64KB)    | NAPOT | --- | rom                     |
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
    ls_clock_control_off(IOPMP_CLOCK);
    ls_reset_line_toggle(IOPMP_RESET);
    ls_clock_control_on(IOPMP_CLOCK);
    for (uint32_t idx = 0; idx < 2; idx++) {
        uint32_t dev = SEC_IOPMP1_ADDR + (idx * 0x400);
        iopmp_config_region_napot4(dev, 0, 0x1000000, KB(64), false, false, false, false);
#if 0
        iopmp_config_region_napot4(dev, 1, 0x8000000, MB(2), false, false, false, false);
#endif
        iopmp_config_region_napot4(dev, 2, 0x10000000, KB(512), false, false, false, false);
        iopmp_config_region_napot4(dev, 3, SEC_SYSC_CPU_SEC_ADDR + 0x28 /* sec_cpu_intr */, 4, true, true, true, false);
        iopmp_config_region_napot4(dev, 4, 0x40000000, KB(256), false, false, false, false);
        iopmp_config_region_napot4(dev, 5, 0x400A0000, KB(32), false, false, false, false);

        iopmp_config_region_napot4(dev, 7, 0x0, (uint64_t)4 * GB(1), true, true, true, false);
        iopmp_config_enable(dev, true);
    }
}

extern void SWINT_Handler_ASM(void);
extern void SystemInit();
extern void psram_init(void);

static void set_trim_params()
{
    REG_FIELD_WR(SEC_PMU->MISC_CTRL0, SEC_PMU_RG_CLK_LDO1_VSEL, 0);
    REG_FIELD_WR(SEC_PMU->MISC_CTRL0, SEC_PMU_RG_CLK_LDO2_VSEL, 0);
}

static void enable_dpll()
{
    CLEAR_BIT(SYSC_SEC_AWO->DPLL1_CTRL1, SYSC_SEC_AWO_DPLL1_CTRL1_PLL1_CLKREF_SEL_MASK); /* clkin */
    SET_BIT(SYSC_SEC_AWO->DPLL1_CTRL1, SYSC_SEC_AWO_DPLL1_CTRL1_PLL1_EN_MASK); /* clr reset */
    SET_BIT(SYSC_SEC_AWO->DPLL1_CTRL1, SYSC_SEC_AWO_DPLL1_CTRL1_PLL1_RSTN_MASK); /* enable pll1 */
    while(0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL1_LOCK_MASK));

    CLEAR_BIT(SYSC_SEC_AWO->DPLL2_CTRL1, SYSC_SEC_AWO_DPLL2_CTRL1_PLL2_CLKREF_SEL_MASK); /* clkin */
    SET_BIT(SYSC_SEC_AWO->DPLL2_CTRL1, SYSC_SEC_AWO_DPLL2_CTRL1_PLL2_EN_MASK); /* clr reset */
    SET_BIT(SYSC_SEC_AWO->DPLL2_CTRL1, SYSC_SEC_AWO_DPLL2_CTRL1_PLL2_RSTN_MASK); /* enable pll2 */
    while(0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL2_LOCK_MASK));
}

static void cpu_600M_ahb_300M_qspi_200M_init()
{
    SYSC_SEC_AWO->PD_AWO_CLK_CTRL1 = FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_PBUS0, 0x0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_PBUS1, 0x0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_PBUS2, 0x0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_PBUS3, 0x0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_PBUS4, 0x3)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_DIV_HBUS, 0x1)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_OTP, 0x1);
    SYSC_SEC_AWO->CLKG_DIV_DPLL = SYSC_SEC_AWO_CLKG_DIV_DPLL_CLR_MASK;
    SYSC_SEC_AWO->PD_AWO_CLK_CTRL0 = 
                                  // FIELD_BUILD(SYSC_SEC_AWO_CLK_DIV_PARA_HBUS_M1, 0x1)
                                     FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS, 0x1)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS_M1, 0x1) /* set ahb_clk = 1/2 * cpu_clk */
                                 //| FIELD_BUILD(SYSC_SEC_AWO_HSE_DCT_EN, 0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_HBUS_FLT_CTRL, 0x9)
                                   | FIELD_BUILD(SYSC_SEC_AWO_QSPI_FLT_CTRL, 0x9)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_QSPI, 0x1)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS_FLT, 0x2)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_QSPI_FLT, 0x2);
    SYSC_SEC_AWO->CLKG_DIV_DPLL = SYSC_SEC_AWO_CLKG_DIV_DPLL_SET_MASK;
    SYSC_SEC_AWO->PD_AWO_CLK_CTRL0 =
                                  // FIELD_BUILD(SYSC_SEC_AWO_CLK_DIV_PARA_HBUS_M1, 0x1)
                                     FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS, 0x10)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS_M1, 0x1)
                                 //| FIELD_BUILD(SYSC_SEC_AWO_HSE_DCT_EN, 0)
                                   | FIELD_BUILD(SYSC_SEC_AWO_HBUS_FLT_CTRL, 0x9)
                                   | FIELD_BUILD(SYSC_SEC_AWO_QSPI_FLT_CTRL, 0x9)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_QSPI, 0x10)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_HBUS_FLT, 0x2)
                                   | FIELD_BUILD(SYSC_SEC_AWO_CLK_SEL_QSPI_FLT, 0x2);
}

#if 0

void lsqsh_emmc_txck_rxck_config(uint32_t base_clock, uint32_t target_clock)
{
    __ASSERT_NO_MSG(base_clock);
    __ASSERT_NO_MSG(target_clock);
    __ASSERT_NO_MSG(base_clock >= target_clock);
    uint16_t div = base_clock / target_clock;

    if (div) {
        div--;
    }
    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_DIV, div);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_SEL, 0x4); /* dpll_200M */
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);

    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_DIV, div);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_SEL, 0x2); /* dpll_200M */
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
}
#else

void lsqsh_emmc_txck_rxck_config(uint32_t base_clock, uint32_t target_clock)
{
    uint16_t div = 0x20;
    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_DIV, div);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_SEL, 0x1); /* hsi */
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);

    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_DIV, div);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_SEL, 0x1); /* hsi */
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
}

#endif

static void peripheral_init()
{
    /* SYSC_APP_AWO->PD_AWO_CLK_CTRL1 */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_PSRAM, 0x10); /* dpll_600M */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_PSRAM_FLT, 0x2); /* bypass */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_USB2, 0x4); /* dpll_48M */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_I3C, 0x4); /* clk_pbus4 */
    /* SYSC_APP_AWO->PD_AWO_CLK_CTRL1 */


    /* SYSC_APP_AWO->ETH_EMMC_RST */
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_TX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_RX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_RMII_RST_N_MASK);

    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_TX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_RX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_RMII_RST_N_MASK);
    /* SYSC_APP_AWO->ETH_EMMC_RST */


    /* SYSC_APP_AWO->ETH_EMMC_RST */
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_TX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_RX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_CORE_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_TIM_RST_N_MASK);

    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_TX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_RX_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_CORE_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_TIM_RST_N_MASK);

    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_LPC1_CLK_RST_N_MASK);
    SET_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_LPC2_CLK_RST_N_MASK);
    /* SYSC_APP_AWO->ETH_EMMC_RST */


    /* SYSC_APP_AWO->ETH1_CLK_CFG */
    REG_FIELD_WR(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_TX_DIV, 0); /* div = 2 */
    REG_FIELD_WR(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_TX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_TX_CG_MASK);

    REG_FIELD_WR(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_RX_DIV, 0); /* div = 2 */
    REG_FIELD_WR(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_RX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH1_CLK_CFG, SYSC_APP_AWO_ETH1_CLK_RX_CG_MASK);

    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_DIV, 0); /* div = 2 */
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->ETH1_CLK_CFG */


    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_DIV_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_SEL, 0x1);
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);

    CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_DIV_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_SEL, 0x1);
    SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */


    /* SYSC_APP_AWO->EMMC1_CORE_TIM_CLK */
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_SEL, 0x2); /* dpll 200M */
    SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_CG_MASK);

    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_SEL, 0x1); /* hsi */
    SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_CG_MASK);
    /* SYSC_APP_AWO->EMMC1_CORE_TIM_CLK */


    /* SYSC_APP_CPU->ETH1_PHY_CTRL */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_PHY_INTF_SEL, 0x1); /* rgmii */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_PHY_SEL, 0x1); /* rgmii */
    /* SYSC_APP_CPU->ETH1_PHY_CTRL */


    /* SYSC_APP_AWO->EMMC2_TX_RX_CLK */
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_SEL, 0x4); /* dpll 200M */
    SET_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_CG_MASK);

    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_SEL, 0x2); /* dpll 200M */
    SET_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->EMMC2_TX_RX_CLK */


    /* SYSC_APP_AWO->EMMC2_CORE_TIM_CLK */
    CLEAR_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_CG_MASK);
    SET_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_DIV_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_SEL, 0x2); /* dpll 200M */
    SET_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_CG_MASK);

    CLEAR_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_TIM_CG_MASK);
    SET_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_TIM_DIV_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_TIM_SEL, 0x1); /* hsi */
    SET_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_TIM_CG_MASK);
    /* SYSC_APP_AWO->EMMC2_CORE_TIM_CLK */


    /* SYSC_APP_AWO->LPC_CLK */
    REG_FIELD_WR(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC1_CLK_DIV, 0x0);
    REG_FIELD_WR(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC1_CLK_SEL, 0x8); /* dpll 50M */
    SET_BIT(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC1_CLK_CG_MASK);

    REG_FIELD_WR(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC2_CLK_DIV, 0x0);
    REG_FIELD_WR(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC2_CLK_SEL, 0x8); /* dpll 50M */
    SET_BIT(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC1_CLK_CG_MASK);
    /* SYSC_APP_AWO->LPC_CLK */
}

void soc_early_init_hook(void)
{
    reset_reason_init();

    __set_MTVT((uint32_t)0);
#if defined(CONFIG_PRECISE_EXCEPTION)
    __set_MHINT(__get_MHINT() | BIT(MHINT_AEE_POS));
    if (BIT(MHINT_AEE_POS) != (__get_MHINT() & BIT(MHINT_AEE_POS))) {
        while(1);
    }
#endif

#if defined(CONFIG_IRQ_NESTED)
    CLIC->CLICCFG = 0x7f;
#endif

    if ((PWR_FULL_RESET == reset_reason_get())
        || (SOFT_FULL_RESET == reset_reason_get())
        || (CPU_FULL_RESET == reset_reason_get())
        || (SYS_IWDT_FULL_RESET == reset_reason_get())
        || (EXT_FULL_RESET == reset_reason_get())
        || (SEC_IWDT_FULL_RESET == reset_reason_get())
        || (SEC_WWDT_FULL_RESET == reset_reason_get())) {
        memset((void *)DT_REG_ADDR(DT_NODELABEL(mbox)), 0, DT_REG_SIZE(DT_NODELABEL(mbox)));
    }

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    if (!is_app_cpu_running()) {
        if ((0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL1_LOCK_MASK))
            && (0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL2_LOCK_MASK))) {
            set_trim_params();
            enable_dpll();
            cpu_600M_ahb_300M_qspi_200M_init();
            peripheral_init();
        }
    }
#endif

    SystemInit();
    // sys_init_none();
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    cpu1_cache_region_init();
#else
    cpu2_cache_region_init();
#endif

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)) && defined(CONFIG_IOPMP)
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

    cpu_sleep_mode_config(0);
    driver_init();
    arch_irq_lock();

#if !defined(CONFIG_INIT_FLASH_FOR_DEBUG)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
#if defined(CONFIG_FLASH)
    flash1.reg = (void *)SEC_QSPI1_ADDR;
    flash1.dual_mode_only = false;
    flash1.continuous_mode_enable = false;
    flash1.writing = false;
    flash1.suspend_count = 0;
    flash1.continuous_mode_on = false;
    flash1.addr4b = DT_PROP(DT_NODELABEL(qspi1), addr4b);
    qspiv2_global_int_ctrl_fn_init();
    if (!is_app_cpu_running()) {
        lscache_cache_enable(1);
    }
#endif
#endif

#else /* !CONFIG_INIT_FLASH_FOR_DEBUG */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (!is_app_cpu_running()) {
        lsqspiv2_msp_init((reg_lsqspiv2_t *)SEC_QSPI1_ADDR);
        pinmux_hal_flash_quad_init();
        flash1.reg = (void *)SEC_QSPI1_ADDR;
        flash1.dual_mode_only = false;
        flash1.continuous_mode_enable = false;
        flash1.writing = false;
        flash1.suspend_count = 0;
        flash1.continuous_mode_on = false;
        flash1.addr4b = DT_PROP(DT_NODELABEL(qspi1), addr4b);
        hal_flash_init();

        lscache_cache_enable(1);
    }
#endif

#if defined(CONFIG_SOC_FLASH_LS)
#if !defined(CONFIG_CPU2_BOOT_ADDR) && !defined(CONFIG_XIP)
    hal_flash_init();
#else
    qspiv2_global_int_ctrl_fn_init();
#endif

    flash_swint_init();

#if !defined(CONFIG_CPU2_BOOT_ADDR) && !defined(CONFIG_XIP)
    hal_flash_xip_mode_reset();
#endif
#endif
#endif /* !CONFIG_INIT_FLASH_FOR_DEBUG */

#if defined(CONFIG_PECI)
    sys_write32(0x0, APP_PMU_RG_APP_ADDR + 0x3e8);
#endif

#if defined(CONFIG_PSRAM)
    if (!is_app_cpu_running()) {
        psram_init();
    }
#endif

    return;
}

__maybe_unused
static void boot_cpu2()
{
    if (is_app_cpu_running()) {
        return;
    }
    app_cpu_reset();
#if (CONFIG_IMAGE_HEADER) \
    && (CONFIG_CPU2_LOAD_ADDR >= CACHE1_ADDR) \
    && (CONFIG_CPU2_LOAD_ADDR < (CACHE1_ADDR + (64 << 20)))

    image_header_t image_header = {};
    flash_read(flash_dev, CONFIG_CPU2_LOAD_ADDR - CONFIG_FLASH_BASE_ADDRESS, &image_header, sizeof(image_header_t));

    if (image_header.test_word[0] != TEST_WORD0 || image_header.test_word[1] != TEST_WORD1)
        return;
    // LOG_I("\t test_word pass");

    uint32_t crc = crc32_ieee((uint8_t *)&image_header, sizeof(image_header_t) - sizeof(uint32_t));
    if (crc != image_header.header_crc)
        return;
    // LOG_I("\t header_crc pass");

    uint32_t exe_addr = 0;
    if (image_header.exe_addr == 0x0) {
        exe_addr = CONFIG_CPU2_LOAD_ADDR + image_header.offset;
    } else {
        exe_addr = image_header.exe_addr;
        flash_read(flash_dev,
                (CONFIG_CPU2_LOAD_ADDR - CONFIG_FLASH_BASE_ADDRESS) + image_header.offset + LSQSPIV2->BACKUP_OFFSET,
                (uint8_t *)image_header.exe_addr, image_header.length);
    }

    app_cpu_dereset_by_addr(exe_addr);
    app_cpu_reset_hold_clr();
#else
#if (DT_REG_SIZE(DT_CHOSEN(zephyr_internal_flash)) > (16 << 20))
    if (CONFIG_CPU2_BOOT_ADDR < SRAM1_ADDR) {
        if (1) {
            printk("boot a_app_image_partition\n");
            hal_flashx_write_ear(&flash1, 0x0);
            uint8_t ear = hal_flashx_read_ear(&flash1);
            if (0x0 != ear) {
                printk("hal_flashx_write_ear err\n");
                while(1);
            }
        } else {
            printk("boot b_app_image_partition_offset\n");
            hal_flashx_write_ear(&flash1, 0x1);
            uint8_t ear = hal_flashx_read_ear(&flash1);
            if (0x1 != ear) {
                printk("hal_flashx_write_ear err\n");
                while(1);
            }
            const uint32_t a_app_image_partition_offset = FIXED_PARTITION_OFFSET(a_app_image_partition);
            const uint32_t b_app_image_partition_offset = FIXED_PARTITION_OFFSET(b_app_image_partition) % MB(16);
            const int32_t offset = b_app_image_partition_offset - a_app_image_partition_offset;
            __ASSERT_NO_MSG(offset >= 0);
            if (0 != offset) {
                if (0 == (offset % KB(16))) {
                    LSQSPIV2->BACKUP_OFFSET = offset >> 14;
                } else {
                    // while(1);
                    printk("0 != ((b_app_image_partition_offset - a_app_image_partition_offset) %% 16KB)\n");
                }
            }
        }
    }
#endif
    app_cpu_dereset_by_addr(CONFIG_CPU2_BOOT_ADDR);
    app_cpu_reset_hold_clr();
#endif /* CONFIG_IMAGE_HEADER */
}

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
void soc_late_init_hook(void)
{
    HAL_IWDG_DeInit(SEC_IWDG);
    SEC_PMU->SFT_CTRL[2] &= ~0xf;
#if defined(CONFIG_BOOT_CPU2)
    boot_cpu2();
#endif /* CONFIG_BOOT_CPU2 */
}
#else
void soc_late_init_hook(void)
{
}
#endif
