#include <zephyr/init.h>
#include <zephyr/platform/hooks.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>
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
#include "reg_app_pmu_rg.h"
#include "reg_sysc_sec_awo.h"
#include "reg_sysc_app_awo.h"
#include "reg_sysc_sec_cpu.h"
#include "ls_hal_iwdgv2.h"
#include "ls_soc_gpio.h"
#include "ls_hal_cache.h"
#include "ls_hal_qspiv2.h"
#include "ls_msp_qspiv2.h"
#include "soc.h"
#include "soc_reset.h"
#include "soc_boot.h"
#include "otbn/otbn_mbox.h"

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#define MHINT_AEE_POS 20
#define SFT_CTRL_REG_NUM_RESET_FLAG (0x2)
#define FLASH_XIP_MODE_RESET_BIT     (4)
BUILD_ASSERT(CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU, "CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU");
BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);
BUILD_ASSERT(CONFIG_FLASH);
BUILD_ASSERT(DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash_controller)));
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
        __ASSERT_NO_MSG(0 == (__nocache_ram_size % CONFIG_SYSMAP_GRANULARITY));
        if (__image_ram_start != __nocache_ram_start) {
            csi_sysmap_config_region(idx++, __nocache_ram_start, CACHEABLE | BUFFERABLE);
        }
        csi_sysmap_config_region(idx++, __nocache_ram_end, WEAK_ORDER);
    }
#endif

    csi_sysmap_config_region(idx++, __image_ram_end, CACHEABLE | BUFFERABLE);
#if DT_NODE_EXISTS(DT_NODELABEL(psram))
    csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_NODELABEL(psram)), WEAK_ORDER); /* 8MB PSRAM */
    csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram))), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */
#endif

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
    if (((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE1_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE2_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE2_ADDR + QSPI_CACHE_SIZE)))) {
        csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), WEAK_ORDER);
        csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + DT_REG_SIZE(DT_CHOSEN(zephyr_flash))), CACHEABLE);
    }
#endif

    csi_sysmap_config_region(idx++, __image_ram_start, WEAK_ORDER);

#if defined(CONFIG_NOCACHE_MEMORY)
    if ((__nocache_ram_size > 0) && (__nocache_ram_size < __image_ram_size)) {
        __ASSERT_NO_MSG(0 == (__nocache_ram_size % CONFIG_SYSMAP_GRANULARITY));
        if (__image_ram_start != __nocache_ram_start) {
            csi_sysmap_config_region(idx++, __nocache_ram_start, CACHEABLE | BUFFERABLE);
        }
        csi_sysmap_config_region(idx++, __nocache_ram_end, WEAK_ORDER);
    }
#endif

    csi_sysmap_config_region(idx++, __image_ram_end, CACHEABLE | BUFFERABLE);
#if DT_NODE_EXISTS(DT_NODELABEL(psram))
    csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_NODELABEL(psram)), WEAK_ORDER); /* 8MB PSRAM */
    csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram))), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */
#endif
    if (idx < 8) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
}

enum iopmp_channel {
    IOPMP_APP_CPUI_APP_CPUD,
    IOPMP_APP_CPUS,
    IOPMP_DMAC1_ETH1_EMMC1,
    IOPMP_DMAC2_ETH2_EMMC2,
    IOPMP_USB2_SHA512_LTPI,
};

#define IOPMP_DMA_CHANNEL_MIN IOPMP_DMAC1_ETH1_EMMC1
#define IOPMP_DMA_CHANNEL_MAX IOPMP_USB2_SHA512_LTPI

void iopmp_region_init(void)
{
    uint32_t dev;
    uint32_t chn;
    uint32_t idx;

    ls_clock_control_off(IOPMP_CLOCK);
    ls_reset_line_toggle(IOPMP_RESET);
    ls_clock_control_on(IOPMP_CLOCK);

    chn = IOPMP_APP_CPUI_APP_CPUD;
    idx = 0;
    dev = SEC_IOPMP1_ADDR + (chn * 0x400);
    iopmp_config_region_napot(dev, idx++, 0x1000000, KB(64), false, false, false, false);
    iopmp_config_region_napot(dev, idx++, 0x10000000 + KB(768), KB(256), false, false, false, false);
    iopmp_config_region_napot(dev, idx++, 0x10000000 + KB(768) + KB(256), KB(256), false, false, false, false);
    iopmp_config_region_napot(dev, idx++, 0x0, (uint64_t)4 * GB(1), true, true, true, false);

    iopmp_config_enable(dev, true);

    chn = IOPMP_APP_CPUS;
    idx = 0;
    dev = SEC_IOPMP1_ADDR + (chn * 0x400);
    iopmp_config_region_napot(dev, idx++, 0x40002800, KB(1), true, true, true, false);    /* calc_sha  0x40002800 0x40002BFF 1K */
    iopmp_config_region_napot(dev, idx++, 0x40004000, KB(16), true, true, true, false);   /* nist_trng 0x40004000 0x40004FFF 4K
                                                                                                                                    sha512    0x40005000 0x40005FFF 4K
                                                                                                                                    otfad_aes 0x40006000 0x40006FFF 4K
                                                                                                                                    nouse     0x40007000 0x40007FFF 4K */
    iopmp_config_region_napot(dev, idx++, 0x40022000 + 0x28, 4, true, true, true, false); /* sec_cpu_intr */
    iopmp_config_region_napot(dev, idx++, 0x40029000, KB(2), true, true, true, false);    /* calc_aes  0x40029000 0x400293FF 1K
                                                                                                                                    calc_sm4  0x40029400 0x400297FF 1K */
#if defined(CONFIG_IOPMP_WHITELIST_I2C1_I3C1)
    iopmp_config_region_napot(dev, idx++, 0x400a0000, KB(4), true, true, true, false);                                          /* i2c1      0x400A0000 0x400A03FF 1K
                                                                                                                                    nouse     0x400A0400 0x400A07FF 1K
                                                                                                                                    i3c1      0x400A0800 0x400A0BFF 1K
                                                                                                                                    nouse     0x400A0C00 0x400A0FFF 1K */
#endif
    iopmp_config_region_napot(dev, idx++, 0x40000000, KB(256), false, false, false, false);
    iopmp_config_region_napot(dev, idx++, 0x400a0000, KB(32), false, false, false, false);

    iopmp_config_region_napot(dev, idx++, 0x0, (uint64_t)4 * GB(1), true, true, true, false);

    iopmp_config_enable(dev, true);

#if defined(CONFIG_IOPMP_DMA)
    for (chn = IOPMP_DMA_CHANNEL_MIN; chn <= IOPMP_DMA_CHANNEL_MAX; chn++) {
        idx = 0;
        dev = SEC_IOPMP1_ADDR + (chn * 0x400);
        iopmp_config_region_napot(dev, idx++, 0x10000000 + KB(768), KB(256), false, false, false, false);
        iopmp_config_region_napot(dev, idx++, 0x10000000 + KB(768) + KB(256), KB(256), false, false, false, false);

        iopmp_config_region_napot(dev, idx++, 0x40002800, KB(1), true, true, true, false);    /* calc_sha  0x40002800 0x40002BFF 1K */
        iopmp_config_region_napot(dev, idx++, 0x40004000, KB(16), true, true, true, false);   /* nist_trng 0x40004000 0x40004FFF 4K
                                                                                                  sha512    0x40005000 0x40005FFF 4K
                                                                                                  otfad_aes 0x40006000 0x40006FFF 4K
                                                                                                  nouse     0x40007000 0x40007FFF 4K */
        iopmp_config_region_napot(dev, idx++, 0x40029000, KB(2), true, true, true, false);    /* calc_aes  0x40029000 0x400293FF 1K
                                                                                                  calc_sm4  0x40029400 0x400297FF 1K */
        iopmp_config_region_napot(dev, idx++, 0x40000000, KB(256), false, false, false, false);
        iopmp_config_region_napot(dev, idx++, 0x400a0000, KB(32), false, false, false, false);

        iopmp_config_region_napot(dev, idx++, 0x0, (uint64_t)4 * GB(1), true, true, true, false);

        iopmp_config_enable(dev, true);
    }
#endif
}

extern void SWINT_Handler_ASM(void);
extern void SystemInit();
extern void psram_init(void);

__maybe_unused __ramfunc static void enable_dpll()
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

__maybe_unused __ramfunc static void cpu_600M_ahb_300M_qspi_200M_init()
{
    LSCACHE->CCR = FIELD_BUILD(LSCACHE_EN, 0);
    MODIFY_REG(LSQSPIV2->QSPI_CTRL1,LSQSPIV2_MODE_DAC_MASK|LSQSPIV2_CAP_DLY_MASK|LSQSPIV2_CAP_NEG_MASK,
                1<<LSQSPIV2_MODE_DAC_POS|QSPI_CAPTURE_DELAY<<LSQSPIV2_CAP_DLY_POS|QSPI_CAPTURE_NEG<<LSQSPIV2_CAP_NEG_POS);
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
    lscache_cache_enable(1);
}

__maybe_unused static void peripheral_init()
{
    /* SYSC_APP_AWO->PD_AWO_CLK_CTRL1 */
#if DT_NODE_EXISTS(DT_NODELABEL(psram))
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_PSRAM, 0x10); /* dpll_600M */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_PSRAM_FLT, 0x2); /* bypass */
#endif
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_USB2, 0x4); /* dpll_48M */
    REG_FIELD_WR(SYSC_APP_AWO->PD_AWO_CLK_CTRL1, SYSC_APP_AWO_CLK_SEL_I3C, 0x2); /* clk_pbus4: 0x4  dpll: 0x2 */
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
    /* SYSC_APP_AWO->ETH1_CLK_CFG */

    /* SYSC_APP_AWO->ETH2_CLK_CFG */
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_TX_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_DIV, 0); /* div = 2 */
    REG_FIELD_WR(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_SEL, 0x2); /* rxck pad */
    SET_BIT(SYSC_APP_AWO->ETH2_CLK_CFG, SYSC_APP_AWO_ETH2_CLK_RX_CG_MASK);
    /* SYSC_APP_AWO->ETH2_CLK_CFG */


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
    CLEAR_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_DIV, 0);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_SEL, 0x2); /* dpll 200M */
    SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_CG_MASK);

    CLEAR_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_CG_MASK);
    SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_DIV_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_SEL, 0x1); /* hsi */
    SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_TIM_CG_MASK);
    /* SYSC_APP_AWO->EMMC1_CORE_TIM_CLK */


    /* SYSC_APP_CPU->ETH1_PHY_CTRL */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_PHY_INTF_SEL, 0x1); /* rgmii */
    REG_FIELD_WR(SYSC_APP_CPU->ETH1_PHY_CTRL, SYSC_APP_CPU_ETH1_PHY_SEL, 0x1); /* rgmii */
    /* SYSC_APP_CPU->ETH1_PHY_CTRL */

    /* SYSC_APP_CPU->ETH2_PHY_CTRL */
    REG_FIELD_WR(SYSC_APP_CPU->ETH2_PHY_CTRL, SYSC_APP_CPU_ETH2_PHY_INTF_SEL, 0x1); /* rgmii */
    REG_FIELD_WR(SYSC_APP_CPU->ETH2_PHY_CTRL, SYSC_APP_CPU_ETH2_PHY_SEL, 0x1); /* rgmii */
    /* SYSC_APP_CPU->ETH2_PHY_CTRL */


    /* SYSC_APP_AWO->EMMC2_CORE_TIM_CLK */
    CLEAR_BIT(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_CG_MASK);
    REG_FIELD_WR(SYSC_APP_AWO->EMMC2_CORE_TIM_CLK, SYSC_APP_AWO_EMMC2_CLK_CORE_DIV, 0);
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

    APP_PMU->PECI_PAD_CFG.PD_PU |= 0x1 << 16;
    APP_PMU->PECI_PAD_CFG.DS_IEN &= ~(0x1);
    APP_PMU->PECI_PAD_CFG.PD_PU |= 0x2 << 16;
    APP_PMU->PECI_PAD_CFG.DS_IEN &= ~(0x2);

    ls_clock_control_off(CALC_SHA_CLOCK);
    ls_reset_line_toggle(CALC_SHA_RESET);
    ls_clock_control_on(CALC_SHA_CLOCK);

    ls_clock_control_off(SHA512_CLOCK);
    ls_reset_line_toggle(SHA512_RESET);
    ls_clock_control_on(SHA512_CLOCK);

    ls_clock_control_off(CALC_SM4_CLOCK);
    ls_reset_line_toggle(CALC_SM4_RESET);
    ls_clock_control_on(CALC_SM4_CLOCK);

    ls_clock_control_off(CRYPT_CLOCK);
    ls_reset_line_toggle(CRYPT_RESET);
    ls_clock_control_on(CRYPT_CLOCK);

    ls_clock_control_off(OTFAD_AES_CLOCK);
    ls_reset_line_toggle(OTFAD_AES_RESET);
    ls_clock_control_on(OTFAD_AES_CLOCK);

    ls_clock_control_off(NIST_TRNG_CLOCK);
    ls_reset_line_toggle(NIST_TRNG_RESET);
    ls_clock_control_on(NIST_TRNG_CLOCK);

#if defined(CONFIG_IOPMP_WHITELIST_I2C1_I3C1)
    ls_clock_control_off(I2C1_CLOCK);
    ls_reset_line_toggle(I2C1_RESET);
    ls_clock_control_on(I2C1_CLOCK);

    ls_clock_control_off(I3C1_CLOCK);
    ls_reset_line_toggle(I3C1_RESET);
    ls_clock_control_on(I3C1_CLOCK);
#endif
}

__maybe_unused void lsqsh_emmc_txck_rxck_config(uint32_t dev, uint32_t base_clock, uint32_t target_clock)
{
    ARG_UNUSED(base_clock);

    __ASSERT_NO_MSG(target_clock);

    uint16_t tx_div;
    uint16_t rx_div;
    uint8_t tx_sel;
    uint8_t rx_sel;

    if (target_clock >= (MHZ(200) / ((SYSC_APP_AWO_EMMC1_CLK_RX_DIV_MASK >> SYSC_APP_AWO_EMMC1_CLK_RX_DIV_POS) + 1))) {
        /* dpll 200M */
        tx_sel = 0x4;
        rx_sel = 0x2;
        tx_div = MHZ(200) / target_clock;
        if (tx_div) {
            tx_div--;
        }
        while ((MHZ(200) / (tx_div + 1)) > target_clock) {
            tx_div++;
        }
        rx_div = tx_div;
        LOG_DBG("target_clock: %d  div: %d", target_clock, tx_div);
        LOG_DBG("real_clock: %d", base_clock / (tx_div + 1));
    } else {
        /* dpll 50M */
        tx_sel = 0x2;
        rx_sel = 0x4;
        tx_div = MHZ(50) / target_clock;
        rx_div = 0;
    }

    if (dev == APP_EMMC1_CFG_ADDR) {
        /* SYSC_APP_AWO->EMMC1_CORE_TIM_CLK */
        CLEAR_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_CG_MASK);
        int core_clk_sel;
        int core_clk_div;
        core_clk_div = MHZ(200) / target_clock;
        if (core_clk_div) {
            core_clk_div--;
        }
        core_clk_sel = 0x2;
        if (core_clk_div > (SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_MASK >> SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_POS)) {
            core_clk_div = MHZ(25) / target_clock;
            if (core_clk_div) {
                core_clk_div--;
            }
            core_clk_sel = 0x1;
        }
        if (core_clk_div > (SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_MASK >> SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_POS)) {
            core_clk_div = SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_MASK >> SYSC_APP_AWO_EMMC1_CLK_CORE_DIV_POS;
        }

        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_DIV, core_clk_div);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_SEL, 0x2); /* dpll 200M */
        SET_BIT(SYSC_APP_AWO->EMMC1_CORE_TIM_CLK, SYSC_APP_AWO_EMMC1_CLK_CORE_CG_MASK);

        /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
        CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_DIV, tx_div);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_SEL, tx_sel);
        SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_TX_CG_MASK);

        CLEAR_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_DIV, rx_div);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_SEL, rx_sel);
        SET_BIT(SYSC_APP_AWO->EMMC1_TX_RX_CLK, SYSC_APP_AWO_EMMC1_CLK_RX_CG_MASK);
        /* SYSC_APP_AWO->EMMC1_TX_RX_CLK */
    } else if (dev == APP_EMMC2_CFG_ADDR) {
        /* SYSC_APP_AWO->EMMC2_TX_RX_CLK */
        CLEAR_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_CG_MASK);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_DIV, tx_div);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_SEL, tx_sel);
        SET_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_TX_CG_MASK);

        CLEAR_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_CG_MASK);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_DIV, rx_div);
        REG_FIELD_WR(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_SEL, rx_sel);
        SET_BIT(SYSC_APP_AWO->EMMC2_TX_RX_CLK, SYSC_APP_AWO_EMMC2_CLK_RX_CG_MASK);
        /* SYSC_APP_AWO->EMMC2_TX_RX_CLK */
    } else {
        while(1);
    }
}

void soc_prep_hook(void)
{
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    cpu1_cache_region_init();
#else
    cpu2_cache_region_init();
#endif
}

void soc_early_init_hook(void)
{
    uint32_t value = __get_MSTATUS();
    MODIFY_REG(value, 0x6000, 0x2000);
    __set_MSTATUS(value);//enable fpu
    value = __get_MHCR();
    value |= (CACHE_MHCR_RS_Msk | CACHE_MHCR_BPE_Msk | CACHE_MHCR_L0BTB_Msk);
    __set_MHCR(value);

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

    for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
        irq_disable(irq);
    }



#if !defined(CONFIG_FORCE_CLOCK_HSI)
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    if (!is_app_cpu_running()) {
        if ((0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL1_LOCK_MASK))
            && (0 == READ_BIT(SYSC_SEC_AWO->DPLL_LOCK, SYSC_SEC_AWO_DPLL2_LOCK_MASK))) {
            enable_dpll();
            cpu_600M_ahb_300M_qspi_200M_init();
        }
        peripheral_init();
    }
#endif
#endif /* CONFIG_FORCE_CLOCK_HSI */

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

    reset_reason_init();

    if ((PWR_FULL_RESET == reset_reason_get())
        || (SOFT_FULL_RESET == reset_reason_get())
        || (CPU_FULL_RESET == reset_reason_get())
        || (SYS_IWDT_FULL_RESET == reset_reason_get())
        || (EXT_FULL_RESET == reset_reason_get())) {
        memset((void *)DT_REG_ADDR(DT_NODELABEL(mbox)), 0, DT_REG_SIZE(DT_NODELABEL(mbox)));
    }

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)) && defined(CONFIG_IOPMP)
    iopmp_region_init();
#endif

    cpu_sleep_mode_config(0);
    driver_init();
    arch_irq_lock();

#if defined(CONFIG_PECI)
    sys_write32(0x0, APP_PMU_RG_APP_ADDR + 0x3e8);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    SET_BIT(SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG], BIT(FLASH_XIP_MODE_RESET_BIT));
#if defined(CONFIG_PSRAM)
    if (!is_app_cpu_running()) {
        psram_init();
    }
#endif
#endif

    return;
}

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
extern uint8_t flash_ls_read_ear(const struct device *dev);
extern uint8_t flash_ls_write_ear(const struct device *dev, uint8_t ear);
extern struct hal_flash_env *flash_ls_env(const struct device *dev);

int flash_xip_prepare(const struct device *flash_dev)
{
    flash_ex_op(flash_dev,FLASH_DRIVER_CLIENT_XIP_ACTIVE,0,NULL);
    if (!is_app_cpu_running()) {
        lscache_cache_disable();
        lscache_cache_enable(1);
    }

    return 0;
}

__maybe_unused static uint32_t cpu2_exe_addr;
__maybe_unused static bool is_app_cpu_xip_in_sec_flash(void)
{
#if defined(CONFIG_CPU2_IMAGE_HEADER)
    return (cpu2_exe_addr >= CACHE1_ADDR) && (cpu2_exe_addr < (CACHE1_ADDR + QSPI_CACHE_SIZE));
#else
    return (CONFIG_CPU2_BOOT_ADDR >= CACHE1_ADDR) && (CONFIG_CPU2_BOOT_ADDR < (CACHE1_ADDR + QSPI_CACHE_SIZE));
#endif
}

__maybe_unused static int flash_read_cpu2_image(const struct device *flash_dev, uint32_t cpu2_boot_addr, uint32_t *addr)
{
    image_header_t image_header = {};
    flash_read(flash_dev, cpu2_boot_addr - CACHE1_ADDR, &image_header, sizeof(image_header_t));

    if (image_header.test_word[0] != TEST_WORD0 || image_header.test_word[1] != TEST_WORD1) {
        LOG_ERR("test_word fail");
        return -EINVAL;
    }

    uint32_t crc = crc32_ieee((uint8_t *)&image_header, sizeof(image_header_t) - sizeof(uint32_t));
    if (crc != image_header.header_crc) {
        LOG_ERR("header_crc fail");
        return -EINVAL;
    }

    if ((image_header.exe_addr >= SRAM1_ADDR)
        && (image_header.exe_addr < (SRAM1_ADDR + SRAM_SIZE))) {
        /* copy zephyr.bin from flash to ram */
        flash_read(flash_dev,
                (cpu2_boot_addr - CACHE1_ADDR) + image_header.offset,
                (uint8_t *)image_header.exe_addr, image_header.length);
        sys_cache_data_flush_range((void *)image_header.exe_addr, image_header.length);
    }

    *addr = image_header.exe_addr;
    cpu2_exe_addr = image_header.exe_addr;

    if (((*addr >= CACHE1_ADDR) && (*addr < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((*addr >= CACHE2_ADDR) && (*addr < (CACHE2_ADDR + QSPI_CACHE_SIZE)))
        || ((*addr >= SRAM1_ADDR) && (*addr < (SRAM1_ADDR + SRAM_SIZE)))) {
        return 0;
    } else {
        return -EINVAL;
    }
}

__maybe_unused static int flash_ear_offset_set(const struct device *flash_dev, uint32_t addr)
{
    struct hal_flash_env *env = flash_ls_env(flash_dev);
    if ((addr - CACHE1_ADDR) < MB(16)) {
        LOG_INF("boot a_app_image_partition");
        flash_ls_write_ear(flash_dev, 0);
        uint8_t ear = flash_ls_read_ear(flash_dev);
        if (0x0 != ear) {
            LOG_ERR("flash_ls_write_ear err");
            while(1);
        }
        int ret = lsqspiv2_backup_offset_set((reg_lsqspiv2_t *)env->reg, 0);
        if (ret) {
            LOG_ERR("lsqspiv2_backup_offset_set err: offset: %#x", 0);
            return ret;
        }
    } else {
        LOG_INF("boot b_app_image_partition");
        flash_ls_write_ear(flash_dev, 0x1);
        uint8_t ear = flash_ls_read_ear(flash_dev);
        if (0x1 != ear) {
            LOG_ERR("flash_ls_write_ear err");
            while(1);
        }
        const uint32_t a_app_image_partition_offset = FIXED_PARTITION_OFFSET(a_app_image_partition);
        const uint32_t b_app_image_partition_offset = FIXED_PARTITION_OFFSET(b_app_image_partition) % MB(16);
        const int32_t offset = b_app_image_partition_offset - a_app_image_partition_offset;
        int ret = lsqspiv2_backup_offset_set((reg_lsqspiv2_t *)env->reg, offset);
        if (ret) {
            LOG_ERR("lsqspiv2_backup_offset_set err: offset: %#x", offset);
            return ret;
        }
    }

    return 0;
}

#define LS_FLASH_CONTROLLER_CHILD(node_id) IF_ENABLED(DT_NODE_HAS_COMPAT(node_id, soc_nv_flash), (DT_REG_SIZE(node_id)))
#define ZEPHYR_INTERNAL_FLASH_SIZE         DT_FOREACH_CHILD_STATUS_OKAY(DT_CHOSEN(zephyr_flash_controller), LS_FLASH_CONTROLLER_CHILD)

__maybe_unused int boot_cpu2(const struct device *flash_dev, uint32_t cpu2_boot_addr)
{
    app_cpu_reset();
    flash_ex_op(flash_dev,FLASH_DRIVER_CLIENT_XIP_INACTIVE,0,NULL);
#if (CONFIG_CPU2_IMAGE_HEADER)
    uint32_t exe_addr;
    if (((cpu2_boot_addr >= CACHE1_ADDR) && (cpu2_boot_addr < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((cpu2_boot_addr >= CACHE2_ADDR) && (cpu2_boot_addr < (CACHE2_ADDR + QSPI_CACHE_SIZE)))) {
        int ret = flash_read_cpu2_image(flash_dev, cpu2_boot_addr, &exe_addr);
        if (ret) {
            return ret;
        }
    } else {
        __ASSERT_NO_MSG(0);
    }
#else
    uint32_t exe_addr = cpu2_boot_addr;
#endif /* CONFIG_CPU2_IMAGE_HEADER */

    if (is_app_cpu_xip_in_sec_flash()) {
#if (ZEPHYR_INTERNAL_FLASH_SIZE > (16 << 20))
        /* do not need to calculate offset cause it is fixed */
        flash_ear_offset_set(flash_dev, cpu2_boot_addr);
#endif
        flash_xip_prepare(flash_dev);
    }
    LOG_INF("%s: address: %#x", __func__, exe_addr);
    app_cpu_dereset_by_addr(exe_addr);
    app_cpu_reset_hold_clr();

    return 0;
}


#define STARTUP_PART_FLAG_MASK               (0xf)
#define SFT_CTRL_REG_NUM_BOOT_RAM_RESET_FLAG (0x5)
#define BOOTRAM_STARTUP_PART_FLAG_MASK       (0xf)
#define BOOTRAM_STARTUP_PART_FLAG_POS        (0)

__maybe_unused void soc_late_init_hook(void)
{
    const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    struct hal_flash_env *env = flash_ls_env(flash_dev);
    REG_FIELD_WR(SEC_IWDG->IWDT_CTRL, IWDT_EN, 0);
    SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG] &= ~STARTUP_PART_FLAG_MASK;
    SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_BOOT_RAM_RESET_FLAG] &= ~BOOTRAM_STARTUP_PART_FLAG_MASK;
    if (env->continuous_mode_enable) {
        SET_BIT(SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG], BIT(FLASH_XIP_MODE_RESET_BIT));
    }

    if (is_app_cpu_running()) {
        flash_xip_prepare(flash_dev);
        return;
    } else {
        pinmux_hal_flash_quad_init();
    }
#if defined(CONFIG_BOOT_CPU2)
    if (((CONFIG_CPU2_BOOT_ADDR >= CACHE1_ADDR) && (CONFIG_CPU2_BOOT_ADDR < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((CONFIG_CPU2_BOOT_ADDR >= CACHE2_ADDR) && (CONFIG_CPU2_BOOT_ADDR < (CACHE2_ADDR + QSPI_CACHE_SIZE)))
        || ((CONFIG_CPU2_BOOT_ADDR >= SRAM1_ADDR) && (CONFIG_CPU2_BOOT_ADDR < (SRAM1_ADDR + SRAM_SIZE)))) {
        int ret = boot_cpu2(flash_dev, CONFIG_CPU2_BOOT_ADDR);
        if (ret) {
            __ASSERT_NO_MSG(0);
        }
    } else {
        LOG_ERR("invalid cpu2 boot address: %#x", CONFIG_CPU2_BOOT_ADDR);
        __ASSERT_NO_MSG(0);
    }
#endif /* CONFIG_BOOT_CPU2 */

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_SERVER)
    ls_otbn_delegation_server_chanels_init();
#endif
}
#else
void soc_late_init_hook(void)
{
#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
    ls_otbn_delegation_client_chanels_init();
#endif
}
#endif
