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
#include "platform.h"
#include "exception_isr.h"
#include "systick.h"
#include "cpu.h"
#include <stdint.h>
#include <string.h>
#include "iopmp.h"
#include "qsh.h"
#include <zephyr/irq.h>
#include "ls_hal_iwdgv2.h"
#include "ls_hal_cache.h"
#include "ls_hal_qspiv2.h"
#include "ls_msp_qspiv2.h"
#include "soc.h"
#include "soc_reset.h"
#include "soc_boot.h"
#include "otbn/ls_otbn_config.h"

#if defined(CONFIG_SMP)
#include "smp/lsqsh_smp.h"
#endif

LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

#define MHINT_AEE_POS 20
#define SFT_CTRL_REG_NUM_RESET_FLAG (0x2)
BUILD_ASSERT(CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU, "CONFIG_NUM_OS <= CONFIG_NUM_USE_CPU");
BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);
BUILD_ASSERT(CONFIG_FLASH);
BUILD_ASSERT(DT_NODE_EXISTS(DT_CHOSEN(zephyr_flash_controller)));
#if defined(CONFIG_CACHE)
IF_ENABLED(CONFIG_DCACHE, (BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE_DETECT)));
IF_ENABLED(CONFIG_DCACHE, (BUILD_ASSERT(CONFIG_DCACHE_LINE_SIZE > 0)));
#endif

void cpu_sleep_mode_config(uint8_t deep)
{
    uint32_t mextstaus = __get_MEXSTATUS();
    MODIFY_REG(mextstaus,MEXSTATUS_SLEEP_Msk,(!deep)<<MEXSTATUS_SLEEP_Pos);
    __set_MEXSTATUS(mextstaus);
}

void systick_start(void){};
void sw_timer_module_init(void){};

#define SYSMAP_REGION_MAX 8
/* strong order | cacheable | bufferable */
/*       2      |     1     |     0      */
#define WEAK_ORDER 0
#define BUFFERABLE SYSMAP_SYSMAPCFG_B_Msk
#define CACHEABLE SYSMAP_SYSMAPCFG_C_Msk
#define STRONG_ORDER SYSMAP_SYSMAPCFG_SO_Msk

#define SYSMAP_PRINT(idx) LOG_INF("SYSMAPADDR%d: %#8.8x SYSMAPCFG%d: %#8.8x", \
                                    idx,                                      \
                                    SYSMAP->SYSMAPADDR##idx,                  \
                                    idx,                                      \
                                    SYSMAP->SYSMAPCFG##idx)

#define SYSMAP_ADDR_ATTR_PRINT(idx) LOG_INF("%d addr: %#8.8x attr: %c%c%c",                              \
                                                idx,                                                     \
                                                SYSMAP->SYSMAPADDR##idx << 12,                           \
                                                SYSMAP->SYSMAPCFG##idx & BUFFERABLE ? 'B' : '-',  \
                                                SYSMAP->SYSMAPCFG##idx & CACHEABLE ? 'C' : '-',   \
                                                SYSMAP->SYSMAPCFG##idx & STRONG_ORDER ? 'S' : '-')

typedef struct {
    volatile uint32_t SYSMAPADDR;             /*!< Offset: 0x000 (R/W)  SYSMAP configure register */
    volatile uint32_t SYSMAPCFG;              /*!< Offset: 0x004 (R/W)  SYSMAP configure register */
} SYSMAP_ITEM_Type;
#define SYSMAP_ITEM              ((SYSMAP_ITEM_Type  *)     SYSMAP_BASE )

void cpu_sysmap_show(void)
{
    SYSMAP_PRINT(0);
    SYSMAP_PRINT(1);
    SYSMAP_PRINT(2);
    SYSMAP_PRINT(3);
    SYSMAP_PRINT(4);
    SYSMAP_PRINT(5);
    SYSMAP_PRINT(6);
    SYSMAP_PRINT(7);
    SYSMAP_ADDR_ATTR_PRINT(0);
    SYSMAP_ADDR_ATTR_PRINT(1);
    SYSMAP_ADDR_ATTR_PRINT(2);
    SYSMAP_ADDR_ATTR_PRINT(3);
    SYSMAP_ADDR_ATTR_PRINT(4);
    SYSMAP_ADDR_ATTR_PRINT(5);
    SYSMAP_ADDR_ATTR_PRINT(6);
    SYSMAP_ADDR_ATTR_PRINT(7);
}

static void cpu_sysmap_erase(void)
{
    for (int i = 0; i < SYSMAP_REGION_MAX; i++) {
        SYSMAP_ITEM[i].SYSMAPADDR = 0;
        SYSMAP_ITEM[i].SYSMAPCFG = 0;
    }
}

static int cpu_sysmap_check(void)
{
    int ret = 0;

    for (int i = 1; i < SYSMAP_REGION_MAX; i++) {
        if (SYSMAP_ITEM[i - 1].SYSMAPADDR > SYSMAP_ITEM[i].SYSMAPADDR) {
            ret = -EINVAL;
            break;
        }
    }

    return ret;
}

__maybe_unused static void cpu1_cache_region_init(void)
{
    const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
    const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
    const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
    const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
    uint8_t idx = 0;

#if defined(CONFIG_XIP)
    if (!(((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE1_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE2_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE2_ADDR + QSPI_CACHE_SIZE))))) {
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

    while (idx < SYSMAP_REGION_MAX) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
}

__maybe_unused static void cpu2_cache_region_init(void)
{
    const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
    const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
    const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
    const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
    const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
    const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
    uint8_t idx = 0;

#if defined(CONFIG_XIP)
    if (!(((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE1_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
        || ((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE2_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE2_ADDR + QSPI_CACHE_SIZE))))) {
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
    while (idx < SYSMAP_REGION_MAX) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
}

__maybe_unused void smp_mode_cache_region_init(void)
{
//     const uint32_t __image_ram_start = (uint32_t)_image_ram_start;
//     const uint32_t __image_ram_end = (uint32_t)_image_ram_end;
//     const uint32_t __image_ram_size = (uint32_t)_image_ram_size;
//     const uint32_t __nocache_ram_start = (uint32_t)_nocache_ram_start;
//     const uint32_t __nocache_ram_end = (uint32_t)_nocache_ram_end;
//     const uint32_t __nocache_ram_size = (uint32_t)_nocache_ram_size;
//     uint8_t idx = 0;

// #if defined(CONFIG_XIP)
//     if (!(((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE1_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE1_ADDR + QSPI_CACHE_SIZE)))
//         || ((DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) >= CACHE2_ADDR) && (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) < (CACHE2_ADDR + QSPI_CACHE_SIZE))))) {
//         csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_CHOSEN(zephyr_flash)), WEAK_ORDER);
//         csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + DT_REG_SIZE(DT_CHOSEN(zephyr_flash))), CACHEABLE);
//     }
// #endif

//     csi_sysmap_config_region(idx++, __image_ram_start, WEAK_ORDER);

// #if defined(CONFIG_NOCACHE_MEMORY)
//     if ((__nocache_ram_size > 0) && (__nocache_ram_size < __image_ram_size)) {
//         __ASSERT_NO_MSG(0 == (__nocache_ram_size % CONFIG_SYSMAP_GRANULARITY));
//         if (__image_ram_start != __nocache_ram_start) {
//             csi_sysmap_config_region(idx++, __nocache_ram_start, CACHEABLE | BUFFERABLE);
//         }
//         csi_sysmap_config_region(idx++, __nocache_ram_end, WEAK_ORDER);
//     }
// #endif

//     csi_sysmap_config_region(idx++, __image_ram_end, CACHEABLE | BUFFERABLE);

// #if DT_NODE_EXISTS(DT_NODELABEL(psram))
//     csi_sysmap_config_region(idx++, DT_REG_ADDR(DT_NODELABEL(psram)), WEAK_ORDER); /* 8MB PSRAM */
//     csi_sysmap_config_region(idx++, (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram))), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */
// #endif

//     while (idx < SYSMAP_REGION_MAX) {
//         csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
//     }
}

extern void SystemInit();
extern void psram_init(void);

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
#define LS_FLASH_CONTROLLER_CHILD_FLASH_SIZE(node_id) \
    IF_ENABLED(DT_NODE_HAS_COMPAT(node_id, soc_nv_flash), (DT_REG_SIZE(node_id)))

static void peripheral_init()
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
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_TX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_RX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH1_CLK_RMII_RST_N_MASK);

    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_TX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_RX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_ETH2_CLK_RMII_RST_N_MASK);
    /* SYSC_APP_AWO->ETH_EMMC_RST */


    /* SYSC_APP_AWO->ETH_EMMC_RST */
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_TX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_RX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_CORE_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC1_CLK_TIM_RST_N_MASK);

    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_TX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_RX_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_CORE_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_EMMC2_CLK_TIM_RST_N_MASK);

    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_LPC1_CLK_RST_N_MASK);
    CLEAR_BIT(SYSC_APP_AWO->ETH_EMMC_RST, SYSC_APP_AWO_LPC2_CLK_RST_N_MASK);
    /* SYSC_APP_AWO->ETH_EMMC_RST */


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
    SET_BIT(SYSC_APP_AWO->LPC_CLK, SYSC_APP_AWO_LPC2_CLK_CG_MASK);
    /* SYSC_APP_AWO->LPC_CLK */

    SET_BIT(SEC_PMU->TRIM0, SEC_PMU_RG_LDO_PECI_EN_MASK);
    APP_PMU->PECI_PAD_CFG.PD_PU |= 0x1 << 16;
    APP_PMU->PECI_PAD_CFG.DS_IEN &= ~(0x1);
    APP_PMU->PECI_PAD_CFG.PD_PU |= 0x2 << 16;
    APP_PMU->PECI_PAD_CFG.DS_IEN &= ~(0x2);

    ls_clock_control_on_reset_line_toggle_once(CALC_SHA_CLOCK, CALC_SHA_RESET);
    ls_clock_control_on_reset_line_toggle_once(SHA512_CLOCK, SHA512_RESET);
    ls_clock_control_on_reset_line_toggle_once(CALC_SM4_CLOCK, CALC_SM4_RESET);
    ls_clock_control_on_reset_line_toggle_once(CRYPT_CLOCK, CRYPT_RESET);
    ls_clock_control_on_reset_line_toggle_once(OTFAD_AES_CLOCK, OTFAD_AES_RESET);
    ls_clock_control_on_reset_line_toggle_once(WWDT1_CLOCK, WWDT1_RESET);
    ls_clock_control_on_reset_line_toggle_once(NIST_TRNG_CLOCK, NIST_TRNG_RESET);
#if defined(CONFIG_IOPMP_WHITELIST_I2C1_I3C1)
    ls_clock_control_on_reset_line_toggle_once(I2C1_CLOCK, I2C1_RESET);
    ls_clock_control_on_reset_line_toggle_once(I3C1_CLOCK, I3C1_RESET);
#endif
}

__ramfunc static void high_frequency_init()
{
    LSCACHE->CCR = FIELD_BUILD(LSCACHE_EN, 0);
    dpll_qspi_clk_config_and_clk_switch();
    struct hal_flash_env env;
    env.reg = (void *)DT_REG_ADDR(DT_CHOSEN(zephyr_flash_controller));
    env.dual_mode_only = !DT_PROP(DT_CHOSEN(zephyr_flash_controller), quad_mode);
    env.continuous_mode_enable = DT_PROP(DT_CHOSEN(zephyr_flash_controller), continuous_mode);
    env.addr4b = (DT_FOREACH_CHILD_STATUS_OKAY(DT_CHOSEN(zephyr_flash_controller), LS_FLASH_CONTROLLER_CHILD_FLASH_SIZE) > (16 << 20));
    env.writing = false;
    env.continuous_mode_on = false;
    if (ls_clock_control_is_on(QSPI1_CLOCK)) {
        hal_flashx_noreset_init(&env);
    } else {
        hal_flashx_init(&env);
    }
    if (!env.dual_mode_only) {
        pinmux_hal_flash_quad_init();
    }
    hal_flashx_continuous_mode_reset(&env);
    hal_flashx_continuous_mode_start(&env);
    if (ls_clock_control_is_on(QSPI1_CLOCK)) {
        lscache_cachex_enable(LSCACHE, 1);
    }
    if (ls_clock_control_is_on(QSPI2_CLOCK)) {
        lscache_cachex_enable(LSCACHE2, 1);
    }
    peripheral_init();
}
#endif /*  DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay) */

__maybe_unused void lsqsh_emmc_txck_rxck_config(uint32_t dev, uint32_t base_clock, uint32_t target_clock)
{
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

__weak void soc_prep_hook(void)
{
#if defined(CONFIG_SMP)
    /* first cpu*/
    // smp_mode_cache_region_init();
#else
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    cpu1_cache_region_init();
#else
    cpu2_cache_region_init();
#endif
#endif
}

__weak void cpu_early_common_config(void)
{
    if (cpu_sysmap_check()) {
        LOG_ERR("cpu_sysmap_check failed, erase sysmap");
        cpu_sysmap_erase();
        cpu_sysmap_show();
    }

    uint32_t value = __get_MSTATUS();
    MODIFY_REG(value, 0x6000, 0x2000);
    __set_MSTATUS(value);//enable fpu
    value = __get_MHCR();
    value |= (CACHE_MHCR_WB_Msk | CACHE_MHCR_WA_Msk | CACHE_MHCR_RS_Msk | CACHE_MHCR_BPE_Msk | CACHE_MHCR_BTB_Msk);
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

}

void soc_early_init_hook(void)
{
    cpu_early_common_config();

#if !defined(CONFIG_FORCE_CLOCK_HSI)
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
    if (!is_app_cpu_running()) {
        high_frequency_init();
    }
#endif
#endif /* CONFIG_FORCE_CLOCK_HSI */

#if defined(CONFIG_CACHE)
#if defined(CONFIG_SMP)
    smp_mode_cache_config();
#else
#if defined(CONFIG_DCACHE)
    csi_dcache_enable();
#endif
#if defined(CONFIG_ICACHE)
    csi_icache_enable();
#endif
#endif
#endif

    reset_reason_init();

    cpu_sleep_mode_config(0);
    arch_irq_lock();

#if defined(CONFIG_PECI)
    sys_write32(0x0, APP_PMU_RG_APP_ADDR + 0x3e8);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
#if defined(CONFIG_MBOX)&&(!defined(CONFIG_SMP))
    if ((PWR_FULL_RESET == reset_reason_get())
        || (SOFT_FULL_RESET == reset_reason_get())
        || (CPU_FULL_RESET == reset_reason_get())
        || (SYS_IWDT_FULL_RESET == reset_reason_get())
        || (EXT_FULL_RESET == reset_reason_get())) {
        memset((void *)DT_REG_ADDR(DT_NODELABEL(mbox_memory)), 0, DT_REG_SIZE(DT_NODELABEL(mbox_memory)));
    }
#endif
    if (!is_app_cpu_running()) {
#if defined(CONFIG_PSRAM)
        psram_init();
#endif
    }
#endif

    return;
}

#if !defined(CONFIG_SMP)
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
extern uint8_t flash_ls_read_ear(const struct device *dev);
extern uint8_t flash_ls_write_ear(const struct device *dev, uint8_t ear);
extern struct hal_flash_env *flash_ls_env(const struct device *dev);

static int flash_xip_prepare(const struct device *flash_dev)
{
    flash_ex_op(flash_dev,FLASH_DRIVER_CLIENT_XIP_ACTIVE,0,NULL);

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

__weak void soc_late_init_hook(void)
{
    enum SEC_PMU_SFT_CTRL_RESET_FLAG_FIELD {
        SEC_PMU_SFT_CTRL_RESET_FLAG_STARTUP_PART_FLAG_MASK = (int)0xf,
        SEC_PMU_SFT_CTRL_RESET_FLAG_STARTUP_PART_FLAG_POS = 0,
        SEC_PMU_SFT_CTRL_RESET_FLAG_FLASH_XIP_MODE_RESET_BIT_MASK = (int)0x10,
        SEC_PMU_SFT_CTRL_RESET_FLAG_FLASH_XIP_MODE_RESET_BIT_POS = 4,
        SEC_PMU_SFT_CTRL_RESET_FLAG_LIFE_CYCLE_UPDATE_REQ_BIT_MASK = (int)0xe0,
        SEC_PMU_SFT_CTRL_RESET_FLAG_LIFE_CYCLE_UPDATE_REQ_BIT_POS = 5,
        SEC_PMU_SFT_CTRL_RESET_FLAG_LIFE_CYCLE_STATUS_OFFSET_MASK = (int)0xff00,
        SEC_PMU_SFT_CTRL_RESET_FLAG_LIFE_CYCLE_STATUS_OFFSET_POS = 8,
        SEC_PMU_SFT_CTRL_RESET_FLAG_BOOTRAM_STARTUP_PART_FLAG_MASK = (int)0xf0000,
        SEC_PMU_SFT_CTRL_RESET_FLAG_BOOTRAM_STARTUP_PART_FLAG_POS = 16,
    };
    const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    struct hal_flash_env *env = flash_ls_env(flash_dev);
    REG_FIELD_WR(SEC_IWDG->IWDT_CTRL, IWDT_EN, 0);
    CLEAR_BIT(SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG], SEC_PMU_SFT_CTRL_RESET_FLAG_STARTUP_PART_FLAG_MASK);
    CLEAR_BIT(SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG], SEC_PMU_SFT_CTRL_RESET_FLAG_BOOTRAM_STARTUP_PART_FLAG_MASK);
    if (env->continuous_mode_enable) {
        SET_BIT(SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG], SEC_PMU_SFT_CTRL_RESET_FLAG_FLASH_XIP_MODE_RESET_BIT_MASK);
    }

    if (is_app_cpu_running() && is_app_cpu_xip_in_sec_flash()) {
        flash_xip_prepare(flash_dev);
        return;
    } else {
        flash_ex_op(flash_dev,FLASH_DRIVER_CLIENT_XIP_INACTIVE,0,NULL);
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
__weak void soc_late_init_hook(void)
{
#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
    ls_otbn_delegation_client_chanels_init();
#endif
}
#endif /*(DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)) */
#endif /*!defined(CONFIG_SMP)*/

#if defined(CONFIG_LINKEDSEMI_TPM_WWDT)

#define LS_TPM_SPIS_DETECT_REG        0x40021018U
#define LS_TPM_SPIS_DETECT_SEL_BIT    BIT(20)
#define LS_TPM_SPIS_DETECT_ENABLE_BIT BIT(21)

#define LS_WWDT1_LOCK_REG      0x400A1D00U
#define LS_WWDT1_UNLOCK_VALUE  0x1ACCE551U
#define LS_WWDT1_TPM_RST_REG   0x400A1C20U
#define LS_WWDT1_TPM_RST_VALUE 0x20000000U
#define LS_WWDT1_TIMEOUT_REG   0x400A1C00U
#define LS_WWDT1_EN_REG        0x400A1C08U
#define LS_WWDT1_EN_VALUE      0xdU

int wwdt1_tpm_init(const struct device *tpm_spis_dev, uint32_t timeout_ms)
{
    uint32_t reg, ticks;
    if (tpm_spis_dev == NULL || !device_is_ready(tpm_spis_dev)) {
        return -ENODEV;
    }
    if (timeout_ms == 0U) {
        return -EINVAL;
    }
    ticks = timeout_ms * 32U;
    if (ticks == 0U) {
        ticks = 1U;
    }
    reg = sys_read32(LS_TPM_SPIS_DETECT_REG);
    reg |= LS_TPM_SPIS_DETECT_ENABLE_BIT;
    reg &= ~LS_TPM_SPIS_DETECT_SEL_BIT;//default tpmspi2

    /* Only compare DEVICE_DT_GET for nodes that are status "okay" in this build. */
    bool sel_matched = false;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(tpm_spis2), okay)
    if (tpm_spis_dev == DEVICE_DT_GET(DT_NODELABEL(tpm_spis2))) {
        // reg &= ~LS_TPM_SPIS_DETECT_SEL_BIT;
        sel_matched = true;
    }
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(tpm_spis1), okay)
    if (!sel_matched && tpm_spis_dev == DEVICE_DT_GET(DT_NODELABEL(tpm_spis1))) {
        reg |= LS_TPM_SPIS_DETECT_SEL_BIT;
        sel_matched = true;
    }
#endif
    if (!sel_matched) {
        return -EINVAL;
    }

    sys_write32(reg, LS_TPM_SPIS_DETECT_REG);
    sys_write32(LS_WWDT1_UNLOCK_VALUE, LS_WWDT1_LOCK_REG);
    sys_write32(LS_WWDT1_TPM_RST_VALUE, LS_WWDT1_TPM_RST_REG);
    sys_write32(ticks, LS_WWDT1_TIMEOUT_REG);
    sys_write32(LS_WWDT1_EN_VALUE, LS_WWDT1_EN_REG);
    return 0;
}

#endif /* CONFIG_LINKEDSEMI_TPM_WWDT */
