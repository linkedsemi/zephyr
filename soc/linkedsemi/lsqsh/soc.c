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
#include <zephyr/drivers/flash/soc_flash_ls_mbox_cpu1.h>
#include <zephyr/drivers/led/led_gpio_corelynx.h>
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
#include "reg_sysc_sec_cpu.h"
#include "ls_hal_iwdgv2.h"
#include "ls_soc_gpio.h"
#include "ls_hal_flash.h"
#include "ls_hal_cache.h"
#include "ls_msp_qspiv2.h"
#include "soc.h"
#include "soc_reset.h"
#include "soc_boot.h"

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
    global_reset_reason_clean();
#endif
    reset_reason_magic_set();
    reset_reason_set(HART_RESET);
    sys_cache_data_flush_all();
    csi_core_reset();
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

    csi_sysmap_config_region(idx++, PSRAM_ADDR + MB(64), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */

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

    csi_sysmap_config_region(idx++, PSRAM_ADDR + MB(64), CACHEABLE | BUFFERABLE); /* 8MB PSRAM */

    if (idx < 8) {
        csi_sysmap_config_region(idx++, 0xffffffff, STRONG_ORDER);
    }
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

void soc_early_init_hook(void)
{
    __set_MTVT((uint32_t)0);
    CLIC->CLICCFG = 0x7f;

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
    lsqspiv2_msp_init((reg_lsqspiv2_t *)SEC_QSPI1_ADDR);
    pinmux_hal_flash_init();
    flash1.reg = (void *)SEC_QSPI1_ADDR;
    flash1.dual_mode_only = false;
    flash1.continuous_mode_enable = false;
    flash1.writing = false;
    flash1.suspend_count = 0;
    flash1.continuous_mode_on = false;
    flash1.addr4b = DT_PROP(DT_NODELABEL(qspi1), addr4b);
    hal_flash_init();

    lscache_cache_enable(1);
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

#if defined(CONFIG_ETH_DRIVER)
    SYSC_APP_CPU->ETH1_PHY_CTRL = 0x9;
#endif

#if defined(CONFIG_PECI)
    sys_write32(0x0, APP_PMU_RG_APP_ADDR + 0x3e8);
#endif

#if defined(CONFIG_PSRAM)
    psram_init();
#endif

    return;
}

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
void soc_late_init_hook(void)
{
#if CONFIG_LED
    led_state_init();
#endif
    HAL_IWDG_DeInit(SEC_IWDG);
    SEC_PMU->SFT_CTRL[2] &= ~0xf;

#if defined(CONFIG_BOOT_CPU2)
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

    app_cpu_reset();
    __NOP();
    app_cpu_dereset_by_addr(exe_addr);
#else
    app_cpu_reset();
    __NOP();
#if (DT_REG_SIZE(DT_CHOSEN(zephyr_internal_flash)) > (16 << 20))
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
#endif
    app_cpu_dereset_by_addr(CONFIG_CPU2_BOOT_ADDR);
#endif /* (CONFIG_CPU2_LOAD_ADDR < 0x10000000) */
#endif /* defined(CONFIG_BOOT_CPU2) */
}
#else
void soc_late_init_hook(void)
{
}
#endif
