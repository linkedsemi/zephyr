#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/cache.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/drivers/flash.h>
#include <soc_reset.h>
#include <platform.h>
#include <ls_hal_flash.h>
#include <ls_hal_qspiv2.h>
#include <hal_flash_int.h>

extern struct hal_flash_env *flash_ls_env(const struct device *dev);

/* SEC_PMU reset-flag register: two 4-bit boot-attempt counters (see the boot
 * chain's boot_port.h). */
#ifndef SFT_CTRL_REG_NUM_RESET_FLAG
#define SFT_CTRL_REG_NUM_RESET_FLAG (0x2)
#endif
#define PMU_STARTUP_PART_FLAG_MASK        (0xF)
#define PMU_BOOTRAM_STARTUP_PART_FLAG_POS (16)

/* A/B slot bases (flash-relative), same layout as boot_ram's boot env and
 * code-mgt's fw_env.h. The XIP window maps app_a_partition_offset as the
 * linker base, so these two values identify the slot it is rebased onto. */
#define SOC_APP_A_OFFSET (0x0010000u)
#define SOC_APP_B_OFFSET (0x1110000u)

void sys_arch_reboot_warm_emul()
{
    irq_lock();
    for (int irq = 0; irq < CONFIG_NUM_IRQS; irq++) {
        irq_disable(irq);
    }
    reset_reason_magic_set();
    sys_cache_data_flush_all();
    sys_cache_data_disable();
    sys_cache_instr_disable();
    void (* goto_rom_region_start)();
    goto_rom_region_start = (void *)__rom_region_start;
    goto_rom_region_start();
}

void sys_arch_reboot(int type)
{
#if defined(CONFIG_OPENBMC_PHOSPHOR_BMC_CODE_MGT)
    /* OpenBMC A/B boot -- MUST run before anything that can reset the chip.
     *
     * Historically this is where the flash read window was handed back to slot
     * A in software (soc_reboot_cold_after_flash_window_reset()), because a
     * reset clears neither the chip's EAR nor the QSPI controller: the window
     * stayed programmed for the running slot, the boot ROM then read the SBL
     * image (flash offset 0) out of the wrong 16MB bank, failed its header
     * check four times and fell back to UART boot until power was removed
     * ("b -> a never boots"; a -> b was fine because an a run leaves the
     * window at 0/0 already).
     *
     * That hand-over is now done by the boot ROM instead: main() programs
     * startup_param.flash_reset_disable = 0 in OTP once (see
     * bmc_otp_program_flash_reset_disable() in the app), and the ROM resets
     * the window on every reset from then on -- a -> b and b -> a both
     * verified working with the software window reset skipped. OTP is one-way,
     * so this cannot be undone, which is why the log states the bit on every
     * boot.
     *
     * soc_reboot_cold_after_flash_window_reset() is kept below, behind
     * SOC_REBOOT_FLASH_WINDOW_RESET, as the software fallback: it costs a
     * couple of register writes and makes b -> a work even on a unit whose OTP
     * block refused the program (main() logs an explicit error in that case).
     * Enable it if the OTP state of a board can never be trusted. */
    soc_flash_window_log("pre-reboot");
#ifdef CONFIG_SOC_REBOOT_FLASH_WINDOW_RESET
    soc_reboot_cold_after_flash_window_reset();
#endif
#endif

    switch(type) {
    case SYS_REBOOT_COLD:
#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
        REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_FROM_SFT, 0x1);
#else
        printf("SYS_REBOOT_COLD is not supported\n");
#endif
        break;
    case SYS_REBOOT_WARM:
#if defined(CONFIG_EMUL_SOFT_RESET)
        sys_arch_reboot_warm_emul();
#endif
        break;
    default:
        printf("type: %d is not supported\n", type);
        break;
    };
    csi_core_reset();
}

/* Diagnostic: dump the state a warm reset would carry into the next boot.
 *
 * Worth logging right before a reboot: the boot chain loads the SBL image from
 * flash offset 0, so a leftover EAR/backup-offset from _this_ boot is exactly
 * what makes the *other* slot unbootable until the board loses power. The two
 * counters in SEC_PMU decide whether the chain still trusts the sbl_env or has
 * already fallen back (and they are only cleared by a power cycle).
 *
 * Call from normal (XIP) context -- never after the window has been cleared. */
void soc_flash_window_log(const char *tag)
{
    const struct device *flashDev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    struct hal_flash_env *env = flash_ls_env(flashDev);
    reg_lsqspiv2_t *qspi = (reg_lsqspiv2_t *)env->reg;
    uint32_t rstFlag = SEC_PMU->SFT_CTRL[SFT_CTRL_REG_NUM_RESET_FLAG];

    printk("flash window[%s]: ear=%u backup_offset=0x%x dac_ctrl=0x%08x "
           "dac_cmd=0x%08x qspi_ctrl1=0x%08x pmu_reset_flag=0x%08x "
           "(startup part=0x%x bootram part=0x%x)\n",
           tag, hal_flashx_read_ear(env), qspi->BACKUP_OFFSET << 14,
           qspi->DAC_CTRL, qspi->DAC_CMD, qspi->QSPI_CTRL1, rstFlag,
           rstFlag & PMU_STARTUP_PART_FLAG_MASK,
           (rstFlag >> PMU_BOOTRAM_STARTUP_PART_FLAG_POS) &
               PMU_STARTUP_PART_FLAG_MASK);
}

/* Which A/B slot the QSPI direct-read (XIP) window is currently mapped onto:
 * 'a', 'b', or 0 when it cannot be determined.
 *
 * This is the hardware ground truth for "which slot am I executing from". The
 * sbl_env carries two selectors (dual_sec_active_image, used by the production
 * secureboot-rom, and dual_app_active_image, used by this tree's boot_ram) and
 * they can disagree -- e.g. sec='a' while the SBL actually booted slot 'b'.
 * An updater that trusts the wrong one erases the flash it is executing from
 * and hangs the CPU without printing anything.
 *
 * Inverse of boot_ram's boot_xip_rebase(): the window maps the linker base
 * (app_a_partition_offset) plus the controller's 16KiB-granular delta, in the
 * 16MiB bank selected by the chip's EAR. */
char soc_flash_window_slot(void)
{
    const struct device *flashDev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

    if ((flashDev == NULL) || !device_is_ready(flashDev)) {
        return 0;
    }

    struct hal_flash_env *env = flash_ls_env(flashDev);
    reg_lsqspiv2_t *qspi = (reg_lsqspiv2_t *)env->reg;
    uint32_t ear = hal_flashx_read_ear(env);
    uint32_t delta = (uint32_t)qspi->BACKUP_OFFSET << 14;
    uint32_t ref = SOC_APP_A_OFFSET; /* linker base of both slots */
    uint32_t window = (ear != 0U ? 16U * 1024U * 1024U : 0U) + ref + delta;

    if (window == SOC_APP_A_OFFSET) {
        return 'a';
    }
    if (window == SOC_APP_B_OFFSET) {
        return 'b';
    }
    return 0;
}

/* Cold reboot that first hands the flash read window back to its neutral
 * state: chip EAR = 0 and QSPI direct-read offset = 0, which is what a boot
 * started from slot A looks like.
 *
 * The A/B chain reprograms that window for whichever slot it boots. Running
 * from slot B leaves EAR=1 and BACKUP_OFFSET=0x40 behind, and a warm reset
 * clears neither the flash chip (the EAR is a volatile chip register) nor the
 * QSPI controller. The *next* boot then reads flash offset 0 -- the SBL image
 * that the boot ROM loads -- out of the second 16MB bank, fails its header
 * check, retries four times and drops into UART boot. That is the "b -> a
 * never boots, only removing power helps" symptom (a -> b is unaffected
 * because a run leaves the window at 0/0 already).
 *
 * Clearing the window invalidates this CPU's own XIP mapping on the spot, so
 * the sequence must live in RAM and must not return: __ramfunc, interrupts
 * off, no logging. Only the epilogue of the HAL's EAR write still executes
 * from flash, and that function is hot at every boot (it is what the SBL uses
 * to set the window), so it is expected to be resident in the LSCACHE. */
__ramfunc void soc_reboot_cold_after_flash_window_reset(void)
{
    const struct device *flashDev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    struct hal_flash_env *env = flash_ls_env(flashDev);
    reg_lsqspiv2_t *qspi = (reg_lsqspiv2_t *)env->reg;

    (void)arch_irq_lock();

    /* Chip back to bank 0 ... */
    hal_flashx_write_ear(env, 0);
    /* ... and the controller back to a 1:1 CPU-to-flash mapping, so offset 0
     * really means offset 0 for the boot ROM and the SBL loader. */
    qspi->BACKUP_OFFSET = 0;

    REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_FROM_SFT, 0x1);
    csi_core_reset();

    for (;;)
    {
        /* A reset was requested; nothing sensible is left to run here. */
    }
}

