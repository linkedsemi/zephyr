#include <zephyr/cache.h>
#include "soc_reset.h"
#include "field_manipulate.h"
#include "ls_msp_iwdg.h"
#include "reg_sec_pmu_rg.h"
#include "reg_sysc_sec_cpu.h"
#include "reg_sysc_app_cpu.h"
#include "reg_sysc_sec_per.h"
#include "reg_sysc_app_per.h"

#define MAGIC_VALUE 0xdeadbeef

static enum reset_reason reset_reason = NO_RESET_REASON;
struct wdt_reset_en wdt_reset_en __noinit;
static volatile uint32_t magic __noinit IF_ENABLED(CONFIG_DCACHE, (__aligned(CONFIG_DCACHE_LINE_SIZE)));

__maybe_unused static void global_reset_reason_clean(void)
{
    REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_SRC_CLR, 1);
    REG_FIELD_WR(SEC_PMU->RST_SFT, SEC_PMU_RG_RST_SRC_CLR, 0);
}

__maybe_unused static void sec_wdt_reset_reason_clean(void)
{
    REG_FIELD_WR(SYSC_SEC_PER->RST_SRC, SYSC_SEC_PER_RST_SRC_CLR, 1);
    REG_FIELD_WR(SYSC_SEC_PER->RST_SRC, SYSC_SEC_PER_RST_SRC_CLR, 0);
}

__maybe_unused static void app_wdt_reset_reason_clean(void)
{
    REG_FIELD_WR(SYSC_APP_PER->RST_SRC, SYSC_APP_PER_RST_SRC_CLR, 1);
    REG_FIELD_WR(SYSC_APP_PER->RST_SRC, SYSC_APP_PER_RST_SRC_CLR, 0);
}

void reset_reasonn_magic_clean(void)
{
    magic = NO_RESET_REASON;
}

void reset_reason_magic_set()
{
    magic = MAGIC_VALUE;
}

enum reset_reason reset_reason_get(void)
{
    return reset_reason;
}

void reset_reason_init(void)
{
    enum reset_reason ret = NO_RESET_REASON;
    uint32_t reset_src = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (SEC_PMU->PMU_STATUS & SEC_PMU_RG_RST_SRC_MASK) {
        reset_src = SEC_PMU->PMU_STATUS & SEC_PMU_RG_RST_SRC_MASK;
        global_reset_reason_clean();
        sec_wdt_reset_reason_clean();
        app_wdt_reset_reason_clean();
        reset_reasonn_magic_clean();
        if (SEC_PMU_RG_RST_SRC_MASK == reset_src) {
            ret = PWR_FULL_RESET;
        } else if (SEC_PMU_RG_RST_SRC_CPU_MASK & reset_src) {
            ret = SOFT_FULL_RESET;
        } else if (SEC_PMU_RG_RST_SRC_IWDT_MASK & reset_src) {
            ret = SYS_IWDT_FULL_RESET;
        } else if (SEC_PMU_RG_RST_SRC_PAD_MASK & reset_src) {
            ret = EXT_FULL_RESET;
        }
    } else if (SYSC_SEC_PER->RST_SRC & SYSC_SEC_PER_RST_SRC_MASK) {
        reset_src = SYSC_SEC_PER->RST_SRC & SYSC_SEC_PER_RST_SRC_MASK;
        sec_wdt_reset_reason_clean();
        if (SYSC_SEC_PER_RST_FROM_IWDT1_MASK & reset_src) {
            if ((IWDT_RSTEN1_ALL_MASK == wdt_reset_en.value1)
                && (IWDT_RSTEN2_ALL_MASK == wdt_reset_en.value2)
                && (IWDT_RSTEN3_ALL_MASK == wdt_reset_en.value3)
                && (IWDT_RSTEN4_ALL_MASK == wdt_reset_en.value4)
                && (IWDT_RSTEN5_ALL_MASK == wdt_reset_en.value5)) {
                ret = SEC_IWDT_FULL_RESET;
            } else if ((0 == wdt_reset_en.value1)
                && (0 == wdt_reset_en.value2)
                && (0 == wdt_reset_en.value3)
                && (0 == wdt_reset_en.value4)
                && (0 == wdt_reset_en.value5)) {
                ret = SEC_IWDT_HART_RESET;
            } else if ((0 != wdt_reset_en.value1)
                || (0 != wdt_reset_en.value2)
                || (0 != wdt_reset_en.value3)
                || (0 != wdt_reset_en.value4)
                || (0 != wdt_reset_en.value5)) {
                ret = SEC_IWDT_PARTIAL_RESET;
            } else {
                /* unreachable */
                for (;;) {}
            }

            if (wdt_reset_en.PSRAM) {
                /* PSRAM xip disable */
                uint32_t psram_cfg = sys_read32(APP_SYSC_CPU_APP_ADDR + 0x80);
                psram_cfg &= ~BIT(0);
                sys_write32(psram_cfg, APP_SYSC_CPU_APP_ADDR + 0x80);
            }
        }
    #if 0
        else if (SYSC_SEC_PER_RST_FROM_WWDT1_MASK & reset_src) {
            if () {
                ret = SEC_WWDT_FULL_RESET;
            } else {
                ret = SEC_WWDT_HART_RESET;
            }
        }
    #endif
        else if (SYSC_SEC_PER_RST_FROM_SEC_CORE_SRST_MASK & reset_src) {
            ret = SOFT_HART_RESET;
        }
    } else if (MAGIC_VALUE == magic) {
        ret = EMUL_SOFT_RESET;
    }
#else  /* DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay) */
    reset_src = SYSC_APP_PER->RST_SRC & SYSC_APP_PER_RST_SRC_MASK;
    if (reset_src) {
        app_wdt_reset_reason_clean();
        if (SYSC_APP_PER_RST_FROM_IWDT2_MASK & reset_src) {
            ret = APP_IWDT_HART_RESET;
        } else if (SYSC_APP_PER_RST_FROM_WWDT2_MASK & reset_src) {
            ret = APP_WWDT_HART_RESET;
        } else if (SYSC_APP_PER_RST_FROM_APP_CORE_SRST_MASK & reset_src) {
            ret = SOFT_HART_RESET;
        }
    } else if (MAGIC_VALUE == magic) {
        ret = EMUL_SOFT_RESET;
    }
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay) */

    reset_reason = ret;
}

struct wdt_reset_en *wdt_reset_en_val_get(void)
{
    return &wdt_reset_en;
}

int sec_iwdt_reset_en_get(struct wdt_reset_en *en)
{
    __ASSERT_NO_MSG(en);
    en->value1 = SEC_IWDG->PER_RSTEN1;
    en->value2 = SEC_IWDG->PER_RSTEN2;
    en->value3 = SEC_IWDG->PER_RSTEN3;
    en->value4 = SEC_IWDG->PER_RSTEN4;
    en->value5 = SEC_IWDG->PER_RSTEN5;

    return 0;
}

int sec_iwdt_reset_en_set(struct wdt_reset_en *en)
{
    __ASSERT_NO_MSG(en);

    SEC_IWDG->PER_RSTEN1 = en->value1;
    SEC_IWDG->PER_RSTEN2 = en->value2;
    SEC_IWDG->PER_RSTEN3 = en->value3;
    SEC_IWDG->PER_RSTEN4 = en->value4;
    SEC_IWDG->PER_RSTEN5 = en->value5;

    return 0;
}

int wdt_reset_en_print(struct wdt_reset_en *en)
{
    __ASSERT_NO_MSG(en);
    printk(
        "BSTIM1 : %d\n"
        "BSTIM2 : %d\n"
        "GPTIMA1 : %d\n"
        "GPTIMA2 : %d\n"
        "GPTIMB1 : %d\n"
        "GPTIMC1 : %d\n"
        "ADTIM1 : %d\n"
        "ADTIM2 : %d\n"
        "PWM : %d\n"
        "TACH : %d\n"
        "I2C1 : %d\n"
        "I2C2 : %d\n"
        "I2C3 : %d\n"
        "I2C4 : %d\n"
        "I2C5 : %d\n"
        "I2C6 : %d\n"
        "I2C7 : %d\n"
        "I2C8 : %d\n"
        "I2C9 : %d\n"
        "I2C10 : %d\n"
        "I2C11 : %d\n"
        "I2C12 : %d\n"
        "I2C13 : %d\n"
        "I2C14 : %d\n"
        "I2C15 : %d\n"
        "I2C16 : %d\n"
        "UART1 : %d\n"
        "UART2 : %d\n"
        "UART3 : %d\n"
        "UART4 : %d\n"
        "UART5 : %d\n"
        "UART6 : %d\n"
        "UART7 : %d\n"
        "UART8 : %d\n"
        "UART9 : %d\n"
        "UART10 : %d\n"
        "UART11 : %d\n"
        "UART12 : %d\n"
        "SGPIO1_MST : %d\n"
        "SGPIO2_MST : %d\n"
        "SGPIO1_MON : %d\n"
        "SGPIO2_MON : %d\n"
        "PECI1 : %d\n"
        "PECI2 : %d\n"
        "SPI1 : %d\n"
        "SPI2 : %d\n"
        "SPI3 : %d\n"
        "SPI4 : %d\n"
        "SPIS1 : %d\n"
        "SPIS2 : %d\n"
        "ADC1 : %d\n"
        "ADC2 : %d\n"
        "EXTI1 : %d\n"
        "EXTI2 : %d\n"
        "EXTI3 : %d\n"
        "EXTI4 : %d\n"
        "MJTAG1 : %d\n"
        "MJTAG2 : %d\n"
        "MJTAG3 : %d\n"
        "I3C1 : %d\n"
        "I3C2 : %d\n"
        "I3C3 : %d\n"
        "I3C4 : %d\n"
        "I3C5 : %d\n"
        "I3C6 : %d\n"
        "I3C7 : %d\n"
        "I3C8 : %d\n"
        "I3C9 : %d\n"
        "I3C10 : %d\n"
        "I3C11 : %d\n"
        "I3C12 : %d\n"
        "I3C13 : %d\n"
        "I3C14 : %d\n"
     /* "RESERVED30 : %d\n" */
     /* "RESERVED31 : %d\n" */
        "KSCAN : %d\n"
        "PS2IF1 : %d\n"
        "PS2IF2 : %d\n"
        "OWM : %d\n"
        "CEC : %d\n"
        "PIS : %d\n"
        "FILTER : %d\n"
        "SPI_FLT1 : %d\n"
        "SPI_FLT2 : %d\n"
        "SPI_FLT3 : %d\n"
        "SPI_FLT4 : %d\n"
        "SMB_FLT1 : %d\n"
        "SMB_FLT2 : %d\n"
        "SMB_FLT3 : %d\n"
        "SMB_FLT4 : %d\n"
        "PDM : %d\n"
        "TRNG1 : %d\n"
        "TRNG2 : %d\n"
     /* "RESERVED32 : %d\n" */
        "PARAL : %d\n"
        "OTBN : %d\n"
        "CACHE1 : %d\n"
        "CACHE2 : %d\n"
        "QSPI1 : %d\n"
        "QSPI2 : %d\n"
        "USB1 : %d\n"
        "USB2 : %d\n"
        "DMAC1 : %d\n"
        "DMAC2 : %d\n"
        "ESPI1 : %d\n"
        "ESPI2 : %d\n"
        "LPC1 : %d\n"
        "LPC2 : %d\n"
        "FDCAN : %d\n"
        "PSRAM : %d\n"
        "ETH1 : %d\n"
        "ETH2 : %d\n"
        "EMMC1 : %d\n"
        "EMMC2 : %d\n"
        "LTPI_SCM : %d\n"
        "LTPI_HPM : %d\n"
        "LTPI_PHY : %d\n"
     /* "RESERVED40 : %d\n" */
        "CALC_CRC : %d\n"
        "CALC : %d\n"
        "CRYPT : %d\n"
        "OTP_CTRL : %d\n"
        "CALC_SHA : %d\n"
        "CALC_SM4 : %d\n"
        "TPM_SPIS1 : %d\n"
        "TPM_SPIS2 : %d\n"
        "NIST_TRNG : %d\n"
     /* "RESERVED41 : %d\n" */
        "SHA512 : %d\n"
        "OTFAD_AES : %d\n",
        en->BSTIM1,
        en->BSTIM2,
        en->GPTIMA1,
        en->GPTIMA2,
        en->GPTIMB1,
        en->GPTIMC1,
        en->ADTIM1,
        en->ADTIM2,
        en->PWM,
        en->TACH,
        en->I2C1,
        en->I2C2,
        en->I2C3,
        en->I2C4,
        en->I2C5,
        en->I2C6,
        en->I2C7,
        en->I2C8,
        en->I2C9,
        en->I2C10,
        en->I2C11,
        en->I2C12,
        en->I2C13,
        en->I2C14,
        en->I2C15,
        en->I2C16,
        en->UART1,
        en->UART2,
        en->UART3,
        en->UART4,
        en->UART5,
        en->UART6,
        en->UART7,
        en->UART8,
        en->UART9,
        en->UART10,
        en->UART11,
        en->UART12,
        en->SGPIO1_MST,
        en->SGPIO2_MST,
        en->SGPIO1_MON,
        en->SGPIO2_MON,
        en->PECI1,
        en->PECI2,
        en->SPI1,
        en->SPI2,
        en->SPI3,
        en->SPI4,
        en->SPIS1,
        en->SPIS2,
        en->ADC1,
        en->ADC2,
        en->EXTI1,
        en->EXTI2,
        en->EXTI3,
        en->EXTI4,
        en->MJTAG1,
        en->MJTAG2,
        en->MJTAG3,
        en->I3C1,
        en->I3C2,
        en->I3C3,
        en->I3C4,
        en->I3C5,
        en->I3C6,
        en->I3C7,
        en->I3C8,
        en->I3C9,
        en->I3C10,
        en->I3C11,
        en->I3C12,
        en->I3C13,
        en->I3C14,
     /* en->RESERVED30, */
     /* en->RESERVED31, */
        en->KSCAN,
        en->PS2IF1,
        en->PS2IF2,
        en->OWM,
        en->CEC,
        en->PIS,
        en->FILTER,
        en->SPI_FLT1,
        en->SPI_FLT2,
        en->SPI_FLT3,
        en->SPI_FLT4,
        en->SMB_FLT1,
        en->SMB_FLT2,
        en->SMB_FLT3,
        en->SMB_FLT4,
        en->PDM,
        en->TRNG1,
        en->TRNG2,
     /* en->RESERVED32, */
        en->PARAL,
        en->OTBN,
        en->CACHE1,
        en->CACHE2,
        en->QSPI1,
        en->QSPI2,
        en->USB1,
        en->USB2,
        en->DMAC1,
        en->DMAC2,
        en->ESPI1,
        en->ESPI2,
        en->LPC1,
        en->LPC2,
        en->FDCAN,
        en->PSRAM,
        en->ETH1,
        en->ETH2,
        en->EMMC1,
        en->EMMC2,
        en->LTPI_SCM,
        en->LTPI_HPM,
        en->LTPI_PHY,
     /* en->RESERVED40, */
        en->CALC_CRC,
        en->CALC,
        en->CRYPT,
        en->OTP_CTRL,
        en->CALC_SHA,
        en->CALC_SM4,
        en->TPM_SPIS1,
        en->TPM_SPIS2,
        en->NIST_TRNG,
     /* en->RESERVED41, */
        en->SHA512,
        en->OTFAD_AES);
    return 0;
}
