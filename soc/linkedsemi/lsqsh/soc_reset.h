
#ifndef _SOC_RESET_H_
#define _SOC_RESET_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>

enum reset_reason {
    NO_RESET_REASON,

    /* sec pmu reg indicate */
    PWR_FULL_RESET, /* reset source: AC machine */
    SOFT_FULL_RESET, /* reset source: 1. write sec pmu system reset reg */
    CPU_FULL_RESET, /* reset source: 1. write cpu1 system reset reg  2. debugger */
    SYS_IWDT_FULL_RESET, /* reset source: 1. SYS_IWDT */
    EXT_FULL_RESET, /* reset source: 1. EXTRST/SRST RESET pin */

    /* sec per reg && sec iwdt/wwdt reg indicate */
    SEC_IWDT_HART_RESET, /* reset source: 1. SEC_IWDT && only hart */
    SEC_WWDT_HART_RESET, /* reset source: 1. SEC_WWDT && only hart */

    /* sec per/app per reg && sec per/app per reg indicate */
    SOFT_HART_RESET, /* reset source: write cpu1/cpu2 core reset reg */

    /* app per reg && app iwdt/wwdt reg indicate */
    APP_IWDT_HART_RESET, /* reset source: 1. APP_IWDT */
    APP_WWDT_HART_RESET, /* reset source: 1. APP_WWDT */

    /* magic word indicate */
    EMUL_SOFT_RESET, /* reset source: set pc to __rom_region_start */
};

#if defined(CONFIG_WDT_RESET_REASON_DETAIL)
struct wdt_reset_en {
    union {
        uint32_t value1;
        struct {
            uint32_t
                WDT_RST_BSTIM1 : 1,
                WDT_RST_BSTIM2 : 1,
                WDT_RST_GPTIMA1 : 1,
                WDT_RST_GPTIMA2 : 1,
                WDT_RST_GPTIMB1 : 1,
                WDT_RST_GPTIMC1 : 1,
                WDT_RST_ADTIM1 : 1,
                WDT_RST_ADTIM2 : 1,
                WDT_RST_PWM : 1,
                WDT_RST_TACH : 1,
                WDT_RST_I2C1 : 1,
                WDT_RST_I2C2 : 1,
                WDT_RST_I2C3 : 1,
                WDT_RST_I2C4 : 1,
                WDT_RST_I2C5 : 1,
                WDT_RST_I2C6 : 1,
                WDT_RST_I2C7 : 1,
                WDT_RST_I2C8 : 1,
                WDT_RST_I2C9 : 1,
                WDT_RST_I2C10 : 1,
                WDT_RST_I2C11 : 1,
                WDT_RST_I2C12 : 1,
                WDT_RST_I2C13 : 1,
                WDT_RST_I2C14 : 1,
                WDT_RST_I2C15 : 1,
                WDT_RST_I2C16 : 1,
                WDT_RST_UART1 : 1,
                WDT_RST_UART2 : 1,
                WDT_RST_UART3 : 1,
                WDT_RST_UART4 : 1,
                WDT_RST_UART5 : 1,
                WDT_RST_UART6 : 1;
        };
    };

    union {
        uint32_t value2;
        struct {
            uint32_t
                WDT_RST_UART7 : 1,
                WDT_RST_UART8 : 1,
                WDT_RST_UART9 : 1,
                WDT_RST_UART10 : 1,
                WDT_RST_UART11 : 1,
                WDT_RST_UART12 : 1,
                WDT_RST_SGPIO1_MST : 1,
                WDT_RST_SGPIO2_MST : 1,
                WDT_RST_SGPIO1_MON : 1,
                WDT_RST_SGPIO2_MON : 1,
                WDT_RST_PECI1 : 1,
                WDT_RST_PECI2 : 1,
                WDT_RST_SPI1 : 1,
                WDT_RST_SPI2 : 1,
                WDT_RST_SPI3 : 1,
                WDT_RST_SPI4 : 1,
                WDT_RST_SPIS1 : 1,
                WDT_RST_SPIS2 : 1,
                WDT_RST_ADC1 : 1,
                WDT_RST_ADC2 : 1,
                WDT_RST_EXTI1 : 1,
                WDT_RST_EXTI2 : 1,
                WDT_RST_EXTI3 : 1,
                WDT_RST_EXTI4 : 1,
                WDT_RST_MJTAG1 : 1,
                WDT_RST_MJTAG2 : 1,
                WDT_RST_MJTAG3 : 1,
                WDT_RST_I3C1 : 1,
                WDT_RST_I3C2 : 1,
                WDT_RST_I3C3 : 1,
                WDT_RST_I3C4 : 1,
                WDT_RST_I3C5 : 1;
        };
    };

    union {
        uint32_t value3;
        struct {
            uint32_t
                WDT_RST_I3C6 : 1,
                WDT_RST_I3C7 : 1,
                WDT_RST_I3C8 : 1,
                WDT_RST_I3C9 : 1,
                WDT_RST_I3C10 : 1,
                WDT_RST_I3C11 : 1,
                WDT_RST_I3C12 : 1,
                WDT_RST_I3C13 : 1,
                WDT_RST_I3C14 : 1,
                WDT_RST_RESERVED30 : 1,
                WDT_RST_RESERVED31 : 1,
                WDT_RST_KSCAN : 1,
                WDT_RST_PS2IF1 : 1,
                WDT_RST_PS2IF2 : 1,
                WDT_RST_OWM : 1,
                WDT_RST_CEC : 1,
                WDT_RST_PIS : 1,
                WDT_RST_FILTER : 1,
                WDT_RST_SPI_FLT1 : 1,
                WDT_RST_SPI_FLT2 : 1,
                WDT_RST_SPI_FLT3 : 1,
                WDT_RST_SPI_FLT4 : 1,
                WDT_RST_SMB_FLT1 : 1,
                WDT_RST_SMB_FLT2 : 1,
                WDT_RST_SMB_FLT3 : 1,
                WDT_RST_SMB_FLT4 : 1,
                WDT_RST_PDM : 1,
                WDT_RST_TRNG1 : 1,
                WDT_RST_TRNG2 : 1,
                WDT_RST_RESERVED32 : 1,
                WDT_RST_PARAL : 1,
                WDT_RST_OTBN : 1;
        };
    };

    union {
        uint32_t value4;
        struct {
            uint32_t
                WDT_RST_CACHE1 : 1,
                WDT_RST_CACHE2 : 1,
                WDT_RST_QSPI1 : 1,
                WDT_RST_QSPI2 : 1,
                WDT_RST_USB1 : 1,
                WDT_RST_USB2 : 1,
                WDT_RST_DMAC1 : 1,
                WDT_RST_DMAC2 : 1,
                WDT_RST_ESPI1 : 1,
                WDT_RST_ESPI2 : 1,
                WDT_RST_LPC1 : 1,
                WDT_RST_LPC2 : 1,
                WDT_RST_FDCAN : 1,
                WDT_RST_PSRAM : 1,
                WDT_RST_ETH1 : 1,
                WDT_RST_ETH2 : 1,
                WDT_RST_EMMC1 : 1,
                WDT_RST_EMMC2 : 1,
                WDT_RST_LTPI_SCM : 1,
                WDT_RST_LTPI_HPM : 1,
                WDT_RST_LTPI_PHY : 1,
                WDT_RST_RESERVED40 : 1,
                WDT_RST_CALC_CRC : 1,
                WDT_RST_CALC : 1,
                WDT_RST_CRYPT : 1,
                WDT_RST_OTP_CTRL : 1,
                WDT_RST_CALC_SHA : 1,
                WDT_RST_CALC_SM4 : 1,
                WDT_RST_TPM_SPIS1 : 1,
                WDT_RST_TPM_SPIS2 : 1,
                WDT_RST_NIST_TRNG : 1,
                WDT_RST_RESERVED41 : 1;
        };
    };

    union {
        uint32_t value5;
        struct {
            uint32_t
                WDT_RST_SHA512 : 1,
                WDT_RST_OTFAD_AES : 1;
        };
    };
};
#endif

void reset_reason_init(void);
uint32_t reset_reason_get(void);
uint32_t reset_reason_app_get(void);
void reset_reason_app_set(uint32_t reason);
void reset_reason_app_clean();
void reset_reason_magic_set(void);
#if defined(CONFIG_WDT_RESET_REASON_DETAIL)
struct wdt_reset_en * wdt_reset_en_val_get(void);
int sec_iwdt_reset_en_get(struct wdt_reset_en *wdt_reset_en);
int sec_iwdt_reset_en_set(struct wdt_reset_en *wdt_reset_en);
int wdt_reset_en_print(struct wdt_reset_en *wdt_reset_en);
void sys_arch_reboot_warm_emul();

static inline int wdt_setup_linkedsemi(const struct device *dev, struct wdt_reset_en *wdt_reset_en)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_PATH(soc, watchdog_400a1800))
    if (DEVICE_DT_GET(DT_PATH(soc, watchdog_400a1800)) == dev) {
        struct wdt_reset_en *noinit_wdt_reset_en = wdt_reset_en_val_get();
        *noinit_wdt_reset_en = *wdt_reset_en;
        sec_iwdt_reset_en_set(wdt_reset_en);
        return wdt_setup(dev, 0);
    } else {
        return -ENOTSUP;
    }
#else
    ARG_UNUSED(dev);
    ARG_UNUSED(wdt_reset_en);
    return -ENOTSUP;
#endif
}
#endif

#endif /* _SOC_RESET_H_ */
