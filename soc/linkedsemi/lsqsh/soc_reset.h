
#ifndef _SOC_RESET_H_
#define _SOC_RESET_H_

#include <zephyr/kernel.h>

enum reset_reason {
    NO_RESET_REASON,

    /* sec pmu reg indicate */
    PWR_FULL_RESET, /* reset source: AC machine */
    SOFT_FULL_RESET, /* reset source: 1. write sec pmu system reset reg */
    CPU_FULL_RESET, /* reset source: 1. write cpu1 system reset reg  2. debugger */
    SYS_IWDT_FULL_RESET, /* reset source: 1. SYS_IWDT */
    EXT_FULL_RESET, /* reset source: 1. EXTRST/SRST RESET pin */

    /* sec per reg && sec iwdt/wwdt reg indicate */
    SEC_IWDT_FULL_RESET, /* reset source: 1. SEC_IWDT && all peripherals */
    SEC_WWDT_FULL_RESET, /* reset source: 1. SEC_WWDT && all peripherals */
    SEC_IWDT_PARTIAL_RESET, /* reset source: 1. SEC_IWDT && partial peripherals */
    SEC_WWDT_PARTIAL_RESET, /* reset source: 1. SEC_WWDT && partial peripherals */
    SEC_IWDT_HART_RESET, /* reset source: 1. SEC_IWDT && only hart */
    SEC_WWDT_HART_RESET, /* reset source: 1. SEC_WWDT && only hart */
    SEC_IWDT_RESET, /* reset source: 1. SEC_IWDT && no reserved information to indicate full/partial/hart reset */
    SEC_WWDT_RESET, /* reset source: 1. SEC_WWDT && no reserved information to indicate full/partial/hart reset */
    /* sec per/app per reg && sec per/app per reg indicate */
    SOFT_HART_RESET, /* reset source: write cpu1/cpu2 core reset reg */

    /* app per reg && app iwdt/wwdt reg indicate */
    APP_IWDT_HART_RESET, /* reset source: 1. APP_IWDT */
    APP_WWDT_HART_RESET, /* reset source: 1. APP_WWDT */

    /* magic word indicate */
    EMUL_SOFT_RESET, /* reset source: set pc to __rom_region_start */
};

struct wdt_reset_en {
    union {
        uint32_t value1;
        struct {
            uint32_t
                BSTIM1 : 1,
                BSTIM2 : 1,
                GPTIMA1 : 1,
                GPTIMA2 : 1,
                GPTIMB1 : 1,
                GPTIMC1 : 1,
                ADTIM1 : 1,
                ADTIM2 : 1,
                PWM : 1,
                TACH : 1,
                I2C1 : 1,
                I2C2 : 1,
                I2C3 : 1,
                I2C4 : 1,
                I2C5 : 1,
                I2C6 : 1,
                I2C7 : 1,
                I2C8 : 1,
                I2C9 : 1,
                I2C10 : 1,
                I2C11 : 1,
                I2C12 : 1,
                I2C13 : 1,
                I2C14 : 1,
                I2C15 : 1,
                I2C16 : 1,
                UART1 : 1,
                UART2 : 1,
                UART3 : 1,
                UART4 : 1,
                UART5 : 1,
                UART6 : 1;
        };
    };

    union {
        uint32_t value2;
        struct {
            uint32_t
                UART7 : 1,
                UART8 : 1,
                UART9 : 1,
                UART10 : 1,
                UART11 : 1,
                UART12 : 1,
                SGPIO1_MST : 1,
                SGPIO2_MST : 1,
                SGPIO1_MON : 1,
                SGPIO2_MON : 1,
                PECI1 : 1,
                PECI2 : 1,
                SPI1 : 1,
                SPI2 : 1,
                SPI3 : 1,
                SPI4 : 1,
                SPIS1 : 1,
                SPIS2 : 1,
                ADC1 : 1,
                ADC2 : 1,
                EXTI1 : 1,
                EXTI2 : 1,
                EXTI3 : 1,
                EXTI4 : 1,
                MJTAG1 : 1,
                MJTAG2 : 1,
                MJTAG3 : 1,
                I3C1 : 1,
                I3C2 : 1,
                I3C3 : 1,
                I3C4 : 1,
                I3C5 : 1;
        };
    };

    union {
        uint32_t value3;
        struct {
            uint32_t
                I3C6 : 1,
                I3C7 : 1,
                I3C8 : 1,
                I3C9 : 1,
                I3C10 : 1,
                I3C11 : 1,
                I3C12 : 1,
                I3C13 : 1,
                I3C14 : 1,
                RESERVED30 : 1,
                RESERVED31 : 1,
                KSCAN : 1,
                PS2IF1 : 1,
                PS2IF2 : 1,
                OWM : 1,
                CEC : 1,
                PIS : 1,
                FILTER : 1,
                SPI_FLT1 : 1,
                SPI_FLT2 : 1,
                SPI_FLT3 : 1,
                SPI_FLT4 : 1,
                SMB_FLT1 : 1,
                SMB_FLT2 : 1,
                SMB_FLT3 : 1,
                SMB_FLT4 : 1,
                PDM : 1,
                TRNG1 : 1,
                TRNG2 : 1,
                RESERVED32 : 1,
                PARAL : 1,
                OTBN : 1;
        };
    };

    union {
        uint32_t value4;
        struct {
            uint32_t
                CACHE1 : 1,
                CACHE2 : 1,
                QSPI1 : 1,
                QSPI2 : 1,
                USB1 : 1,
                USB2 : 1,
                DMAC1 : 1,
                DMAC2 : 1,
                ESPI1 : 1,
                ESPI2 : 1,
                LPC1 : 1,
                LPC2 : 1,
                FDCAN : 1,
                PSRAM : 1,
                ETH1 : 1,
                ETH2 : 1,
                EMMC1 : 1,
                EMMC2 : 1,
                LTPI_SCM : 1,
                LTPI_HPM : 1,
                LTPI_PHY : 1,
                RESERVED40 : 1,
                CALC_CRC : 1,
                CALC : 1,
                CRYPT : 1,
                OTP_CTRL : 1,
                CALC_SHA : 1,
                CALC_SM4 : 1,
                TPM_SPIS1 : 1,
                TPM_SPIS2 : 1,
                NIST_TRNG : 1,
                RESERVED41 : 1;
        };
    };

    union {
        uint32_t value5;
        struct {
            uint32_t
                SHA512 : 1,
                OTFAD_AES : 1;
        };
    };
};

void reset_reason_init(void);
enum reset_reason reset_reason_get(void);
void reset_reason_magic_set(void);
struct wdt_reset_en * wdt_reset_en_val_get(void);
int sec_iwdt_reset_en_get(struct wdt_reset_en *wdt_reset_en);
int sec_iwdt_reset_en_set(struct wdt_reset_en *wdt_reset_en);
int wdt_reset_en_print(struct wdt_reset_en *wdt_reset_en);

#endif /* _SOC_RESET_H_ */
