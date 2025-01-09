/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_FUNC_1_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LSQSH_PINCTRL_FUNC_1_H_

#define LS_PINMUX_RSV_FUNC1_IDX0_PA(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM1_CH1_FUNC1_IDX1_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH1N_FUNC1_IDX2_PA(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2_FUNC1_IDX3_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2N_FUNC1_IDX4_PA(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3_FUNC1_IDX5_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3N_FUNC1_IDX6_PA(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH4_FUNC1_IDX7_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM1 */
#define LS_PINMUX_ADTIM1_ETR_FUNC1_IDX8_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM1 */
#define LS_PINMUX_ADTIM1_BK_FUNC1_IDX9_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM1 */
#define LS_PINMUX_I2C1_SCL_FUNC1_IDX10_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 1 */
#define LS_PINMUX_I2C1_SDA_FUNC1_IDX11_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 1 */
#define LS_PINMUX_I2C1_SMBA_FUNC1_IDX12_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 1 */
#define LS_PINMUX_RSV_FUNC1_IDX13_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* TX data function for UART 1 */
#define LS_PINMUX_RSV_FUNC1_IDX14_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RX data function for UART 1 */
#define LS_PINMUX_RSV_FUNC1_IDX15_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* CTS function for UART 1 */
#define LS_PINMUX_RSV_FUNC1_IDX16_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* RTS function for UART 1 */
#define LS_PINMUX_RSV_FUNC1_IDX17_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* the clock signal for the uart1 */
#define LS_PINMUX_SPI1_SCK_FUNC1_IDX18_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* SCK function for SPI 2 */
#define LS_PINMUX_SPI1_NSS_FUNC1_IDX19_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* chip select function for SPI 1 */
#define LS_PINMUX_SPI1_MOSI_FUNC1_IDX20_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* MOSI function for SPI 1 */
#define LS_PINMUX_SPI1_MISO_FUNC1_IDX21_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MISO function for SPI 1 */
#define LS_PINMUX_SPI1_I2S_FUNC1_IDX22_PA(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* IIS clock function of SPI 1 */
#define LS_PINMUX_FDCAN_TXD_FUNC1_IDX23_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* TX data of CAN */
#define LS_PINMUX_FDCAN_RXD_FUNC1_IDX24_PA(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* RX data of CAN */
#define LS_PINMUX_PS2IF1_CLK_FUNC1_IDX25_PA(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* ps2if1 clock */
#define LS_PINMUX_PS2IF1_DAT_FUNC1_IDX26_PA(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* ps2if1 dat */
#define LS_PINMUX_PWM1_FUNC1_IDX27_PA(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* pwm1 */
#define LS_PINMUX_CAP1_FUNC1_IDX28_PA(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* cap1 */
#define LS_PINMUX_OWM_FUNC1_IDX29_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* owm */
#define LS_PINMUX_CEC_FUNC1_IDX30_PA(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cec */
#define LS_PINMUX_PIS_CH1_FUNC1_IDX31_PA(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PA##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 1 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PB(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM2_CH1_FUNC1_IDX1_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH1N_FUNC1_IDX2_PB(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2_FUNC1_IDX3_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2N_FUNC1_IDX4_PB(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3_FUNC1_IDX5_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3N_FUNC1_IDX6_PB(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH4_FUNC1_IDX7_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM2 */
#define LS_PINMUX_ADTIM2_ETR_FUNC1_IDX8_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM2 */
#define LS_PINMUX_ADTIM2_BK_FUNC1_IDX9_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM2 */
#define LS_PINMUX_I2C2_SCL_FUNC1_IDX10_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 2 */
#define LS_PINMUX_I2C2_SDA_FUNC1_IDX11_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 2 */
#define LS_PINMUX_I2C2_SMBA_FUNC1_IDX12_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 2 */
#define LS_PINMUX_RSV_FUNC1_IDX13_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* TX data function for UART 2 */
#define LS_PINMUX_RSV_FUNC1_IDX14_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RX data function for UART 2 */
#define LS_PINMUX_RSV_FUNC1_IDX15_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* CTS function for UART 2 */
#define LS_PINMUX_RSV_FUNC1_IDX16_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* RTS function for UART 2 */
#define LS_PINMUX_RSV_FUNC1_IDX17_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* the clock signal for the uart2 */
#define LS_PINMUX_SPI2_SCK_FUNC1_IDX18_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* SCK function for SPI 2 */
#define LS_PINMUX_SPI2_NSS_FUNC1_IDX19_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* chip select function for SPI 2 */
#define LS_PINMUX_SPI2_MOSI_FUNC1_IDX20_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* MOSI function for SPI 2 */
#define LS_PINMUX_SPI2_MISO_FUNC1_IDX21_PB(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MISO function for SPI 2 */
#define LS_PINMUX_SPI2_I2S_FUNC1_IDX22_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* IIS clock function of SPI 2 */
#define LS_PINMUX_USB1_DP_FUNC1_IDX23_PB(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* DP function of USB1 */
#define LS_PINMUX_USB1_DM_FUNC1_IDX24_PB(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* DM function of USB1 */
#define LS_PINMUX_USB1_CID_FUNC1_IDX25_PB(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* CID function of USB1 */
#define LS_PINMUX_PS2IF2_CLK_FUNC1_IDX26_PB(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* ps2if2 clock */
#define LS_PINMUX_PS2IF2_DAT_FUNC1_IDX27_PB(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* ps2if2 dat */
#define LS_PINMUX_PWM2_FUNC1_IDX28_PB(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm2 */
#define LS_PINMUX_CAP2_FUNC1_IDX29_PB(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap2 */
#define LS_PINMUX_OWM_FUNC1_IDX30_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* owm */
#define LS_PINMUX_CEC_FUNC1_IDX31_PB(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PB##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* cec */
#define LS_PINMUX_RSV_FUNC1_IDX0_PC(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C3_SCL_FUNC1_IDX6_PC(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 1 */
#define LS_PINMUX_I2C3_SDA_FUNC1_IDX7_PC(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 1 */
#define LS_PINMUX_I2C3_SMBA_FUNC1_IDX8_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 1 */
#define LS_PINMUX_UART3_TXD_FUNC1_IDX9_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* TX data function for UART 3 */
#define LS_PINMUX_UART3_RXD_FUNC1_IDX10_PC(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* RX data function for UART 3 */
#define LS_PINMUX_UART3_CTS_N_FUNC1_IDX11_PC(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* CTS function for UART 3 */
#define LS_PINMUX_UART3_DSR_N_FUNC1_IDX12_PC(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DSR function for UART 3 */
#define LS_PINMUX_UART3_DCD_N_FUNC1_IDX13_PC(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* DCD function for UART 3 */
#define LS_PINMUX_UART3_RI_N_FUNC1_IDX14_PC(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RI function for UART 3 */
#define LS_PINMUX_UART3_RTS_N_FUNC1_IDX15_PC(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* RTS function for UART 3 */
#define LS_PINMUX_UART3_DTR_N_FUNC1_IDX16_PC(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DTR function for UART 3 */
#define LS_PINMUX_UART3_DE_FUNC1_IDX17_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DE function for UART 3 */
#define LS_PINMUX_UART3_RE_FUNC1_IDX18_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RE function for UART 3 */
#define LS_PINMUX_SPI1_SCK_FUNC1_IDX19_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* SCK function for SPI 1 */
#define LS_PINMUX_SPI1_NSS_FUNC1_IDX20_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* chip select function for SPI 1 */
#define LS_PINMUX_SPI1_MOSI_FUNC1_IDX21_PC(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MOSI function for SPI 1 */
#define LS_PINMUX_SPI1_MISO_FUNC1_IDX22_PC(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* MISO function for SPI 1 */
#define LS_PINMUX_SPI1_I2S_FUNC1_IDX23_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* IIS clock function of SPI 1 */
#define LS_PINMUX_USB1_DP_FUNC1_IDX24_PC(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* DP function of USB1 */
#define LS_PINMUX_USB1_DM_FUNC1_IDX25_PC(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* DM function of USB1 */
#define LS_PINMUX_USB1_CID_FUNC1_IDX26_PC(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* CID function of USB1 */
#define LS_PINMUX_PWM3_FUNC1_IDX27_PC(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* pwm3 */
#define LS_PINMUX_PWM4_FUNC1_IDX28_PC(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm4 */
#define LS_PINMUX_CAP3_FUNC1_IDX29_PC(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap3 */
#define LS_PINMUX_CAP4_FUNC1_IDX30_PC(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cap4 */
#define LS_PINMUX_PIS_CH2_FUNC1_IDX31_PC(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PC##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 2 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PD(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C4_SCL_FUNC1_IDX6_PD(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 4 */
#define LS_PINMUX_I2C4_SDA_FUNC1_IDX7_PD(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 4 */
#define LS_PINMUX_I2C4_SMBA_FUNC1_IDX8_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 4 */
#define LS_PINMUX_UART4_TXD_FUNC1_IDX9_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* TX data function for UART 4 */
#define LS_PINMUX_UART4_RXD_FUNC1_IDX10_PD(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* RX data function for UART 4 */
#define LS_PINMUX_UART4_CTS_N_FUNC1_IDX11_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* CTS function for UART 4 */
#define LS_PINMUX_UART4_DSR_N_FUNC1_IDX12_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DSR function for UART 4 */
#define LS_PINMUX_UART4_DCD_N_FUNC1_IDX13_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* DCD function for UART 4 */
#define LS_PINMUX_UART4_RI_N_FUNC1_IDX14_PD(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RI function for UART 4 */
#define LS_PINMUX_UART4_RTS_N_FUNC1_IDX15_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* RTS function for UART 4 */
#define LS_PINMUX_UART4_DTR_N_FUNC1_IDX16_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DTR function for UART 4 */
#define LS_PINMUX_UART4_DE_FUNC1_IDX17_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DE function for UART 4 */
#define LS_PINMUX_UART4_RE_FUNC1_IDX18_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RE function for UART 4 */
#define LS_PINMUX_SPI2_SCK_FUNC1_IDX19_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* SCK function for SPI 1 */
#define LS_PINMUX_SPI2_NSS_FUNC1_IDX20_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* chip select function for SPI 1 */
#define LS_PINMUX_SPI2_MOSI_FUNC1_IDX21_PD(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MOSI function for SPI 1 */
#define LS_PINMUX_SPI2_MISO_FUNC1_IDX22_PD(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* MISO function for SPI 1 */
#define LS_PINMUX_SPI2_I2S_FUNC1_IDX23_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* IIS clock function of SPI 1 */
#define LS_PINMUX_MJTAG1_TRST_FUNC1_IDX24_PD(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 1 trst */
#define LS_PINMUX_MJTAG1_TDI_FUNC1_IDX25_PD(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 1 tdi */
#define LS_PINMUX_MJTAG1_TCK_FUNC1_IDX26_PD(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* jtag master 1 tck */
#define LS_PINMUX_MJTAG1_TMS_FUNC1_IDX27_PD(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* jtag master 1 tms */
#define LS_PINMUX_MJTAG1_TDO_FUNC1_IDX28_PD(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* jtag master 1 tdo */
#define LS_PINMUX_DBG_SCL1_FUNC1_IDX29_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* I2C Debug SCL1 */
#define LS_PINMUX_DBG_SDA1_FUNC1_IDX30_PD(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* I2C Debug SDA1 */
#define LS_PINMUX_PIS_CH3_FUNC1_IDX31_PD(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PD##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 3 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PE(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C5_SCL_FUNC1_IDX6_PE(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 5 */
#define LS_PINMUX_I2C5_SDA_FUNC1_IDX7_PE(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 5 */
#define LS_PINMUX_I2C5_SMBA_FUNC1_IDX8_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 5 */
#define LS_PINMUX_UART5_TXD_FUNC1_IDX9_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* TX data function for UART 5 */
#define LS_PINMUX_UART5_RXD_FUNC1_IDX10_PE(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* RX data function for UART 5 */
#define LS_PINMUX_UART5_CTS_N_FUNC1_IDX11_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* CTS function for UART 5 */
#define LS_PINMUX_UART5_DSR_N_FUNC1_IDX12_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DSR function for UART 5 */
#define LS_PINMUX_UART5_DCD_N_FUNC1_IDX13_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* DCD function for UART 5 */
#define LS_PINMUX_UART5_RI_N_FUNC1_IDX14_PE(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RI function for UART 5 */
#define LS_PINMUX_UART5_RTS_N_FUNC1_IDX15_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* RTS function for UART 5 */
#define LS_PINMUX_UART5_DTR_N_FUNC1_IDX16_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DTR function for UART 5 */
#define LS_PINMUX_UART5_DE_FUNC1_IDX17_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DE function for UART 5 */
#define LS_PINMUX_UART5_RE_FUNC1_IDX18_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RE function for UART 5 */
#define LS_PINMUX_DBG_HBUS_CLK_FUNC1_IDX19_PE(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* HBUS clock test output */
#define LS_PINMUX_DBG_SRC_CLK_FUNC1_IDX20_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* HBUS clock test output */
#define LS_PINMUX_PDM_DAT0_FUNC1_IDX21_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* pdm channel0 data */
#define LS_PINMUX_PDM_DAT1_FUNC1_IDX22_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* pdm channel1 data */
#define LS_PINMUX_PDM_CLK_FUNC1_IDX23_PE(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* pdm clock */
#define LS_PINMUX_MJTAG2_TRST_FUNC1_IDX24_PE(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 2 trst */
#define LS_PINMUX_MJTAG2_TDI_FUNC1_IDX25_PE(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 2 tdi */
#define LS_PINMUX_MJTAG2_TCK_FUNC1_IDX26_PE(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* jtag master 2 tck */
#define LS_PINMUX_MJTAG2_TMS_FUNC1_IDX27_PE(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* jtag master 2 tms */
#define LS_PINMUX_MJTAG2_TDO_FUNC1_IDX28_PE(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* jtag master 2 tdo */
#define LS_PINMUX_DBG_SCL2_FUNC1_IDX29_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* I2C Debug SCL2 */
#define LS_PINMUX_DBG_SDA2_FUNC1_IDX30_PE(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* I2C Debug SDA2 */
#define LS_PINMUX_PIS_CH4_FUNC1_IDX31_PE(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PE##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 4 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PF(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C6_SCL_FUNC1_IDX5_PF(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* SCL function for IIC 2 */
#define LS_PINMUX_I2C6_SDA_FUNC1_IDX6_PF(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SDA function for IIC 2 */
#define LS_PINMUX_I2C6_SMBA_FUNC1_IDX7_PF(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SMBA function for IIC 2 */
#define LS_PINMUX_UART6_TXD_FUNC1_IDX8_PF(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* TX data function for UART 6 */
#define LS_PINMUX_UART6_RXD_FUNC1_IDX9_PF(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* RX data function for UART 6 */
#define LS_PINMUX_UART6_CTS_N_FUNC1_IDX10_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* CTS function for UART 6 */
#define LS_PINMUX_UART6_DSR_N_FUNC1_IDX11_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* DSR function for UART 6 */
#define LS_PINMUX_UART6_DCD_N_FUNC1_IDX12_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DCD function for UART 6 */
#define LS_PINMUX_UART6_RI_N_FUNC1_IDX13_PF(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* RI function for UART 6 */
#define LS_PINMUX_UART6_RTS_N_FUNC1_IDX14_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RTS function for UART 6 */
#define LS_PINMUX_UART6_DTR_N_FUNC1_IDX15_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* DTR function for UART 6 */
#define LS_PINMUX_UART6_DE_FUNC1_IDX16_PF(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DE function for UART 6 */
#define LS_PINMUX_UART6_RE_FUNC1_IDX17_PF(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* RE function for UART 6 */
#define LS_PINMUX_EMMC2_LED_CTRL_FUNC1_IDX18_PF(XX)  ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* SD card communication indicator led */
#define LS_PINMUX_EMMC1_LED_CTRL_FUNC1_IDX19_PF(XX)  ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* SD card communication indicator led */
#define LS_PINMUX_PDM_DATA0_FUNC1_IDX20_PF(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* pdm channel0 data */
#define LS_PINMUX_PDM_DATA1_FUNC1_IDX21_PF(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* pdm channel1 data */
#define LS_PINMUX_PDM_CLK_FUNC1_IDX22_PF(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* pdm clock */
#define LS_PINMUX_MJTAG3_TRST_FUNC1_IDX23_PF(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* jtag master 3 trst */
#define LS_PINMUX_MJTAG3_TDI_FUNC1_IDX24_PF(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 3 tdi */
#define LS_PINMUX_MJTAG3_TCK_FUNC1_IDX25_PF(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 3 tck */
#define LS_PINMUX_MJTAG3_TMS_FUNC1_IDX26_PF(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* jtag master 3 tms */
#define LS_PINMUX_MJTAG3_TDO_FUNC1_IDX27_PF(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* jtag master 3 tdo */
#define LS_PINMUX_PWM5_FUNC1_IDX28_PF(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm5 */
#define LS_PINMUX_PWM6_FUNC1_IDX29_PF(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* pwm6 */
#define LS_PINMUX_CAP5_FUNC1_IDX30_PF(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cap5 */
#define LS_PINMUX_CAP6_FUNC1_IDX31_PF(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PF##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* cap6 */
#define LS_PINMUX_RSV_FUNC1_IDX0_PG(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM1_CH1_FUNC1_IDX1_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH1N_FUNC1_IDX2_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2_FUNC1_IDX3_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2N_FUNC1_IDX4_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3_FUNC1_IDX5_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3N_FUNC1_IDX6_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH4_FUNC1_IDX7_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM1 */
#define LS_PINMUX_ADTIM1_ETR_FUNC1_IDX8_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM1 */
#define LS_PINMUX_ADTIM1_BK_FUNC1_IDX9_PG(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM1 */
#define LS_PINMUX_I2C7_SCL_FUNC1_IDX10_PG(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 7 */
#define LS_PINMUX_I2C7_SDA_FUNC1_IDX11_PG(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 7 */
#define LS_PINMUX_I2C7_SMBA_FUNC1_IDX12_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 7 */
#define LS_PINMUX_UART7_TXD_FUNC1_IDX13_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* TX data function for UART 7 */
#define LS_PINMUX_UART7_RXD_FUNC1_IDX14_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RX data function for UART 7 */
#define LS_PINMUX_UART7_CTS_N_FUNC1_IDX15_PG(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* CTS function for UART 7 */
#define LS_PINMUX_UART7_DSR_N_FUNC1_IDX16_PG(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DSR function for UART 7 */
#define LS_PINMUX_UART7_DCD_N_FUNC1_IDX17_PG(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DCD function for UART 7 */
#define LS_PINMUX_UART7_RI_N_FUNC1_IDX18_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RI function for UART 7 */
#define LS_PINMUX_UART7_RTS_N_FUNC1_IDX19_PG(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* RTS function for UART 7 */
#define LS_PINMUX_UART7_DTR_N_FUNC1_IDX20_PG(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* DTR function for UART 7 */
#define LS_PINMUX_UART7_DE_FUNC1_IDX21_PG(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* DE function for UART 7 */
#define LS_PINMUX_UART7_RE_FUNC1_IDX22_PG(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* RE function for UART 7 */
#define LS_PINMUX_FDCAN_TXD_FUNC1_IDX23_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* TX data of CAN */
#define LS_PINMUX_FDCAN_RXD_FUNC1_IDX24_PG(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* RX data of CAN */
#define LS_PINMUX_PS2IF1_CLK_FUNC1_IDX25_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* ps2if1 clock */
#define LS_PINMUX_PS2IF1_DAT_FUNC1_IDX26_PG(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* ps2if1 dat */
#define LS_PINMUX_PWM7_FUNC1_IDX27_PG(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* pwm7 */
#define LS_PINMUX_CAP7_FUNC1_IDX28_PG(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* cap7 */
#define LS_PINMUX_OWM_FUNC1_IDX29_PG(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* owm */
#define LS_PINMUX_CEC_FUNC1_IDX30_PG(XX)             ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cec */
#define LS_PINMUX_PIS_CH5_FUNC1_IDX31_PG(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PG##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 5 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PH(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM2_CH1_FUNC1_IDX1_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH1N_FUNC1_IDX2_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2_FUNC1_IDX3_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2N_FUNC1_IDX4_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3_FUNC1_IDX5_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3N_FUNC1_IDX6_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH4_FUNC1_IDX7_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM2 */
#define LS_PINMUX_ADTIM2_ETR_FUNC1_IDX8_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM2 */
#define LS_PINMUX_ADTIM2_BK_FUNC1_IDX9_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM2 */
#define LS_PINMUX_I2C8_SCL_FUNC1_IDX10_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 8 */
#define LS_PINMUX_I2C8_SDA_FUNC1_IDX11_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 8 */
#define LS_PINMUX_I2C8_SMBA_FUNC1_IDX12_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 8 */
#define LS_PINMUX_UART8_TXD_FUNC1_IDX13_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* TX data function for UART 8 */
#define LS_PINMUX_UART8_RXD_FUNC1_IDX14_PH(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RX data function for UART 8 */
#define LS_PINMUX_UART8_CTS_N_FUNC1_IDX15_PH(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* CTS function for UART 8 */
#define LS_PINMUX_UART8_DSR_N_FUNC1_IDX16_PH(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DSR function for UART 8 */
#define LS_PINMUX_UART8_DCD_N_FUNC1_IDX17_PH(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DCD function for UART 8 */
#define LS_PINMUX_UART8_RI_N_FUNC1_IDX18_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RI function for UART 8 */
#define LS_PINMUX_UART8_RTS_N_FUNC1_IDX19_PH(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* RTS function for UART 8 */
#define LS_PINMUX_UART8_DTR_N_FUNC1_IDX20_PH(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* DTR function for UART 8 */
#define LS_PINMUX_UART8_DE_FUNC1_IDX21_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* DE function for UART 8 */
#define LS_PINMUX_UART8_RE_FUNC1_IDX22_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* RE function for UART 8 */
#define LS_PINMUX_USB1_DP_FUNC1_IDX23_PH(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* DP function of USB1 */
#define LS_PINMUX_USB1_DM_FUNC1_IDX24_PH(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* DM function of USB1 */
#define LS_PINMUX_USB1_CID_FUNC1_IDX25_PH(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* CID function of USB1 */
#define LS_PINMUX_PS2IF2_CLK_FUNC1_IDX26_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* ps2if2 clock */
#define LS_PINMUX_PS2IF2_DAT_FUNC1_IDX27_PH(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* ps2if2 dat */
#define LS_PINMUX_PWM8_FUNC1_IDX28_PH(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm8 */
#define LS_PINMUX_CAP8_FUNC1_IDX29_PH(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap8 */
#define LS_PINMUX_DBG_CACHE2_MISS_FUNC1_IDX30_PH(XX) ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cache2 miss test output */
#define LS_PINMUX_DBG_CACHE2_HIT_FUNC1_IDX31_PH(XX)  ((PINMUX_FUNC1 << LS_FUNC_POS) | (PH##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* cache2 hit test output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PI(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C9_SCL_FUNC1_IDX6_PI(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 9 */
#define LS_PINMUX_I2C9_SDA_FUNC1_IDX7_PI(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 9 */
#define LS_PINMUX_I2C9_SMBA_FUNC1_IDX8_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 9 */
#define LS_PINMUX_UART9_TXD_FUNC1_IDX9_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* TX data function for UART 3 */
#define LS_PINMUX_UART9_RXD_FUNC1_IDX10_PI(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* RX data function for UART 3 */
#define LS_PINMUX_UART9_CTS_N_FUNC1_IDX11_PI(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* CTS function for UART 3 */
#define LS_PINMUX_UART9_DSR_N_FUNC1_IDX12_PI(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DSR function for UART 3 */
#define LS_PINMUX_UART9_DCD_N_FUNC1_IDX13_PI(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* DCD function for UART 3 */
#define LS_PINMUX_UART9_RI_N_FUNC1_IDX14_PI(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RI function for UART 3 */
#define LS_PINMUX_UART9_RTS_N_FUNC1_IDX15_PI(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* RTS function for UART 3 */
#define LS_PINMUX_UART9_DTR_N_FUNC1_IDX16_PI(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DTR function for UART 3 */
#define LS_PINMUX_UART9_DE_FUNC1_IDX17_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DE function for UART 3 */
#define LS_PINMUX_UART9_RE_FUNC1_IDX18_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RE function for UART 3 */
#define LS_PINMUX_SPI1_SCK_FUNC1_IDX19_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* SCK function for SPI 1 */
#define LS_PINMUX_SPI1_NSS_FUNC1_IDX20_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* chip select function for SPI 1 */
#define LS_PINMUX_SPI1_MOSI_FUNC1_IDX21_PI(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MOSI function for SPI 1 */
#define LS_PINMUX_SPI1_MISO_FUNC1_IDX22_PI(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* MISO function for SPI 1 */
#define LS_PINMUX_SPI1_I2S_FUNC1_IDX23_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* IIS clock function of SPI 1 */
#define LS_PINMUX_USB1_DP_FUNC1_IDX24_PI(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* DP function of USB1 */
#define LS_PINMUX_USB1_DM_FUNC1_IDX25_PI(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* DM function of USB1 */
#define LS_PINMUX_USB1_CID_FUNC1_IDX26_PI(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* CID function of USB1 */
#define LS_PINMUX_PWM9_FUNC1_IDX27_PI(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* pwm9 */
#define LS_PINMUX_PWM10_FUNC1_IDX28_PI(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm10 */
#define LS_PINMUX_CAP9_FUNC1_IDX29_PI(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap9 */
#define LS_PINMUX_CAP10_FUNC1_IDX30_PI(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cap10 */
#define LS_PINMUX_PIS_CH6_FUNC1_IDX31_PI(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PI##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 6 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PJ(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C10_SCL_FUNC1_IDX6_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 10 */
#define LS_PINMUX_I2C10_SDA_FUNC1_IDX7_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 10 */
#define LS_PINMUX_I2C10_SMBA_FUNC1_IDX8_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 10 */
#define LS_PINMUX_UART10_TXD_FUNC1_IDX9_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* TX data function for UART 10 */
#define LS_PINMUX_UART10_RXD_FUNC1_IDX10_PJ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* RX data function for UART 10 */
#define LS_PINMUX_UART10_CTS_N_FUNC1_IDX11_PJ(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* CTS function for UART 10 */
#define LS_PINMUX_UART10_DSR_N_FUNC1_IDX12_PJ(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* DSR function for UART 10 */
#define LS_PINMUX_UART10_DCD_N_FUNC1_IDX13_PJ(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* DCD function for UART 10 */
#define LS_PINMUX_UART10_RI_N_FUNC1_IDX14_PJ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* RI function for UART 10 */
#define LS_PINMUX_UART10_RTS_N_FUNC1_IDX15_PJ(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* RTS function for UART 10 */
#define LS_PINMUX_UART10_DTR_N_FUNC1_IDX16_PJ(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DTR function for UART 10 */
#define LS_PINMUX_UART10_DE_FUNC1_IDX17_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* DE function for UART 10 */
#define LS_PINMUX_UART10_RE_FUNC1_IDX18_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RE function for UART 10 */
#define LS_PINMUX_SPI2_SCK_FUNC1_IDX19_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* SCK function for SPI 2 */
#define LS_PINMUX_SPI2_NSS_FUNC1_IDX20_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* chip select function for SPI 2 */
#define LS_PINMUX_SPI2_MOSI_FUNC1_IDX21_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* MOSI function for SPI 2 */
#define LS_PINMUX_SPI2_MISO_FUNC1_IDX22_PJ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* MISO function for SPI 2 */
#define LS_PINMUX_SPI2_I2S_FUNC1_IDX23_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* IIS clock function of SPI 2 */
#define LS_PINMUX_MJTAG1_TRST_FUNC1_IDX24_PJ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 1 trst */
#define LS_PINMUX_MJTAG1_TDI_FUNC1_IDX25_PJ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 1 tdi */
#define LS_PINMUX_MJTAG1_TCK_FUNC1_IDX26_PJ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* jtag master 1 tck */
#define LS_PINMUX_MJTAG1_TMS_FUNC1_IDX27_PJ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* jtag master 1 tms */
#define LS_PINMUX_MJTAG1_TDO_FUNC1_IDX28_PJ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* jtag master 1 tdo */
#define LS_PINMUX_DBG_SCL1_FUNC1_IDX29_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* I2C Debug SCL1 */
#define LS_PINMUX_DBG_SDA1_FUNC1_IDX30_PJ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* I2C Debug SDA1 */
#define LS_PINMUX_PIS_CH7_FUNC1_IDX31_PJ(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PJ##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 7 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PK(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C11_SCL_FUNC1_IDX6_PK(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SCL function for IIC 11 */
#define LS_PINMUX_I2C11_SDA_FUNC1_IDX7_PK(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SDA function for IIC 11 */
#define LS_PINMUX_I2C11_SMBA_FUNC1_IDX8_PK(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SMBA function for IIC 11 */
#define LS_PINMUX_DBG_HBUS_CLK_FUNC1_IDX9_PK(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* HBUS clock test output */
#define LS_PINMUX_DBG_CACHE1_MISS_FUNC1_IDX10_PK(XX) ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* cache1 miss test output */
#define LS_PINMUX_DBG_CACHE1_HIT_FUNC1_IDX11_PK(XX)  ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* cache1 hit test output */
#define LS_PINMUX_UART11_TXD_FUNC1_IDX12_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* TX data function for UART 11 */
#define LS_PINMUX_UART11_RXD_FUNC1_IDX13_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* RX data function for UART 11 */
#define LS_PINMUX_UART11_CTS_N_FUNC1_IDX14_PK(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* CTS function for UART 11 */
#define LS_PINMUX_UART11_DSR_N_FUNC1_IDX15_PK(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* DSR function for UART 11 */
#define LS_PINMUX_UART11_DCD_N_FUNC1_IDX16_PK(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* DCD function for UART 11 */
#define LS_PINMUX_UART11_RI_N_FUNC1_IDX17_PK(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* RI function for UART 11 */
#define LS_PINMUX_UART11_RTS_N_FUNC1_IDX18_PK(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* RTS function for UART 11 */
#define LS_PINMUX_UART11_DTR_N_FUNC1_IDX19_PK(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* DTR function for UART 11 */
#define LS_PINMUX_UART11_DE_FUNC1_IDX20_PK(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* DE function for UART 11 */
#define LS_PINMUX_UART11_RE_FUNC1_IDX21_PK(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* RE function for UART 11 */
#define LS_PINMUX_MJTAG2_TRST_FUNC1_IDX22_PK(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* jtag master 2 trst */
#define LS_PINMUX_MJTAG2_TDI_FUNC1_IDX23_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* jtag master 2 tdi */
#define LS_PINMUX_MJTAG2_TCK_FUNC1_IDX24_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 2 tck */
#define LS_PINMUX_MJTAG2_TMS_FUNC1_IDX25_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 2 tms */
#define LS_PINMUX_MJTAG2_TDO_FUNC1_IDX26_PK(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* jtag master 2 tdo */
#define LS_PINMUX_DBG_SCL2_FUNC1_IDX27_PK(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* I2C Debug SCL2 */
#define LS_PINMUX_DBG_SDA2_FUNC1_IDX28_PK(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* I2C Debug SDA2 */
#define LS_PINMUX_PWM7_FUNC1_IDX29_PK(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* pwm7 */
#define LS_PINMUX_CAP7_FUNC1_IDX30_PK(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cap7 */
#define LS_PINMUX_PIS_CH8_FUNC1_IDX31_PK(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PK##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 8 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PM(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_I2C12_SCL_FUNC1_IDX5_PM(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* SCL function for IIC 12 */
#define LS_PINMUX_I2C12_SDA_FUNC1_IDX6_PM(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* SDA function for IIC 12 */
#define LS_PINMUX_I2C12_SMBA_FUNC1_IDX7_PM(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* SMBA function for IIC 12 */
#define LS_PINMUX_EMMC2_LED_CTRL_FUNC1_IDX8_PM(XX)   ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* SD card communication indicator led */
#define LS_PINMUX_EMMC1_LED_CTRL_FUNC1_IDX9_PM(XX)   ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* SD card communication indicator led */
#define LS_PINMUX_DBG_SRC_CLK_FUNC1_IDX10_PM(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* HBUS clock test output */
#define LS_PINMUX_UART12_TXD_FUNC1_IDX11_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* TX data function for UART 12 */
#define LS_PINMUX_UART12_RXD_FUNC1_IDX12_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* RX data function for UART 12 */
#define LS_PINMUX_UART12_CTS_N_FUNC1_IDX13_PM(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* CTS function for UART 12 */
#define LS_PINMUX_UART12_DSR_N_FUNC1_IDX14_PM(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* DSR function for UART 12 */
#define LS_PINMUX_UART12_DCD_N_FUNC1_IDX15_PM(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* DCD function for UART 12 */
#define LS_PINMUX_UART12_RI_N_FUNC1_IDX16_PM(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* RI function for UART 12 */
#define LS_PINMUX_UART12_RTS_N_FUNC1_IDX17_PM(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* RTS function for UART 12 */
#define LS_PINMUX_UART12_DTR_N_FUNC1_IDX18_PM(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* DTR function for UART 12 */
#define LS_PINMUX_UART12_DE_FUNC1_IDX19_PM(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* DE function for UART 12 */
#define LS_PINMUX_UART12_RE_FUNC1_IDX20_PM(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* RE function for UART 12 */
#define LS_PINMUX_MJTAG3_TRST_FUNC1_IDX21_PM(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* jtag master 3 trst */
#define LS_PINMUX_MJTAG3_TDI_FUNC1_IDX22_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* jtag master 3 tdi */
#define LS_PINMUX_MJTAG3_TCK_FUNC1_IDX23_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* jtag master 3 tck */
#define LS_PINMUX_MJTAG3_TMS_FUNC1_IDX24_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* jtag master 3 tms */
#define LS_PINMUX_MJTAG3_TDO_FUNC1_IDX25_PM(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* jtag master 3 tdo */
#define LS_PINMUX_PWM8_FUNC1_IDX26_PM(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* pwm8 */
#define LS_PINMUX_PWM9_FUNC1_IDX27_PM(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* pwm9 */
#define LS_PINMUX_PWM10_FUNC1_IDX28_PM(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* pwm10 */
#define LS_PINMUX_CAP8_FUNC1_IDX29_PM(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap8 */
#define LS_PINMUX_CAP9_FUNC1_IDX30_PM(XX)            ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* cap9 */
#define LS_PINMUX_CAP10_FUNC1_IDX31_PM(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PM##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* cap10 */
#define LS_PINMUX_RSV_FUNC1_IDX0_PN(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM1_CH1_FUNC1_IDX1_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH1N_FUNC1_IDX2_PN(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2_FUNC1_IDX3_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH2N_FUNC1_IDX4_PN(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3_FUNC1_IDX5_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH3N_FUNC1_IDX6_PN(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM1 */
#define LS_PINMUX_ADTIM1_CH4_FUNC1_IDX7_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM1 */
#define LS_PINMUX_ADTIM1_ETR_FUNC1_IDX8_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM1 */
#define LS_PINMUX_ADTIM1_BK_FUNC1_IDX9_PN(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM1 */
#define LS_PINMUX_I2C13_SCL_FUNC1_IDX10_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 13 */
#define LS_PINMUX_I2C13_SDA_FUNC1_IDX11_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 13 */
#define LS_PINMUX_I2C13_SMBA_FUNC1_IDX12_PN(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 13 */
#define LS_PINMUX_I2C14_SCL_FUNC1_IDX13_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* SCL function for IIC 14 */
#define LS_PINMUX_I2C14_SDA_FUNC1_IDX14_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* SDA function for IIC 14 */
#define LS_PINMUX_I2C14_SMBA_FUNC1_IDX15_PN(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* SMBA function for IIC 14 */
#define LS_PINMUX_UART1_TXD_FUNC1_IDX16_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* TX data function for UART 1 */
#define LS_PINMUX_UART1_RXD_FUNC1_IDX17_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* RX data function for UART 1 */
#define LS_PINMUX_UART1_CTS_N_FUNC1_IDX18_PN(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* CTS function for UART 1 */
#define LS_PINMUX_UART1_RTS_N_FUNC1_IDX19_PN(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* RTS function for UART 1 */
#define LS_PINMUX_UART3_TXD_FUNC1_IDX20_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* TX data function for UART 3 */
#define LS_PINMUX_UART3_RXD_FUNC1_IDX21_PN(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* RX data function for UART 3 */
#define LS_PINMUX_UART3_CTS_N_FUNC1_IDX22_PN(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* CTS function for UART 3 */
#define LS_PINMUX_UART3_RTS_N_FUNC1_IDX23_PN(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* RTS function for UART 3 */
#define LS_PINMUX_PWM11_FUNC1_IDX24_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* pwm11 */
#define LS_PINMUX_PWM12_FUNC1_IDX25_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* pwm12 */
#define LS_PINMUX_PWM13_FUNC1_IDX26_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* pwm13 */
#define LS_PINMUX_CAP11_FUNC1_IDX27_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* cap11 */
#define LS_PINMUX_CAP12_FUNC1_IDX28_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* cap12 */
#define LS_PINMUX_CAP13_FUNC1_IDX29_PN(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap13 */
#define LS_PINMUX_PIS_CH5_FUNC1_IDX30_PN(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* PIS channel 5 output */
#define LS_PINMUX_PIS_CH6_FUNC1_IDX31_PN(XX)         ((PINMUX_FUNC1 << LS_FUNC_POS) | (PN##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* PIS channel 6 output */
#define LS_PINMUX_RSV_FUNC1_IDX0_PQ(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_ADTIM2_CH1_FUNC1_IDX1_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* positive function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH1N_FUNC1_IDX2_PQ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* negative function of channel 1 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2_FUNC1_IDX3_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* positive function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH2N_FUNC1_IDX4_PQ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* negative function of channel 2 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3_FUNC1_IDX5_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* positive function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH3N_FUNC1_IDX6_PQ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* negative function of channel 3 of ADTIM2 */
#define LS_PINMUX_ADTIM2_CH4_FUNC1_IDX7_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* positive function of channel 4 of ADTIM2 */
#define LS_PINMUX_ADTIM2_ETR_FUNC1_IDX8_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* external trigger function of ADTIM2 */
#define LS_PINMUX_ADTIM2_BK_FUNC1_IDX9_PQ(XX)        ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* break function of ADTIM2 */
#define LS_PINMUX_I2C15_SCL_FUNC1_IDX10_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* SCL function for IIC 15 */
#define LS_PINMUX_I2C15_SDA_FUNC1_IDX11_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* SDA function for IIC 15 */
#define LS_PINMUX_I2C15_SMBA_FUNC1_IDX12_PQ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* SMBA function for IIC 15 */
#define LS_PINMUX_I2C16_SCL_FUNC1_IDX13_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* SCL function for IIC 16 */
#define LS_PINMUX_I2C16_SDA_FUNC1_IDX14_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* SDA function for IIC 16 */
#define LS_PINMUX_I2C16_SMBA_FUNC1_IDX15_PQ(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* SMBA function for IIC 16 */
#define LS_PINMUX_UART2_TXD_FUNC1_IDX16_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* TX data function for UART 2 */
#define LS_PINMUX_UART2_RXD_FUNC1_IDX17_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* RX data function for UART 2 */
#define LS_PINMUX_UART2_CTS_N_FUNC1_IDX18_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* CTS function for UART 2 */
#define LS_PINMUX_UART2_RTS_N_FUNC1_IDX19_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* RTS function for UART 2 */
#define LS_PINMUX_UART4_TXD_FUNC1_IDX20_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* TX data function for UART 4 */
#define LS_PINMUX_UART4_RXD_FUNC1_IDX21_PQ(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* RX data function for UART 4 */
#define LS_PINMUX_UART4_CTS_N_FUNC1_IDX22_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* CTS function for UART 4 */
#define LS_PINMUX_UART4_RTS_N_FUNC1_IDX23_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* RTS function for UART 4 */
#define LS_PINMUX_PWM14_FUNC1_IDX24_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* pwm14 */
#define LS_PINMUX_PWM15_FUNC1_IDX25_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* pwm15 */
#define LS_PINMUX_PWM16_FUNC1_IDX26_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* pwm16 */
#define LS_PINMUX_CAP14_FUNC1_IDX27_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* cap14 */
#define LS_PINMUX_CAP15_FUNC1_IDX28_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* cap15 */
#define LS_PINMUX_CAP16_FUNC1_IDX29_PQ(XX)           ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* cap16 */
#define LS_PINMUX_USB20_DBG14_FUNC1_IDX30_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG15_FUNC1_IDX31_PQ(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PQ##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* */
#define LS_PINMUX_RSV_FUNC1_IDX0_PT(XX)              ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (0 << LS_ALT_POS)) /* reserved for future use */
#define LS_PINMUX_PARAL_DATA0_FUNC1_IDX1_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (1 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA1_FUNC1_IDX2_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (2 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA2_FUNC1_IDX3_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (3 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA3_FUNC1_IDX4_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (4 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA4_FUNC1_IDX5_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (5 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA5_FUNC1_IDX6_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (6 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA6_FUNC1_IDX7_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (7 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_DATA7_FUNC1_IDX8_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (8 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_PERROR_FUNC1_IDX9_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (9 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_XFLAG_FUNC1_IDX10_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (10 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_BUSY_FUNC1_IDX11_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (11 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_FAULT_FUNC1_IDX12_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (12 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_ACK_FUNC1_IDX13_PT(XX)       ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (13 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_STROBE_FUNC1_IDX14_PT(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (14 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_AUTOFD_FUNC1_IDX15_PT(XX)    ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (15 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_INIT_FUNC1_IDX16_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (16 << LS_ALT_POS)) /* */
#define LS_PINMUX_PARAL_SELECTIN_FUNC1_IDX17_PT(XX)  ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (17 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG0_FUNC1_IDX18_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (18 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG1_FUNC1_IDX19_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (19 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG2_FUNC1_IDX20_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (20 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG3_FUNC1_IDX21_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (21 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG4_FUNC1_IDX22_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (22 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG5_FUNC1_IDX23_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (23 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG6_FUNC1_IDX24_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (24 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG7_FUNC1_IDX25_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (25 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG8_FUNC1_IDX26_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (26 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG9_FUNC1_IDX27_PT(XX)      ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (27 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG10_FUNC1_IDX28_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (28 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG11_FUNC1_IDX29_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (29 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG12_FUNC1_IDX30_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (30 << LS_ALT_POS)) /* */
#define LS_PINMUX_USB20_DBG13_FUNC1_IDX31_PT(XX)     ((PINMUX_FUNC1 << LS_FUNC_POS) | (PT##XX << LS_PIN_POS) | (31 << LS_ALT_POS)) /* */

#endif
