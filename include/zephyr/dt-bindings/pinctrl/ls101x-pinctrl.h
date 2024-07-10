/*
 * Copyright (c) 2024 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS101X_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS101X_PINCTRL_H_

#define FIO_GPTIMA1_CH1     1
#define FIO_GPTIMA1_CH2     2
#define FIO_GPTIMA1_CH3     3
#define FIO_GPTIMA1_CH4     4
#define FIO_GPTIMA1_ETR     5
#define FIO_GPTIMA2_CH1     6
#define FIO_GPTIMA2_CH2     7
#define FIO_GPTIMA2_CH3     8
#define FIO_GPTIMA2_CH4     9
#define FIO_GPTIMA2_ETR     10
#define FIO_GPTIMB1_CH1     11
#define FIO_GPTIMB1_CH2     12
#define FIO_GPTIMB1_CH3     13
#define FIO_GPTIMB1_CH4     14
#define FIO_GPTIMB1_ETR     15
#define FIO_GPTIMC1_CH1     16
#define FIO_GPTIMC1_CH1N    17
#define FIO_GPTIMC1_CH2     18
#define FIO_GPTIMC1_BK      19
#define FIO_ADTIM1_CH1      20
#define FIO_ADTIM1_CH1N     21
#define FIO_ADTIM1_CH2      22
#define FIO_ADTIM1_CH2N     23
#define FIO_ADTIM1_CH3      24
#define FIO_ADTIM1_CH3N     25
#define FIO_ADTIM1_CH4      26
#define FIO_ADTIM1_ETR      27
#define FIO_ADTIM1_BK       28
#define FIO_ADTIM2_CH1      29
#define FIO_ADTIM2_CH1N     30
#define FIO_ADTIM2_CH2      31
#define FIO_ADTIM2_CH2N     32
#define FIO_ADTIM2_CH3      33
#define FIO_ADTIM2_CH3N     34
#define FIO_ADTIM2_CH4      35
#define FIO_ADTIM2_ETR      36
#define FIO_ADTIM2_BK       37
#define FIO_ADTIM3_CH1      38
#define FIO_ADTIM3_CH1N     39
#define FIO_ADTIM3_CH2      40
#define FIO_ADTIM3_CH2N     41
#define FIO_ADTIM3_CH3      42
#define FIO_ADTIM3_CH3N     43
#define FIO_ADTIM3_CH4      44
#define FIO_ADTIM3_ETR      45
#define FIO_ADTIM3_BK       46
#define FIO_ADTIM4_CH1      47
#define FIO_ADTIM4_CH1N     48
#define FIO_ADTIM4_CH2      49
#define FIO_ADTIM4_CH2N     50
#define FIO_ADTIM4_CH3      51
#define FIO_ADTIM4_CH3N     52
#define FIO_ADTIM4_CH4      53
#define FIO_ADTIM4_ETR      54
#define FIO_ADTIM4_BK       55
#define FIO_I2C1_SCL        56
#define FIO_I2C1_SDA        57
#define FIO_I2C1_SMBA       58
#define FIO_I2C2_SCL        59
#define FIO_I2C2_SDA        60
#define FIO_I2C2_SMBA       61
#define FIO_I2C3_SDA        62
#define FIO_I2C3_SCL        63
#define FIO_I2C3_SMBA       64
#define FIO_I2C4_SCL        65
#define FIO_I2C4_SDA        66
#define FIO_I2C4_SMBA       67
#define FIO_I2C5_SCL        68
#define FIO_I2C5_SDA        69
#define FIO_I2C5_SMBA       70
#define FIO_I2C6_SDA        71
#define FIO_I2C6_SCL        72
#define FIO_I2C6_SMBA       73
#define FIO_UART1_TXD       74
#define FIO_UART1_RXD       75
#define FIO_UART1_CTSN      76
#define FIO_UART1_RTSN      77
#define FIO_UART1_CK        78
#define FIO_UART2_TXD       79
#define FIO_UART2_RXD       80
#define FIO_UART2_CTSN      81
#define FIO_UART2_RTSN      82
#define FIO_UART3_TXD       83
#define FIO_UART3_RXD       84
#define FIO_UART3_CTSN      85
#define FIO_UART3_RTSN      86
#define FIO_DWUART1_TXD       87
#define FIO_DWUART1_RXD       88
#define FIO_DWUART1_CTSN      89
#define FIO_DWUART1_RTSN      90
#define FIO_DWUART1_DE        91
#define FIO_DWUART1_RE        92
#define FIO_DWUART2_TXD       93
#define FIO_DWUART2_RXD       94
#define FIO_DWUART2_CTSN      95
#define FIO_DWUART2_RTSN      96
#define FIO_DWUART2_DE        97
#define FIO_DWUART2_RE        98
#define FIO_SPI1_NSS0       99
#define FIO_SPI1_NSS1       100
#define FIO_SPI1_SCK        101
#define FIO_SPI1_DQ0        102
#define FIO_SPI1_DQ1        103
#define FIO_SPI1_DQ2        104
#define FIO_SPI1_DQ3        105
#define FIO_SPI2_SCK        106
#define FIO_SPI2_NSS        107
#define FIO_SPI2_MOSI       108
#define FIO_SPI2_MISO       109
#define FIO_SPI2_I2S        110
#define FIO_SPI3_SCK        111
#define FIO_SPI3_NSS        112
#define FIO_SPI3_MOSI       113
#define FIO_SPI3_MISO       114
#define FIO_SPI3_I2S        115
#define FIO_PDM_CLK         116
#define FIO_PDM_DATA0       117
#define FIO_PDM_DATA1       118
#define FIO_PIS_CH0         119
#define FIO_PIS_CH1         120
#define FIO_FDCAN_TXD       121
#define FIO_FDCAN_RXD       122
#define FIO_USB_DP          123
#define FIO_USB_DM          124
#define FIO_COMP1           125
#define FIO_COMP2           126
#define FIO_COMP3           127
#define FIO_PWM1            128
#define FIO_PWM2            129
#define FIO_PWM3            130
#define FIO_PWM4            131
#define FIO_PWM5            132
#define FIO_PWM6            133
#define FIO_PWM7            134
#define FIO_PWM8            135
#define FIO_CAP1            136
#define FIO_CAP2            137
#define FIO_CAP3            138
#define FIO_CAP4            139
#define FIO_CAP5            140
#define FIO_CAP6            141
#define FIO_CAP7            142
#define FIO_CAP8            143
#define FIO_OWM             144
#define FIO_PS2H1_CLK      145
#define FIO_PS2H1_DAT      146
#define FIO_PS2H2_CLK      147
#define FIO_PS2H2_DAT      148
#define FIO_PS2H3_CLK      149
#define FIO_PS2H3_DAT      150
#define FIO_PS2H4_CLK      151
#define FIO_PS2H4_DAT      152
#define FIO_CEC             153
#define FIO_PECI            154
#define FIO_I2C_DBG_SCL     155
#define FIO_I2C_DBG_SDA     156
#define FIO_USB_CID         163

#define FIO_ADC				164
#define FIO_DAC				166



/* GP TIM A */
#define GPTIMA1_CH1  FIO_GPTIMA1_CH1
#define GPTIMA1_CH2  FIO_GPTIMA1_CH2
#define GPTIMA1_CH3  FIO_GPTIMA1_CH3
#define GPTIMA1_CH4  FIO_GPTIMA1_CH4
#define GPTIMA1_ETR  FIO_GPTIMA1_ETR
/* GP TIM B */
#define GPTIMB1_CH1  FIO_GPTIMB1_CH1
#define GPTIMB1_CH2  FIO_GPTIMB1_CH2
#define GPTIMB1_CH3  FIO_GPTIMB1_CH3
#define GPTIMB1_CH4  FIO_GPTIMB1_CH4
#define GPTIMB1_ETR  FIO_GPTIMB1_ETR
/* GP TIM C */
#define GPTIMC1_CH1  FIO_GPTIMC1_CH1
#define GPTIMC1_CH1N  FIO_GPTIMC1_CH1N
#define GPTIMC1_CH2  FIO_GPTIMC1_CH2
#define GPTIMC1_BK   FIO_GPTIMC1_BK
/* AD TIM 1 */
#define ADTIM1_CH1   FIO_ADTIM1_CH1
#define ADTIM1_CH1N  FIO_ADTIM1_CH1N
#define ADTIM1_CH2   FIO_ADTIM1_CH2
#define ADTIM1_CH2N  FIO_ADTIM1_CH2N
#define ADTIM1_CH3   FIO_ADTIM1_CH3
#define ADTIM1_CH3N  FIO_ADTIM1_CH3N
#define ADTIM1_CH4   FIO_ADTIM1_CH4
#define ADTIM1_ETR   FIO_ADTIM1_ETR
#define ADTIM1_BK    FIO_ADTIM1_BK
/* AD TIM 2 */
#define ADTIM2_CH1   FIO_ADTIM2_CH1
#define ADTIM2_CH1N  FIO_ADTIM2_CH1N
#define ADTIM2_CH2   FIO_ADTIM2_CH2
#define ADTIM2_CH2N  FIO_ADTIM2_CH2N
#define ADTIM2_CH3   FIO_ADTIM2_CH3
#define ADTIM2_CH3N  FIO_ADTIM2_CH3N
#define ADTIM2_CH4   FIO_ADTIM2_CH4
#define ADTIM2_ETR   FIO_ADTIM2_ETR
#define ADTIM2_BK    FIO_ADTIM2_BK
/* I2C 1 */
#define IIC1_SCL     FIO_I2C1_SCL
#define IIC1_SDA     FIO_I2C1_SDA
#define IIC1_SMBA    FIO_I2C1_SMBA
/* I2C 2 */
#define IIC2_SCL     FIO_I2C2_SCL
#define IIC2_SDA     FIO_I2C2_SDA
#define IIC2_SMBA    FIO_I2C2_SMBA
/* I2C 3 */
#define IIC3_SDA     FIO_I2C3_SDA
#define IIC3_SCL     FIO_I2C3_SCL
#define IIC3_SMBA    FIO_I2C3_SMBA
/* I2C 4 */
#define IIC4_SCL     FIO_I2C4_SCL
#define IIC4_SDA     FIO_I2C4_SDA
#define IIC4_SMBA    FIO_I2C4_SMBA
/* I2C 5 */
#define IIC5_SCL     FIO_I2C5_SCL
#define IIC5_SDA     FIO_I2C5_SDA
#define IIC5_SMBA    FIO_I2C5_SMBA
/* I2C 6 */
#define IIC6_SDA     FIO_I2C6_SDA
#define IIC6_SCL     FIO_I2C6_SCL
#define IIC6_SMBA    FIO_I2C6_SMBA
/* UART 1 */
#define UART1_TXD    FIO_UART1_TXD
#define UART1_RXD    FIO_UART1_RXD
#define UART1_CTSN   FIO_UART1_CTSN
#define UART1_RTSN   FIO_UART1_RTSN
#define UART1_CK     FIO_UART1_CK
/* UART 2 */
#define UART2_TXD    FIO_UART2_TXD
#define UART2_RXD    FIO_UART2_RXD
/* UART 3 */
#define UART3_TXD    FIO_UART3_TXD
#define UART3_RXD    FIO_UART3_RXD
/* UART 4 */
#define DWUART1_TXD    FIO_DWUART1_TXD
#define DWUART1_RXD    FIO_DWUART1_RXD
/* UART 5 */
#define DWUART2_TXD    FIO_DWUART2_TXD
#define DWUART2_RXD    FIO_DWUART2_RXD
/* SPI 1 */
#define SPI1_NSS0    FIO_SPI1_NSS0
#define SPI1_NSS1    FIO_SPI1_NSS1
#define SPI1_SCK     FIO_SPI1_SCK
#define SPI1_DQ0     FIO_SPI1_DQ0
#define SPI1_DQ1     FIO_SPI1_DQ1
#define SPI1_DQ2     FIO_SPI1_DQ2
#define SPI1_DQ3     FIO_SPI1_DQ3
/* SPI 2 */
#define SPI2_SCK     FIO_SPI2_SCK
#define SPI2_NSS     FIO_SPI2_NSS
#define SPI2_MOSI    FIO_SPI2_MOSI
#define SPI2_MISO    FIO_SPI2_MISO
#define SPI2_IIS     FIO_SPI2_I2S
/* SPI 3 */
#define SPI3_SCK     FIO_SPI3_SCK
#define SPI3_NSS     FIO_SPI3_NSS
#define SPI3_MOSI    FIO_SPI3_MOSI
#define SPI3_MISO    FIO_SPI3_MISO
#define SPI3_IIS     FIO_SPI3_I2S
/* PDM */
#define PDM_CLK      FIO_PDM_CLK
#define PDM_DATA0    FIO_PDM_DATA0
#define PDM_DATA1    FIO_PDM_DATA1
/* PIS */
#define PIS_CH0      FIO_PIS_CH0
#define PIS_CH1      FIO_PIS_CH1
/* FDCAN */
#define FDCAN_TXD    FIO_FDCAN_TXD
#define FDCAN_RXD    FIO_FDCAN_RXD
/* USB */
#define USB_DP       FIO_USB_DP
#define USB_DM       FIO_USB_DM
#define USB_CID       FIO_USB_CID
/* COMPx */
#define COMP1        FIO_COMP1
#define COMP2        FIO_COMP2
#define COMP3        FIO_COMP3
/* I2C_DBG */
#define I2C_DBG_CLK  FIO_I2C_DBG_SCL
#define I2C_DBG_DAT  FIO_I2C_DBG_SDA
/* CEC */
#define CEC_CH       FIO_CEC
/* PS2 */
#define PS2H1_CLK    FIO_PS2H1_CLK
#define PS2H1_DAT    FIO_PS2H1_DAT
#define PS2H2_CLK    FIO_PS2H2_CLK
#define PS2H2_DAT    FIO_PS2H2_DAT
#define PS2H3_CLK    FIO_PS2H3_CLK
#define PS2H3_DAT    FIO_PS2H3_DAT
#define PS2H4_CLK    FIO_PS2H4_CLK
#define PS2H4_DAT    FIO_PS2H4_DAT
/* OWM */
#define OWM_CH       FIO_OWM
/* PWM */
#define PWM_CH1      FIO_PWM1
#define PWM_CH2      FIO_PWM2
#define PWM_CH3      FIO_PWM3
#define PWM_CH4      FIO_PWM4
#define PWM_CH5      FIO_PWM5
#define PWM_CH6      FIO_PWM6
#define PWM_CH7      FIO_PWM7
#define PWM_CH8      FIO_PWM8
/* PECI */
#define PECI_DAT     FIO_PECI

// RSV07
// ANT0
// ANT1
// ANT2
// ANT3
// ANT4
// ANT5
// ANT6
// ANT7

/**
 * @brief Bit Masks
 */
#define LS_PORT_POS				0
#define LS_PORT_MASK       		0xf

#define LS_PIN_POS				4
#define LS_PIN_MASK				0xf

#define LS_PULL_DOWN_POS		8     
#define LS_PULL_DOWN_MASK		0x1

#define LS_PULL_UP_POS			9     
#define LS_PULL_UP_MASK			0x1

#define LS_PUSH_PULL_POS        10
#define LS_PUSH_PULL_MASK    	0x1

#define LS_OPEN_DRAIN_POS	 	11
#define LS_OPEN_DRAIN_MASK 		0x1  

#define LS_CFG_INPUT_POS     	12
#define LS_CFG_INPUT_MASK		0x1    

#define LS_CFG_OUTPUT_POS		13    
#define LS_CFG_OUTPUT_MASK		0x1   

#define LS_OUT_HIGH_POS      	14
#define LS_OUT_HIGH_MASK     	0x1

#define LS_OUT_LOW_POS			15       
#define LS_OUT_LOW_MASK			0x1      

#define LS_DRIVE_POS    		16     
#define LS_DRIVE_MASK  			0x3  

#define LS_DIN_POS				18
#define LS_DIN_MASK				0x1 

#define LS_ALT_POS	         	19
#define LS_ALT_MASK         	0xff

/* Encode the IO port pin into a numeric format */
#define		LSPIN(port, pin)	((port) << 4 | (pin))

/* Set and get macro definitions for the pin reuse configuration */

#define LS_PINMUX_SET(port, pin, alt_fun)			\
	(((port) - 'A') << LS_PORT_POS | (pin) << LS_PIN_POS | (alt_fun) << LS_ALT_POS)

#define	LS_PINMUX_GET_PORT(mx)		   ((mx >> LS_PORT_POS)       & LS_PORT_MASK)
#define LS_PINMUX_GET_PIN(mx)	       ((mx >> LS_PIN_POS)        & LS_PIN_MASK)
#define LS_PINMUX_GET_PULL_DOWN(mx)    ((mx >> LS_PULL_DOWN_POS)  & LS_PULL_DOWN_MASK)
#define LS_PINMUX_GET_PULL_UP(mx)      ((mx >> LS_PULL_UP_POS)    & LS_PULL_UP_MASK)
#define LS_PINMUX_GET_PUSH_PULL(mx)    ((mx >> LS_PUSH_PULL_POS)  & LS_PUSH_PULL_MASK)
#define LS_PINMUX_GET_OPEN_DRAIN(mx)   ((mx >> LS_OPEN_DRAIN_POS) & LS_OPEN_DRAIN_MASK)
#define LS_PINMUX_GET_OUT_HIGH(mx)	   ((mx >> LS_OUT_HIGH_POS)   & LS_OUT_HIGH_MASK)
#define LS_PINMUX_GET_OUT_LOW(mx)	   ((mx >> LS_OUT_LOW_POS)    & LS_OUT_LOW_MASK)
#define LS_PINMUX_GET_INPUT(mx)        ((mx >> LS_CFG_INPUT_POS)  & LS_CFG_INPUT_MASK)
#define LS_PINMUX_GET_OUTPUT(mx)       ((mx >> LS_CFG_OUTPUT_POS) & LS_CFG_OUTPUT_MASK)
#define LS_PINMUX_GET_DRIVE(mx)	       ((mx >> LS_DRIVE_POS)      & LS_DRIVE_MASK)
#define LS_PINMUX_GET_ALT(mx)	       ((mx >> LS_ALT_POS)        & LS_ALT_MASK)
#define LS_PINMUX_GET_ANALOG(mx)	   ((mx >> LS_ANALOG_EN_POS)  & LS_ANALOG_EN_MASK)
#define LS_PINMUX_GET_LOCK(mx)	       ((mx >> LS_LOCK_POS)       & LS_LOCK_MASK)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS101X_PINCTRL_H_ */