/*
 * Copyright (c) 2023 Linkedsemi Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS_PINCTRL_H_

/**
 * @brief Pin modes
 */
#define SET_GPIO_MODE_GPIO      0x00000000u     /*!< gpio Mode    */
#define SET_GPIO_MODE_ANALOG    0x00000001u     /*!< Analog Mode  */
#define SET_GPIO_MODE_AF        0x00000002u     /*!< Alternate Function Mode */
#define SET_GPIO_MODE_TEST      0x00000003u     /*!< Test Mode */

/**
 * @brief Alternate Function
 */
#define AF_JLINK_SWD            0x00
#define AF_JLINK_SCK            0x01
#define AF_UART1_CK             0x02
#define AF_UART1_TXD            0x03
#define AF_UART1_RXD            0x04
#define AF_UART1_CTSN           0x05
#define AF_UART1_RTSN           0x06
#define AF_ANT_SW0              0x07
#define AF_UART2_TXD            0x08
#define AF_UART2_RXD            0x09
#define AF_ANT_SW1              0x0a
#define AF_ANT_SW2              0x0b
#define AF_PIS_CH0_OUT          0x0c
#define AF_PIS_CH1_OUT          0x0d
#define AF_ANT_SW3              0x0e
#define AF_ANT_SW4              0x0f
#define AF_I2C1_SCL             0x10
#define AF_I2C1_SDA             0x11
#define AF_I2C1_SMBA            0x12
#define AF_I2C2_SCL             0x13
#define AF_I2C2_SDA             0x14
#define AF_SPI1_NSS0            0x15
#define AF_SPI1_SCK             0x16
#define AF_SPI1_DQ0             0x17
#define AF_SPI1_DQ1             0x18
#define AF_SPI1_DQ2             0x19
#define AF_SPI1_DQ3             0x1a
#define AF_SPI2_SCK             0x1b
#define AF_SPI2_NSS             0x1c
#define AF_SPI2_MOSI            0x1d
#define AF_SPI2_MISO            0x1e
#define AF_ADTIM1_CH1           0x1f
#define AF_ADTIM1_CH1N          0x20
#define AF_ADTIM1_CH2           0x21
#define AF_ADTIM1_CH2N          0x22
#define AF_ADTIM1_CH3           0x23
#define AF_ADTIM1_CH3N          0x24
#define AF_ADTIM1_CH4           0x25
#define AF_ADTIM1_ETR           0x26
#define AF_ADTIM1_BK            0x27
#define AF_GPTIMA1_CH1          0x28
#define AF_GPTIMA1_CH2          0x29
#define AF_GPTIMA1_CH3          0x2a
#define AF_GPTIMA1_CH4          0x2b
#define AF_GPTIMA1_ETR          0x2c
#define AF_GPTIMB1_CH1          0x2d
#define AF_GPTIMB1_CH2          0x2e
#define AF_GPTIMB1_CH3          0x2f
#define AF_GPTIMB1_CH4          0x30
#define AF_GPTIMB1_ETR          0x31
#define AF_GPTIMC1_CH1          0x32
#define AF_GPTIMC1_CH1N         0x33
#define AF_GPTIMC1_CH2          0x34
#define AF_GPTIMC1_BK           0x35
#define AF_LPTIM_OUT            0x36
#define AF_ANT_SW5              0x37
#define AF_PDM_CLK              0x38
#define AF_UART3_TXD            0x39
#define AF_UART3_RXD            0x3a
#define AF_PDM_DATA0            0x3b
#define AF_PDM_DATA1            0x3c
#define AF_ANT_SW6              0x3d
#define AF_SPI1_NSS1            0x3e
#define AF_I2S_CLK              0x3f

#define ANA_FUNC_DIS            0x40
#define ANA_FUNC1               0x41 
#define ANA_FUNC2               0x42               

/**
 * @brief Bit Masks
 */
#define LS_PORT_POS          0
#define LS_PORT_MASK         0xf

#define LS_PIN_POS	         4
#define LS_PIN_MASK          0xf

#define LS_PULL_DOWN_POS     8
#define LS_PULL_DOWN_MASK    0x1

#define LS_PULL_UP_POS       9
#define LS_PULL_UP_MASK      0x1

#define LS_PUSH_PULL_POS     10
#define LS_PUSH_PULL_MASK    0x1

#define LS_OPEN_DRAIN_POS	 11
#define LS_OPEN_DRAIN_MASK   0x1

#define LS_CFG_INPUT_POS     12
#define LS_CFG_INPUT_MASK    0x1

#define LS_CFG_OUTPUT_POS    13
#define LS_CFG_OUTPUT_MASK   0x1

#define LS_OUT_HIGH_POS      14
#define LS_OUT_HIGH_MASK     0x1

#define LS_OUT_LOW_POS       15
#define LS_OUT_LOW_MASK      0x1

#define LS_DRIVE_POS         16
#define LS_DRIVE_MASK        0x3

#define LS_ALT_POS	         18
#define LS_ALT_MASK          0x7f

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */
#define LSPIN(_port, _pin)      (_port << 4 | _pin)

/**
 * @brief pinmux setters and getters
 */
#define LS_PINMUX_SET(port, pin, alt_fun)              \
	(((port) - 'A')  << LS_PORT_POS | (pin) << LS_PIN_POS | (alt_fun) << LS_ALT_POS)

#define LS_PINMUX_GET_PORT(mx)	       ((mx >> LS_PORT_POS)       & LS_PORT_MASK)
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

#endif  /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_LS_PINCTRL_H_ */
