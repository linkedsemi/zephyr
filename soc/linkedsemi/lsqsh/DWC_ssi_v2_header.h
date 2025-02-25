#ifndef DWC_SSI_V2_HEADER_H_
#define DWC_SSI_V2_HEADER_H_
/*
 * ABSTRACT : Register definition header file for 
 */
#define ssic_address_block_BaseAddress 0x0

/* Register CTRLR0 */
/* This register controls the serial data transfer. It is impossible to write to this register when the DWC_ssi is enabled. */
#define SSIV2_CTRLR0 (ssic_address_block_BaseAddress + 0x0)
#define SSIV2_CTRLR0_RegisterSize 32
#define SSIV2_CTRLR0_RegisterResetValue 0x4007
#define SSIV2_CTRLR0_RegisterResetMask 0xffffffff

/* Register Field information for CTRLR0 */

/* Register CTRLR0 field DFS */
/* Data Frame Size.
Selects the data frame length. When the data frame size is programmed to be less than 32 bits, the receive data is automatically right-justified by the receive logic, with the upper bits of the receive FIFO zero-padded.
You must right-justify transmit data before writing into the transmit FIFO. The transmit logic ignores the upper unused bits when transmitting the data. 

Note: When SSIC_SPI_MODE is set to "Dual", "Quad" or "Octal" mode and SPI_FRF is not set to 2'b00:
 - DFS value must be a multiple of 2 if SPI_FRF = 01
 - DFS value must be multiple of 4 if SPI_FRF = 10
 - DFS value must be multiple of 8 if SPI_FRF = 11 */
#define SSIV2_CTRLR0_DFS_BitAddressOffset 0
#define SSIV2_CTRLR0_DFS_RegisterSize 5

/* Register CTRLR0 field RSVD_CTRLR0_5 */
/* Reserved bits - read as zero */
#define SSIV2_CTRLR0_RSVD_CTRLR0_5_BitAddressOffset 5
#define SSIV2_CTRLR0_RSVD_CTRLR0_5_RegisterSize 1

/* Register CTRLR0 field FRF */
/* Frame Format.
Selects which serial protocol transfers the data. */
#define SSIV2_CTRLR0_FRF_BitAddressOffset 6
#define SSIV2_CTRLR0_FRF_RegisterSize 2

/* Register CTRLR0 field SCPH */
/* Serial Clock Phase.
Valid when the frame format (FRF) is set to Motorola SPI. The serial clock phase selects the relationship of the serial clock with the slave select signal.
When SCPH = 0, data are captured on the first edge of the serial clock. When SCPH = 1, the serial clock starts toggling one cycle after the slave select line is activated, and data are captured on the second edge of the serial clock. */
#define SSIV2_CTRLR0_SCPH_BitAddressOffset 8
#define SSIV2_CTRLR0_SCPH_RegisterSize 1

/* Register CTRLR0 field SCPOL */
/* Serial Clock Polarity.
Valid when the frame format (FRF) is set to Motorola SPI. Used to select the polarity of the inactive serial clock, which is held inactive when the DWC_ssi master is not actively transferring data on the serial bus. */
#define SSIV2_CTRLR0_SCPOL_BitAddressOffset 9
#define SSIV2_CTRLR0_SCPOL_RegisterSize 1

/* Register CTRLR0 field TMOD */
/* Transfer Mode.
Selects the mode of transfer for serial communication. This field does not affect the transfer duplicity. Only indicates whether the receive or transmit data are valid.
In transmit-only mode, data received from the external device is not valid and is not stored in the receive FIFO memory; it is overwritten on the next transfer.
In receive-only mode, transmitted data are not valid. After the first write to the transmit FIFO, the same word is retransmitted for the duration of the transfer.
In transmit-and-receive mode, both transmit and receive data are valid. The transfer continues until the transmit FIFO is empty. Data received from the external device are stored into the receive FIFO memory, where it can be accessed by the host processor. */
#define SSIV2_CTRLR0_TMOD_BitAddressOffset 10
#define SSIV2_CTRLR0_TMOD_RegisterSize 2

/* Register CTRLR0 field SLV_OE */
/* Slave Output Enable.
Relevant only when the DWC_ssi is configured as a serial-slave
device. When configured as a serial master, this bit field has no functionality. This bit enables or disables the setting of the ssi_oe_n output from the DWC_ssi serial slave. When SLV_OE = 1, the ssi_oe_n output can never be active. When the ssi_oe_n output controls the tri-state buffer on the txd output from the slave, a high impedance state is always present on the slave txd output when SLV_OE = 1.
This is useful when the master transmits in broadcast mode (master transmits data to all slave devices). Only one slave may respond with data on the master rxd line. This bit is enabled after reset and must be disabled by software (when broadcast mode is used), if you do not want this device to respond with data.
When SSIC_SLV_SPI_MODE is set to 1 and SPI is programmed to work in Enhanced SPI mode, then for correct operation this bit should be programmed to 0. */
#define SSIV2_CTRLR0_SLV_OE_BitAddressOffset 12
#define SSIV2_CTRLR0_SLV_OE_RegisterSize 1

/* Register CTRLR0 field SRL */
/* Shift Register Loop.
Used for testing purposes only. When internally
active, connects the transmit shift register output to the receive shift register input. Can be used in both serial-slave and serial-master modes. When the DWC_ssi is configured as a slave in loopback mode, the ss_in_n and ssi_clk signals must be provided by an external source. In this mode, the slave cannot generate these signals because there is nothing to which to loop back. */
#define SSIV2_CTRLR0_SRL_BitAddressOffset 13
#define SSIV2_CTRLR0_SRL_RegisterSize 1

/* Register CTRLR0 field SSTE */
/* Slave Select Toggle Enable.
While operating in SPI mode with clock phase (SCPH) set to 0, this register controls the behavior of the slave select line (ss_*_n) between data frames. */
#define SSIV2_CTRLR0_SSTE_BitAddressOffset 14
#define SSIV2_CTRLR0_SSTE_RegisterSize 1

/* Register CTRLR0 field RSVD_CTRLR0_15 */
/* Reserved bits - read as zero */
#define SSIV2_CTRLR0_RSVD_CTRLR0_15_BitAddressOffset 15
#define SSIV2_CTRLR0_RSVD_CTRLR0_15_RegisterSize 1

/* Register CTRLR0 field CFS */
/* Control Frame Size.
Selects the length of the control word for the Microwire frame format. */
#define SSIV2_CTRLR0_CFS_BitAddressOffset 16
#define SSIV2_CTRLR0_CFS_RegisterSize 4

/* Register CTRLR0 field RSVD_CTRLR0_20_21 */
/* Reserved bits - read as zero */
#define SSIV2_CTRLR0_RSVD_CTRLR0_20_21_BitAddressOffset 20
#define SSIV2_CTRLR0_RSVD_CTRLR0_20_21_RegisterSize 2

/* Register CTRLR0 field SPI_FRF */
/* SPI Frame Format
Selects data frame format for Transmitting/Receiving the data. Bits only valid when SSIC_SPI_MODE is either set to "Dual" or "Quad" or "Octal" mode. */
#define SSIV2_CTRLR0_SPI_FRF_BitAddressOffset 22
#define SSIV2_CTRLR0_SPI_FRF_RegisterSize 2

/* Register CTRLR0 field SPI_HYPERBUS_EN */
/* SPI Hyperbus Frame format enable.
Selects if data frame format for Transmitting/Receiving the data is in Hyperbus mode. This field is effective only when CTRLRO.FRF is set to SPI frame format. */
#define SSIV2_CTRLR0_SPI_HYPERBUS_EN_BitAddressOffset 24
#define SSIV2_CTRLR0_SPI_HYPERBUS_EN_RegisterSize 1

/* Register CTRLR0 field SPI_DWS_EN */
/* Enable Dynamic wait states in SPI mode of operation. This field is only applicable when CTRLR0.FRF is set to 0 (Motorola SPI Frame Format). */
#define SSIV2_CTRLR0_SPI_DWS_EN_BitAddressOffset 25
#define SSIV2_CTRLR0_SPI_DWS_EN_RegisterSize 1

/* Register CTRLR0 field RSVD_CTRLR0_26_31 */
/* Reserved bits - read as zero */
#define SSIV2_CTRLR0_RSVD_CTRLR0_26_31_BitAddressOffset 26
#define SSIV2_CTRLR0_RSVD_CTRLR0_26_31_RegisterSize 5

/* Register CTRLR0 field SSI_IS_MST */
/* This field selects if DWC_ssi is working in Master or Slave mode */
#define SSIV2_CTRLR0_SSI_IS_MST_BitAddressOffset 31
#define SSIV2_CTRLR0_SSI_IS_MST_RegisterSize 1

/* End of Register Definition for CTRLR0 */

/* Register CTRLR1 */
/* This register exists only when the DWC_ssi is configured as a master device. When the DWC_ssi is configured as a serial slave, writing to this location has no effect; reading from this location returns 0. Control register 1 controls the end of serial transfers when in receive-only mode. It is impossible to write to this register when the DWC_ssi is enabled. */
#define SSIV2_CTRLR1 (ssic_address_block_BaseAddress + 0x4)
#define SSIV2_CTRLR1_RegisterSize 32
#define SSIV2_CTRLR1_RegisterResetValue 0x0
#define SSIV2_CTRLR1_RegisterResetMask 0xffffffff

/* Register Field information for CTRLR1 */

/* Register CTRLR1 field NDF */
/* Number of Data Frames.
When TMOD = 10 or TMOD = 11 , this register field sets the number of data frames to be continuously received by the DWC_ssi. The DWC_ssi continues to receive serial data until the number of data frames received is equal to this register value plus 1, which enables you to receive up to 64 KB of data in a continuous transfer.
 When SPI_CTRLR0.CLK_STRETCH_EN=1 and TMOD = 01, this register field sets the number of data frames to be continously transmitted by DWC_ssi. If the Transmit FIFO goes empty in-between, DWC_ssi masks the serial clock (sclk_out) and wait for rest of the data until the programmed amount of frames are transferred successfully.
When the DWC_ssi is configured as a serial slave, the transfer continues for as long as the slave is selected. Therefore, this register serves no purpose and is not present when the DWC_ssi is configured as a serial slave. */
#define SSIV2_CTRLR1_NDF_BitAddressOffset 0
#define SSIV2_CTRLR1_NDF_RegisterSize 16

/* Register CTRLR1 field RSVD_CTRLR1 */
/* Reserved bits - read as zero */
#define SSIV2_CTRLR1_RSVD_CTRLR1_BitAddressOffset 16
#define SSIV2_CTRLR1_RSVD_CTRLR1_RegisterSize 16

/* End of Register Definition for CTRLR1 */

/* Register SSIENR */
/* This register enables and disables the DWC_ssi. */
#define SSIV2_SSIENR (ssic_address_block_BaseAddress + 0x8)
#define SSIV2_SSIENR_RegisterSize 32
#define SSIV2_SSIENR_RegisterResetValue 0x0
#define SSIV2_SSIENR_RegisterResetMask 0xffffffff

/* Register Field information for SSIENR */

/* Register SSIENR field SSIC_EN */
/* SSI Enable.
Enables and disables all DWC_ssi operations. When disabled, all serial transfers are halted immediately. Transmit and receive FIFO buffers are cleared when the device is disabled. It is impossible to program some of the DWC_ssi control registers when enabled. When disabled, the ssi sleep output is set (after delay) to inform the system that it is safe to remove the ssi_clk, thus saving power consumption in the system. */
#define SSIV2_SSIENR_SSIC_EN_BitAddressOffset 0
#define SSIV2_SSIENR_SSIC_EN_RegisterSize 1

/* Register SSIENR field RSVD_SSIENR */
/* Reserved bits - read as zero */
#define SSIV2_SSIENR_RSVD_SSIENR_BitAddressOffset 1
#define SSIV2_SSIENR_RSVD_SSIENR_RegisterSize 31

/* End of Register Definition for SSIENR */

/* Register MWCR */
/* This register controls the direction of the data word for the half-duplex Microwire serial protocol. It is impossible to write to this register when the DWC_ssi is enabled. */
#define SSIV2_MWCR (ssic_address_block_BaseAddress + 0xc)
#define SSIV2_MWCR_RegisterSize 32
#define SSIV2_MWCR_RegisterResetValue 0x0
#define SSIV2_MWCR_RegisterResetMask 0xffffffff

/* Register Field information for MWCR */

/* Register MWCR field MWMOD */
/* Microwire Transfer Mode.
Defines whether the Microwire transfer is sequential or non-sequential. When sequential mode is used, only one control word is needed to transmit or receive a block of data words. When non-sequential mode is used, there must be a control word for each data word that is transmitted or received. */
#define SSIV2_MWCR_MWMOD_BitAddressOffset 0
#define SSIV2_MWCR_MWMOD_RegisterSize 1

/* Register MWCR field MDD */
/* Microwire Control.
Defines the direction of the data word when the Microwire serial protocol is used. When this bit is set to 0, the data word is received by the DWC_ssi MacroCell from the external serial device. When this bit is set to 1, the data word is transmitted from the DWC_ssi MacroCell to the external serial device. */
#define SSIV2_MWCR_MDD_BitAddressOffset 1
#define SSIV2_MWCR_MDD_RegisterSize 1

/* Register MWCR field MHS */
/* Microwire Handshaking.
Relevant only when the DWC_ssi is configured as a serial-master device. When configured as a serial slave, this bit field has no functionality. Used to enable and disable the busy/ready handshaking interface for the Microwire protocol. When enabled, the DWC_ssi checks for a ready status from the target slave, after the transfer of the last data/control bit, before clearing the BUSY status in the SR register. */
#define SSIV2_MWCR_MHS_BitAddressOffset 2
#define SSIV2_MWCR_MHS_RegisterSize 1

/* Register MWCR field RSVD_MWCR */
/* Reserved bits - read as zero */
#define SSIV2_MWCR_RSVD_MWCR_BitAddressOffset 3
#define SSIV2_MWCR_RSVD_MWCR_RegisterSize 29

/* End of Register Definition for MWCR */

/* Register SER */
/* This register is valid only when the DWC_ssi is configured as a master device. When the DWC_ssi is configured as a serial slave, writing to this location has no effect; reading from this location returns 0. The register enables the individual slave select output lines from the DWC_ssi master. Up to 16 slave-select output pins are available on the DWC_ssi master. You cannot write to this register when DWC_ssi is busy and when SSIC_EN = 1. */
#define SSIV2_SER (ssic_address_block_BaseAddress + 0x10)
#define SSIV2_SER_RegisterSize 32
#define SSIV2_SER_RegisterResetValue 0x0
#define SSIV2_SER_RegisterResetMask 0xffffffff

/* Register Field information for SER */

/* Register SER field SER */
/* Slave Select Enable Flag.
Each bit in this register corresponds to a slave select line (ss_x_n) from the DWC_ssi master. When a bit in this register is set (1), the corresponding slave select line from the master is activated when a serial transfer begins. It should be noted that setting or clearing bits in this register have no effect on the corresponding slave select outputs until a transfer is started. Before beginning a transfer, you should enable the bit in this register that corresponds to the slave device with which the master wants to communicate. When not operating in broadcast mode, only one bit in this field should be set. */
#define SSIV2_SER_SER_BitAddressOffset 0
#define SSIV2_SER_SER_RegisterSize 2

/* Register SER field RSVD_SER */
/* Reserved bits - read as zero */
#define SSIV2_SER_RSVD_SER_BitAddressOffset 2
#define SSIV2_SER_RSVD_SER_RegisterSize 30

/* End of Register Definition for SER */

/* Register BAUDR */
/* This register is valid only when the DWC_ssi is configured as a master device. When the DWC_ssi is configured as a serial slave, writing to this location has no effect; reading from this location returns 0. The register derives the frequency of the serial clock that regulates the data transfer. The 16-bit field in this register defines the ssi_clk divider value. It is impossible to write to this register when the DWC_ssi is enabled. */
#define SSIV2_BAUDR (ssic_address_block_BaseAddress + 0x14)
#define SSIV2_BAUDR_RegisterSize 32
#define SSIV2_BAUDR_RegisterResetValue 0x0
#define SSIV2_BAUDR_RegisterResetMask 0xffffffff

/* Register Field information for BAUDR */

/* Register BAUDR field RSVD_BAUDR_0 */
/* Reserved bits - read as zero */
#define SSIV2_BAUDR_RSVD_BAUDR_0_BitAddressOffset 0
#define SSIV2_BAUDR_RSVD_BAUDR_0_RegisterSize 1

/* Register BAUDR field SCKDV */
/* SSI Clock Divider.
The LSB for this field is always set to 0 and is unaffected by a write operation, which ensures an even value is held in this register. If the value is 0, the serial output clock (sclk_out) is disabled. The frequency of the sclk_out is derived from the following equation:

Fsclk_out = Fssi_clk/BAUDR
where BAUDR is any even value between 2 and 65534 (BAUDR = {SCKDV*2}). For example: for Fssi_clk = 3.6864MHz and BAUDR =2 Fsclk_out = 3.6864/2 = 1.8432MHz */
#define SSIV2_BAUDR_SCKDV_BitAddressOffset 1
#define SSIV2_BAUDR_SCKDV_RegisterSize 15

/* Register BAUDR field RSVD_BAUDR_16_31 */
/* Reserved bits - read as zero */
#define SSIV2_BAUDR_RSVD_BAUDR_16_31_BitAddressOffset 16
#define SSIV2_BAUDR_RSVD_BAUDR_16_31_RegisterSize 16

/* End of Register Definition for BAUDR */

/* Register TXFTLR */
/* This register controls the threshold value for the transmit FIFO memory.. */
#define SSIV2_TXFTLR (ssic_address_block_BaseAddress + 0x18)
#define SSIV2_TXFTLR_RegisterSize 32
#define SSIV2_TXFTLR_RegisterResetValue 0x0
#define SSIV2_TXFTLR_RegisterResetMask 0xffffffff

/* Register Field information for TXFTLR */

/* Register TXFTLR field TFT */
/* Transmit FIFO Threshold.
Controls the level of entries (or below) at which the transmit FIFO controller triggers an interrupt. The FIFO depth is configurable in the range 8-256; this register is sized to the number of address bits needed to access the FIFO. If you attempt to set this value greater than or equal to the depth of the FIFO, this field is not written and retains its current value. When the number of transmit FIFO entries is less than or equal to this value, the transmit FIFO empty interrupt is triggered. */
#define SSIV2_TXFTLR_TFT_BitAddressOffset 0
#define SSIV2_TXFTLR_TFT_RegisterSize 4

/* Register TXFTLR field RSVD_TXFTLR */
/* Reserved bits - read as zero */
#define SSIV2_TXFTLR_RSVD_TXFTLR_BitAddressOffset 4
#define SSIV2_TXFTLR_RSVD_TXFTLR_RegisterSize 12

/* Register TXFTLR field TXFTHR */
/* Transfer start FIFO level.
Used to control the level of entries in transmit FIFO above which transfer will start on serial line. This register can be used to ensure that sufficient data is present in transmit FIFO before starting a write operation on serial line.
In Internal DMA mode, this field sets the minimum amount of data frames present in the FIFO after which DWC_ssi starts the transfer.
This field is valid only for Master mode of operation. */
#define SSIV2_TXFTLR_TXFTHR_BitAddressOffset 16
#define SSIV2_TXFTLR_TXFTHR_RegisterSize 4

/* Register TXFTLR field RSVD_TXFTHR */
/* Reserved bits - read as zero */
#define SSIV2_TXFTLR_RSVD_TXFTHR_BitAddressOffset 20
#define SSIV2_TXFTLR_RSVD_TXFTHR_RegisterSize 12

/* End of Register Definition for TXFTLR */

/* Register RXFTLR */
/* This register controls the threshold value for the receive FIFO memory.. */
#define SSIV2_RXFTLR (ssic_address_block_BaseAddress + 0x1c)
#define SSIV2_RXFTLR_RegisterSize 32
#define SSIV2_RXFTLR_RegisterResetValue 0x0
#define SSIV2_RXFTLR_RegisterResetMask 0xffffffff

/* Register Field information for RXFTLR */

/* Register RXFTLR field RFT */
/* Receive FIFO Threshold.
Controls the level of entries (or above) at which the receive FIFO controller triggers an interrupt. The FIFO depth is configurable in the range 8-256. This register is sized to the number of address bits needed to access the FIFO. If you attempt to set this value greater than the depth of the FIFO, this field is not written and retains its current value. When the number of receive FIFO entries is greater than or equal to this value + 1, the receive FIFO full interrupt is triggered. */
#define SSIV2_RXFTLR_RFT_BitAddressOffset 0
#define SSIV2_RXFTLR_RFT_RegisterSize 4

/* Register RXFTLR field RSVD_RXFTLR */
/* Reserved bits - read as zero */
#define SSIV2_RXFTLR_RSVD_RXFTLR_BitAddressOffset 4
#define SSIV2_RXFTLR_RSVD_RXFTLR_RegisterSize 28

/* End of Register Definition for RXFTLR */

/* Register TXFLR */
/* This register contains the number of valid data entries in the transmit FIFO memory. */
#define SSIV2_TXFLR (ssic_address_block_BaseAddress + 0x20)
#define SSIV2_TXFLR_RegisterSize 32
#define SSIV2_TXFLR_RegisterResetValue 0x0
#define SSIV2_TXFLR_RegisterResetMask 0xffffffff

/* Register Field information for TXFLR */

/* Register TXFLR field TXTFL */
/* Transmit FIFO Level.
Contains the number of valid data entries in the transmit FIFO. */
#define SSIV2_TXFLR_TXTFL_BitAddressOffset 0
#define SSIV2_TXFLR_TXTFL_RegisterSize 5

/* Register TXFLR field RSVD_TXFLR */
/* Reserved bits - read as zero */
#define SSIV2_TXFLR_RSVD_TXFLR_BitAddressOffset 5
#define SSIV2_TXFLR_RSVD_TXFLR_RegisterSize 27

/* End of Register Definition for TXFLR */

/* Register RXFLR */
/* This register contains the number of valid data entries in the receive FIFO memory. This register can be read at any time. */
#define SSIV2_RXFLR (ssic_address_block_BaseAddress + 0x24)
#define SSIV2_RXFLR_RegisterSize 32
#define SSIV2_RXFLR_RegisterResetValue 0x0
#define SSIV2_RXFLR_RegisterResetMask 0xffffffff

/* Register Field information for RXFLR */

/* Register RXFLR field RXTFL */
/* Receive FIFO Level.
Contains the number of valid data entries in the receive FIFO. */
#define SSIV2_RXFLR_RXTFL_BitAddressOffset 0
#define SSIV2_RXFLR_RXTFL_RegisterSize 5

/* Register RXFLR field RSVD_RXFLR */
/* Reserved bits - read as zero */
#define SSIV2_RXFLR_RSVD_RXFLR_BitAddressOffset 5
#define SSIV2_RXFLR_RSVD_RXFLR_RegisterSize 27

/* End of Register Definition for RXFLR */

/* Register SR */
/* This is a read-only register used to indicate the current transfer status, FIFO status, and any transmission/reception errors that may have occurred. The status register may be read at any time. None of the bits in this register request an interrupt. */
#define SSIV2_SR (ssic_address_block_BaseAddress + 0x28)
#define SSIV2_SR_RegisterSize 32
#define SSIV2_SR_RegisterResetValue 0x6
#define SSIV2_SR_RegisterResetMask 0xffffffff

/* Register Field information for SR */

/* Register SR field BUSY */
/* SSI Busy Flag.
When set, indicates that a serial transfer is in progress; when cleared indicates that the DWC_ssi is idle or disabled. */
#define SSIV2_SR_BUSY_BitAddressOffset 0
#define SSIV2_SR_BUSY_RegisterSize 1

/* Register SR field TFNF */
/* Transmit FIFO Not Full.
Set when the transmit FIFO contains one or more empty locations, and is cleared when the FIFO is full. */
#define SSIV2_SR_TFNF_BitAddressOffset 1
#define SSIV2_SR_TFNF_RegisterSize 1

/* Register SR field TFE */
/* Transmit FIFO Empty.
When the transmit FIFO is completely empty, this bit is set. When the transmit FIFO contains one or more valid entries, this bit is cleared. This bit field does not request an interrupt. */
#define SSIV2_SR_TFE_BitAddressOffset 2
#define SSIV2_SR_TFE_RegisterSize 1

/* Register SR field RFNE */
/* Receive FIFO Not Empty.
Set when the receive FIFO contains one or more entries and is cleared when the receive FIFO is empty. This bit can be polled by software to completely empty the receive FIFO. */
#define SSIV2_SR_RFNE_BitAddressOffset 3
#define SSIV2_SR_RFNE_RegisterSize 1

/* Register SR field RFF */
/* Receive FIFO Full.
When the receive FIFO is completely full, this bit is set. When the receive FIFO contains one or more empty location, this bit is cleared. */
#define SSIV2_SR_RFF_BitAddressOffset 4
#define SSIV2_SR_RFF_RegisterSize 1

/* Register SR field TXE */
/* Transmission Error.
Set if the transmit FIFO is empty when a transfer is started. This bit can be set only when the DWC_ssi is configured as a slave device. Data from the previous transmission is resent on the txd line. This bit is cleared when read. */
#define SSIV2_SR_TXE_BitAddressOffset 5
#define SSIV2_SR_TXE_RegisterSize 1

/* Register SR field DCOL */
/* Data Collision Error.
Relevant only when the DWC_ssi is configured as a master device. This bit will be set if ss_in_n input is asserted by other master, when the DWC_ssi master is in the middle of the transfer. This informs the processor that the last data transfer was halted before completion. This bit is cleared when read. */
#define SSIV2_SR_DCOL_BitAddressOffset 6
#define SSIV2_SR_DCOL_RegisterSize 1

/* Register SR field RSVD_SR */
/* Reserved bits - read as zero */
#define SSIV2_SR_RSVD_SR_BitAddressOffset 7
#define SSIV2_SR_RSVD_SR_RegisterSize 8

/* Register SR field CMPLTD_DF */
/* Completed Data frames
This field indicates total data frames transferred in the previous internal DMA transfer */
#define SSIV2_SR_CMPLTD_DF_BitAddressOffset 15
#define SSIV2_SR_CMPLTD_DF_RegisterSize 17

/* End of Register Definition for SR */

/* Register IMR */
/* This read/write register masks or enables all interrupts generated by the DWC_ssi. When the DWC_ssi is configured as a slave device, the MSTIM bit field is not present. This changes the reset value from 0x3F for serial-master configurations to 0x1F for serial-slave configurations. */
#define SSIV2_IMR (ssic_address_block_BaseAddress + 0x2c)
#define SSIV2_IMR_RegisterSize 32
#define SSIV2_IMR_RegisterResetValue 0xff
#define SSIV2_IMR_RegisterResetMask 0xffffffff

/* Register Field information for IMR */

/* Register IMR field TXEIM */
/* Transmit FIFO Empty Interrupt Mask */
#define SSIV2_IMR_TXEIM_BitAddressOffset 0
#define SSIV2_IMR_TXEIM_RegisterSize 1

/* Register IMR field TXOIM */
/* Transmit FIFO Overflow Interrupt Mask */
#define SSIV2_IMR_TXOIM_BitAddressOffset 1
#define SSIV2_IMR_TXOIM_RegisterSize 1

/* Register IMR field RXUIM */
/* Receive FIFO Underflow Interrupt Mask */
#define SSIV2_IMR_RXUIM_BitAddressOffset 2
#define SSIV2_IMR_RXUIM_RegisterSize 1

/* Register IMR field RXOIM */
/* Receive FIFO Overflow Interrupt Mask */
#define SSIV2_IMR_RXOIM_BitAddressOffset 3
#define SSIV2_IMR_RXOIM_RegisterSize 1

/* Register IMR field RXFIM */
/* Receive FIFO Full Interrupt Mask
0 - ssi_rxf_intr interrupt is masked
1 - ssi_rxf_intr interrupt is not masked */
#define SSIV2_IMR_RXFIM_BitAddressOffset 4
#define SSIV2_IMR_RXFIM_RegisterSize 1

/* Register IMR field MSTIM */
/* Multi-Master Contention Interrupt Mask. This bit field is not present if the DWC_ssi is configured as a serial-master device. */
#define SSIV2_IMR_MSTIM_BitAddressOffset 5
#define SSIV2_IMR_MSTIM_RegisterSize 1

/* Register IMR field XRXOIM */
/* XIP Receive FIFO Overflow Interrupt Mask */
#define SSIV2_IMR_XRXOIM_BitAddressOffset 6
#define SSIV2_IMR_XRXOIM_RegisterSize 1

/* Register IMR field TXUIM */
/* Transmit FIFO Underflow Interrupt Mask */
#define SSIV2_IMR_TXUIM_BitAddressOffset 7
#define SSIV2_IMR_TXUIM_RegisterSize 1

/* Register IMR field AXIEM */
/* AXI Error Interrupt Mask */
#define SSIV2_IMR_AXIEM_BitAddressOffset 8
#define SSIV2_IMR_AXIEM_RegisterSize 1

/* Register IMR field RSVD_9_IMR */
/* Reserved bits - read as zero */
#define SSIV2_IMR_RSVD_9_IMR_BitAddressOffset 9
#define SSIV2_IMR_RSVD_9_IMR_RegisterSize 1

/* Register IMR field SPITEM */
/* SPI Transmit Error Interrupt Mask */
#define SSIV2_IMR_SPITEM_BitAddressOffset 10
#define SSIV2_IMR_SPITEM_RegisterSize 1

/* Register IMR field DONEM */
/* SSI Done Interrupt Mask */
#define SSIV2_IMR_DONEM_BitAddressOffset 11
#define SSIV2_IMR_DONEM_RegisterSize 1

/* Register IMR field RSVD_12_32_IMR */
/* Reserved bits - read as zero */
#define SSIV2_IMR_RSVD_12_32_IMR_BitAddressOffset 12
#define SSIV2_IMR_RSVD_12_32_IMR_RegisterSize 20

/* End of Register Definition for IMR */

/* Register ISR */
/* This register reports the status of the DWC_ssi interrupts after they have been masked. */
#define SSIV2_ISR (ssic_address_block_BaseAddress + 0x30)
#define SSIV2_ISR_RegisterSize 32
#define SSIV2_ISR_RegisterResetValue 0x0
#define SSIV2_ISR_RegisterResetMask 0xffffffff

/* Register Field information for ISR */

/* Register ISR field TXEIS */
/* Transmit FIFO Empty Interrupt Status */
#define SSIV2_ISR_TXEIS_BitAddressOffset 0
#define SSIV2_ISR_TXEIS_RegisterSize 1

/* Register ISR field TXOIS */
/* Transmit FIFO Overflow Interrupt Status */
#define SSIV2_ISR_TXOIS_BitAddressOffset 1
#define SSIV2_ISR_TXOIS_RegisterSize 1

/* Register ISR field RXUIS */
/* Receive FIFO Underflow Interrupt Status */
#define SSIV2_ISR_RXUIS_BitAddressOffset 2
#define SSIV2_ISR_RXUIS_RegisterSize 1

/* Register ISR field RXOIS */
/* Receive FIFO Overflow Interrupt Status */
#define SSIV2_ISR_RXOIS_BitAddressOffset 3
#define SSIV2_ISR_RXOIS_RegisterSize 1

/* Register ISR field RXFIS */
/* Receive FIFO Full Interrupt Status */
#define SSIV2_ISR_RXFIS_BitAddressOffset 4
#define SSIV2_ISR_RXFIS_RegisterSize 1

/* Register ISR field MSTIS */
/* Multi-Master Contention Interrupt Status. This bit field is not present
if the DWC_ssi is configured as a serial-slave device. */
#define SSIV2_ISR_MSTIS_BitAddressOffset 5
#define SSIV2_ISR_MSTIS_RegisterSize 1

/* Register ISR field XRXOIS */
/* XIP Receive FIFO Overflow Interrupt Status */
#define SSIV2_ISR_XRXOIS_BitAddressOffset 6
#define SSIV2_ISR_XRXOIS_RegisterSize 1

/* Register ISR field TXUIS */
/* Transmit FIFO Underflow Interrupt Status */
#define SSIV2_ISR_TXUIS_BitAddressOffset 7
#define SSIV2_ISR_TXUIS_RegisterSize 1

/* Register ISR field AXIES */
/* AXI Error Interrupt Status */
#define SSIV2_ISR_AXIES_BitAddressOffset 8
#define SSIV2_ISR_AXIES_RegisterSize 1

/* Register ISR field RSVD_9_RISR */
/* Reserved bits - read as zero */
#define SSIV2_ISR_RSVD_9_RISR_BitAddressOffset 9
#define SSIV2_ISR_RSVD_9_RISR_RegisterSize 1

/* Register ISR field SPITES */
/* SPI Transmit Error Interrupt */
#define SSIV2_ISR_SPITES_BitAddressOffset 10
#define SSIV2_ISR_SPITES_RegisterSize 1

/* Register ISR field DONES */
/* SSI Done Interrupt Status */
#define SSIV2_ISR_DONES_BitAddressOffset 11
#define SSIV2_ISR_DONES_RegisterSize 1

/* Register ISR field RSVD_12_32_RISR */
/* Reserved bits - read as zero */
#define SSIV2_ISR_RSVD_12_32_RISR_BitAddressOffset 12
#define SSIV2_ISR_RSVD_12_32_RISR_RegisterSize 20

/* End of Register Definition for ISR */

/* Register RISR */
/* Raw Interrupt Status Register */
#define SSIV2_RISR (ssic_address_block_BaseAddress + 0x34)
#define SSIV2_RISR_RegisterSize 32
#define SSIV2_RISR_RegisterResetValue 0x0
#define SSIV2_RISR_RegisterResetMask 0xffffffff

/* Register Field information for RISR */

/* Register RISR field TXEIR */
/* Transmit FIFO Empty Raw Interrupt Status */
#define SSIV2_RISR_TXEIR_BitAddressOffset 0
#define SSIV2_RISR_TXEIR_RegisterSize 1

/* Register RISR field TXOIR */
/* Transmit FIFO Overflow Raw Interrupt Status */
#define SSIV2_RISR_TXOIR_BitAddressOffset 1
#define SSIV2_RISR_TXOIR_RegisterSize 1

/* Register RISR field RXUIR */
/* Receive FIFO Underflow Raw Interrupt Status */
#define SSIV2_RISR_RXUIR_BitAddressOffset 2
#define SSIV2_RISR_RXUIR_RegisterSize 1

/* Register RISR field RXOIR */
/* Receive FIFO Overflow Raw Interrupt Status */
#define SSIV2_RISR_RXOIR_BitAddressOffset 3
#define SSIV2_RISR_RXOIR_RegisterSize 1

/* Register RISR field RXFIR */
/* Receive FIFO Full Raw Interrupt Status */
#define SSIV2_RISR_RXFIR_BitAddressOffset 4
#define SSIV2_RISR_RXFIR_RegisterSize 1

/* Register RISR field MSTIR */
/* Multi-Master Contention Raw Interrupt Status. This bit field is not present if the DWC_ssi is configured as a serial-slave device. */
#define SSIV2_RISR_MSTIR_BitAddressOffset 5
#define SSIV2_RISR_MSTIR_RegisterSize 1

/* Register RISR field XRXOIR */
/* XIP Receive FIFO Overflow Raw Interrupt Status */
#define SSIV2_RISR_XRXOIR_BitAddressOffset 6
#define SSIV2_RISR_XRXOIR_RegisterSize 1

/* Register RISR field TXUIR */
/* Transmit FIFO Underflow Interrupt Raw Status */
#define SSIV2_RISR_TXUIR_BitAddressOffset 7
#define SSIV2_RISR_TXUIR_RegisterSize 1

/* Register RISR field AXIER */
/* AXI Error Interrupt Raw Status */
#define SSIV2_RISR_AXIER_BitAddressOffset 8
#define SSIV2_RISR_AXIER_RegisterSize 1

/* Register RISR field RSVD_9_RISR */
/* Reserved bits - read as zero */
#define SSIV2_RISR_RSVD_9_RISR_BitAddressOffset 9
#define SSIV2_RISR_RSVD_9_RISR_RegisterSize 1

/* Register RISR field SPITER */
/* SPI Transmit Error Interrupt status.
This bit gets set, If SPI Master fails to get a READY status from the slave until the amount of time defined in SPI_CTRLR1.MAX_WS field, then it will stop the SPI transfer and the FIFO is flushed (in case of write operation). */
#define SSIV2_RISR_SPITER_BitAddressOffset 10
#define SSIV2_RISR_SPITER_RegisterSize 1

/* Register RISR field DONER */
/* SSI Done Interrupt Raw Status */
#define SSIV2_RISR_DONER_BitAddressOffset 11
#define SSIV2_RISR_DONER_RegisterSize 1

/* Register RISR field RSVD_12_32_RISR */
/* Reserved bits - read as zero */
#define SSIV2_RISR_RSVD_12_32_RISR_BitAddressOffset 12
#define SSIV2_RISR_RSVD_12_32_RISR_RegisterSize 20

/* End of Register Definition for RISR */

/* Register TXEICR */
/* Transmit FIFO Error Interrupt Clear Register */
#define SSIV2_TXEICR (ssic_address_block_BaseAddress + 0x38)
#define SSIV2_TXEICR_RegisterSize 32
#define SSIV2_TXEICR_RegisterResetValue 0x0
#define SSIV2_TXEICR_RegisterResetMask 0xffffffff

/* Register Field information for TXEICR */

/* Register TXEICR field TXEICR */
/* Clear Transmit FIFO Overflow/Underflow Interrupt.
This register reflects the status of the interrupt. A read from this register clears the ssi_txo_intr/ssi_txu_intr interrupt; writing has no effect. */
#define SSIV2_TXEICR_TXEICR_BitAddressOffset 0
#define SSIV2_TXEICR_TXEICR_RegisterSize 1

/* Register TXEICR field RSVD_TXEICR */
/* Reserved bits - read as zero */
#define SSIV2_TXEICR_RSVD_TXEICR_BitAddressOffset 1
#define SSIV2_TXEICR_RSVD_TXEICR_RegisterSize 31

/* End of Register Definition for TXEICR */

/* Register RXOICR */
/* Receive FIFO Overflow Interrupt Clear Register */
#define SSIV2_RXOICR (ssic_address_block_BaseAddress + 0x3c)
#define SSIV2_RXOICR_RegisterSize 32
#define SSIV2_RXOICR_RegisterResetValue 0x0
#define SSIV2_RXOICR_RegisterResetMask 0xffffffff

/* Register Field information for RXOICR */

/* Register RXOICR field RXOICR */
/* Clear Receive FIFO Overflow Interrupt.
This register reflects the status of the interrupt. A read from this register clears the ssi_rxo_intr interrupt; writing has no effect. */
#define SSIV2_RXOICR_RXOICR_BitAddressOffset 0
#define SSIV2_RXOICR_RXOICR_RegisterSize 1

/* Register RXOICR field RSVD_RXOICR */
/* Reserved bits - read as zero */
#define SSIV2_RXOICR_RSVD_RXOICR_BitAddressOffset 1
#define SSIV2_RXOICR_RSVD_RXOICR_RegisterSize 31

/* End of Register Definition for RXOICR */

/* Register RXUICR */
/* Receive FIFO Underflow Interrupt Clear Register */
#define SSIV2_RXUICR (ssic_address_block_BaseAddress + 0x40)
#define SSIV2_RXUICR_RegisterSize 32
#define SSIV2_RXUICR_RegisterResetValue 0x0
#define SSIV2_RXUICR_RegisterResetMask 0xffffffff

/* Register Field information for RXUICR */

/* Register RXUICR field RXUICR */
/* Clear Receive FIFO Underflow Interrupt.
This register reflects the status of the interrupt. A read from this register clears the ssi_rxu_intr interrupt; writing has no effect. */
#define SSIV2_RXUICR_RXUICR_BitAddressOffset 0
#define SSIV2_RXUICR_RXUICR_RegisterSize 1

/* Register RXUICR field RSVD_RXUICR */
/* Reserved bits - read as zero */
#define SSIV2_RXUICR_RSVD_RXUICR_BitAddressOffset 1
#define SSIV2_RXUICR_RSVD_RXUICR_RegisterSize 31

/* End of Register Definition for RXUICR */

/* Register MSTICR */
/* Multi-Master Interrupt Clear Register */
#define SSIV2_MSTICR (ssic_address_block_BaseAddress + 0x44)
#define SSIV2_MSTICR_RegisterSize 32
#define SSIV2_MSTICR_RegisterResetValue 0x0
#define SSIV2_MSTICR_RegisterResetMask 0xffffffff

/* Register Field information for MSTICR */

/* Register MSTICR field MSTICR */
/* Clear Multi-Master Contention Interrupt.
This register reflects the status of the interrupt. A read from this register clears the ssi_mst_intr interrupt; writing has no effect. */
#define SSIV2_MSTICR_MSTICR_BitAddressOffset 0
#define SSIV2_MSTICR_MSTICR_RegisterSize 1

/* Register MSTICR field RSVD_MSTICR */
/* Reserved bits - read as zero */
#define SSIV2_MSTICR_RSVD_MSTICR_BitAddressOffset 1
#define SSIV2_MSTICR_RSVD_MSTICR_RegisterSize 31

/* End of Register Definition for MSTICR */

/* Register ICR */
/* Interrupt Clear Register */
#define SSIV2_ICR (ssic_address_block_BaseAddress + 0x48)
#define SSIV2_ICR_RegisterSize 32
#define SSIV2_ICR_RegisterResetValue 0x0
#define SSIV2_ICR_RegisterResetMask 0xffffffff

/* Register Field information for ICR */

/* Register ICR field ICR */
/* Clear Interrupts.
This register is set if any of the interrupts below are active. A read clears the ssi_txo_intr, ssi_rxu_intr, ssi_rxo_intr, and the ssi_mst_intr interrupts. Writing to this register has no effect. */
#define SSIV2_ICR_ICR_BitAddressOffset 0
#define SSIV2_ICR_ICR_RegisterSize 1

/* Register ICR field RSVD_ICR */
/* Reserved bits - read as zero */
#define SSIV2_ICR_RSVD_ICR_BitAddressOffset 1
#define SSIV2_ICR_RSVD_ICR_RegisterSize 31

/* End of Register Definition for ICR */

/* Register DMACR */
/* DMA Control Register.
This register is only valid when DWC_ssi is configured with a set of DMA Controller interface signals (SSIC_HAS_DMA = 1) or Internal DMA operation (SSIC_HAS_DMA = 2). When DWC_ssi is not configured for DMA operation, this register will not exist and writing to the register's address will have no effect; reading from this register address will return zero. */
#define SSIV2_DMACR (ssic_address_block_BaseAddress + 0x4c)
#define SSIV2_DMACR_RegisterSize 32
#define SSIV2_DMACR_RegisterResetValue 0x0
#define SSIV2_DMACR_RegisterResetMask 0xffffffff

/* Register Field information for DMACR */

/* Register DMACR field RDMAE */
/* Receive DMA Enable. This bit enables/disables the receive FIFO DMA channel. */
#define SSIV2_DMACR_RDMAE_BitAddressOffset 0
#define SSIV2_DMACR_RDMAE_RegisterSize 1

/* Register DMACR field TDMAE */
/* Transmit DMA Enable. This bit enables/disables the transmit FIFO DMA channel. */
#define SSIV2_DMACR_TDMAE_BitAddressOffset 1
#define SSIV2_DMACR_TDMAE_RegisterSize 1

/* Register DMACR field IDMAE */
/* Internal DMA Enable. This bit should be enabled only when CTRLR0.FRF = 0 (Motorola SPI) and CTRLR0.SPI_FRF > 0. */
#define SSIV2_DMACR_IDMAE_BitAddressOffset 2
#define SSIV2_DMACR_IDMAE_RegisterSize 1

/* Register DMACR field ATW */
/* AXI transfer width for DMA transfers mapped to arsize/awsize. This value must be less than or equal to SSIC_AXI_DW.   Values:

   0x0: 1 byte

   0x1: 2 bytes

   0x2: 4 bytes

   0x3: 8 bytes

 Note: When SSIC_AXI_DW is set to 32 bits, if user programs this field to 0x8(3 bytes). DWC_ssi will use 4 bytes as transfer size for the AXI transfers. */
#define SSIV2_DMACR_ATW_BitAddressOffset 3
#define SSIV2_DMACR_ATW_RegisterSize 2

/* Register DMACR field RSVD_DMACR5 */
/* Reserved bits - read as zero */
#define SSIV2_DMACR_RSVD_DMACR5_BitAddressOffset 5
#define SSIV2_DMACR_RSVD_DMACR5_RegisterSize 1

/* Register DMACR field AINC */
/* Address Increment. Indicates whether to increment the AXI address on every transfer.
   1 = Increment
   0 = No Change
 Note: Increment aligns the address to the next DMACR.ATW boundary */
#define SSIV2_DMACR_AINC_BitAddressOffset 6
#define SSIV2_DMACR_AINC_RegisterSize 1

/* Register DMACR field RSVD_DMACR7 */
/* Reserved bits - read as zero */
#define SSIV2_DMACR_RSVD_DMACR7_BitAddressOffset 7
#define SSIV2_DMACR_RSVD_DMACR7_RegisterSize 1

/* Register DMACR field ACACHE */
/* AXI arcache/awcache signal value. */
#define SSIV2_DMACR_ACACHE_BitAddressOffset 8
#define SSIV2_DMACR_ACACHE_RegisterSize 4

/* Register DMACR field APROT */
/* AXI arprot/awprot signal value. */
#define SSIV2_DMACR_APROT_BitAddressOffset 12
#define SSIV2_DMACR_APROT_RegisterSize 3

/* Register DMACR field AID */
/* AXI awid/arid signal value. */
#define SSIV2_DMACR_AID_BitAddressOffset 15
#define SSIV2_DMACR_AID_RegisterSize 6

/* Register DMACR field RSVD_DMACR */
/* Reserved bits - read as zero */
#define SSIV2_DMACR_RSVD_DMACR_BitAddressOffset 21
#define SSIV2_DMACR_RSVD_DMACR_RegisterSize 11

/* End of Register Definition for DMACR */

/* Register DMATDLR */
/* This register is only valid when the DWC_ssi is configured with a set of DMA interface signals (SSIC_HAS_DMA = 1). When DWC_ssi is not configured for DMA operation, this register will not exist and writing to its address will have no effect; reading from its address will return zero. */
#define SSIV2_DMATDLR (ssic_address_block_BaseAddress + 0x50)
#define SSIV2_DMATDLR_RegisterSize 32
#define SSIV2_DMATDLR_RegisterResetValue 0x0
#define SSIV2_DMATDLR_RegisterResetMask 0xffffffff

/* Register Field information for DMATDLR */

/* Register DMATDLR field DMATDL */
/* Transmit Data Level. This bit field controls the level at which a DMA request is made by the transmit logic. It is equal to the watermark level; that is, the dma_tx_req signal is generated when the number of valid data entries in the transmit FIFO is equal to or below this field value, and TDMAE = 1. */
#define SSIV2_DMATDLR_DMATDL_BitAddressOffset 0
#define SSIV2_DMATDLR_DMATDL_RegisterSize 4

/* Register DMATDLR field RSVD_DMATDLR */
/* Reserved bits - read as zero */
#define SSIV2_DMATDLR_RSVD_DMATDLR_BitAddressOffset 4
#define SSIV2_DMATDLR_RSVD_DMATDLR_RegisterSize 28

/* End of Register Definition for DMATDLR */

/* Register DMARDLR */
/* This register is only valid when DWC_ssi is configured with a set of DMA interface signals (SSIC_HAS_DMA = 1). When DWC_ssi is not configured for DMA operation, this register will not exist and writing to its address will have no effect; reading from its address will return zero. */
#define SSIV2_DMARDLR (ssic_address_block_BaseAddress + 0x54)
#define SSIV2_DMARDLR_RegisterSize 32
#define SSIV2_DMARDLR_RegisterResetValue 0x0
#define SSIV2_DMARDLR_RegisterResetMask 0xffffffff

/* Register Field information for DMARDLR */

/* Register DMARDLR field DMARDL */
/* Receive Data Level. This bit field controls the level at which a DMA request is made by the receive logic. The watermark level = DMARDL+1; that is, dma_rx_req is generated when the number of valid data entries in the receive FIFO is equal to or above this field value + 1, and RDMAE=1. */
#define SSIV2_DMARDLR_DMARDL_BitAddressOffset 0
#define SSIV2_DMARDLR_DMARDL_RegisterSize 4

/* Register DMARDLR field RSVD_DMARDLR */
/* Reserved bits - read as zero */
#define SSIV2_DMARDLR_RSVD_DMARDLR_BitAddressOffset 4
#define SSIV2_DMARDLR_RSVD_DMARDLR_RegisterSize 28

/* End of Register Definition for DMARDLR */

/* Register IDR */
/* This register contains the peripherals identification code, which is written into the register at configuration time using coreConsultant. */
#define SSIV2_IDR (ssic_address_block_BaseAddress + 0x58)
#define SSIV2_IDR_RegisterSize 32
#define SSIV2_IDR_RegisterResetValue 0xffffffff
#define SSIV2_IDR_RegisterResetMask 0xffffffff

/* Register Field information for IDR */

/* Register IDR field IDCODE */
/* Identification code. The register contains the peripheral's identification code, which is written into the register at configuration time using CoreConsultant. */
#define SSIV2_IDR_IDCODE_BitAddressOffset 0
#define SSIV2_IDR_IDCODE_RegisterSize 32

/* End of Register Definition for IDR */

/* Register SSIC_VERSION_ID */
/* This read-only register stores the specific DWC_ssi component version. */
#define SSIV2_SSIC_VERSION_ID (ssic_address_block_BaseAddress + 0x5c)
#define SSIV2_SSIC_VERSION_ID_RegisterSize 32
#define SSIV2_SSIC_VERSION_ID_RegisterResetValue 0x3130332a
#define SSIV2_SSIC_VERSION_ID_RegisterResetMask 0xffffffff

/* Register Field information for SSIC_VERSION_ID */

/* Register SSIC_VERSION_ID field SSIC_COMP_VERSION */
/* Contains the hex representation of the Synopsys component version. Consists of ASCII value for each number in the version, followed by *. For example 31_30_33_2A represents the version 1.03*. */
#define SSIV2_SSIC_VERSION_ID_SSIC_COMP_VERSION_BitAddressOffset 0
#define SSIV2_SSIC_VERSION_ID_SSIC_COMP_VERSION_RegisterSize 32

/* End of Register Definition for SSIC_VERSION_ID */

/* Register DR0 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR0 (ssic_address_block_BaseAddress + 0x60)
#define SSIV2_DR0_RegisterSize 32
#define SSIV2_DR0_RegisterResetValue 0x0
#define SSIV2_DR0_RegisterResetMask 0xffffffff

/* Register Field information for DR0 */

/* Register DR0 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR0_DR_BitAddressOffset 0
#define SSIV2_DR0_DR_RegisterSize 32

/* End of Register Definition for DR0 */

/* Register DR1 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR1 (ssic_address_block_BaseAddress + 0x64)
#define SSIV2_DR1_RegisterSize 32
#define SSIV2_DR1_RegisterResetValue 0x0
#define SSIV2_DR1_RegisterResetMask 0xffffffff

/* Register Field information for DR1 */

/* Register DR1 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR1_DR_BitAddressOffset 0
#define SSIV2_DR1_DR_RegisterSize 32

/* End of Register Definition for DR1 */

/* Register DR2 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR2 (ssic_address_block_BaseAddress + 0x68)
#define SSIV2_DR2_RegisterSize 32
#define SSIV2_DR2_RegisterResetValue 0x0
#define SSIV2_DR2_RegisterResetMask 0xffffffff

/* Register Field information for DR2 */

/* Register DR2 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR2_DR_BitAddressOffset 0
#define SSIV2_DR2_DR_RegisterSize 32

/* End of Register Definition for DR2 */

/* Register DR3 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR3 (ssic_address_block_BaseAddress + 0x6c)
#define SSIV2_DR3_RegisterSize 32
#define SSIV2_DR3_RegisterResetValue 0x0
#define SSIV2_DR3_RegisterResetMask 0xffffffff

/* Register Field information for DR3 */

/* Register DR3 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR3_DR_BitAddressOffset 0
#define SSIV2_DR3_DR_RegisterSize 32

/* End of Register Definition for DR3 */

/* Register DR4 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR4 (ssic_address_block_BaseAddress + 0x70)
#define SSIV2_DR4_RegisterSize 32
#define SSIV2_DR4_RegisterResetValue 0x0
#define SSIV2_DR4_RegisterResetMask 0xffffffff

/* Register Field information for DR4 */

/* Register DR4 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR4_DR_BitAddressOffset 0
#define SSIV2_DR4_DR_RegisterSize 32

/* End of Register Definition for DR4 */

/* Register DR5 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR5 (ssic_address_block_BaseAddress + 0x74)
#define SSIV2_DR5_RegisterSize 32
#define SSIV2_DR5_RegisterResetValue 0x0
#define SSIV2_DR5_RegisterResetMask 0xffffffff

/* Register Field information for DR5 */

/* Register DR5 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR5_DR_BitAddressOffset 0
#define SSIV2_DR5_DR_RegisterSize 32

/* End of Register Definition for DR5 */

/* Register DR6 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR6 (ssic_address_block_BaseAddress + 0x78)
#define SSIV2_DR6_RegisterSize 32
#define SSIV2_DR6_RegisterResetValue 0x0
#define SSIV2_DR6_RegisterResetMask 0xffffffff

/* Register Field information for DR6 */

/* Register DR6 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR6_DR_BitAddressOffset 0
#define SSIV2_DR6_DR_RegisterSize 32

/* End of Register Definition for DR6 */

/* Register DR7 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR7 (ssic_address_block_BaseAddress + 0x7c)
#define SSIV2_DR7_RegisterSize 32
#define SSIV2_DR7_RegisterResetValue 0x0
#define SSIV2_DR7_RegisterResetMask 0xffffffff

/* Register Field information for DR7 */

/* Register DR7 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR7_DR_BitAddressOffset 0
#define SSIV2_DR7_DR_RegisterSize 32

/* End of Register Definition for DR7 */

/* Register DR8 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR8 (ssic_address_block_BaseAddress + 0x80)
#define SSIV2_DR8_RegisterSize 32
#define SSIV2_DR8_RegisterResetValue 0x0
#define SSIV2_DR8_RegisterResetMask 0xffffffff

/* Register Field information for DR8 */

/* Register DR8 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR8_DR_BitAddressOffset 0
#define SSIV2_DR8_DR_RegisterSize 32

/* End of Register Definition for DR8 */

/* Register DR9 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR9 (ssic_address_block_BaseAddress + 0x84)
#define SSIV2_DR9_RegisterSize 32
#define SSIV2_DR9_RegisterResetValue 0x0
#define SSIV2_DR9_RegisterResetMask 0xffffffff

/* Register Field information for DR9 */

/* Register DR9 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR9_DR_BitAddressOffset 0
#define SSIV2_DR9_DR_RegisterSize 32

/* End of Register Definition for DR9 */

/* Register DR10 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR10 (ssic_address_block_BaseAddress + 0x88)
#define SSIV2_DR10_RegisterSize 32
#define SSIV2_DR10_RegisterResetValue 0x0
#define SSIV2_DR10_RegisterResetMask 0xffffffff

/* Register Field information for DR10 */

/* Register DR10 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR10_DR_BitAddressOffset 0
#define SSIV2_DR10_DR_RegisterSize 32

/* End of Register Definition for DR10 */

/* Register DR11 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR11 (ssic_address_block_BaseAddress + 0x8c)
#define SSIV2_DR11_RegisterSize 32
#define SSIV2_DR11_RegisterResetValue 0x0
#define SSIV2_DR11_RegisterResetMask 0xffffffff

/* Register Field information for DR11 */

/* Register DR11 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR11_DR_BitAddressOffset 0
#define SSIV2_DR11_DR_RegisterSize 32

/* End of Register Definition for DR11 */

/* Register DR12 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR12 (ssic_address_block_BaseAddress + 0x90)
#define SSIV2_DR12_RegisterSize 32
#define SSIV2_DR12_RegisterResetValue 0x0
#define SSIV2_DR12_RegisterResetMask 0xffffffff

/* Register Field information for DR12 */

/* Register DR12 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR12_DR_BitAddressOffset 0
#define SSIV2_DR12_DR_RegisterSize 32

/* End of Register Definition for DR12 */

/* Register DR13 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR13 (ssic_address_block_BaseAddress + 0x94)
#define SSIV2_DR13_RegisterSize 32
#define SSIV2_DR13_RegisterResetValue 0x0
#define SSIV2_DR13_RegisterResetMask 0xffffffff

/* Register Field information for DR13 */

/* Register DR13 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR13_DR_BitAddressOffset 0
#define SSIV2_DR13_DR_RegisterSize 32

/* End of Register Definition for DR13 */

/* Register DR14 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR14 (ssic_address_block_BaseAddress + 0x98)
#define SSIV2_DR14_RegisterSize 32
#define SSIV2_DR14_RegisterResetValue 0x0
#define SSIV2_DR14_RegisterResetMask 0xffffffff

/* Register Field information for DR14 */

/* Register DR14 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR14_DR_BitAddressOffset 0
#define SSIV2_DR14_DR_RegisterSize 32

/* End of Register Definition for DR14 */

/* Register DR15 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR15 (ssic_address_block_BaseAddress + 0x9c)
#define SSIV2_DR15_RegisterSize 32
#define SSIV2_DR15_RegisterResetValue 0x0
#define SSIV2_DR15_RegisterResetMask 0xffffffff

/* Register Field information for DR15 */

/* Register DR15 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR15_DR_BitAddressOffset 0
#define SSIV2_DR15_DR_RegisterSize 32

/* End of Register Definition for DR15 */

/* Register DR16 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR16 (ssic_address_block_BaseAddress + 0xa0)
#define SSIV2_DR16_RegisterSize 32
#define SSIV2_DR16_RegisterResetValue 0x0
#define SSIV2_DR16_RegisterResetMask 0xffffffff

/* Register Field information for DR16 */

/* Register DR16 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR16_DR_BitAddressOffset 0
#define SSIV2_DR16_DR_RegisterSize 32

/* End of Register Definition for DR16 */

/* Register DR17 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR17 (ssic_address_block_BaseAddress + 0xa4)
#define SSIV2_DR17_RegisterSize 32
#define SSIV2_DR17_RegisterResetValue 0x0
#define SSIV2_DR17_RegisterResetMask 0xffffffff

/* Register Field information for DR17 */

/* Register DR17 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR17_DR_BitAddressOffset 0
#define SSIV2_DR17_DR_RegisterSize 32

/* End of Register Definition for DR17 */

/* Register DR18 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR18 (ssic_address_block_BaseAddress + 0xa8)
#define SSIV2_DR18_RegisterSize 32
#define SSIV2_DR18_RegisterResetValue 0x0
#define SSIV2_DR18_RegisterResetMask 0xffffffff

/* Register Field information for DR18 */

/* Register DR18 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR18_DR_BitAddressOffset 0
#define SSIV2_DR18_DR_RegisterSize 32

/* End of Register Definition for DR18 */

/* Register DR19 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR19 (ssic_address_block_BaseAddress + 0xac)
#define SSIV2_DR19_RegisterSize 32
#define SSIV2_DR19_RegisterResetValue 0x0
#define SSIV2_DR19_RegisterResetMask 0xffffffff

/* Register Field information for DR19 */

/* Register DR19 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR19_DR_BitAddressOffset 0
#define SSIV2_DR19_DR_RegisterSize 32

/* End of Register Definition for DR19 */

/* Register DR20 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR20 (ssic_address_block_BaseAddress + 0xb0)
#define SSIV2_DR20_RegisterSize 32
#define SSIV2_DR20_RegisterResetValue 0x0
#define SSIV2_DR20_RegisterResetMask 0xffffffff

/* Register Field information for DR20 */

/* Register DR20 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR20_DR_BitAddressOffset 0
#define SSIV2_DR20_DR_RegisterSize 32

/* End of Register Definition for DR20 */

/* Register DR21 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR21 (ssic_address_block_BaseAddress + 0xb4)
#define SSIV2_DR21_RegisterSize 32
#define SSIV2_DR21_RegisterResetValue 0x0
#define SSIV2_DR21_RegisterResetMask 0xffffffff

/* Register Field information for DR21 */

/* Register DR21 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR21_DR_BitAddressOffset 0
#define SSIV2_DR21_DR_RegisterSize 32

/* End of Register Definition for DR21 */

/* Register DR22 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR22 (ssic_address_block_BaseAddress + 0xb8)
#define SSIV2_DR22_RegisterSize 32
#define SSIV2_DR22_RegisterResetValue 0x0
#define SSIV2_DR22_RegisterResetMask 0xffffffff

/* Register Field information for DR22 */

/* Register DR22 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR22_DR_BitAddressOffset 0
#define SSIV2_DR22_DR_RegisterSize 32

/* End of Register Definition for DR22 */

/* Register DR23 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR23 (ssic_address_block_BaseAddress + 0xbc)
#define SSIV2_DR23_RegisterSize 32
#define SSIV2_DR23_RegisterResetValue 0x0
#define SSIV2_DR23_RegisterResetMask 0xffffffff

/* Register Field information for DR23 */

/* Register DR23 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR23_DR_BitAddressOffset 0
#define SSIV2_DR23_DR_RegisterSize 32

/* End of Register Definition for DR23 */

/* Register DR24 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR24 (ssic_address_block_BaseAddress + 0xc0)
#define SSIV2_DR24_RegisterSize 32
#define SSIV2_DR24_RegisterResetValue 0x0
#define SSIV2_DR24_RegisterResetMask 0xffffffff

/* Register Field information for DR24 */

/* Register DR24 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR24_DR_BitAddressOffset 0
#define SSIV2_DR24_DR_RegisterSize 32

/* End of Register Definition for DR24 */

/* Register DR25 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR25 (ssic_address_block_BaseAddress + 0xc4)
#define SSIV2_DR25_RegisterSize 32
#define SSIV2_DR25_RegisterResetValue 0x0
#define SSIV2_DR25_RegisterResetMask 0xffffffff

/* Register Field information for DR25 */

/* Register DR25 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR25_DR_BitAddressOffset 0
#define SSIV2_DR25_DR_RegisterSize 32

/* End of Register Definition for DR25 */

/* Register DR26 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR26 (ssic_address_block_BaseAddress + 0xc8)
#define SSIV2_DR26_RegisterSize 32
#define SSIV2_DR26_RegisterResetValue 0x0
#define SSIV2_DR26_RegisterResetMask 0xffffffff

/* Register Field information for DR26 */

/* Register DR26 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR26_DR_BitAddressOffset 0
#define SSIV2_DR26_DR_RegisterSize 32

/* End of Register Definition for DR26 */

/* Register DR27 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR27 (ssic_address_block_BaseAddress + 0xcc)
#define SSIV2_DR27_RegisterSize 32
#define SSIV2_DR27_RegisterResetValue 0x0
#define SSIV2_DR27_RegisterResetMask 0xffffffff

/* Register Field information for DR27 */

/* Register DR27 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR27_DR_BitAddressOffset 0
#define SSIV2_DR27_DR_RegisterSize 32

/* End of Register Definition for DR27 */

/* Register DR28 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR28 (ssic_address_block_BaseAddress + 0xd0)
#define SSIV2_DR28_RegisterSize 32
#define SSIV2_DR28_RegisterResetValue 0x0
#define SSIV2_DR28_RegisterResetMask 0xffffffff

/* Register Field information for DR28 */

/* Register DR28 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR28_DR_BitAddressOffset 0
#define SSIV2_DR28_DR_RegisterSize 32

/* End of Register Definition for DR28 */

/* Register DR29 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR29 (ssic_address_block_BaseAddress + 0xd4)
#define SSIV2_DR29_RegisterSize 32
#define SSIV2_DR29_RegisterResetValue 0x0
#define SSIV2_DR29_RegisterResetMask 0xffffffff

/* Register Field information for DR29 */

/* Register DR29 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR29_DR_BitAddressOffset 0
#define SSIV2_DR29_DR_RegisterSize 32

/* End of Register Definition for DR29 */

/* Register DR30 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR30 (ssic_address_block_BaseAddress + 0xd8)
#define SSIV2_DR30_RegisterSize 32
#define SSIV2_DR30_RegisterResetValue 0x0
#define SSIV2_DR30_RegisterResetMask 0xffffffff

/* Register Field information for DR30 */

/* Register DR30 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR30_DR_BitAddressOffset 0
#define SSIV2_DR30_DR_RegisterSize 32

/* End of Register Definition for DR30 */

/* Register DR31 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR31 (ssic_address_block_BaseAddress + 0xdc)
#define SSIV2_DR31_RegisterSize 32
#define SSIV2_DR31_RegisterResetValue 0x0
#define SSIV2_DR31_RegisterResetMask 0xffffffff

/* Register Field information for DR31 */

/* Register DR31 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR31_DR_BitAddressOffset 0
#define SSIV2_DR31_DR_RegisterSize 32

/* End of Register Definition for DR31 */

/* Register DR32 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR32 (ssic_address_block_BaseAddress + 0xe0)
#define SSIV2_DR32_RegisterSize 32
#define SSIV2_DR32_RegisterResetValue 0x0
#define SSIV2_DR32_RegisterResetMask 0xffffffff

/* Register Field information for DR32 */

/* Register DR32 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR32_DR_BitAddressOffset 0
#define SSIV2_DR32_DR_RegisterSize 32

/* End of Register Definition for DR32 */

/* Register DR33 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR33 (ssic_address_block_BaseAddress + 0xe4)
#define SSIV2_DR33_RegisterSize 32
#define SSIV2_DR33_RegisterResetValue 0x0
#define SSIV2_DR33_RegisterResetMask 0xffffffff

/* Register Field information for DR33 */

/* Register DR33 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR33_DR_BitAddressOffset 0
#define SSIV2_DR33_DR_RegisterSize 32

/* End of Register Definition for DR33 */

/* Register DR34 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR34 (ssic_address_block_BaseAddress + 0xe8)
#define SSIV2_DR34_RegisterSize 32
#define SSIV2_DR34_RegisterResetValue 0x0
#define SSIV2_DR34_RegisterResetMask 0xffffffff

/* Register Field information for DR34 */

/* Register DR34 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR34_DR_BitAddressOffset 0
#define SSIV2_DR34_DR_RegisterSize 32

/* End of Register Definition for DR34 */

/* Register DR35 */
/* The DWC_ssi data register is a 32-bit read/write buffer for the transmit/receive FIFOs. When the register is read, data in the receive FIFO buffer is accessed. When it is written to, data are moved into the transmit FIFO buffer; a write can occur only when SSIC_EN = 1. FIFOs are reset when SSIC_EN = 0.
Note The DR register in the DWC_ssi occupies thirty-six 32-bit address locations of the memory map to facilitate AHB burst transfers. Writing to any of these address locations has the same effect as pushing the data from the pwdata bus into the transmit FIFO. Reading from any of these locations has the same effect as popping data from the receive FIFO onto the hrdata bus. The FIFO buffers on the DWC_ssi are not addressable. */
#define SSIV2_DR35 (ssic_address_block_BaseAddress + 0xec)
#define SSIV2_DR35_RegisterSize 32
#define SSIV2_DR35_RegisterResetValue 0x0
#define SSIV2_DR35_RegisterResetMask 0xffffffff

/* Register Field information for DR35 */

/* Register DR35 field DR */
/* Data Register. When writing to this register, you must right-justify the data. Read data are automatically right-justified.
Read = Receive FIFO buffer
Write = Transmit FIFO buffer. */
#define SSIV2_DR35_DR_BitAddressOffset 0
#define SSIV2_DR35_DR_RegisterSize 32

/* End of Register Definition for DR35 */

/* Register RX_SAMPLE_DELAY */
/* This register is only valid when the DWC_ssi is configured with rxd sample delay logic (SSIC_HAS_RX_SAMPLE_DELAY==1). When the DWC_ssi is not configured with rxd sample delay logic, this register will not exist and writing to its address location will have no effect; reading from its address will return zero.
This register control the number of ssi_clk cycles that are delayed (from the default sample time) before the actual sample of the rxd input occurs. It is impossible to write to this register when the DWC_ssi is enabled. */
#define SSIV2_RX_SAMPLE_DELAY (ssic_address_block_BaseAddress + 0xf0)
#define SSIV2_RX_SAMPLE_DELAY_RegisterSize 32
#define SSIV2_RX_SAMPLE_DELAY_RegisterResetValue 0x0
#define SSIV2_RX_SAMPLE_DELAY_RegisterResetMask 0xffffffff

/* Register Field information for RX_SAMPLE_DELAY */

/* Register RX_SAMPLE_DELAY field RSD */
/* Receive Data (rxd) Sample Delay. This register is used to delay the sample of the rxd input port. Each value
represents a single ssi_clk delay on the sample of rxd.
Note; If this register is programmed with a value that exceeds the depth of the internal shift registers (SSIC_RX_DLY_SR_DEPTH) zero delay will be applied to the rxd sample. */
#define SSIV2_RX_SAMPLE_DELAY_RSD_BitAddressOffset 0
#define SSIV2_RX_SAMPLE_DELAY_RSD_RegisterSize 8

/* Register RX_SAMPLE_DELAY field RSVD0_RX_SAMPLE_DLY */
/* Reserved bits - read as zero */
#define SSIV2_RX_SAMPLE_DELAY_RSVD0_RX_SAMPLE_DLY_BitAddressOffset 8
#define SSIV2_RX_SAMPLE_DELAY_RSVD0_RX_SAMPLE_DLY_RegisterSize 8

/* Register RX_SAMPLE_DELAY field SE */
/* Receive Data (rxd) Sampling Edge. This register is used to decide the sampling edge for RXD signal with ssi_clk. Then this bit is set to 1 then negative edge of ssi_clk will be used to sample the incoming data, otherwise positive edge will be used for sampling. */
#define SSIV2_RX_SAMPLE_DELAY_SE_BitAddressOffset 16
#define SSIV2_RX_SAMPLE_DELAY_SE_RegisterSize 1

/* Register RX_SAMPLE_DELAY field RSVD1_RX_SAMPLE_DLY */
/* Reserved bits - read as zero */
#define SSIV2_RX_SAMPLE_DELAY_RSVD1_RX_SAMPLE_DLY_BitAddressOffset 17
#define SSIV2_RX_SAMPLE_DELAY_RSVD1_RX_SAMPLE_DLY_RegisterSize 15

/* End of Register Definition for RX_SAMPLE_DELAY */

/* Register SPI_CTRLR0 */
/* This register is used to control the serial data transfer in enhanced SPI mode of operation. The register is relevant only when either CTRLR0.SPI_DWS_EN is set to 1 or CTRLR0.SPI_FRF is set to either 01 or 10 or 11. It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_SPI_CTRLR0 (ssic_address_block_BaseAddress + 0xf4)
#define SSIV2_SPI_CTRLR0_RegisterSize 32
#define SSIV2_SPI_CTRLR0_RegisterResetValue 0x200
#define SSIV2_SPI_CTRLR0_RegisterResetMask 0xffffffff

/* Register Field information for SPI_CTRLR0 */

/* Register SPI_CTRLR0 field TRANS_TYPE */
/* Address and instruction transfer format.
Selects whether DWC_ssi will transmit instruction/address either in Standard SPI mode or the SPI mode selected in CTRLR0.SPI_FRF field. */
#define SSIV2_SPI_CTRLR0_TRANS_TYPE_BitAddressOffset 0
#define SSIV2_SPI_CTRLR0_TRANS_TYPE_RegisterSize 2

/* Register SPI_CTRLR0 field ADDR_L */
/* This bit defines Length of Address to be transmitted. Only after this much bits are programmed in to the FIFO the transfer can begin. */
#define SSIV2_SPI_CTRLR0_ADDR_L_BitAddressOffset 2
#define SSIV2_SPI_CTRLR0_ADDR_L_RegisterSize 4

/* Register SPI_CTRLR0 field RSVD_SPI_CTRLR0_6 */
/* Reserved bits - read as zero */
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_6_BitAddressOffset 6
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_6_RegisterSize 1

/* Register SPI_CTRLR0 field XIP_MD_BIT_EN */
/* Mode bits enable in XIP mode. If this bit is set to 1, then in XIP mode of operation DWC_ssi will insert mode bits after the address phase. These bits are set in register XIP_MODE_BITS register. The length of mode bits is always set to 8 bits. */
#define SSIV2_SPI_CTRLR0_XIP_MD_BIT_EN_BitAddressOffset 7
#define SSIV2_SPI_CTRLR0_XIP_MD_BIT_EN_RegisterSize 1

/* Register SPI_CTRLR0 field INST_L */
/* Dual/Quad/Octal mode instruction length in bits.
 */
#define SSIV2_SPI_CTRLR0_INST_L_BitAddressOffset 8
#define SSIV2_SPI_CTRLR0_INST_L_RegisterSize 2

/* Register SPI_CTRLR0 field RSVD_SPI_CTRLR0_10 */
/* Reserved bits - read as zero */
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_10_BitAddressOffset 10
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_10_RegisterSize 1

/* Register SPI_CTRLR0 field WAIT_CYCLES */
/* Wait cycles in Dual/Quad/Octal mode between control frames transmit and data reception. Specified as number of SPI clock cycles. */
#define SSIV2_SPI_CTRLR0_WAIT_CYCLES_BitAddressOffset 11
#define SSIV2_SPI_CTRLR0_WAIT_CYCLES_RegisterSize 5

/* Register SPI_CTRLR0 field SPI_DDR_EN */
/* SPI DDR Enable bit. This will enable Dual-data rate transfers in Dual/Quad/Octal frame formats of SPI. */
#define SSIV2_SPI_CTRLR0_SPI_DDR_EN_BitAddressOffset 16
#define SSIV2_SPI_CTRLR0_SPI_DDR_EN_RegisterSize 1

/* Register SPI_CTRLR0 field INST_DDR_EN */
/* Instruction DDR Enable bit. This will enable Dual-data rate transfer for Instruction phase. */
#define SSIV2_SPI_CTRLR0_INST_DDR_EN_BitAddressOffset 17
#define SSIV2_SPI_CTRLR0_INST_DDR_EN_RegisterSize 1

/* Register SPI_CTRLR0 field SPI_RXDS_EN */
/* Read data strobe enable bit. Once this bit is set to 1 DWC_ssi will use Read data strobe (rxds) to capture read data in DDR mode. */
#define SSIV2_SPI_CTRLR0_SPI_RXDS_EN_BitAddressOffset 18
#define SSIV2_SPI_CTRLR0_SPI_RXDS_EN_RegisterSize 1

/* Register SPI_CTRLR0 field XIP_DFS_HC */
/* Fix DFS for XIP transfers. If this bit is set to 1 then data frame size for XIP transfers will be fixed to the programmed value in CTRLR0.DFS. The number of data frames to fetch will be determined by HSIZE and HBURST signals. If this bit is set to 0 then data frame size and number of data frames to fetch will be determined by HSIZE and HBURST signals */
#define SSIV2_SPI_CTRLR0_XIP_DFS_HC_BitAddressOffset 19
#define SSIV2_SPI_CTRLR0_XIP_DFS_HC_RegisterSize 1

/* Register SPI_CTRLR0 field XIP_INST_EN */
/* XIP instruction enable bit. If this bit is set to 1 then XIP transfers will also have instruction phase. The instruction op-codes will be chosen from XIP_INCR_INST or XIP_WRAP_INST registers bases on AHB transfer type. */
#define SSIV2_SPI_CTRLR0_XIP_INST_EN_BitAddressOffset 20
#define SSIV2_SPI_CTRLR0_XIP_INST_EN_RegisterSize 1

/* Register SPI_CTRLR0 field SSIC_XIP_CONT_XFER_EN */
/* Enable continuous transfer in XIP mode. If this bit is set to 1 then continuous transfer mode in XIP will be enabled, in this mode DWC_ssi will keep slave selected until a non-XIP transfer is detected on the AHB interface. */
#define SSIV2_SPI_CTRLR0_SSIC_XIP_CONT_XFER_EN_BitAddressOffset 21
#define SSIV2_SPI_CTRLR0_SSIC_XIP_CONT_XFER_EN_RegisterSize 1

/* Register SPI_CTRLR0 field RSVD_SPI_CTRLR0_22_23 */
/* Reserved bits - read as zero */
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_22_23_BitAddressOffset 22
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_22_23_RegisterSize 2

/* Register SPI_CTRLR0 field SPI_DM_EN */
/* SPI data mask enable bit.
When this bit is enabled, the txd_dm signal is used to mask the data on the txd data line. This bit is enabled only when the SSIC_DM_EN parameter is set to 1. */
#define SSIV2_SPI_CTRLR0_SPI_DM_EN_BitAddressOffset 24
#define SSIV2_SPI_CTRLR0_SPI_DM_EN_RegisterSize 1

/* Register SPI_CTRLR0 field SPI_RXDS_SIG_EN */
/* Enable rxds signaling during address and command phase of Hypebus transfer.
This bit enables rxds signaling by Hyperbus slave devices during Command-Address (CA) phase. If the rxds signal is set to 1 during the CA phase of transfer, DWC_ssi transmits (2*SPI_CTRLR0.WAIT_CYCLES-1) wait cycles after the address phase is complete. */
#define SSIV2_SPI_CTRLR0_SPI_RXDS_SIG_EN_BitAddressOffset 25
#define SSIV2_SPI_CTRLR0_SPI_RXDS_SIG_EN_RegisterSize 1

/* Register SPI_CTRLR0 field XIP_MBL */
/* XIP Mode bits length. Sets the length of mode bits in XIP mode of operation. These bits are valid only when SPI_CTRLR0.XIP_MD_BIT_EN is set to 1. */
#define SSIV2_SPI_CTRLR0_XIP_MBL_BitAddressOffset 26
#define SSIV2_SPI_CTRLR0_XIP_MBL_RegisterSize 2

/* Register SPI_CTRLR0 field RSVD_SPI_CTRLR0_28 */
/* Reserved bits - read as zero */
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_28_BitAddressOffset 28
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_28_RegisterSize 1

/* Register SPI_CTRLR0 field XIP_PREFETCH_EN */
/* Enables XIP pre-fetch functionality in DWC_ssi.
Once enabled DWC_ssi will pre-fetch data frames from next contigous location, to reduce the latency for the upcoming contiguous transfer. If the next XIP request is not contigous then pre-fetched bits will be discarded. */
#define SSIV2_SPI_CTRLR0_XIP_PREFETCH_EN_BitAddressOffset 29
#define SSIV2_SPI_CTRLR0_XIP_PREFETCH_EN_RegisterSize 1

/* Register SPI_CTRLR0 field CLK_STRETCH_EN */
/* Enables clock stretching capability in SPI transfers.

In case of write, if the FIFO becomes empty DWC_ssi will stretch the clock until FIFO has enough data to continue the transfer.

In case of read, if the receive FIFO becomes full DWC_ssi will stop the clock until data has been read from the FIFO. */
#define SSIV2_SPI_CTRLR0_CLK_STRETCH_EN_BitAddressOffset 30
#define SSIV2_SPI_CTRLR0_CLK_STRETCH_EN_RegisterSize 1

/* Register SPI_CTRLR0 field RSVD_SPI_CTRLR0 */
/* Reserved bits - read as zero */
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_BitAddressOffset 31
#define SSIV2_SPI_CTRLR0_RSVD_SPI_CTRLR0_RegisterSize 1

/* End of Register Definition for SPI_CTRLR0 */

/* Register DDR_DRIVE_EDGE */
/* This Register is valid only when SSIC_HAS_DDR is equal to 1. This register is used to control the driving edge of TXD register in DDR mode.
 It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_DDR_DRIVE_EDGE (ssic_address_block_BaseAddress + 0xf8)
#define SSIV2_DDR_DRIVE_EDGE_RegisterSize 32
#define SSIV2_DDR_DRIVE_EDGE_RegisterResetValue 0x0
#define SSIV2_DDR_DRIVE_EDGE_RegisterResetMask 0xffffffff

/* Register Field information for DDR_DRIVE_EDGE */

/* Register DDR_DRIVE_EDGE field TDE */
/* TXD Drive edge register which decided the driving edge of transmit data.
The maximum value of this register is = (BAUDR/2) -1. */
#define SSIV2_DDR_DRIVE_EDGE_TDE_BitAddressOffset 0
#define SSIV2_DDR_DRIVE_EDGE_TDE_RegisterSize 8

/* Register DDR_DRIVE_EDGE field RSVD_DDR_DRIVE_EDGE */
/* Reserved bits - read as zero */
#define SSIV2_DDR_DRIVE_EDGE_RSVD_DDR_DRIVE_EDGE_BitAddressOffset 8
#define SSIV2_DDR_DRIVE_EDGE_RSVD_DDR_DRIVE_EDGE_RegisterSize 24

/* End of Register Definition for DDR_DRIVE_EDGE */

/* Register XIP_MODE_BITS */
/* This register carries the mode bits which are sent in the XIP mode of operation after address phase. This is a 8 bit register and can only be written when SSIENR register is set to 0. */
#define SSIV2_XIP_MODE_BITS (ssic_address_block_BaseAddress + 0xfc)
#define SSIV2_XIP_MODE_BITS_RegisterSize 32
#define SSIV2_XIP_MODE_BITS_RegisterResetValue 0x0
#define SSIV2_XIP_MODE_BITS_RegisterResetMask 0xffffffff

/* Register Field information for XIP_MODE_BITS */

/* Register XIP_MODE_BITS field XIP_MD_BITS */
/* XIP mode bits to be sent after address phase of XIP transfer. */
#define SSIV2_XIP_MODE_BITS_XIP_MD_BITS_BitAddressOffset 0
#define SSIV2_XIP_MODE_BITS_XIP_MD_BITS_RegisterSize 16

/* Register XIP_MODE_BITS field RSVD_XIP_MD_BITS */
/* Reserved bits - read as zero */
#define SSIV2_XIP_MODE_BITS_RSVD_XIP_MD_BITS_BitAddressOffset 16
#define SSIV2_XIP_MODE_BITS_RSVD_XIP_MD_BITS_RegisterSize 16

/* End of Register Definition for XIP_MODE_BITS */

#define ssic_address_block2_BaseAddress 0x100

/* Register XIP_INCR_INST */
/* This Register is valid only when SSIC_XIP_EN is equal to 1. This register is used to store the instruction op-code to be used in INCR transactions when the same is requested on AHB interface.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_INCR_INST (ssic_address_block2_BaseAddress + 0x0)
#define SSIV2_XIP_INCR_INST_RegisterSize 32
#define SSIV2_XIP_INCR_INST_RegisterResetValue 0x0
#define SSIV2_XIP_INCR_INST_RegisterResetMask 0xffffffff

/* Register Field information for XIP_INCR_INST */

/* Register XIP_INCR_INST field INCR_INST */
/* XIP INCR transfer opcode.
 When SPI_CTRLR0.XIP_INST_EN bit is set to 1, DWC_ssi sends instruction for all XIP transfers, this register field stores the instruction op-code to be sent when an INCR type transfer is requested on AHB bus. The number of bits to be send in instruction phase is determined by SPI_CTRL0.INST_L field. */
#define SSIV2_XIP_INCR_INST_INCR_INST_BitAddressOffset 0
#define SSIV2_XIP_INCR_INST_INCR_INST_RegisterSize 16

/* Register XIP_INCR_INST field RSVD_INCR_INST */
/* Reserved bits - read as zero */
#define SSIV2_XIP_INCR_INST_RSVD_INCR_INST_BitAddressOffset 16
#define SSIV2_XIP_INCR_INST_RSVD_INCR_INST_RegisterSize 16

/* End of Register Definition for XIP_INCR_INST */

/* Register XIP_WRAP_INST */
/* This Register is valid only when SSIC_XIP_EN is equal to 1. This register is used to store the instruction op-code to be used in WRAP transactions when the same is requested on AHB interface.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_WRAP_INST (ssic_address_block2_BaseAddress + 0x4)
#define SSIV2_XIP_WRAP_INST_RegisterSize 32
#define SSIV2_XIP_WRAP_INST_RegisterResetValue 0x0
#define SSIV2_XIP_WRAP_INST_RegisterResetMask 0xffffffff

/* Register Field information for XIP_WRAP_INST */

/* Register XIP_WRAP_INST field WRAP_INST */
/* XIP WRAP transfer opcode.
 When SPI_CTRLR0.XIP_INST_EN bit is set to 1, DWC_ssi sends instruction for all XIP transfers, this register field stores the instruction op-code to be sent when an WRAP type transfer is requested on AHB bus. The number of bits to be send in instruction phase is determined by SPI_CTRL0.INST_L field. */
#define SSIV2_XIP_WRAP_INST_WRAP_INST_BitAddressOffset 0
#define SSIV2_XIP_WRAP_INST_WRAP_INST_RegisterSize 16

/* Register XIP_WRAP_INST field RSVD_WRAP_INST */
/* Reserved bits - read as zero */
#define SSIV2_XIP_WRAP_INST_RSVD_WRAP_INST_BitAddressOffset 16
#define SSIV2_XIP_WRAP_INST_RSVD_WRAP_INST_RegisterSize 16

/* End of Register Definition for XIP_WRAP_INST */

/* Register XIP_CTRL */
/* This Register is valid only when SSIC_CONCURRENT_XIP_EN is equal to 1. This register is used to store the control information that the XIP transfer will be using in the concurrent mode.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_CTRL (ssic_address_block2_BaseAddress + 0x8)
#define SSIV2_XIP_CTRL_RegisterSize 32
#define SSIV2_XIP_CTRL_RegisterResetValue 0x8000401
#define SSIV2_XIP_CTRL_RegisterResetMask 0xffffffff

/* Register Field information for XIP_CTRL */

/* Register XIP_CTRL field FRF */
/* SPI Frame Format
Selects data frame format for Transmitting/Receiving the data. */
#define SSIV2_XIP_CTRL_FRF_BitAddressOffset 0
#define SSIV2_XIP_CTRL_FRF_RegisterSize 2

/* Register XIP_CTRL field TRANS_TYPE */
/* Address and instruction transfer format.
Selects whether DWC_ssi will transmit instruction/address either in Standard SPI mode or the SPI mode selected in CTRLR0.SPI_FRF field. */
#define SSIV2_XIP_CTRL_TRANS_TYPE_BitAddressOffset 2
#define SSIV2_XIP_CTRL_TRANS_TYPE_RegisterSize 2

/* Register XIP_CTRL field ADDR_L */
/* This bit defines Length of Address to be transmitted. Only after this much bits are programmed in to the FIFO the transfer can begin. */
#define SSIV2_XIP_CTRL_ADDR_L_BitAddressOffset 4
#define SSIV2_XIP_CTRL_ADDR_L_RegisterSize 4

/* Register XIP_CTRL field RSVD_XIP_CTRL_8 */
/* Reserved bits - read as zero */
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_8_BitAddressOffset 8
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_8_RegisterSize 1

/* Register XIP_CTRL field INST_L */
/* Dual/Quad/Octal mode instruction length in bits.
 */
#define SSIV2_XIP_CTRL_INST_L_BitAddressOffset 9
#define SSIV2_XIP_CTRL_INST_L_RegisterSize 2

/* Register XIP_CTRL field RSVD_SPI_CTRLR0_11 */
/* Reserved bits - read as zero */
#define SSIV2_XIP_CTRL_RSVD_SPI_CTRLR0_11_BitAddressOffset 11
#define SSIV2_XIP_CTRL_RSVD_SPI_CTRLR0_11_RegisterSize 1

/* Register XIP_CTRL field MD_BITS_EN */
/* Mode bits enable in XIP mode. If this bit is set to 1, then in XIP mode of operation DWC_ssi will insert mode bits after the address phase. These bits are set in register XIP_MODE_BITS register. The length of mode bits is always set to 8 bits. */
#define SSIV2_XIP_CTRL_MD_BITS_EN_BitAddressOffset 12
#define SSIV2_XIP_CTRL_MD_BITS_EN_RegisterSize 1

/* Register XIP_CTRL field WAIT_CYCLES */
/* Wait cycles in Dual/Quad/Octal mode between control frames transmit and data reception. Specified as number of SPI clock cycles. */
#define SSIV2_XIP_CTRL_WAIT_CYCLES_BitAddressOffset 13
#define SSIV2_XIP_CTRL_WAIT_CYCLES_RegisterSize 5

/* Register XIP_CTRL field DFS_HC */
/* Fix DFS for XIP transfers. If this bit is set to 1 then data frame size for XIP transfers will be fixed to the programmed value in CTRLR0.DFS. The number of data frames to fetch will be determined by HSIZE and HBURST signals. If this bit is set to 0 then data frame size and number of data frames to fetch will be determined by HSIZE and HBURST signals */
#define SSIV2_XIP_CTRL_DFS_HC_BitAddressOffset 18
#define SSIV2_XIP_CTRL_DFS_HC_RegisterSize 1

/* Register XIP_CTRL field DDR_EN */
/* SPI DDR Enable bit. This will enable Dual-data rate transfers in Dual/Quad/Octal frame formats of SPI. */
#define SSIV2_XIP_CTRL_DDR_EN_BitAddressOffset 19
#define SSIV2_XIP_CTRL_DDR_EN_RegisterSize 1

/* Register XIP_CTRL field INST_DDR_EN */
/* Instruction DDR Enable bit. This will enable Dual-data rate transfer for Instruction phase. */
#define SSIV2_XIP_CTRL_INST_DDR_EN_BitAddressOffset 20
#define SSIV2_XIP_CTRL_INST_DDR_EN_RegisterSize 1

/* Register XIP_CTRL field RXDS_EN */
/* Read data strobe enable bit. Once this bit is set to 1 DWC_ssi will use Read data strobe (rxds) to capture read data in DDR mode. */
#define SSIV2_XIP_CTRL_RXDS_EN_BitAddressOffset 21
#define SSIV2_XIP_CTRL_RXDS_EN_RegisterSize 1

/* Register XIP_CTRL field INST_EN */
/* XIP instruction enable bit. If this bit is set to 1 then XIP transfers will also have instruction phase. The instruction op-codes will be chosen from XIP_INCR_INST or XIP_WRAP_INST registers bases on AHB transfer type. */
#define SSIV2_XIP_CTRL_INST_EN_BitAddressOffset 22
#define SSIV2_XIP_CTRL_INST_EN_RegisterSize 1

/* Register XIP_CTRL field CONT_XFER_EN */
/* Enable continuous transfer in XIP mode. If this bit is set to 1 then continuous transfer mode in XIP will be enabled, in this mode DWC_ssi will keep slave selected until a non-XIP transfer is detected on the AHB interface. */
#define SSIV2_XIP_CTRL_CONT_XFER_EN_BitAddressOffset 23
#define SSIV2_XIP_CTRL_CONT_XFER_EN_RegisterSize 1

/* Register XIP_CTRL field XIP_HYPERBUS_EN */
/* SPI Hyperbus Frame format enable for XIP transfers.
Selects if data frame format for XIP transfers is in Hyperbus mode. This field is effective only when CTRLRO.FRF is set to SPI frame format. */
#define SSIV2_XIP_CTRL_XIP_HYPERBUS_EN_BitAddressOffset 24
#define SSIV2_XIP_CTRL_XIP_HYPERBUS_EN_RegisterSize 1

/* Register XIP_CTRL field RXDS_SIG_EN */
/* Enable rxds signaling during address and command phase of Hyperbus transfer.
This bit enables rxds signaling by Hyperbus slave devices during Command-Address (CA) phase. If the rxds signal is
set to 1 during the CA phase of transfer, DWC_ssi transmits (2*SPI_CTRLR0.WAIT_CYCLES-1) wait cycles after the address phase is complete. */
#define SSIV2_XIP_CTRL_RXDS_SIG_EN_BitAddressOffset 25
#define SSIV2_XIP_CTRL_RXDS_SIG_EN_RegisterSize 1

/* Register XIP_CTRL field XIP_MBL */
/* XIP Mode bits length. Sets the length of mode bits in XIP mode of operation. These bits are valid only when XIP_CTRL.XIP_MD_BIT_EN is set to 1. */
#define SSIV2_XIP_CTRL_XIP_MBL_BitAddressOffset 26
#define SSIV2_XIP_CTRL_XIP_MBL_RegisterSize 2

/* Register XIP_CTRL field RSVD_XIP_CTRL_28 */
/* Reserved bits - read as zero */
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_28_BitAddressOffset 28
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_28_RegisterSize 1

/* Register XIP_CTRL field XIP_PREFETCH_EN */
/* Enables XIP pre-fetch functionality in DWC_ssi.
Once enabled DWC_ssi will pre-fetch data frames from next contigous location, to reduce the latency for the upcoming contiguous transfer. If the next XIP request is not contigous then pre-fetched bits will be discarded. */
#define SSIV2_XIP_CTRL_XIP_PREFETCH_EN_BitAddressOffset 29
#define SSIV2_XIP_CTRL_XIP_PREFETCH_EN_RegisterSize 1

/* Register XIP_CTRL field RSVD_XIP_CTRL */
/* Reserved bits - read as zero */
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_BitAddressOffset 30
#define SSIV2_XIP_CTRL_RSVD_XIP_CTRL_RegisterSize 2

/* End of Register Definition for XIP_CTRL */

/* Register XRXOICR */
/* XIP Receive FIFO Overflow Interrupt Clear Register */
#define SSIV2_XRXOICR (ssic_address_block2_BaseAddress + 0x10)
#define SSIV2_XRXOICR_RegisterSize 32
#define SSIV2_XRXOICR_RegisterResetValue 0x0
#define SSIV2_XRXOICR_RegisterResetMask 0xffffffff

/* Register Field information for XRXOICR */

/* Register XRXOICR field XRXOICR */
/* Clear XIP Receive FIFO Overflow Interrupt.
This register reflects the status of the interrupt. A read from this register clears the ssi_xrxo_intr(_n) interrupt; writing has no effect. */
#define SSIV2_XRXOICR_XRXOICR_BitAddressOffset 0
#define SSIV2_XRXOICR_XRXOICR_RegisterSize 1

/* Register XRXOICR field RSVD_XRXOICR */
/* Reserved bits - read as zero */
#define SSIV2_XRXOICR_RSVD_XRXOICR_BitAddressOffset 1
#define SSIV2_XRXOICR_RSVD_XRXOICR_RegisterSize 31

/* End of Register Definition for XRXOICR */

/* Register XIP_CNT_TIME_OUT */
/* XIP count down register for continuous mode. The counter is used to de-select the slave during continuous transfer mode.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_CNT_TIME_OUT (ssic_address_block2_BaseAddress + 0x14)
#define SSIV2_XIP_CNT_TIME_OUT_RegisterSize 32
#define SSIV2_XIP_CNT_TIME_OUT_RegisterResetValue 0x0
#define SSIV2_XIP_CNT_TIME_OUT_RegisterResetMask 0xffffffff

/* Register Field information for XIP_CNT_TIME_OUT */

/* Register XIP_CNT_TIME_OUT field XTOC */
/* XIP time out value in terms of hclk. Once slave is selected in continuous XIP mode this counter will be used to de-select the slave if there is no request for the time specified in the counter. */
#define SSIV2_XIP_CNT_TIME_OUT_XTOC_BitAddressOffset 0
#define SSIV2_XIP_CNT_TIME_OUT_XTOC_RegisterSize 8

/* Register XIP_CNT_TIME_OUT field RSVD_XTOC */
/* Reserved bits - read as zero */
#define SSIV2_XIP_CNT_TIME_OUT_RSVD_XTOC_BitAddressOffset 8
#define SSIV2_XIP_CNT_TIME_OUT_RSVD_XTOC_RegisterSize 24

/* End of Register Definition for XIP_CNT_TIME_OUT */

/* Register XIP_WRITE_INCR_INST */
/* This Register is valid only when both SSIC_XIP_WRITE_REG_EN is set to 1. This register is used to store the instruction op-code to be used in INCR transactions for XIP Write when the same is requested on AHB interface.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_WRITE_INCR_INST (ssic_address_block2_BaseAddress + 0x40)
#define SSIV2_XIP_WRITE_INCR_INST_RegisterSize 32
#define SSIV2_XIP_WRITE_INCR_INST_RegisterResetValue 0x0
#define SSIV2_XIP_WRITE_INCR_INST_RegisterResetMask 0xffffffff

/* Register Field information for XIP_WRITE_INCR_INST */

/* Register XIP_WRITE_INCR_INST field INCR_WRITE_INST */
/* XIP Write INCR transfer opcode.
 When XIP_WRITE_CTRL.INST_L is not equal to 0, DWC_ssi sends instruction for all XIP write transfers, this register field stores the instruction op-code to be sent when an INCR type XIP Write transfer is requested on AHB bus. The number of bits to be send in instruction phase is determined by XIP_WRITE_CTRL.INST_L field. */
#define SSIV2_XIP_WRITE_INCR_INST_INCR_WRITE_INST_BitAddressOffset 0
#define SSIV2_XIP_WRITE_INCR_INST_INCR_WRITE_INST_RegisterSize 16

/* Register XIP_WRITE_INCR_INST field RSVD_INCR_INST_16to31 */
/* Reserved bits - Read Only */
#define SSIV2_XIP_WRITE_INCR_INST_RSVD_INCR_INST_16to31_BitAddressOffset 16
#define SSIV2_XIP_WRITE_INCR_INST_RSVD_INCR_INST_16to31_RegisterSize 16

/* End of Register Definition for XIP_WRITE_INCR_INST */

/* Register XIP_WRITE_WRAP_INST */
/* This Register is valid only when both SSIC_XIP_WRITE_REG_EN is set to 1. This register is used to store the instruction op-code to be used in WRAP transactions for XIP Write when the same is requested on AHB interface.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_WRITE_WRAP_INST (ssic_address_block2_BaseAddress + 0x44)
#define SSIV2_XIP_WRITE_WRAP_INST_RegisterSize 32
#define SSIV2_XIP_WRITE_WRAP_INST_RegisterResetValue 0x0
#define SSIV2_XIP_WRITE_WRAP_INST_RegisterResetMask 0xffffffff

/* Register Field information for XIP_WRITE_WRAP_INST */

/* Register XIP_WRITE_WRAP_INST field WRAP_WRITE_INST */
/* XIP Write WRAP transfer opcode.
 When XIP_WRITE_CTRL.INST_L is not equal to 0, DWC_ssi sends instruction for all XIP write transfers, this register field stores the instruction op-code to be sent when an WRAP type XIP Write transfer is requested on AHB bus. The number of bits to be send in instruction phase is determined by XIP_WRITE_CTRL.INST_L field. */
#define SSIV2_XIP_WRITE_WRAP_INST_WRAP_WRITE_INST_BitAddressOffset 0
#define SSIV2_XIP_WRITE_WRAP_INST_WRAP_WRITE_INST_RegisterSize 16

/* Register XIP_WRITE_WRAP_INST field RSVD_WRAP_INST_16to31 */
/* Reserved bits - Read Only */
#define SSIV2_XIP_WRITE_WRAP_INST_RSVD_WRAP_INST_16to31_BitAddressOffset 16
#define SSIV2_XIP_WRITE_WRAP_INST_RSVD_WRAP_INST_16to31_RegisterSize 16

/* End of Register Definition for XIP_WRITE_WRAP_INST */

/* Register XIP_WRITE_CTRL */
/* This Register is valid only when SSIC_XIP_WRITE_REG_EN is equal to 1. This register is used to store the control information that the XIP write transfer will be using in the xip mode.
It is not possible to write to this register when the DWC_ssi is enabled (SSIC_EN=1). */
#define SSIV2_XIP_WRITE_CTRL (ssic_address_block2_BaseAddress + 0x48)
#define SSIV2_XIP_WRITE_CTRL_RegisterSize 32
#define SSIV2_XIP_WRITE_CTRL_RegisterResetValue 0x73
#define SSIV2_XIP_WRITE_CTRL_RegisterResetMask 0xffffffff

/* Register Field information for XIP_WRITE_CTRL */

/* Register XIP_WRITE_CTRL field WR_FRF */
/* SPI Frame Format
Selects data frame format for Transmitting the data. */
#define SSIV2_XIP_WRITE_CTRL_WR_FRF_BitAddressOffset 0
#define SSIV2_XIP_WRITE_CTRL_WR_FRF_RegisterSize 2

/* Register XIP_WRITE_CTRL field WR_TRANS_TYPE */
/* Address and instruction transfer format.
Selects whether DWC_ssi will transmit instruction/address either in Standard SPI mode or the SPI mode selected in XIP_WRITE_CTRL.FRF field. */
#define SSIV2_XIP_WRITE_CTRL_WR_TRANS_TYPE_BitAddressOffset 2
#define SSIV2_XIP_WRITE_CTRL_WR_TRANS_TYPE_RegisterSize 2

/* Register XIP_WRITE_CTRL field WR_ADDR_L */
/* This bit defines Length of Address to be transmitted. Only after this much bits are programmed in to the FIFO the transfer can begin. */
#define SSIV2_XIP_WRITE_CTRL_WR_ADDR_L_BitAddressOffset 4
#define SSIV2_XIP_WRITE_CTRL_WR_ADDR_L_RegisterSize 4

/* Register XIP_WRITE_CTRL field WR_INST_L */
/* Dual/Quad/Octal mode instruction length in bits.
 */
#define SSIV2_XIP_WRITE_CTRL_WR_INST_L_BitAddressOffset 8
#define SSIV2_XIP_WRITE_CTRL_WR_INST_L_RegisterSize 2

/* Register XIP_WRITE_CTRL field WR_SPI_DDR_EN */
/* SPI DDR Enable bit. This will enable Dual-data rate transfers in Dual/Quad/Octal frame formats of SPI. */
#define SSIV2_XIP_WRITE_CTRL_WR_SPI_DDR_EN_BitAddressOffset 10
#define SSIV2_XIP_WRITE_CTRL_WR_SPI_DDR_EN_RegisterSize 1

/* Register XIP_WRITE_CTRL field WR_INST_DDR_EN */
/* Instruction DDR Enable bit. This will enable Dual-data rate transfer for Instruction phase. */
#define SSIV2_XIP_WRITE_CTRL_WR_INST_DDR_EN_BitAddressOffset 11
#define SSIV2_XIP_WRITE_CTRL_WR_INST_DDR_EN_RegisterSize 1

/* Register XIP_WRITE_CTRL field XIPWR_HYPERBUS_EN */
/* SPI Hyperbus Frame format enable for XIP Write transfers.
Selects if data frame format for XIP Write transfers is in Hyperbus mode. This field is effective only when CTRLRO.FRF is set to SPI frame format. */
#define SSIV2_XIP_WRITE_CTRL_XIPWR_HYPERBUS_EN_BitAddressOffset 12
#define SSIV2_XIP_WRITE_CTRL_XIPWR_HYPERBUS_EN_RegisterSize 1

/* Register XIP_WRITE_CTRL field XIPWR_RXDS_SIG_EN */
/* Enable rxds signaling during address and command phase of Hyperbus transfer.
This bit enables rxds signaling by Hyperbus slave devices during Command-Address (CA) phase. If the rxds signal is
set to 1 during the CA phase of transfer, DWC_ssi transmits (2*XIP_WRITE_CTRL.WAIT_CYCLES-1) wait cycles after the address phase is complete. */
#define SSIV2_XIP_WRITE_CTRL_XIPWR_RXDS_SIG_EN_BitAddressOffset 13
#define SSIV2_XIP_WRITE_CTRL_XIPWR_RXDS_SIG_EN_RegisterSize 1

/* Register XIP_WRITE_CTRL field RSVD_XIP_WRITECTRL_14to15 */
/* Reserved bits - Read Only */
#define SSIV2_XIP_WRITE_CTRL_RSVD_XIP_WRITECTRL_14to15_BitAddressOffset 14
#define SSIV2_XIP_WRITE_CTRL_RSVD_XIP_WRITECTRL_14to15_RegisterSize 2

/* Register XIP_WRITE_CTRL field XIPWR_WAIT_CYCLES */
/* Wait cycles in Dual/Quad/Octal mode between control frames transmit and data reception. Specified as number of SPI clock cycles. */
#define SSIV2_XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES_BitAddressOffset 16
#define SSIV2_XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES_RegisterSize 5

/* Register XIP_WRITE_CTRL field RSVD_XIP_WRITECTRL_21to31 */
/* Reserved bits - Read Only */
#define SSIV2_XIP_WRITE_CTRL_RSVD_XIP_WRITECTRL_21to31_BitAddressOffset 21
#define SSIV2_XIP_WRITE_CTRL_RSVD_XIP_WRITECTRL_21to31_RegisterSize 11

/* End of Register Definition for XIP_WRITE_CTRL */


#endif /* DWC_SSI_HEADER_H_ */
