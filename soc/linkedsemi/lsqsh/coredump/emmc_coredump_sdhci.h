/*
 * Copyright (c) 2024 LinkedSemi Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief eMMC/SDHCI definitions and operations for coredump
 *
 * This provides:
 * 1. SDHCI register definitions compatible with Zephyr's sdhci.h
 * 2. HAL eMMC function declarations for card initialization
 * 3. Bare-metal coredump operation functions
 */

#ifndef SDHCI_OF_LINKEDSEMI_EMMC_PROTECTION_H_
#define SDHCI_OF_LINKEDSEMI_EMMC_PROTECTION_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*
 * ============================================================================
 * SDHCI Register Definitions - compatible with Zephyr's sdhci.h
 * ============================================================================
 */
#define SDHCI_DMA_ADDRESS       0x00
#define SDHCI_ARGUMENT2         SDHCI_DMA_ADDRESS
#define SDHCI_32BIT_BLK_CNT    SDHCI_DMA_ADDRESS

#define SDHCI_BLOCK_SIZE        0x04
#define SDHCI_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

#define SDHCI_BLOCK_COUNT       0x06
#define SDHCI_ARGUMENT          0x08
#define SDHCI_TRANSFER_MODE     0x0C
#define SDHCI_TRNS_DMA          0x01
#define SDHCI_TRNS_BLK_CNT_EN   0x02
#define SDHCI_TRNS_AUTO_CMD12    0x04
#define SDHCI_TRNS_AUTO_CMD23    0x08
#define SDHCI_TRNS_READ          0x10
#define SDHCI_TRNS_MULTI         0x20

#define SDHCI_COMMAND           0x0E
#define SDHCI_CMD_CRC           0x08
#define SDHCI_CMD_INDEX         0x10
#define SDHCI_CMD_DATA          0x20
#define SDHCI_MAKE_CMD(c, f)    (((c & 0xff) << 8) | (f & 0xff))

#define SDHCI_RESPONSE          0x10
#define SDHCI_BUFFER            0x20

#define SDHCI_PRESENT_STATE     0x24
#define SDHCI_CMD_INHIBIT       0x00000001
#define SDHCI_DATA_INHIBIT      0x00000002
#define SDHCI_DATA0_LINE_LEVEL_FLAG (1U << 20)

#define SDHCI_HOST_CONTROL      0x28
#define SDHCI_CLOCK_CONTROL     0x2C
#define SDHCI_CLOCK_CARD_EN     0x0004
#define SDHCI_CLOCK_INT_STABLE  0x0002
#define SDHCI_CLOCK_INT_EN      0x0001

#define SDHCI_TIMEOUT_CONTROL   0x2E
#define SDHCI_SOFTWARE_RESET    0x2F
#define SDHCI_RESET_ALL         0x01
#define SDHCI_RESET_CMD         0x02
#define SDHCI_RESET_DATA        0x04

#define SDHCI_INT_STATUS        0x30
#define SDHCI_INT_ENABLE        0x34
#define SDHCI_SIGNAL_ENABLE     0x38
#define SDHCI_INT_RESPONSE      0x00000001
#define SDHCI_INT_DATA_END      0x00000002
#define SDHCI_INT_DMA_END       0x00000008
#define SDHCI_INT_DATA_AVAIL    0x00000020
#define SDHCI_INT_SPACE_AVAIL   0x00000010
#define SDHCI_INT_ERROR         0x00008000
#define SDHCI_INT_TIMEOUT       0x00010000
#define SDHCI_INT_DATA_TIMEOUT  0x00100000
#define SDHCI_INT_DATA_CRC      0x00200000
#define SDHCI_INT_DATA_END_BIT  0x00400000
#define SDHCI_INT_CMD_MASK      (SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT)
#define SDHCI_INT_DATA_MASK     (SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
                                 SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
                                 SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
                                 SDHCI_INT_DATA_END_BIT)
#define SDHCI_INT_ALL_MASK      0xFFFFFFFF

#define SDHCI_HOST_CONTROL2     0x3E
#define SDHCI_CAPABILITIES      0x40
#define SDHCI_CAPABILITIES_1    0x44
#define SDHCI_MAX_CURRENT       0x48
#define SDHCI_ADMA_ADDRESS      0x58
#define SDHCI_ADMA_ADDRESS_HI   0x5C

/*
 * ============================================================================
 * Configuration Constants
 * ============================================================================
 */
#define CONFIG_SDHC_BUFFER_ALIGNMENT  32
#define CONFIG_SD_DATA_RETRIES        3
#define CONFIG_SD_CMD_TIMEOUT         200
#define CONFIG_SD_DATA_TIMEOUT        10000
#define CONFIG_SD_CMD_RETRIES         0
#define CONFIG_SD_RETRY_COUNT         10
#define CONFIG_MMC_RCA                0x2
#define MMC_RCA_ARG                   (CONFIG_MMC_RCA << 16U)
#define MMC_REL_ADR_ARG              (CONFIG_MMC_RCA << 16U)
#define SDMMC_R1_TRANSFER            4
#define LINKEDSEMI_SDHCI_DEFAULT_TIMEOUT 5000U
#define SDHCI_DEFAULT_BOUNDARY_ARG    7
#define MMC_EXT_CSD_BYTES            512
#define SDHCI_NATIVE_RESPONSE_MASK   0xF

/*
 * ============================================================================
 * SD/MMC Command Opcodes - use Zephyr's sd_spec.h for standard opcodes
 * Additional opcodes defined here for eMMC specific commands
 * ============================================================================
 */
#define SDIO_SEND_OP_COND      5
#define SD_SWITCH              6
#define MMC_CHECK_BUS_TEST     14
#define MMC_SEND_BUS_TEST      19
#define MMC_SEND_TUNING_BLOCK  21
#define SD_ERASE_BLOCK_START   32
#define SD_ERASE_BLOCK_END     33
#define SD_ERASE_BLOCK_OPERATION 38
#define SD_APP_CMD             55

/*
 * ============================================================================
 * Card Response Types
 * ============================================================================
 */
#define CARD_RESPONSE_TYPE_NONE  0U
#define CARD_RESPONSE_TYPE_R1    1U
#define CARD_RESPONSE_TYPE_R1B   2U
#define CARD_RESPONSE_TYPE_R2   3U
#define CARD_RESPONSE_TYPE_R3    4U
#define CARD_RESPONSE_TYPE_R5    6U
#define CARD_RESPONSE_TYPE_R5B   7U
#define CARD_RESPONSE_TYPE_R6    8U
#define CARD_RESPONSE_TYPE_R7    9U

/*
 * ============================================================================
 * Command Types
 * ============================================================================
 */
#define CARD_COMMAND_TYPE_NORMAL  0U
#define CARD_COMMAND_TYPE_SUSPEND 1U
#define CARD_COMMAND_TYPE_RESUME   2U
#define CARD_COMMAND_TYPE_ABORT    3U

/*
 * ============================================================================
 * Response Type Masks
 * ============================================================================
 */
#define SD_RSP_TYPE_NONE     0U
#define SD_RSP_TYPE_R1       1U
#define SD_RSP_TYPE_R1b      2U
#define SD_RSP_TYPE_R2       3U
#define SD_RSP_TYPE_R3       4U
#define SD_SPI_RSP_TYPE_R1   (1U << 4)
#define SD_SPI_RSP_TYPE_R2   (3U << 4)

/*
 * ============================================================================
 * SD R1 Status Flags
 * ============================================================================
 */
#define SD_R1_ERR_FLAGS      0xFFF9C000
#define SD_R1_RDY_DATA       BIT(8)
#define SD_R1_CUR_STATE      (0xFU << 9)
#define SD_OCR_PWR_BUSY_FLAG BIT(31)

/*
 * ============================================================================
 * Bit helpers
 * ============================================================================
 */
#define BIT(n)  (1UL << (n))

/*
 * ============================================================================
 * Internal SDHCI Command/Data Structures (for bare-metal operations)
 * ============================================================================
 */
struct sdhci_command {
    uint32_t index;
    uint32_t argument;
    uint8_t type;
    uint8_t response_type;
    uint32_t response[4U];
    uint32_t response_error_flags;
    uint16_t flags;
    uint16_t flags2;
    uint32_t timeout_ms;
};

struct sdhci_data {
    bool enable_auto_command12;
    bool enable_auto_command23;
    bool enableIgnoreError;
    uint32_t block_size;
    uint32_t block_count;
    uint32_t *rx_data;
    const uint32_t *tx_data;
    uint32_t timeout_ms;
};

/*
 * ============================================================================
 * eMMC operation states (for coredump)
 * ============================================================================
 */
typedef enum {
    EMMC_OP_STATE_IDLE = 0,
    EMMC_OP_STATE_READING,
    EMMC_OP_STATE_WRITING,
    EMMC_OP_STATE_ABORTING,
} emmc_op_state_t;

/*
 * ============================================================================
 * HAL eMMC Functions (for internal use by emmc_baremetal.c)
 * ============================================================================
 */
uint32_t ls_sdhci_card_busy(uint32_t mapbase);
uint32_t ls_mmc_card_init(uint32_t mapbase);
uint32_t ls_mmc_read_blocks(uint32_t mapbase, uint8_t *rbuf, uint32_t start_block, uint32_t num_blocks);
uint32_t ls_mmc_write_blocks(uint32_t mapbase, const uint8_t *wbuf, uint32_t start_block, uint32_t num_blocks);

/*
 * ============================================================================
 * Bare-metal Coredump Functions
 * ============================================================================
 */

/**
 * @brief Initialize eMMC protection
 *
 * Called at system initialization.
 */
void emmc_protection_init(void);

/**
 * @brief Bare-metal abort operation for fatal error context
 *
 * This function completely bypasses the normal SDHC driver and OS synchronization.
 * It does direct hardware register access with polling only.
 * NO OS primitives are used - no mutex, no semaphore, no sleeping.
 *
 * Checks COMMAND_INHIBIT and DATA_INHIBIT flags:
 * - If busy: returns -EBUSY, caller should skip write operations
 * - If idle: returns 0, card is ready for write
 *
 * @return 0 if card is idle and ready for write
 * @return -EBUSY if card is busy (command or data inhibit set)
 */
int emmc_abort_operation_baremetal(void);

/**
 * @brief Reinitialize eMMC after abort
 *
 * @return 0 if successful, negative errno on error
 */
int emmc_reinitialize(void);

/**
 * @brief Bare-metal eMMC write for fatal error context
 *
 * @param offset Byte offset from coredump region start
 * @param buf Data buffer to write
 * @param len Number of bytes to write
 * @return 0 on success, negative errno on failure
 */
int emmc_write_baremetal(size_t offset, const uint8_t *buf, size_t len);

/**
 * @brief Bare-metal eMMC read for fatal error context
 *
 * @param offset Byte offset from coredump region start
 * @param buf Data buffer to read into
 * @param len Number of bytes to read
 * @return 0 on success, negative errno on failure
 */
int emmc_read_baremetal(size_t offset, uint8_t *buf, size_t len);

#if defined(CONFIG_DEBUG_COREDUMP_BACKEND_EMMC)
/**
 * @brief Get eMMC coredump region information
 *
 * @param dev SDHC device (not used, for API compatibility)
 * @param start_block Output: pointer to store coredump start block
 * @param block_count Output: pointer to store reserved block count
 * @return 0 if successful, -ENODEV if coredump region not configured
 */
int linkedsemi_sdhci_get_coredump_info(const struct device *dev, uint32_t *start_block, uint32_t *block_count);

/**
 * @brief Check if eMMC card is ready for bare-metal operations
 *
 * @param dev SDHC device
 * @return true if card is ready, false otherwise
 */
bool linkedsemi_sdhci_card_ready(const struct device *dev);
#endif

#endif /* SDHCI_OF_LINKEDSEMI_EMMC_PROTECTION_H_ */