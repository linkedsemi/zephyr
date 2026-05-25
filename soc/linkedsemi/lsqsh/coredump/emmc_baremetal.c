/*
 * Copyright (c) 2024 LinkedSemi Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief eMMC bare-metal operations for coredump
 *
 * These functions provide bare-metal eMMC operations that can be
 * called from fatal error context without using OS primitives.
 *
 * This file contains a minimal SDHCI driver that can be synchronized
 * with zephyr/drivers/sdhc/sdhci-of-linkedsemi.c
 *
 * Key design decisions for bare-metal context:
 * - NO OS primitives (no mutex, semaphore, sleeping)
 * - Short timeouts only (max 100ms per operation)
 * - No retry loops that can cause minute-level hangs
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/bitarray.h>
// #include <zephyr/sys_cache.h>
#include <zephyr/cache.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include "emmc_coredump_sdhci.h"

/* SDHCI register definitions - see emmc_coredump_sdhci.h
 * SDHCI_DMA_ADDRESS, SDHCI_BLOCK_SIZE, SDHCI_BLOCK_COUNT,
 * SDHCI_ARGUMENT, SDHCI_TRANSFER_MODE, SDHCI_COMMAND,
 * SDHCI_RESPONSE, SDHCI_BUFFER, etc. are defined there
 */

/* Additional register definitions for bare-metal operations
 * SDHCI_HOST_CONTROL2, SDHCI_CAPABILITIES, etc. are in emmc_coredump_sdhci.h */
#define SDHCI_CTRL_UHS_MASK     0x0007
#define SDHCI_CTRL_EMMC_LEGACY  0x0000
#define SDHCI_CTRL_EMMC_HIGH_SPEED_SDR 0x0001

#define SDHCI_ADMA_ADDRESS      0x58
#define SDHCI_ADMA_ADDRESS_HI   0x5C

/* Vendor-specific registers */
#define DWC_MSHC_PTR_VENDOR1    0x500
#define MSHC_CTRL_R             (DWC_MSHC_PTR_VENDOR1 + 0x08)

/* SD/MMC command opcodes */
#define SD_GO_IDLE_STATE        0
#define MMC_SEND_OP_COND        1
#define SD_ALL_SEND_CID         2
#define SD_SEND_RELATIVE_ADDR   3
#define MMC_SEND_RELATIVE_ADDR  3
#define SD_SELECT_CARD          7
#define SD_SEND_CSD             9
#define SD_SEND_CID             10
#define SD_STOP_TRANSMISSION    12
#define SD_SEND_STATUS          13
#define MMC_CHECK_BUS_TEST     14
#define SD_SET_BLOCK_SIZE       16
#define SD_READ_SINGLE_BLOCK    17
#define SD_READ_MULTIPLE_BLOCK  18
#define MMC_SEND_BUS_TEST       19
#define MMC_SEND_EXT_CSD        8
#define SD_SET_BLOCK_COUNT       23
#define SD_WRITE_SINGLE_BLOCK   24
#define SD_WRITE_MULTIPLE_BLOCK 25
#define SD_APP_CMD              55

/* Configuration constants - matching ls_hal_emmc.h for consistency */
#define CONFIG_SDHC_BUFFER_ALIGNMENT  32
#define BM_CONFIG_SD_CMD_TIMEOUT        2000
#define CONFIG_SD_DATA_TIMEOUT       10000
#define CONFIG_SD_DATA_RETRIES      3
#define CONFIG_SD_CMD_RETRIES        0
#define CONFIG_SD_RETRY_COUNT        10
#define CONFIG_MMC_RCA               0x2
#define MMC_RCA_ARG                  (CONFIG_MMC_RCA << 16U)
#define MMC_REL_ADR_ARG              (CONFIG_MMC_RCA << 16U)
#define SDMMC_R1_TRANSFER            4
#define SDHCI_DEFAULT_BOUNDARY_ARG   7
#define MMC_EXT_CSD_BYTES            512
#define SDHCI_NATIVE_RESPONSE_MASK    0xF

/* Card response types */
#define CARD_RESPONSE_TYPE_NONE  0U
#define CARD_RESPONSE_TYPE_R1    1U
#define CARD_RESPONSE_TYPE_R1B   2U
#define CARD_RESPONSE_TYPE_R2    3U
#define CARD_RESPONSE_TYPE_R3    4U
#define CARD_RESPONSE_TYPE_R6    8U
#define CARD_RESPONSE_TYPE_R7    9U

/* Command types */
#define CARD_COMMAND_TYPE_NORMAL 0U
#define CARD_COMMAND_TYPE_ABORT  3U

/* Response type masks */
#define SD_RSP_TYPE_NONE     0U
#define SD_RSP_TYPE_R1       1U
#define SD_RSP_TYPE_R1b      2U
#define SD_RSP_TYPE_R2       3U
#define SD_RSP_TYPE_R3       4U
#define SD_SPI_RSP_TYPE_R1   (1U << 4)
#define SD_SPI_RSP_TYPE_R2   (3U << 4)

/* SDHCI command flags */
#define SDHCI_CMD_RESP_SHORT     0x2
#define SDHCI_CMD_RESP_LONG      0x1
#define SDHCI_CMD_RESP_SHORT_BUSY 0x3
#define SDHCI_ENABLE_CMD_CRC_FLAG     SDHCI_CMD_CRC
#define SDHCI_ENABLE_CMD_INDEX_CHK_FLAG SDHCI_CMD_INDEX
#define SDHCI_ENABLE_CMD_DATA_PRESENT_FLAG SDHCI_CMD_DATA
#define SDHCI_ENABLE_COMMAND_TYPE_NORMAL   0x00
#define SDHCI_ENABLE_COMMAND_TYPE_ABORT    0xC0
#define SDHCI_ENABLE_DMA_FLAG    SDHCI_TRNS_DMA
#define SDHCI_ENABLE_BLOCK_COUNT_FLAG SDHCI_TRNS_BLK_CNT_EN
#define SDHCI_ENABLE_AUTO_COMMAND12_FLAG SDHCI_TRNS_AUTO_CMD12
#define SDHCI_DATA_READ_FLAG     SDHCI_TRNS_READ
#define SDHCI_MULTIPLE_BLOCK_FLAG SDHCI_TRNS_MULTI
#define SDHCI_ENABLE_AUTO_COMMAND23_FLAG SDHCI_TRNS_AUTO_CMD23

/* HAL struct for sdhci_request compatibility */
struct sdhc_command {
    uint32_t opcode;
    uint32_t arg;
    uint32_t response[4];
    uint32_t response_type;
    unsigned int retries;
    int timeout_ms;
};

struct sdhc_data {
    unsigned int block_addr;
    unsigned int block_size;
    unsigned int blocks;
    unsigned int bytes_xfered;
    void *data;
    int timeout_ms;
};

/*
 * Bare-metal delay functions - using architecture-independent delay
 * These are simple polling loops that work in fatal error context
 */
static inline void baremetal_delay_us(uint32_t us)
{
    volatile uint32_t count = us * 100;  /* Approximate cycles per us */
    while (count--) {
        __asm__ volatile ("nop");
    }
}

static inline void baremetal_delay_ms(uint32_t ms)
{
    baremetal_delay_us(ms * 1000);
}

/*
 * SDHCI register accessors - using sys_io ops for bare-metal compatibility
 */
static inline uint32_t sdhci_readl(uint32_t mapbase, uint32_t reg)
{
    return sys_read32(mapbase + reg);
}

static inline void sdhci_writel(uint32_t mapbase, uint32_t val, uint32_t reg)
{
    sys_write32(val, mapbase + reg);
}

static inline uint16_t sdhci_readw(uint32_t mapbase, int reg)
{
    return sys_read16(mapbase + reg);
}

static inline void sdhci_writew(uint32_t mapbase, uint16_t val, uint32_t reg)
{
    sys_write16(val, mapbase + reg);
}

static inline uint8_t sdhci_readb(uint32_t mapbase, uint32_t reg)
{
    return sys_read8(mapbase + reg);
}

static inline void sdhci_writeb(uint32_t mapbase, uint8_t val, uint32_t reg)
{
    sys_write8(val, mapbase + reg);
}

/* Forward declarations of internal functions */
static uint32_t sdhci_get_present_status_flag(uint32_t mapbase);
static uint32_t sdhci_get_int_status_flag(uint32_t mapbase);
static void sdhci_clear_int_status_flag(uint32_t mapbase, uint32_t mask);
static uint32_t ls_sdhci_receive_command_response(uint32_t mapbase, struct sdhci_command *command);
static uint32_t linkedsemi_sdhci_wait_command_done(uint32_t mapbase, struct sdhci_command *command);
static void ls_sdhci_reset(uint32_t mapbase, uint8_t mask);
static uint32_t ls_sdhci_set_transfer_config(struct sdhci_command *sdhci_command, struct sdhci_data *sdhci_data);
static void ls_sdhci_send_command(uint32_t mapbase, struct sdhci_command *command, struct sdhci_data *data);
static int32_t linkedsemi_sdhci_transfer_data_blocking(uint32_t mapbase, struct sdhci_data *data);
static uint32_t linkedsemi_sdhci_transfer_blocking(uint32_t mapbase, struct sdhci_command *command, struct sdhci_data *data);

/* ============================================================================
 * Core SDHCI functions - can be synchronized with sdhci-of-linkedsemi.c
 * ============================================================================
 */

/**
 * @brief Get present status flags
 */
static uint32_t sdhci_get_present_status_flag(uint32_t mapbase)
{
    return sdhci_readl(mapbase, SDHCI_PRESENT_STATE);
}

/**
 * @brief Get interrupt status flags
 */
static uint32_t sdhci_get_int_status_flag(uint32_t mapbase)
{
    return sdhci_readl(mapbase, SDHCI_INT_STATUS);
}

/**
 * @brief Clear interrupt status flags
 */
static void sdhci_clear_int_status_flag(uint32_t mapbase, uint32_t mask)
{
    sdhci_writel(mapbase, mask, SDHCI_INT_STATUS);
}

/**
 * @brief Receive command response from card
 */
static uint32_t ls_sdhci_receive_command_response(uint32_t mapbase, struct sdhci_command *command)
{
    if (command->response_type == CARD_RESPONSE_TYPE_R2) {
        /* CRC is stripped so we need to do some shifting. */
        for (uint8_t i = 0; i < 4; i++) {
            command->response[3 - i] = sdhci_readl(mapbase, SDHCI_RESPONSE + (3 - i) * 4) << 8;
            if (i != 3) {
                command->response[3 - i] |= sdhci_readb(mapbase, SDHCI_RESPONSE + (3 - i) * 4 - 1);
            }
        }
    } else {
        command->response[0] = sdhci_readl(mapbase, SDHCI_RESPONSE);
    }
    /* check response error flag */
    if ((command->response_error_flags != 0U)
        && ((command->response_type == CARD_RESPONSE_TYPE_R1)
        || (command->response_type == CARD_RESPONSE_TYPE_R1B)
        || (command->response_type == CARD_RESPONSE_TYPE_R6))) {
        if (((command->response_error_flags) & (command->response[0U])) != 0U) {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Wait for command completion (bare-metal, short timeout)
 */
static uint32_t linkedsemi_sdhci_wait_command_done(uint32_t mapbase, struct sdhci_command *command)
{
    while (!(sdhci_get_int_status_flag(mapbase) & SDHCI_INT_RESPONSE)) {
        baremetal_delay_us(1);
    }
    sdhci_clear_int_status_flag(mapbase, SDHCI_INT_RESPONSE);
    return ls_sdhci_receive_command_response(mapbase, command);
}

/**
 * @brief Reset SDHCI controller or specific line
 */
void ls_sdhci_reset(uint32_t mapbase, uint8_t mask)
{
    sdhci_writeb(mapbase, mask, SDHCI_SOFTWARE_RESET);
    while (sdhci_readb(mapbase, SDHCI_SOFTWARE_RESET) & mask) {
        baremetal_delay_ms(1);
    }
}

/**
 * @brief Set transfer configuration based on command and data
 */
static uint32_t ls_sdhci_set_transfer_config(struct sdhci_command *sdhci_command, struct sdhci_data *sdhci_data)
{
    /* Define the flag corresponding to each response type. */
    switch (sdhci_command->response_type) {
    case CARD_RESPONSE_TYPE_NONE:
        break;
    case CARD_RESPONSE_TYPE_R1:
    case CARD_RESPONSE_TYPE_R6:
    case CARD_RESPONSE_TYPE_R7:
        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT | SDHCI_ENABLE_CMD_CRC_FLAG | SDHCI_ENABLE_CMD_INDEX_CHK_FLAG);
        break;
    case CARD_RESPONSE_TYPE_R1B:
        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_ENABLE_CMD_CRC_FLAG | SDHCI_ENABLE_CMD_INDEX_CHK_FLAG);
        break;
    case CARD_RESPONSE_TYPE_R2:
        sdhci_command->flags |= (SDHCI_CMD_RESP_LONG | SDHCI_ENABLE_CMD_CRC_FLAG);
        break;
    case CARD_RESPONSE_TYPE_R3:
        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT);
        break;
    default:
        break;
    }

    if (sdhci_command->type == CARD_COMMAND_TYPE_ABORT) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_ABORT;
    } else if (sdhci_command->type == CARD_COMMAND_TYPE_NORMAL) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_NORMAL;
    }

    if (sdhci_data) {
        sdhci_command->flags |= SDHCI_ENABLE_CMD_DATA_PRESENT_FLAG;
        sdhci_command->flags2 |= SDHCI_ENABLE_BLOCK_COUNT_FLAG;

        if (sdhci_data->rx_data) {
            sdhci_command->flags2 |= SDHCI_DATA_READ_FLAG;
        }
        if (sdhci_data->block_count > 1U) {
            sdhci_command->flags2 |= (SDHCI_MULTIPLE_BLOCK_FLAG);
            if (sdhci_data->enable_auto_command12) {
                sdhci_command->flags2 |= SDHCI_ENABLE_AUTO_COMMAND12_FLAG;
            }
            if (sdhci_data->enable_auto_command23) {
                sdhci_command->flags2 |= SDHCI_ENABLE_AUTO_COMMAND23_FLAG;
            }
        }
    }
    return 0;
}

/**
 * @brief Send command to card
 */
static void ls_sdhci_send_command(uint32_t mapbase, struct sdhci_command *command, struct sdhci_data *data)
{
    uint32_t cmd_r, xfermode;
    cmd_r = SDHCI_MAKE_CMD(command->index, command->flags);
    if (data != NULL) {
        uint32_t start_addr;
        if (data->rx_data) {
            start_addr = (uint32_t)((uint8_t *)data->rx_data);
        } else {
            start_addr = (uint32_t)((uint8_t *)data->tx_data);
            /* Flush cache for write - push data to memory */
            sys_cache_data_flush_range((void *)start_addr, data->block_size * data->block_count);
        }
        command->flags2 |= SDHCI_ENABLE_DMA_FLAG;
        sdhci_writel(mapbase, start_addr, SDHCI_DMA_ADDRESS);

        sdhci_writew(mapbase, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG, data->block_size), SDHCI_BLOCK_SIZE);
        sdhci_writew(mapbase, data->block_count, SDHCI_BLOCK_COUNT);
    }
    xfermode = command->flags2 & 0x1ff;

    sdhci_writew(mapbase, xfermode, SDHCI_TRANSFER_MODE);
    sdhci_writel(mapbase, command->argument, SDHCI_ARGUMENT);
    sdhci_writew(mapbase, cmd_r, SDHCI_COMMAND);
}

/**
 * @brief Transfer data with blocking/polling (bare-metal)
 */
static int32_t linkedsemi_sdhci_transfer_data_blocking(uint32_t mapbase, struct sdhci_data *data)
{
    uint32_t loop_count = 0;
    while (1) {
        uint32_t status = sdhci_get_int_status_flag(mapbase);

        if (status & SDHCI_INT_ERROR) {
            sdhci_clear_int_status_flag(mapbase, SDHCI_INT_ERROR);
            return -1;
        }
        if (status & SDHCI_INT_DMA_END) {
            sdhci_clear_int_status_flag(mapbase, SDHCI_INT_DMA_END);
            sdhci_writel(mapbase, SDHCI_INT_DMA_END, SDHCI_INT_STATUS);
            sdhci_writel(mapbase, sdhci_readl(mapbase, SDHCI_DMA_ADDRESS), SDHCI_DMA_ADDRESS);
        }
        if (status & SDHCI_INT_DATA_END) {
            sdhci_clear_int_status_flag(mapbase, SDHCI_INT_DATA_END);
            /* Invalidate cache for read - ensure CPU reads actual DMA data */
            if (data->rx_data) {
                sys_cache_data_invd_range(data->rx_data, data->block_size * data->block_count);
            }
            return 0;
        }

        loop_count++;
        if (loop_count > 100000) {
            return -1;
        }
        baremetal_delay_us(1);
    }
}

/**
 * @brief Complete SDHCI transfer blocking (bare-metal)
 */
static uint32_t linkedsemi_sdhci_transfer_blocking(uint32_t mapbase, struct sdhci_command *command, struct sdhci_data *data)
{
    uint8_t ret = 0;
    /* Wait until command/data bus out of busy status. */
    while (sdhci_get_present_status_flag(mapbase) & SDHCI_CMD_INHIBIT) {
        baremetal_delay_us(1);
    }
    while (data && (sdhci_get_present_status_flag(mapbase) & SDHCI_DATA_INHIBIT)) {
        baremetal_delay_us(1);
    }

    sdhci_writel(mapbase, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);

    ret = ls_sdhci_set_transfer_config(command, data);
    if (ret != 0) {
        return ret;
    }
    sdhci_writel(mapbase, sdhci_readl(mapbase, SDHCI_SIGNAL_ENABLE) | SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK, SDHCI_SIGNAL_ENABLE);

    ls_sdhci_send_command(mapbase, command, data);
    /* wait command done */
    ret = linkedsemi_sdhci_wait_command_done(mapbase, command);
    /* transfer data */
    if ((data != NULL) && (ret == 0) && (!(sdhci_get_int_status_flag(mapbase) & SDHCI_INT_ERROR))) {
        ret = linkedsemi_sdhci_transfer_data_blocking(mapbase, data);
    }
    while ((sdhci_get_present_status_flag(mapbase) & SDHCI_CMD_INHIBIT) && (!(sdhci_get_int_status_flag(mapbase) & SDHCI_INT_ERROR))) {
        baremetal_delay_us(1);
    }
    while ((data && (sdhci_get_present_status_flag(mapbase) & SDHCI_DATA_INHIBIT) && (!(sdhci_get_int_status_flag(mapbase) & SDHCI_INT_ERROR)))) {
        baremetal_delay_us(1);
    }
    sdhci_writel(mapbase, sdhci_readl(mapbase, SDHCI_SIGNAL_ENABLE) & ~(SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK), SDHCI_SIGNAL_ENABLE);
    sdhci_writel(mapbase, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
    ls_sdhci_reset(mapbase, SDHCI_RESET_CMD);
    ls_sdhci_reset(mapbase, SDHCI_RESET_DATA);
    return ret;
}

/* ============================================================================
 * Public SDHCI functions - compatible with ls_hal_emmc.h interface
 * ============================================================================
 */

/**
 * @brief Check if card is busy (data0 line low)
 */
uint32_t ls_sdhci_card_busy(uint32_t mapbase)
{
    return (!(sdhci_get_present_status_flag(mapbase) & SDHCI_DATA0_LINE_LEVEL_FLAG));
}

/**
 * @brief Send SDHCI request (bare-metal compatible)
 *
 * This is the core function for all SD/MMC commands.
 * Uses short timeouts suitable for fatal error context.
 */
uint32_t ls_sdhci_request(uint32_t mapbase, struct sdhc_command *cmd, struct sdhc_data *data)
{
    uint32_t ret;
    struct sdhci_data sdhci_data = { 0 };
    struct sdhci_command sdhci_command = { 0 };
    struct sdhci_data *sdhci_data_ptr = &sdhci_data;

    sdhci_command.index = cmd->opcode;
    sdhci_command.argument = cmd->arg;

    /* Mask out part of response type field used for SPI commands */
    sdhci_command.response_type = (cmd->response_type & SDHCI_NATIVE_RESPONSE_MASK);
    if (cmd->opcode == SD_STOP_TRANSMISSION) {
        sdhci_command.type = CARD_COMMAND_TYPE_ABORT;
    } else {
        sdhci_command.type = CARD_COMMAND_TYPE_NORMAL;
    }

    if (data) {
        sdhci_data.block_size = data->block_size;
        sdhci_data.block_count = data->blocks;

        switch (cmd->opcode) {
        case SD_WRITE_SINGLE_BLOCK:
        case SD_WRITE_MULTIPLE_BLOCK:
            sdhci_data.enable_auto_command12 = true;
            sdhci_data.tx_data = data->data;
            break;
        case MMC_SEND_BUS_TEST:
            sdhci_data.tx_data = data->data;
            break;
        case SD_READ_SINGLE_BLOCK:
        case SD_READ_MULTIPLE_BLOCK:
            sdhci_data.enable_auto_command12 = true;
            sdhci_data.rx_data = data->data;
            break;
        case MMC_SEND_EXT_CSD:
            sdhci_data.rx_data = data->data;
            break;
        default:
            return -ENOTSUP;
        }
        sdhci_data.timeout_ms = data->timeout_ms;
        sdhci_data_ptr = &sdhci_data;
    } else {
        sdhci_data_ptr = NULL;
    }
    sdhci_command.timeout_ms = cmd->timeout_ms;

    do {
        ret = linkedsemi_sdhci_transfer_blocking(mapbase, &sdhci_command, sdhci_data_ptr);

        if (data && ret) {
            /* Send CMD12 to stop transmission after error - wait indefinitely */
            while (ls_sdhci_card_busy(mapbase)) {
                baremetal_delay_us(125);
            }
        } else {
            cmd->response[0] = sdhci_command.response[0];
            cmd->response[1] = sdhci_command.response[1];
            cmd->response[2] = sdhci_command.response[2];
            cmd->response[3] = sdhci_command.response[3];
        }
} while (ret != 0 && (cmd->retries-- > 0));

    return ret;
}

/* ============================================================================
 * eMMC card initialization functions
 * ============================================================================
 */

static uint32_t mmc_send_op_cond(uint32_t mapbase, uint32_t ocr)
{
    struct sdhc_command cmd = {0};
    uint32_t ret = 0;
    uint32_t retries;

    cmd.opcode = MMC_SEND_OP_COND;
    cmd.arg = ocr;
    cmd.response_type = SD_RSP_TYPE_R3;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;

    for (retries = 0;
         retries < 1000 && !(cmd.response[0] & SD_OCR_PWR_BUSY_FLAG);
         retries++) {
        ret = ls_sdhci_request(mapbase, &cmd, NULL);
        if (ret) {
            return ret;
        }
        if (ocr == 0) {
            return 0;
        }
        baremetal_delay_ms(10);
    }
    if (retries >= 1000) {
        return -ETIMEDOUT;
    }
    return 0;
}

uint32_t ls_sd_idle(uint32_t mapbase)
{
    struct sdhc_command cmd;

    cmd.opcode = SD_GO_IDLE_STATE;
    cmd.arg = 0x0;
    cmd.response_type = (SD_RSP_TYPE_NONE | SD_SPI_RSP_TYPE_R1);
    cmd.retries = CONFIG_SD_CMD_RETRIES;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;
    return ls_sdhci_request(mapbase, &cmd, NULL);
}

static uint32_t sdmmc_read_cxd(uint32_t mapbase, uint32_t opcode, uint32_t rca, uint32_t *cxd)
{
    struct sdhc_command cmd;
    uint32_t ret;

    cmd.opcode = opcode;
    cmd.arg = (rca << 16);
    cmd.response_type = SD_RSP_TYPE_R2;
    cmd.retries = CONFIG_SD_CMD_RETRIES;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;

    ret = ls_sdhci_request(mapbase, &cmd, NULL);
    if (ret) {
        return ret;
    }
    memcpy(cxd, cmd.response, 16);
    return 0;
}

static uint32_t ls_card_read_cid(uint32_t mapbase)
{
    uint32_t cid[4];
    return sdmmc_read_cxd(mapbase, SD_ALL_SEND_CID, 0, cid);
}

static inline uint32_t sd_check_response(struct sdhc_command *cmd)
{
    if (cmd->response_type == SD_RSP_TYPE_R1) {
        return (cmd->response[0] & SD_R1_ERR_FLAGS);
    }
    return 0;
}

static uint32_t mmc_set_rca(uint32_t mapbase)
{
    struct sdhc_command cmd = {0};
    uint32_t ret;

    cmd.opcode = MMC_SEND_RELATIVE_ADDR;
    cmd.arg = MMC_RCA_ARG;
    cmd.response_type = SD_RSP_TYPE_R1;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;

    ret = ls_sdhci_request(mapbase, &cmd, NULL);
    if (ret) {
        return ret;
    }
    ret = sd_check_response(&cmd);
    if (ret) {
        return ret;
    }
    return 0;
}

static uint32_t mmc_read_csd(uint32_t mapbase)
{
    uint32_t ret;
    struct sdhc_command cmd = {0};

    cmd.opcode = SD_SEND_CSD;
    cmd.arg = MMC_REL_ADR_ARG;
    cmd.response_type = SD_RSP_TYPE_R2;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;

    ret = ls_sdhci_request(mapbase, &cmd, NULL);
    if (ret) {
        return ret;
    }
    return 0;
}

uint32_t ls_sdmmc_select_card(uint32_t mapbase)
{
    struct sdhc_command cmd = {0};
    uint32_t ret;

    cmd.opcode = SD_SELECT_CARD;
    cmd.arg = (CONFIG_MMC_RCA << 16U);
    cmd.response_type = SD_RSP_TYPE_R1;
    cmd.retries = CONFIG_SD_CMD_RETRIES;
    cmd.timeout_ms = BM_CONFIG_SD_CMD_TIMEOUT;

    ret = ls_sdhci_request(mapbase, &cmd, NULL);
    if (ret) {
        return ret;
    }
    ret = sd_check_response(&cmd);
    if (ret) {
        return ret;
    }
    return 0;
}

/**
 * @brief Initialize eMMC card (bare-metal compatible)
 */
uint32_t ls_mmc_card_init(uint32_t mapbase)
{
    uint32_t ret = 0;
    uint32_t ocr_arg = 0U;

    /* Modern SDHC always at least supports 512 byte block sizes */
    ocr_arg |= BIT(30) | BIT(7);  /* SECTOR_MODE | VDD170_195FLAG */

    /* CMD1 */
    ret = mmc_send_op_cond(mapbase, ocr_arg);
    if (ret) {
        return ret;
    }

    /* CMD2 */
    ret = ls_card_read_cid(mapbase);
    if (ret) {
        return ret;
    }

    /* CMD3 */
    ret = mmc_set_rca(mapbase);
    if (ret) {
        return ret;
    }

    /* CMD9 */
    ret = mmc_read_csd(mapbase);
    if (ret) {
        return ret;
    }

    /* CMD7 */
    ret = ls_sdmmc_select_card(mapbase);
    if (ret) {
        return ret;
    }

    return 0;
}

/* ============================================================================
 * Public bare-metal eMMC functions for coredump
 * ============================================================================
 */

/**
 * @brief Initialize eMMC protection
 *
 * Called at system initialization.
 * Currently a placeholder - bare-metal functions don't require initialization.
 */
void emmc_protection_init(void)
{
    /* No initialization needed for bare-metal operations */
}

/**
 * @brief Bare-metal abort operation for fatal error context
 *
 * This function completely bypasses the normal SDHC driver and OS synchronization.
 * It does direct hardware register access with polling only.
 * NO OS primitives are used - no mutex, no semaphore, no sleeping.
 * Use this when normal OS services may not be available.
 *
 * Checks COMMAND_INHIBIT and DATA_INHIBIT flags:
 * - If busy: returns -EBUSY, caller should skip write operations
 * - If idle: returns 0, card is ready for write (actual write will succeed or fail naturally)
 *
 * @return 0 if card is idle and ready for write
 * @return -EBUSY if card is busy (command or data inhibit set)
 */
int emmc_abort_operation_baremetal(void)
{
    uint32_t sdhci_base = DT_REG_ADDR(DT_ALIAS(sdhc0));
    uint32_t present_state;

    /* Read present state register */
    present_state = sys_read32(sdhci_base + SDHCI_PRESENT_STATE);

    /* Check if card is busy - command or data inhibit set */
    if (present_state & (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
        /* Card is busy with ongoing operation, cannot write */
        return -EBUSY;
    }

    /* Clear any pending interrupt flags */
    sys_write32(SDHCI_INT_ALL_MASK, sdhci_base + SDHCI_INT_STATUS);

    return 0;
}

/**
 * @brief eMMC write for fatal error context
 *
 * Write data to eMMC directly via ls_sdhci_request().
 * Assumes SDHC and eMMC are already initialized and in working state.
 * If card is busy, skips the write operation.
 *
 * @param offset Absolute byte offset in eMMC (already includes start_block)
 * @param buf Data buffer to write
 * @param len Number of bytes to write
 * @return 0 on success, negative errno on failure (-EBUSY if card busy)
 */
int emmc_write_baremetal(size_t offset, const uint8_t *buf, size_t len)
{
    uint32_t sdhci_base = DT_REG_ADDR(DT_ALIAS(sdhc0));

    uint32_t block_addr = offset / 512;
    uint32_t block_count = (len + 511) / 512;

    struct sdhc_command cmd = {0};
    struct sdhc_data data = {0};
    uint32_t ret;

    /* Build write command */
    cmd.opcode = (block_count == 1) ? 24 : 25;  /* SD_WRITE_SINGLE_BLOCK=24, SD_WRITE_MULTIPLE_BLOCK=25 */
    cmd.arg = block_addr;
    cmd.response_type = CARD_RESPONSE_TYPE_R1;
    cmd.retries = 0;
    cmd.timeout_ms = 2000;

    /* Build data descriptor */
    data.block_addr = block_addr;
    data.block_size = 512;
    data.blocks = block_count;
    data.data = (void *)buf;
    data.timeout_ms = 2000;

    /* Call HAL request directly - bypasses ls_mmc_write_blocks() to avoid
     * ls_sdmmc_wait_ready() which can timeout after 10 seconds
     */
    ret = ls_sdhci_request(sdhci_base, &cmd, &data);

    /* Clear any pending SDHC interrupt flags to prevent spurious interrupts
     * when returning to Zephyr context
     */
    sys_write32(SDHCI_INT_ALL_MASK, sdhci_base + SDHCI_INT_STATUS);

    return (ret == 0) ? 0 : -EIO;
}

/**
 * @brief eMMC read for fatal error context - optimized version
 *
 * Read data from eMMC directly via ls_sdhci_request().
 * This bypasses ls_mmc_read_blocks() which includes ls_sdmmc_wait_ready()
 * that can timeout after 10 seconds when card state is uncertain.
 *
 * @param offset Absolute byte offset in eMMC (already includes start_block)
 * @param buf Data buffer to read into
 * @param len Number of bytes to read
 * @return 0 on success, negative errno on failure
 */
int emmc_read_baremetal(size_t offset, uint8_t *buf, size_t len)
{
    uint32_t sdhci_base = DT_REG_ADDR(DT_ALIAS(sdhc0));

    uint32_t block_addr = offset / 512;
    uint32_t block_count = (len + 511) / 512;

    struct sdhc_command cmd = {0};
    struct sdhc_data data = {0};
    uint32_t ret;

    /* Build read command - optimized for bare-metal */
    cmd.opcode = (block_count == 1) ? 17 : 18;  /* SD_READ_SINGLE_BLOCK=17, SD_READ_MULTIPLE_BLOCK=18 */
    cmd.arg = block_addr;
    cmd.response_type = CARD_RESPONSE_TYPE_R1;
    cmd.retries = 0;  /* No retry in fatal error context */
    cmd.timeout_ms = 2000;  /* 2000ms command timeout - increased for reliability */

    /* Build data descriptor */
    data.block_addr = block_addr;
    data.block_size = 512;
    data.blocks = block_count;
    data.data = buf;
    data.timeout_ms = 2000;  /* 2000ms data timeout - increased for reliability */

    /* Call HAL request directly - bypasses ls_mmc_read_blocks() to avoid
     * ls_sdmmc_wait_ready() which can timeout after 10 seconds
     */
    ret = ls_sdhci_request(sdhci_base, &cmd, &data);

    /* Clear any pending SDHC interrupt flags */
    sys_write32(SDHCI_INT_ALL_MASK, sdhci_base + SDHCI_INT_STATUS);

    return (ret == 0) ? 0 : -EIO;
}

/**
 * @brief Reinitialize eMMC after abort
 *
 * Called after abort to restore eMMC to a clean state
 * for coredump storage.
 *
 * @return 0 if successful, negative errno on error
 */
int emmc_reinitialize(void)
{
    uint32_t sdhci_base = DT_REG_ADDR(DT_ALIAS(sdhc0));

    /* Re-init eMMC using HAL - this clears interrupt state and re-initializes card */
    return ls_mmc_card_init(sdhci_base);
}