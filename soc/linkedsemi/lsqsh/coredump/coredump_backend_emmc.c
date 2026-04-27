/*
 * Copyright (c) 2024 LinkedSemi Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief eMMC coredump backend for Zephyr
 *
 * This backend stores coredump data directly to eMMC storage,
 * bypassing the filesystem layer for more reliable operation
 * in fatal error contexts.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/debug/coredump.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <emmc_coredump_sdhci.h>

#include "coredump_internal.h"

LOG_MODULE_REGISTER(coredump_emmc, CONFIG_KERNEL_LOG_LEVEL);

/* HW_STACK_PROTECTION must be disabled for eMMC coredump backend */
#if defined(CONFIG_TEST_HW_STACK_PROTECTION) && CONFIG_TEST_HW_STACK_PROTECTION
#error "TEST_HW_STACK_PROTECTION must be disabled (CONFIG_TEST_HW_STACK_PROTECTION=n) for eMMC coredump backend"
#endif

/* eMMC coredump configuration */
#define COREDUMP_EMMC_DEVICE     DEVICE_DT_GET(DT_ALIAS(sdhc0))
#define COREDUMP_EMMC_TIMEOUT_MS 2000

/* Header related */
#define HDR_VER                  1
#define EMMC_BLOCK_SIZE          512

/* Checksum algorithm: simple 16-bit sum */
#define CHECKSUM_TYPE            uint32_t
#define CHECKSUM_INITIAL         0

/* Buffer size for write operations - avoids malloc in fatal context */
#define EMMC_BUF_SIZE            MAX(CONFIG_DEBUG_COREDUMP_EMMC_SECTION_SIZE, 512)

/* Header stored at beginning of coredump region */
struct emmc_hdr_t {
    /* 'C', 'D' */
    char     id[2];

    /* Header version */
    uint16_t hdr_version;

    /* Coredump size, excluding this header */
    size_t   size;

    /* Flags */
    uint16_t flags;

    /* Checksum */
    CHECKSUM_TYPE checksum;

    /* Error */
    int      error;

    /* Reserved for future use */
    uint32_t reserved[3];
} __packed;

/* Ensure header is aligned to emmc block size */
BUILD_ASSERT(sizeof(struct emmc_hdr_t) <= EMMC_BLOCK_SIZE,
             "emmc_hdr_t must fit in one block");

/* Backend context - all static to avoid malloc in fatal context */
static struct {
    /* Device handle for SDHC */
    const struct device *sdhc_dev;

    /* Checksum of data so far */
    CHECKSUM_TYPE checksum;

    /* Error encountered */
    int error;

    /* Number of bytes written (excluding header) */
    size_t bytes_written;

    /* Flag if backend is initialized */
    bool initialized;

    /* Flag if coredump is in progress */
    bool dump_in_progress;

    /* Current write offset within coredump region */
    size_t current_offset;

    /* Reserved block count for coredump region (from driver) */
    uint32_t reserved_blocks;

    /* Start block number for coredump region (from driver) */
    uint32_t start_block;
} backend_ctx;

/* Static buffers for write operations - aligned for DMA */
static uint8_t header_buf[EMMC_BLOCK_SIZE] __aligned(32);
static uint8_t read_buf[EMMC_BUF_SIZE] __aligned(32);  /* For reading data */

/* Block-aligned write buffer for accumulating partial blocks */
static uint8_t block_buf[EMMC_BLOCK_SIZE] __aligned(32);
static size_t block_buf_used;  /* Number of bytes currently in block_buf */

/* Forward declaration */
static void flush_block_buf_if_needed(void);

/* Semaphore for exclusive eMMC access (when not in fatal context) */
K_SEM_DEFINE(emmc_sem, 1, 1);

#define EMMC_BACKEND_SEM_TIMEOUT (k_is_in_isr() ? K_NO_WAIT : K_FOREVER)

/* Callback type for processing stored dump data */
typedef int (*data_read_cb_t)(void *arg, uint8_t *buf, size_t len);

/**
 * @brief Compute checksum of buffer
 *
 * @param buf Data buffer
 * @param len Number of bytes in buffer
 * @return Checksum value
 */
static CHECKSUM_TYPE compute_checksum(const uint8_t *buf, size_t len)
{
    CHECKSUM_TYPE sum = CHECKSUM_INITIAL;
    size_t i;

    for (i = 0; i < len; i++) {
        sum += buf[i];
    }

    return sum;
}

/**
 * @brief Read from eMMC using Zephyr SD stack (for normal context)
 *
 * @param offset Byte offset from start of coredump region
 * @param buf Buffer to read into
 * @param len Number of bytes to read
 * @return 0 if successful, error otherwise
 */
static int emmc_read_zephyr(size_t offset, uint8_t *buf, size_t len)
{
    uint32_t block_addr = offset / EMMC_BLOCK_SIZE;
    uint32_t block_count = (len + EMMC_BLOCK_SIZE - 1) / EMMC_BLOCK_SIZE;
    struct sdhc_command cmd = {0};
    struct sdhc_data data = {0};
    int ret;

    /* Lazy initialization of SDHC device on first use */
    if (backend_ctx.sdhc_dev == NULL) {
        backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;
    }

    cmd.opcode = (block_count == 1) ? SD_READ_SINGLE_BLOCK : SD_READ_MULTIPLE_BLOCK;
    cmd.arg = block_addr;
    cmd.response_type = SD_RSP_TYPE_R1;
    cmd.timeout_ms = 2000;

    data.data = buf;
    data.block_size = EMMC_BLOCK_SIZE;
    data.blocks = block_count;
    data.timeout_ms = 2000;

    ret = sdhc_request(backend_ctx.sdhc_dev, &cmd, &data);

    return (ret == 0) ? 0 : -EIO;
}

/**
 * @brief Read from eMMC - wrapper for normal context
 *
 * @param offset Byte offset from start of coredump region
 * @param buf Buffer to read into
 * @param len Number of bytes to read
 * @return 0 if successful, error otherwise
 */
static int emmc_read(size_t offset, uint8_t *buf, size_t len)
{
    return emmc_read_zephyr(offset, buf, len);
}

/**
 * @brief Write to eMMC using bare-metal HAL (for exception context)
 *
 * @param offset Byte offset from start of coredump region
 * @param buf Buffer containing data to write
 * @param len Number of bytes to write
 * @return 0 if successful, error otherwise
 */
static int emmc_write(size_t offset, const uint8_t *buf, size_t len)
{
    uint32_t block_addr = offset / EMMC_BLOCK_SIZE;
    uint32_t block_count = (len + EMMC_BLOCK_SIZE - 1) / EMMC_BLOCK_SIZE;
    struct sdhc_command cmd = {0};
    struct sdhc_data data = {0};
    int ret;

    /* Lazy initialization of SDHC device on first use */
    if (backend_ctx.sdhc_dev == NULL) {
        backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;
    }

    cmd.opcode = (block_count == 1) ? SD_WRITE_SINGLE_BLOCK : SD_WRITE_MULTIPLE_BLOCK;
    cmd.arg = block_addr;
    cmd.response_type = SD_RSP_TYPE_R1;
    cmd.timeout_ms = 2000;

    data.data = (uint8_t *)buf;
    data.block_size = EMMC_BLOCK_SIZE;
    data.blocks = block_count;
    data.timeout_ms = 2000;

    ret = sdhc_request(backend_ctx.sdhc_dev, &cmd, &data);

    return (ret == 0) ? 0 : -EIO;
}

// /**
//  * @brief Erase eMMC coredump region
//  *
//  * @return 0 if successful, error otherwise
//  */
// static int emmc_erase_region(void)
// {
//     struct sdhc_command cmd = {0};
//     uint32_t start_block, block_count;
//     int ret;

//     /* Lazy initialization of SDHC device on first use */
//     if (backend_ctx.sdhc_dev == NULL) {
//         backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;
//     }

//     if (backend_ctx.sdhc_dev == NULL) {
//         return -ENODEV;
//     }

//     /* Get coredump region info from driver */
//     ret = linkedsemi_sdhci_get_coredump_info(backend_ctx.sdhc_dev, &start_block, &block_count);
//     if (ret != 0) {
//         LOG_ERR("Failed to get coredump region info: %d", ret);
//         return ret;
//     }

//     /* Erase start block */
//     cmd.opcode = SD_ERASE_BLOCK_START;
//     cmd.arg = start_block;
//     cmd.response_type = SD_RSP_TYPE_R1;
//     cmd.retries = 3;
//     cmd.timeout_ms = 2000;

//     ret = sdhc_request(backend_ctx.sdhc_dev, &cmd, NULL);
//     if (ret != 0) {
//         LOG_ERR("eMMC erase start failed: %d", ret);
//         return ret;
//     }

//     /* Erase end block */
//     cmd.opcode = SD_ERASE_BLOCK_END;
//     cmd.arg = start_block + block_count - 1;
//     cmd.response_type = SD_RSP_TYPE_R1;
//     cmd.retries = 3;
//     cmd.timeout_ms = 2000;

//     ret = sdhc_request(backend_ctx.sdhc_dev, &cmd, NULL);
//     if (ret != 0) {
//         LOG_ERR("eMMC erase end failed: %d", ret);
//         return ret;
//     }

//     /* Execute erase */
//     cmd.opcode = SD_ERASE_BLOCK_OPERATION;
//     cmd.arg = 0;
//     cmd.response_type = SD_RSP_TYPE_R1b;
//     cmd.retries = 3;
//     cmd.timeout_ms = 30000; /* Erase can take a while */

//     ret = sdhc_request(backend_ctx.sdhc_dev, &cmd, NULL);
//     if (ret != 0) {
//         LOG_ERR("eMMC erase operation failed: %d", ret);
//         return ret;
//     }

//     LOG_DBG("eMMC region erased: blocks %u to %u", start_block, start_block + block_count - 1);
//     return 0;
// }

/**
 * @brief Read the stored coredump header from eMMC.
 *
 * @param hdr Pointer to header structure to fill.
 * @return 0 if successful, negative errno on error.
 */
static int read_header(struct emmc_hdr_t *hdr)
{
    int ret;

    ret = emmc_read_zephyr(backend_ctx.start_block * EMMC_BLOCK_SIZE, header_buf, sizeof(struct emmc_hdr_t));
    if (ret != 0) {
        return ret;
    }

    memcpy(hdr, header_buf, sizeof(struct emmc_hdr_t));

    return 0;
}

/**
 * @brief Write the coredump header to eMMC.
 *
 * @param hdr Header to write.
 * @return 0 if successful, negative errno on error.
 */
/**
 * @brief Write the coredump header to eMMC (Zephyr SDHCI stack - for normal context)
 *
 * @param hdr Header to write.
 * @return 0 if successful, negative errno on error.
 */
static int write_header(const struct emmc_hdr_t *hdr)
{
    int ret;

    /* Copy header to aligned buffer */
    memset(header_buf, 0, sizeof(header_buf));
    memcpy(header_buf, hdr, sizeof(struct emmc_hdr_t));

    /* Write header block using Zephyr SDHCI stack */
    ret = emmc_write(backend_ctx.start_block * EMMC_BLOCK_SIZE, header_buf, EMMC_BLOCK_SIZE);

    return ret;
}

/**
 * @brief Write the coredump header to eMMC (baremetal - for fatal error context)
 *
 * @param hdr Header to write.
 * @return 0 if successful, negative errno on error.
 */
static int write_header_baremetal(const struct emmc_hdr_t *hdr)
{
    int ret;

    /* Copy header to aligned buffer */
    memset(header_buf, 0, sizeof(header_buf));
    memcpy(header_buf, hdr, sizeof(struct emmc_hdr_t));

    /* Write header block using baremetal SDHCI */
    ret = emmc_write_baremetal(backend_ctx.start_block * EMMC_BLOCK_SIZE, header_buf, EMMC_BLOCK_SIZE);

    return ret;
}

/**
 * @brief Process the stored coredump in eMMC.
 *
 * This reads the stored coredump data and processes it via
 * the callback function.
 *
 * @param cb callback to process the stored coredump data
 * @param cb_arg argument passed to callback
 * @return 1 if successful; 0 if stored coredump is not found
 *         or is not valid; error otherwise
 */
static int process_stored_dump(data_read_cb_t cb, void *cb_arg)
{
    int ret;
    struct emmc_hdr_t hdr;
    size_t remaining;
    size_t read_len;
    size_t data_offset;

    /* Read header - using bare-metal, no semaphore needed */
    ret = read_header(&hdr);
    if (ret != 0) {
        goto out;
    }

    /* Verify header signature */
    if ((hdr.id[0] != 'C') || (hdr.id[1] != 'D')) {
        ret = 0;
        goto out;
    }

    /* Error encountered while dumping, return the stored error */
    if (hdr.error != 0) {
        ret = hdr.error;
        goto out;
    }

    backend_ctx.checksum = 0;
    remaining = hdr.size;

    /* Read data and process via callback - using static buffer */
    /* Data starts at EMMC_BLOCK_SIZE (512), right after header block */
    data_offset = backend_ctx.start_block * EMMC_BLOCK_SIZE + EMMC_BLOCK_SIZE;
    while (remaining > 0) {
        /* Read at most one block at a time to avoid crossing block boundaries */
        read_len = MIN(remaining, MIN((size_t)EMMC_BUF_SIZE, (size_t)EMMC_BLOCK_SIZE));

        ret = emmc_read(data_offset, read_buf, read_len);
        if (ret != 0) {
            goto out;
        }

        /* Calculate block checksum - read_len might span partial blocks */
        CHECKSUM_TYPE block_checksum = compute_checksum(read_buf, read_len);

        /* Update checksum */
        backend_ctx.checksum += block_checksum;

    // printk("[SYNC] process_stored_dump: checksum so far=%u\n", (unsigned int)backend_ctx.checksum);

        data_offset += EMMC_BLOCK_SIZE;  /* Advance by block size */

        /* Process via callback */
        if (cb != NULL) {
            ret = cb(cb_arg, read_buf, read_len);
            if (ret != 0) {
                goto out;
            }
        }

        remaining -= read_len;
    }

    /* Signal end of data to callback */
    if (cb != NULL) {
        ret = cb(cb_arg, NULL, 0);
        if (ret != 0) {
            goto out;
        }
    }

    /* Verify checksum */
    ret = (backend_ctx.checksum == hdr.checksum) ? 1 : 0;

out:
    return ret;
}

/**
 * @brief Callback to calculate checksum from stored dump.
 *
 * @param arg callback argument (not being used)
 * @param buf data buffer
 * @param len number of bytes in buffer to process
 * @return 0
 */
static int cb_calc_checksum(void *arg, uint8_t *buf, size_t len)
{
    ARG_UNUSED(arg);

    /* Note: checksum is already accumulated in process_stored_dump
     * before this callback is invoked. This callback exists for
     * other purposes (like printing), not for checksum calculation.
     * Do NOT add to checksum here to avoid double-counting.
     */
    ARG_UNUSED(buf);
    ARG_UNUSED(len);

    return 0;
}

/**
 * @brief Get the stored coredump size.
 *
 * @return dump size if successful; 0 if stored coredump is not found
 *         or is not valid; negative errno otherwise
 */
static ssize_t get_stored_dump_size(void)
{
    struct emmc_hdr_t hdr;
    int ret;

    ret = read_header(&hdr);
    if (ret != 0) {
        return (ssize_t)ret;
    }

    /* Verify header signature */
    if ((hdr.id[0] != 'C') || (hdr.id[1] != 'D')) {
        return 0;
    }

    /* Error encountered while dumping */
    if (hdr.error != 0) {
        return 0;
    }

    return (ssize_t)hdr.size;
}

/**
 * @brief Get the stored coredump data.
 *
 * @param off offset to start reading from
 * @param dst destination buffer (can be NULL to just get size)
 * @param len number of bytes to read
 * @return number of bytes read if successful; 0 if stored coredump is not found
 *         or is not valid; negative errno otherwise
 */
static ssize_t get_stored_dump(off_t off, uint8_t *dst, size_t len)
{
    struct emmc_hdr_t hdr;
    ssize_t ret;

    ret = read_header(&hdr);
    if (ret != 0) {
        goto out;
    }

    /* Verify header signature */
    if ((hdr.id[0] != 'C') || (hdr.id[1] != 'D')) {
        ret = 0;
        goto out;
    }

    /* Error encountered while dumping, return the stored error */
    if (hdr.error != 0) {
        ret = (ssize_t)hdr.error;
        goto out;
    }

    /* Return size if no destination buffer */
    if (dst == NULL) {
        ret = (ssize_t)hdr.size;
        goto out;
    }

    /* Offset larger than dump size */
    if (off >= (off_t)hdr.size) {
        ret = 0;
        goto out;
    }

    /* Read data - data starts at EMMC_BLOCK_SIZE, not sizeof(hdr) */
    ret = emmc_read(backend_ctx.start_block * EMMC_BLOCK_SIZE + EMMC_BLOCK_SIZE + off, dst, len);
    if (ret != 0) {
        ret = -EIO;
    } else {
        ret = (ssize_t)len;  /* Return bytes read on success */
    }

out:
    return ret;
}

/**
 * @brief Erase (invalidate) the stored coredump.
 *
 * @return 0 if successful, negative errno on error.
 */
static int erase_stored_dump(void)
{
    struct emmc_hdr_t hdr;

    /* Invalidate header by clearing ID */
    memset(&hdr, 0, sizeof(hdr));
    return write_header(&hdr);
}

/**
 * @brief Start of coredump session.
 *
 * This prepares the eMMC for coredump storage.
 */
static void coredump_emmc_backend_start(void)
{
    int ret;

    /* Lazy initialization of SDHC device */
    if (backend_ctx.sdhc_dev == NULL) {
        backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;
    }

    /* Check if card is ready for bare-metal operations */
    if (!linkedsemi_sdhci_card_ready(backend_ctx.sdhc_dev)) {
        LOG_DBG("coredump_emmc_backend_start: card not ready, skipping");
        backend_ctx.error = -EBUSY;
        return;
    }

    /* Clear any pending SDHC interrupts */
    sys_write32(SDHCI_INT_ALL_MASK, DT_REG_ADDR(DT_ALIAS(sdhc0)) + 0x30);

    /* Get coredump region info from driver */
    uint32_t start_block, block_count;
    ret = linkedsemi_sdhci_get_coredump_info(backend_ctx.sdhc_dev, &start_block, &block_count);
    if (ret != 0) {
        LOG_ERR("Failed to get coredump region info: %d", ret);
        backend_ctx.error = ret;
        return;
    }
    backend_ctx.reserved_blocks = block_count;
    backend_ctx.start_block = start_block;

    /* Clear backend context */
    backend_ctx.checksum = CHECKSUM_INITIAL;
    backend_ctx.error = 0;
    backend_ctx.bytes_written = 0;
    backend_ctx.dump_in_progress = true;
    backend_ctx.current_offset = 0;

    /*
     * Write placeholder header - overwrites any existing data
     */

    /* Write placeholder header - overwrites any existing data */
    LOG_DBG("coredump_emmc_backend_start: writing initial header\n");
    struct emmc_hdr_t hdr = {
        .id = {'C', 'D'},
        .hdr_version = HDR_VER,
        .size = 0,
        .flags = 0,
        .checksum = 0,
        .error = 0,
    };

    ret = write_header_baremetal(&hdr);
    LOG_DBG("coredump_emmc_backend_start: write_header_baremetal returned %d", ret);
    if (ret != 0) {
        LOG_DBG("coredump_emmc_backend_start: write_header_baremetal failed %d\n", ret);
        backend_ctx.error = ret;
        return;
    }

    /* Advance offset past header - skip full block to avoid overwriting header */
    backend_ctx.current_offset = EMMC_BLOCK_SIZE;

    LOG_DBG("coredump_emmc_backend_start: done, current_offset=%zu",
           backend_ctx.current_offset);
}

/**
 * @brief End of coredump session.
 *
 * This finalizes the coredump and updates the header
 * with the final size and checksum.
 */
static void coredump_emmc_backend_end(void)
{
    struct emmc_hdr_t hdr;
    int ret;

    LOG_DBG("backend_end: dump_in_progress=%d", backend_ctx.dump_in_progress);

    if (!backend_ctx.dump_in_progress) {
        return;
    }

    backend_ctx.dump_in_progress = false;

    LOG_DBG("backend_end: flushing buffer, bytes_written=%zu", backend_ctx.bytes_written);

    /* Flush any remaining data in block buffer */
    flush_block_buf_if_needed();

    /* Skip busy check in end - just clear interrupts and proceed */
    sys_write32(SDHCI_INT_ALL_MASK, DT_REG_ADDR(DT_ALIAS(sdhc0)) + 0x30);

    /* Directly update header with final values */
    hdr.id[0] = 'C';
    hdr.id[1] = 'D';
    hdr.hdr_version = HDR_VER;
    hdr.size = backend_ctx.bytes_written;
    hdr.checksum = backend_ctx.checksum;
    hdr.error = backend_ctx.error;
    hdr.flags = 0;

    LOG_DBG("backend_end: writing final header: size=%zu, checksum=%u, error=%d",
           hdr.size, hdr.checksum, hdr.error);

    /* Write updated header */
    ret = write_header_baremetal(&hdr);
    if (ret != 0) {
        backend_ctx.error = ret;
    }

    LOG_DBG("backend_end done");
}

/**
 * @brief Flush block buffer to eMMC if there's data in it
 */
static void flush_block_buf_if_needed(void)
{
    int ret;
    size_t reserved_size;

    if (block_buf_used == 0) {
        return;
    }

    /* Check if we have space to write this block */
    reserved_size = (size_t)backend_ctx.reserved_blocks * EMMC_BLOCK_SIZE;
    if (backend_ctx.current_offset + EMMC_BLOCK_SIZE > reserved_size) {
        LOG_ERR("eMMC coredump space exhausted! offset=%zu, reserved=%zu",
                backend_ctx.current_offset, reserved_size);
        backend_ctx.error = -ENOSPC;
        block_buf_used = 0;
        return;
    }

    LOG_DBG("flush: block_buf_used=%zu, current_offset=%zu", block_buf_used, backend_ctx.current_offset);

    /* Pad remaining bytes with zeros */
    if (block_buf_used < EMMC_BLOCK_SIZE) {
        memset(block_buf + block_buf_used, 0, EMMC_BLOCK_SIZE - block_buf_used);
    }

    /* Calculate checksum for this block before writing */
    // CHECKSUM_TYPE block_checksum = compute_checksum(block_buf, EMMC_BLOCK_SIZE);

    ret = emmc_write_baremetal(backend_ctx.start_block * EMMC_BLOCK_SIZE + backend_ctx.current_offset, block_buf, EMMC_BLOCK_SIZE);
    LOG_DBG("flush: emmc_write_baremetal returned %d", ret);
    if (ret != 0) {
        LOG_ERR("eMMC flush error: %d at offset %zu", ret, backend_ctx.current_offset);
        backend_ctx.error = ret;
        return;
    }

    backend_ctx.current_offset += EMMC_BLOCK_SIZE;
    block_buf_used = 0;
}

/**
 * @brief Write a buffer to the coredump storage.
 * Accumulates data in block_buf and writes full blocks to eMMC.
 *
 * @param buf buffer of data to write
 * @param buflen number of bytes to write
 */
static void coredump_emmc_backend_buffer_output(uint8_t *buf, size_t buflen)
{
    size_t remaining = buflen;
    const uint8_t *ptr = buf;
    size_t copy_len;

    if (backend_ctx.error != 0) {
        return;
    }

    /*
     * Since the system is still running, memory content may be constantly
     * changing (e.g. stack of this thread). We need to make a copy of
     * the buffer so that the checksum corresponds to what is being written.
     */
    while (remaining > 0) {
        /* Copy data to block buffer */
        copy_len = MIN(remaining, EMMC_BLOCK_SIZE - block_buf_used);
        memcpy(block_buf + block_buf_used, ptr, copy_len);

        /* Update checksum with copied data */
        backend_ctx.checksum += compute_checksum(block_buf + block_buf_used, copy_len);

        block_buf_used += copy_len;
        ptr += copy_len;
        remaining -= copy_len;
        backend_ctx.bytes_written += copy_len;

        /* If block is full, flush it */
        if (block_buf_used == EMMC_BLOCK_SIZE) {
            flush_block_buf_if_needed();
            continue;  /* Don't process remaining data in same iteration */
        }
    }
}

/**
 * @brief Perform query on this backend.
 *
 * @param query_id ID of query
 * @param arg argument of query
 * @return depends on query
 */
static int coredump_emmc_backend_query(enum coredump_query_id query_id,
                   void *arg)
{
    int ret;

    switch (query_id) {
    case COREDUMP_QUERY_GET_ERROR:
        ret = backend_ctx.error;
        break;
    case COREDUMP_QUERY_HAS_STORED_DUMP:
        ret = process_stored_dump(cb_calc_checksum, NULL);
        break;
    case COREDUMP_QUERY_GET_STORED_DUMP_SIZE:
        ret = (int)get_stored_dump_size();
        break;
    default:
        ret = -ENOTSUP;
        break;
    }

    return ret;
}

/**
 * @brief Perform command on this backend.
 *
 * @param cmd_id command ID
 * @param arg argument of command
 * @return depends on command
 */
static int coredump_emmc_backend_cmd(enum coredump_cmd_id cmd_id,
                  void *arg)
{
    int ret;

    switch (cmd_id) {
    case COREDUMP_CMD_CLEAR_ERROR:
        ret = 0;
        backend_ctx.error = 0;
        break;
    case COREDUMP_CMD_VERIFY_STORED_DUMP:
        ret = process_stored_dump(cb_calc_checksum, NULL);
        break;
    case COREDUMP_CMD_ERASE_STORED_DUMP:
        ret = erase_stored_dump();
        break;
    case COREDUMP_CMD_COPY_STORED_DUMP:
        if (arg) {
            struct coredump_cmd_copy_arg *copy_arg =
                (struct coredump_cmd_copy_arg *)arg;

            ret = (int)get_stored_dump(copy_arg->offset,
                            copy_arg->buffer,
                            copy_arg->length);
        } else {
            ret = -EINVAL;
        }
        break;
    case COREDUMP_CMD_INVALIDATE_STORED_DUMP:
        ret = erase_stored_dump();
        break;
    default:
        ret = -ENOTSUP;
        break;
    }

    return ret;
}

struct coredump_backend_api coredump_backend_emmc = {
    .start = coredump_emmc_backend_start,
    .end = coredump_emmc_backend_end,
    .buffer_output = coredump_emmc_backend_buffer_output,
    .query = coredump_emmc_backend_query,
    .cmd = coredump_emmc_backend_cmd,
};

/**
 * @brief Initialize the eMMC coredump backend
 *
 * This should be called at system initialization to prepare
 * the backend for use.
 *
 * @return 0 if successful, negative errno on error
 */
int coredump_emmc_backend_init(void)
{
    if (backend_ctx.initialized) {
        return 0;
    }

    /* Initialize SDHC device handle for Zephyr SD stack operations */
    backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;

    backend_ctx.initialized = true;
    LOG_INF("eMMC coredump backend initialized");

    return 0;
}

#ifdef CONFIG_DEBUG_COREDUMP_SHELL
#include <zephyr/shell/shell.h>

/* Length of buffer of printable size */
#define PRINT_BUF_SZ      64

/* Length of buffer of printable size plus null character */
#define PRINT_BUF_SZ_RAW  (PRINT_BUF_SZ + 1)

/* Print buffer */
static char print_buf[PRINT_BUF_SZ_RAW];
static off_t print_buf_ptr;

/**
 * @brief Flush the print buffer to shell.
 *
 * @param sh shell instance.
 */
static void flush_print_buf(const struct shell *sh)
{
    shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, print_buf);
    print_buf_ptr = 0;
    (void)memset(print_buf, 0, sizeof(print_buf));
}

/**
 * @brief Callback to print stored coredump to shell
 *
 * @param arg shell instance
 * @param buf binary data buffer
 * @param len number of bytes in buffer to be printed
 * @return 0 if no issues; -EINVAL if error converting data
 */
static int cb_print_stored_dump(void *arg, uint8_t *buf, size_t len)
{
    int ret = 0;
    size_t i = 0;
    size_t remaining = len;
    const struct shell *sh = (const struct shell *)arg;

    if (len == 0) {
        /* Flush print buffer */
        flush_print_buf(sh);
        goto out;
    }

    /* Do checksum for process_stored_dump() */
    cb_calc_checksum(arg, buf, len);

    while (remaining > 0) {
        if (hex2char(buf[i] >> 4, &print_buf[print_buf_ptr]) < 0) {
            ret = -EINVAL;
            break;
        }
        print_buf_ptr++;

        if (hex2char(buf[i] & 0xf, &print_buf[print_buf_ptr]) < 0) {
            ret = -EINVAL;
            break;
        }
        print_buf_ptr++;

        remaining--;
        i++;

        if (print_buf_ptr == PRINT_BUF_SZ) {
            flush_print_buf(sh);
        }
    }

out:
    return ret;
}

/**
 * @brief Shell command to get backend error.
 *
 * Reads error from stored header on eMMC (not from memory, since
 * memory is lost on reboot).
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_error_get(const struct shell *sh,
                  size_t argc, char **argv)
{
    struct emmc_hdr_t hdr;
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* Lazy initialization of SDHC device */
    if (backend_ctx.sdhc_dev == NULL) {
        backend_ctx.sdhc_dev = COREDUMP_EMMC_DEVICE;
    }

    /* Read header from eMMC to get stored error */
    ret = read_header(&hdr);
    if (ret != 0) {
        shell_print(sh, "Failed to read header: %d", ret);
        return ret;
    }

    /* Verify it's a valid coredump header */
    if (hdr.id[0] != 'C' || hdr.id[1] != 'D') {
        shell_print(sh, "No stored coredump found.");
        return -ENODATA;
    }

    if (hdr.error == 0) {
        shell_print(sh, "No error. Size: %u bytes, checksum: %u",
                    hdr.size, hdr.checksum);
    } else {
        /* Provide human-readable error description */
        const char *err_str;
        switch (hdr.error) {
        case -ENOSPC:
            err_str = "Space exhausted - coredump truncated";
            break;
        case -EIO:
            err_str = "I/O error during write";
            break;
        case -ETIMEDOUT:
            err_str = "Write operation timed out";
            break;
        case -EBUSY:
            err_str = "Card was busy";
            break;
        default:
            err_str = "Unknown error";
            break;
        }
        shell_print(sh, "Error: %d (%s)", hdr.error, err_str);
        shell_print(sh, "Size: %u bytes, checksum: %u",
                    hdr.size, hdr.checksum);
    }

    return 0;
}

/**
 * @brief Shell command to clear backend error.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_error_clear(const struct shell *sh,
                    size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    backend_ctx.error = 0;

    shell_print(sh, "In-memory error cleared (lost on reboot).");
    shell_print(sh, "To clear stored error in eMMC, use: coredump_emmc erase");

    return 0;
}

/**
 * @brief Shell command to see if there is a stored coredump.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_has_stored_dump(const struct shell *sh,
                    size_t argc, char **argv)
{
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ret = coredump_emmc_backend_query(COREDUMP_QUERY_HAS_STORED_DUMP, NULL);

    if (ret == 1) {
        shell_print(sh, "Stored coredump found.");
    } else if (ret == 0) {
        shell_print(sh, "Stored coredump NOT found.");
    } else {
        /* Interpret the stored error */
        const char *err_str;
        switch (ret) {
        case -ENOSPC:
            err_str = "Space exhausted";
            break;
        case -EIO:
            err_str = "I/O error";
            break;
        case -ETIMEDOUT:
            err_str = "Timeout";
            break;
        case -EBUSY:
            err_str = "Card busy";
            break;
        default:
            err_str = "Unknown error";
            break;
        }
        shell_print(sh, "Stored coredump has error: %d (%s). "
                   "Use 'coredump_emmc erase' to clear.",
                   ret, err_str);
    }

    return 0;
}

/**
 * @brief Shell command to verify if the stored coredump is valid.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_verify_stored_dump(const struct shell *sh,
                       size_t argc, char **argv)
{
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ret = coredump_emmc_backend_cmd(COREDUMP_CMD_VERIFY_STORED_DUMP, NULL);

    if (ret == 1) {
        shell_print(sh, "Stored coredump verified.");
    } else if (ret == 0) {
        shell_print(sh, "Stored coredump verification failed "
                   "or there is no stored coredump.");
    } else {
        const char *err_str;
        switch (ret) {
        case -ENOSPC:
            err_str = "Space exhausted";
            break;
        case -EIO:
            err_str = "I/O error";
            break;
        default:
            err_str = "Unknown error";
            break;
        }
        shell_print(sh, "Stored coredump has error: %d (%s). "
                   "Use 'coredump_emmc erase' to clear.",
                   ret, err_str);
    }

    return 0;
}

/**
 * @brief Shell command to print stored coredump data to shell
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_print_stored_dump(const struct shell *sh,
                      size_t argc, char **argv)
{
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* Verify first to see if stored dump is valid */
    ret = coredump_emmc_backend_cmd(COREDUMP_CMD_VERIFY_STORED_DUMP, NULL);

    if (ret == 0) {
        shell_print(sh, "Stored coredump verification failed "
                   "or there is no stored coredump.");
        goto out;
    } else if (ret != 1) {
        shell_print(sh, "Failed to perform verify command: %d", ret);
        goto out;
    }

    /* If valid, start printing to shell */
    print_buf_ptr = 0;
    (void)memset(print_buf, 0, sizeof(print_buf));

    shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, COREDUMP_BEGIN_STR);

    ret = process_stored_dump(cb_print_stored_dump, (void *)sh);
    if (print_buf_ptr != 0) {
        shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, print_buf);
    }

    if (backend_ctx.error != 0) {
        shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, COREDUMP_ERROR_STR);
    }

    shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, COREDUMP_END_STR);

    if (ret == 1) {
        shell_print(sh, "Stored coredump printed.");
    } else if (ret == 0) {
        shell_print(sh, "Stored coredump verification failed "
                   "or there is no stored coredump.");
    } else {
        shell_print(sh, "Failed to print: %d", ret);
    }

out:
    return 0;
}

/**
 * @brief Shell command to erase stored coredump.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_erase_stored_dump(const struct shell *sh,
                      size_t argc, char **argv)
{
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ret = coredump_emmc_backend_cmd(COREDUMP_CMD_ERASE_STORED_DUMP, NULL);

    if (ret == 0) {
        shell_print(sh, "Stored coredump erased.");
    } else {
        shell_print(sh, "Failed to perform erase command: %d", ret);
    }

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_coredump_emmc_error,
    SHELL_CMD(clear, NULL, "Clear Coredump error",
          cmd_coredump_error_clear),
    SHELL_CMD(get, NULL, "Get Coredump error", cmd_coredump_error_get),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_coredump_emmc,
    SHELL_CMD(error, &sub_coredump_emmc_error,
          "Get/clear backend error.", NULL),
    SHELL_CMD(erase, NULL,
          "Erase stored coredump",
          cmd_coredump_erase_stored_dump),
    SHELL_CMD(find, NULL,
          "Query if there is a stored coredump",
          cmd_coredump_has_stored_dump),
    SHELL_CMD(print, NULL,
          "Print stored coredump to shell",
          cmd_coredump_print_stored_dump),
    SHELL_CMD(verify, NULL,
          "Verify stored coredump",
          cmd_coredump_verify_stored_dump),
    SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(coredump_emmc, &sub_coredump_emmc,
           "Coredump commands (eMMC backend)", NULL);

#endif /* CONFIG_DEBUG_COREDUMP_SHELL */
