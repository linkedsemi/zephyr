#ifndef _LS_OTBN_CONFIG_
#define _LS_OTBN_CONFIG_

#include <string.h>
#include <stdbool.h>
/**
 * @file ls_otbn_config.h
 * @brief LinkedSemi OTBN RTOS abstraction layer API.
 *
 * This header provides the high-level API used by crypto libraries
 * (mbedtls/wolfssl) and user applications to access the OTBN hardware.
 *
 * @section otbn_session OTBN Session Model
 *
 * OTBN is a single, exclusive hardware accelerator. All operations must
 * follow the acquire-use-release pattern:
 *
 * @code
 * ret = ls_otbn_session_acquire(firmware_id, timeout);
 * if (ret != 0) {
 *     // OTBN is busy or timeout expired
 * }
 *
 * // Load firmware/data, run commands...
 * ls_otbn_imem_write(0, firmware_code, code_size);
 * ls_otbn_cmd(OTBN_CMD_EXECUTE);
 *
 * ls_otbn_session_release();
 * @endcode
 *
 * Important rules:
 *   - Only one thread may hold an OTBN session at a time.
 *   - A thread that does not own the session will get -EACCES from memory
 *     access functions and commands.
 *   - Reentrant acquire by the same thread is rejected with -EBUSY.
 *   - @ref ls_otbn_session_release must only be called by the thread that
 *     successfully acquired the session.
 *   - @ref ls_otbn_module_deinit may be called to force deinitialization;
 *     it waits for the current owner to release the session and for any
 *     running OTBN command to complete.
 *
 * @section otbn_firmware Firmware IDs
 *
 * The firmware ID identifies which OTBN firmware image is requested.
 * Built-in values are defined below (e.g. OTBN_FIRMWARE_SHA256).
 * User-defined firmware IDs may start at @ref OTBN_FIRMWARE_USER_BASE.
 */

/** @brief OTBN command types */
enum otbn_cmd_t
{
    OTBN_CMD_EXECUTE            = 0xd8, /**< Start firmware execution */
    OTBN_CMD_SEC_WIPE_DMEM      = 0xc3, /**< Securely wipe data memory */
    OTBN_CMD_SEC_WIPE_IMEM      = 0x1e, /**< Securely wipe instruction memory */
};

#define OTBN_FIRMWARE_UNUSED         0 /**< No firmware loaded */
#define OTBN_FIRMWARE_SHA256         1 /**< SHA-256 firmware */
#define OTBN_FIRMWARE_SHA384         2 /**< SHA-384 firmware */
#define OTBN_FIRMWARE_SHA512         3 /**< SHA-512 firmware */
#define OTBN_FIRMWARE_SM3            4 /**< SM3 firmware */
#define OTBN_FIRMWARE_ECDSA_P256     5 /**< ECDSA over P-256 firmware */
#define OTBN_FIRMWARE_ECDSA_P384     6 /**< ECDSA over P-384 firmware */
#define OTBN_FIRMWARE_SM2            7 /**< SM2 firmware */
#define OTBN_FIRMWARE_RSA_MODEXP     8 /**< RSA modexp firmware */
#define OTBN_FIRMWARE_RSA_KEYGEN     9 /**< RSA key generation firmware */

/** @brief Base value for user-defined custom firmware IDs */
#define OTBN_FIRMWARE_USER_BASE     0x100

typedef uint32_t otbn_firmware_t;   /**< OTBN firmware identifier */
typedef uint32_t otbn_timeout_s;    /**< Timeout in seconds for session acquire */
typedef uint32_t (*otbn_rand_cb)(void);

typedef void (*otbn_done_callback_t )(void *param);

/**
 * @brief Register TRNG/PRNG callbacks for OTBN EDN requests.
 *
 * OTBN requires external random numbers during operation. If no callbacks
 * are registered, the built-in fallback uses rand() (not cryptographically
 * secure). For production use, register a hardware entropy source.
 *
 * @param trng_cb Callback for true random number requests.
 * @param prng_cb Callback for pseudo random number requests.
 */
void ls_otbn_random_callback_register(otbn_rand_cb trng_cb, otbn_rand_cb prng_cb);

/**
 * @brief Register a callback invoked when OTBN execution completes.
 *
 * The callback is called from ISR context when OTBN reports completion.
 *
 * @param handler Callback function pointer.
 * @param param   User parameter passed to the callback.
 */
void ls_otbn_done_callback_register(otbn_done_callback_t handler, void *param);

/** @brief Unregister the OTBN completion callback. */
void ls_otbn_done_callback_unregister(void);

/**
 * @brief Initialize the OTBN hardware and related OS resources.
 *
 * This function is idempotent; it does nothing if OTBN is already
 * initialized. It is called automatically by @ref ls_otbn_session_acquire.
 */
void ls_otbn_module_init(void);

/**
 * @brief Deinitialize the OTBN hardware and release OS resources.
 *
 * This function waits for any active OTBN session to finish and for any
 * running OTBN command to complete before deinitializing the hardware.
 * If the calling thread currently owns the session, it is released first.
 *
 * @return 0 on success, negative errno on failure (e.g. -ETIMEDOUT if a
 *         running command does not complete within
 *         @ref OTBN_OPERATION_TIMEOUT_MS).
 */
int ls_otbn_module_deinit(void);

/**
 * @brief Acquire an exclusive OTBN session for the current thread.
 *
 * This function is non-reentrant: calling it again from the same thread
 * before releasing the session returns -EBUSY. Calling it from a different
 * thread while a session is active returns -EBUSY or blocks until the
 * timeout expires.
 *
 * @param firmware_id Identifier of the firmware to be loaded.
 * @param timeout     Maximum time to wait in seconds.
 *
 * @return 0 on success, -EBUSY if the session is already held by another
 *         thread or by the same thread, or if the timeout expires.
 */
int ls_otbn_session_acquire(otbn_firmware_t firmware_id, otbn_timeout_s timeout);

/**
 * @brief Check whether the current thread holds the OTBN session.
 *
 * This function is mainly intended for use cases where an OTBN operation is
 * split into multiple stages and the caller needs to verify ownership before
 * each stage, e.g. hash algorithms (init/update/final) where the same session
 * is held across several API calls. In normal one-shot operations it is not
 * necessary to call this explicitly; just acquire, use the OTBN, and release.
 *
 * @return true if the current thread is the owner, false otherwise.
 */
bool ls_otbn_session_is_owner(void);

/**
 * @brief Release the OTBN session held by the current thread.
 *
 * This function must only be called by the thread that successfully called
 * @ref ls_otbn_session_acquire. Releasing without owning a session is a
 * programming error.
 */
int ls_otbn_session_release(void);

/**
 * @brief Mark @p firmware_id as the firmware currently held in OTBN IMEM.
 *
 * Modules that program IMEM with a complete firmware image must call this
 * after a successful write, so that the shared IMEM state stays accurate.
 * The state persists across sessions: IMEM is hardware memory, it is not
 * cleared by session acquire/release. It is reset when the OTBN module is
 * (re)initialized, since that resets the core.
 *
 * @param firmware_id Firmware image now resident in IMEM.
 */
void ls_otbn_imem_firmware_confirm(otbn_firmware_t firmware_id);

/**
 * @brief Get the firmware currently held in OTBN IMEM per the last
 *        @ref ls_otbn_imem_firmware_confirm call.
 *
 * Modules that cache the loaded firmware (hash, ECC) use this to decide
 * whether IMEM still holds their image or must be reloaded.
 *
 * @return Current IMEM firmware, or OTBN_FIRMWARE_UNUSED if unknown.
 */
otbn_firmware_t ls_otbn_imem_firmware_get(void);

/**
 * @brief Send a command to OTBN and wait for completion.
 *
 * The caller must hold an active OTBN session. The function blocks until
 * OTBN reports completion or the internal timeout expires.
 *
 * @param cmd Command to execute, see @ref otbn_cmd_t.
 *
 * @return 0 on success, -ETIMEDOUT on timeout, other negative errno on error.
 */
int ls_otbn_cmd(enum otbn_cmd_t cmd);

/**
 * @brief Write data to OTBN instruction memory (IMEM).
 *
 * The caller must hold an active OTBN session. Both @p offset and @p size
 * must be 4-byte aligned. The caller should also ensure OTBN is idle; if a
 * command is still running, the function returns -EBUSY.
 *
 * @param offset Byte offset in IMEM, must be 4-byte aligned.
 * @param src    Source data buffer.
 * @param size   Number of bytes to write, must be 4-byte aligned.
 *
 * @return 0 on success, -EINVAL for invalid parameters (e.g. unaligned or
 *         out-of-bounds), -EACCES if the current thread does not hold the
 *         session, -EBUSY if OTBN is not idle, other negative errno on
 *         hardware error.
 */
int ls_otbn_imem_write(uint32_t offset, const uint32_t *src, uint32_t size);

/**
 * @brief Read data from OTBN instruction memory (IMEM).
 *
 * The caller must hold an active OTBN session. Both @p offset and @p size
 * must be 4-byte aligned. The caller should also ensure OTBN is idle; if a
 * command is still running, the function returns -EBUSY.
 *
 * @param offset Byte offset in IMEM, must be 4-byte aligned.
 * @param dst    Destination data buffer.
 * @param size   Number of bytes to read, must be 4-byte aligned.
 *
 * @return 0 on success, -EINVAL for invalid parameters, -EACCES if the
 *         current thread does not hold the session, -EBUSY if OTBN is not
 *         idle, other negative errno on hardware error.
 */
int ls_otbn_imem_read(uint32_t offset, uint32_t *dst, uint32_t size);

/**
 * @brief Write data to OTBN data memory (DMEM).
 *
 * The caller must hold an active OTBN session. Both @p offset and @p size
 * must be 4-byte aligned. The caller should also ensure OTBN is idle; if a
 * command is still running, the function returns -EBUSY.
 *
 * @param offset Byte offset in DMEM, must be 4-byte aligned.
 * @param src    Source data buffer.
 * @param size   Number of bytes to write, must be 4-byte aligned.
 *
 * @return 0 on success, -EINVAL for invalid parameters, -EACCES if the
 *         current thread does not hold the session, -EBUSY if OTBN is not
 *         idle, other negative errno on hardware error.
 */
int ls_otbn_dmem_write(uint32_t offset, const uint32_t *src, uint32_t size);

/**
 * @brief Read data from OTBN data memory (DMEM).
 *
 * The caller must hold an active OTBN session. Both @p offset and @p size
 * must be 4-byte aligned. The caller should also ensure OTBN is idle; if a
 * command is still running, the function returns -EBUSY.
 *
 * @param offset Byte offset in DMEM, must be 4-byte aligned.
 * @param dst    Destination data buffer.
 * @param size   Number of bytes to read, must be 4-byte aligned.
 *
 * @return 0 on success, -EINVAL for invalid parameters, -EACCES if the
 *         current thread does not hold the session, -EBUSY if OTBN is not
 *         idle, other negative errno on hardware error.
 */
int ls_otbn_dmem_read(uint32_t offset, uint32_t *dst, uint32_t size);

/**
 * @brief Set a region of OTBN data memory (DMEM) to a fixed value.
 *
 * The caller must hold an active OTBN session. Both @p offset and @p size
 * must be 4-byte aligned. The caller should also ensure OTBN is idle; if a
 * command is still running, the function returns -EBUSY.
 *
 * @param offset Byte offset in DMEM, must be 4-byte aligned.
 * @param data   32-bit data value to fill.
 * @param size   Number of bytes to set, must be 4-byte aligned.
 *
 * @return 0 on success, -EINVAL for invalid parameters, -EACCES if the
 *         current thread does not hold the session, -EBUSY if OTBN is not
 *         idle, other negative errno on hardware error.
 */
int ls_otbn_dmem_set(uint32_t offset, uint32_t data, uint32_t size);



#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_SERVER)
/** @brief Initialize wolfSSL OTBN delegation server mailboxes. */
void ls_otbn_delegation_server_chanels_init(void);
#endif

#if defined(CONFIG_WOLFSSL_LINKEDSEMI_OTBN_DELEGATION_CLIENT)
/** @brief Initialize wolfSSL OTBN delegation client mailboxes. */
void ls_otbn_delegation_client_chanels_init(void);
#endif

#endif
