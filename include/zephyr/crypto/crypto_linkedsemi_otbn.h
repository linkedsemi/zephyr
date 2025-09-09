


#include <zephyr/device.h>
#include <zephyr/sys/util.h>



/**
 * Generate the prefix to add to an OTBN symbol name used on the Ibex side
 *
 * The result is a pointer to Ibex's rodata that should be used to initialise
 * memory for that symbol.
 *
 * This is needed by the OTBN driver to support DMEM/IMEM ranges but
 * application code shouldn't need to use this. Use the `otbn_addr_t` type and
 * supporting macros instead.
 */
#define OTBN_SYMBOL_PTR(app_name, sym) _otbn_local_app_##app_name##_##sym

/**
 * Generate the prefix to add to an OTBN symbol name used on the OTBN side
 *
 * The result is a pointer whose integer value is the address by which the
 * symbol should be accessed in OTBN memory.
 *
 * This is an internal macro used in `OTBN_DECLARE_SYMBOL_ADDR` and
 * `OTBN_ADDR_T_INIT` but application code shouldn't need to use it directly.
 */
#define OTBN_SYMBOL_ADDR(app_name, sym) _otbn_remote_app_##app_name##_##sym

/**
 * Makes a symbol in the OTBN application image available.
 *
 * This is needed by the OTBN driver to support DMEM/IMEM ranges but
 * application code shouldn't need to use this. To get access to OTBN
 * addresses, use `OTBN_DECLARE_SYMBOL_ADDR` instead.
 */
#define OTBN_DECLARE_SYMBOL_PTR(app_name, symbol_name) \
  extern const uint32_t OTBN_SYMBOL_PTR(app_name, symbol_name)[]

/**
 * Makes the OTBN address of a symbol in the OTBN application available.
 *
 * Symbols are typically function or data pointers, i.e. labels in assembly
 * code. Unlike OTBN_DECLARE_SYMBOL_PTR, this will work for symbols in the .bss
 * section (which exist on the OTBN side, even though they don't have backing
 * data on Ibex).
 *
 * Use this macro instead of manually declaring the symbols as symbol names
 * might change.
 *
 * @param app_name    Name of the application the function is contained in.
 * @param symbol_name Name of the symbol (function, label).
 */
#define OTBN_DECLARE_SYMBOL_ADDR(app_name, symbol_name) \
  extern const uint32_t OTBN_SYMBOL_ADDR(app_name, symbol_name)[]

/**
 * Makes an embedded OTBN application image available for use.
 *
 * Make symbols available that indicate the start and the end of instruction
 * and data memory regions, as they are stored in the device memory.
 *
 * Use this macro instead of manually declaring the symbols as symbol names
 * might change.
 *
 * @param app_name Name of the application to load, which is typically the
 *                 name of the main (assembly) source file.
 */
#define OTBN_DECLARE_APP_SYMBOLS(app_name)              \
  OTBN_DECLARE_SYMBOL_PTR(app_name, _imem_start);       \
  OTBN_DECLARE_SYMBOL_PTR(app_name, _imem_end);         \
  OTBN_DECLARE_SYMBOL_PTR(app_name, _dmem_data_start);  \
  OTBN_DECLARE_SYMBOL_PTR(app_name, _dmem_data_end);    \
  OTBN_DECLARE_SYMBOL_ADDR(app_name, _dmem_data_start); \
  OTBN_DECLARE_SYMBOL_ADDR(app_name, _checksum)

typedef uint32_t otbn_addr_t; 

  /**
 * Initializes an `otbn_addr_t`.
 */
#define OTBN_ADDR_T_INIT(app_name, symbol_name) \
  ((uint32_t)OTBN_SYMBOL_ADDR(app_name, symbol_name))

  /**
 * Information about an embedded OTBN application image.
 *
 * All pointers reference data in the normal CPU address space.
 * uint32_t values are addresses in the OTBN address space.
 *
 * Use `OTBN_DECLARE_APP_SYMBOLS()` together with `OTBN_APP_T_INIT()` to
 * initialize this structure.
 */
// typedef struct otbn_app {
//   /**
//    * Start of OTBN instruction memory in the embedded program.
//    *
//    * This pointer references Ibex's memory.
//    */
//   const uint32_t *imem_start;
//   /**
//    * The first word after OTBN instruction memory in the embedded program.
//    *
//    * This pointer references Ibex's memory.
//    *
//    * This address satifies `imem_start < imem_end`.
//    */
//   const uint32_t *imem_end;
//   /**
//    * Start of initialized OTBN data in the embedded program.
//    *
//    * This pointer references Ibex's memory.
//    *
//    * Data in between `dmem_data_start` and `dmem_data_end` will be copied to
//    * OTBN at app load time.
//    */
//   const uint32_t *dmem_data_start;
//   /**
//    * The first word after initialized OTBN data in the embedded program.
//    *
//    * This pointer references Ibex's memory.
//    *
//    * Should satisfy `dmem_data_start <= dmem_data_end`.
//    */
//   const uint32_t *dmem_data_end;
//   /**
//    * Start of initialized data section in OTBN's DMEM.
//    *
//    * This pointer references OTBN's memory and is used to copy data at app load
//    * time.
//    */
//   const otbn_addr_t dmem_data_start_addr;
//   /**
//    * Application checksum.
//    *
//    * This value represents a CRC32 checksum over IMEM and the `.data` portion
//    * of DMEM.
//    */
//   const uint32_t checksum;
// } otbn_app_t;
typedef struct otbn_app {
    uint16_t curve;
    uint8_t *dmem_image;
    uint8_t *imem_image;
    uint32_t kOtbnAppImemSize;
    uint32_t kOtbnAppDmemSize;
    uint32_t kOtbnAppDmemEnd;
} otbn_app_t;

/**
 * Initializes the OTBN application information structure.
 *
 * After making all required symbols from the application image available
 * through `OTBN_DECLARE_APP_SYMBOLS()`, use this macro to initialize an
 * `otbn_app_t` struct with those symbols.
 *
 * @param app_name Name of the application to load.
 * @see OTBN_DECLARE_APP_SYMBOLS()
 */
#define OTBN_APP_T_INIT(app_name)                                           \
  ((otbn_app_t){                                                            \
      .imem_start = OTBN_SYMBOL_PTR(app_name, _imem_start),                 \
      .imem_end = OTBN_SYMBOL_PTR(app_name, _imem_end),                     \
      .dmem_data_start = OTBN_SYMBOL_PTR(app_name, _dmem_data_start),       \
      .dmem_data_end = OTBN_SYMBOL_PTR(app_name, _dmem_data_end),           \
      .dmem_data_start_addr = OTBN_ADDR_T_INIT(app_name, _dmem_data_start), \
      .checksum = OTBN_ADDR_T_INIT(app_name, _checksum),                    \
  })

/**
 * OpenTitan's status_t is a single 32-bit word conveying either a result code
 * or an error code.
 * - The otcrypto has only one result code: The `Ok` value which is
 *   equivalent in value to kHardenedBoolTrue.
 * - The otcrypto error codes all have the MSB set and encode an error type of
 *   absl_status_t in the lower 5 bits.
 *
 *   This definition is supplied to provide the status_t definition when
 *   otcrypto is exported out of the OpenTitan repository.
 */
typedef uint32_t status_t;

/**
 * OTBN status
 */
typedef enum otbn_status {
  kOtbnStatusIdle = 0x00,
  kOtbnStatusBusyExecute = 0x01,
  kOtbnStatusBusySecWipeDmem = 0x02,
  kOtbnStatusBusySecWipeImem = 0x03,
  kOtbnStatusBusySecWipeInt = 0x04,
  kOtbnStatusLocked = 0xFF,
} otbn_status_t;

/**
 * OTBN commands
 */
typedef enum otbn_cmd {
  kOtbnCmdExecute = 0xd8,
  kOtbnCmdSecWipeDmem = 0xc3,
  kOtbnCmdSecWipeImem = 0x1e,
} otbn_cmd_t;

enum{
  /**
   * ERR_BITS register value in the case of no errors.
   *
   * Although some parts of the ERR_BITS register are marked reserved, the OTBN
   * documentation explicitly guarantees that ERR_BITS is zero for a successful
   * execution:
   *   https://opentitan.org/book/hw/ip/otbn/doc/theory_of_operation.html#software-execution-design-details
   */
  kOtbnErrBitsNoError = 0,
};

enum currnt_imem_image_t
{
    OTBN_FREE,
    OTBN_SM2,
    OTBN_ECC_P256,
    OTBN_ECC_P384,

};

struct otbn_ops_api_t{
    status_t (*otbn_imem_sec_wipe)(const struct device *dev);
    status_t (*otbn_dmem_sec_wipe)(const struct device *dev);
    uint32_t (*otbn_err_bits_get)(const struct device *dev);
    status_t (*otbn_execute)(const struct device *dev);
    int (*otbn_busy_wait_for_done)(const struct device *dev);
    int (*otbn_dmem_read)(const struct device *dev, uint16_t num_words, otbn_addr_t src, uint32_t *dest);
    int (*otbn_dmem_set)(const struct device *dev, uint16_t num_words, const uint32_t data, otbn_addr_t dest);
    int (*otbn_dmem_write)(const struct device *dev, uint16_t num_words, const uint32_t *src, otbn_addr_t dest);
    status_t (*otbn_load_app)(const struct device *dev, const otbn_app_t *app_info);
};


struct ls_otbn_data{
    struct k_sem mutex;
    struct k_sem completion_sem;
    enum currnt_imem_image_t mode;
    void (*app_callback)(const struct device *dev);
};
