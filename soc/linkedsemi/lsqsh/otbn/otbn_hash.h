#ifndef OTBN_HASH_H
#define OTBN_HASH_H

#include <stdint.h>
#include <stdbool.h>
#include "ls_otbn_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OTBN_HASH_ALGO_SHA224 = 0,
    OTBN_HASH_ALGO_SHA256,
    OTBN_HASH_ALGO_SHA384,
    OTBN_HASH_ALGO_SHA512,
    OTBN_HASH_ALGO_SM3,
} otbn_hash_algo_t;

#define OTBN_HASH_BLOCK_SIZE_MAX    128  /* SHA-384/512 */
#define OTBN_HASH_STATE_SIZE_MAX    256  /* SHA-384/512 DMEM state */

typedef struct __aligned(4) {
    otbn_hash_algo_t algo;
    otbn_firmware_t  firmware_id;

    uint64_t total_len;                 /* total bytes processed */
    uint32_t remain_len;                /* bytes in remain_data */
    uint32_t dmem_msg_idx;              /* next DMEM message write offset */

    uint8_t remain_data[OTBN_HASH_BLOCK_SIZE_MAX] __aligned(4);
    uint8_t state_bytes[OTBN_HASH_STATE_SIZE_MAX] __aligned(4);
} otbn_hash_ctx_t;

int otbn_hash_init(otbn_hash_ctx_t *ctx, otbn_hash_algo_t algo);
int otbn_hash_update(otbn_hash_ctx_t *ctx, const uint8_t *data, uint32_t len);
int otbn_hash_final(otbn_hash_ctx_t *ctx, uint8_t *digest);
int otbn_hash_transform(otbn_hash_ctx_t *ctx, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* OTBN_HASH_H */
