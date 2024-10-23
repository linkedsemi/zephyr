#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <string.h>
#include <zephyr/crypto/crypto.h>
#include <zephyr/sys/byteorder.h>
#include "crypto_linkedsemi.h"
LOG_MODULE_DECLARE(crypto_linkedsem);
#include "crypto_linkedsemi_sha.h"

static void sha_fifo_write_byte(const struct device *dev, uint8_t byte)
{
    const struct crypto_linkedsemi_config *dev_config = dev->config;
    struct crypto_linkedsemi_data *dev_data = dev->data;
    dev_data->sha_pkt_in_buf_index++;
    dev_data->sha_current_block_index++;
    ((uint8_t *)(&dev_data->sha_fifo))[dev_data->sha_fifo_index++] = byte;
    if (dev_data->sha_fifo_index == 4) {
        dev_data->sha_fifo_index = 0;
        sys_write32(dev_data->sha_fifo, dev_config->reg_calc_sha + SHA_FIFO_DAT);
    }
}

void linkedsemi_sha_isr(const struct device *dev)
{
    const struct crypto_linkedsemi_config *dev_config = dev->config;
    struct crypto_linkedsemi_data *dev_data = dev->data;

    union sha_reg_intr sha_reg_intr_stat_un;
    sha_reg_intr_stat_un.value = sys_read32(dev_config->reg_calc_sha + SHA_INTR_S);

    if (sha_reg_intr_stat_un.field.FSM_END) {
        union sha_reg_intr sha_reg_intr_un = { .field = { .FSM_END = 1, }, };
        sys_write32(sha_reg_intr_un.value, dev_config->reg_calc_sha + SHA_INTR_C);

        k_sem_give(&dev_data->hash_device_sync_sem);
    }
}

int crypto_linkedsemi_sha(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
    const struct device *dev = ctx->device;
    const struct crypto_linkedsemi_config *dev_config = dev->config;
    struct crypto_linkedsemi_data *dev_data = dev->data;
    int ret = 0;

    union sha_reg_ctrl sha_reg_ctrl_un = {
        .field = {
            .FST_DAT = (dev_data->sha_total_len == 0) ? 1 : 0,
            .CALC_SHA224 = (dev_data->hash_algo == CRYPTO_HASH_ALGO_SHA224) ? 1 : 0,
            .CALC_SM3 = 0,
            /* LEN start from 0. write 0 means 1 block */
            .LEN = 0,
        },
    };
    union sha_reg_intr sha_reg_intr_un = { .field = { .FSM_END = 1, .FSM_EMPT = 0, }, };
    const union sha_reg_start sha_reg_start_un = { .field = { .FSM_START = 1, }, };

    dev_data->sha_total_len += pkt->in_len;
    dev_data->sha_pkt_in_buf_index = 0;

    sys_write32(sha_reg_ctrl_un.value, dev_config->reg_calc_sha + SHA_CTRL);
    sys_write32(sha_reg_intr_un.value, dev_config->reg_calc_sha + SHA_INTR_M);
    sys_write32(sha_reg_start_un.value, dev_config->reg_calc_sha + SHA_START);

    sha_reg_ctrl_un.field.FST_DAT = 0;
    sys_write32(sha_reg_ctrl_un.value, dev_config->reg_calc_sha + SHA_CTRL);

    /* copy buffer to hash engine */
    for (uint64_t i = dev_data->sha_pkt_in_buf_index; i < pkt->in_len; i++) {
        sha_fifo_write_byte(dev, pkt->in_buf[i]);
        if(dev_data->sha_current_block_index == SHA_BLOCK_LEN_BYTE) {
            dev_data->sha_current_block_index = 0;
            k_sem_take(&dev_data->hash_device_sync_sem, K_FOREVER);
            sys_write32(sha_reg_start_un.value, dev_config->reg_calc_sha + SHA_START);
        }
    }

    if (finish) {
        /* write tail to hash engine */
        sha_fifo_write_byte(dev, 0x80);
        while (dev_data->sha_current_block_index != SHA_PADDING_MOD_LEN_BYTE) {
            if(dev_data->sha_current_block_index == SHA_BLOCK_LEN_BYTE) {
                dev_data->sha_current_block_index = 0;
                k_sem_take(&dev_data->hash_device_sync_sem, K_FOREVER);
                sys_write32(sha_reg_start_un.value, dev_config->reg_calc_sha + SHA_START);
            }
            sha_fifo_write_byte(dev, 0x00);
        }
        uint64_t sha_total_len_bit = dev_data->sha_total_len * 8;
        sha_fifo_write_byte(dev, sha_total_len_bit >> 56);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 48);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 40);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 32);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 24);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 16);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 8);
        sha_fifo_write_byte(dev, sha_total_len_bit >> 0);
        k_sem_take(&dev_data->hash_device_sync_sem, K_FOREVER);

        uint8_t result_len = 0;
        switch(dev_data->hash_algo) {
        case CRYPTO_HASH_ALGO_SHA224:
            result_len = 7;
            break;
        case CRYPTO_HASH_ALGO_SHA256:
            result_len = 8;
            break;
        case CRYPTO_HASH_ALGO_SHA384:
        case CRYPTO_HASH_ALGO_SHA512:
            break;
        default:
            LOG_ERR("Unsupported mode");
            return -ENOTSUP;
        }
        for (uint8_t i = 0; i < result_len; i++) {
            uint32_t val = sys_read32(dev_config->reg_calc_sha + SHA_RSLT0 + i * 0x4);
            val = BSWAP_32(val);
            memcpy(pkt->out_buf + i * sizeof(uint32_t), &val, sizeof(uint32_t));
        }

        dev_data->sha_fifo_index = 0;
        dev_data->sha_current_block_index = 0;
        dev_data->sha_total_len = 0;
    }

    return ret;
}
