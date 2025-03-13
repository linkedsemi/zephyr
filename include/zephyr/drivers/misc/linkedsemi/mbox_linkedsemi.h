#ifndef __MBOX_LINKEDSEMI_H
#define __MBOX_LINKEDSEMI_H

#include <zephyr/kernel.h>
#include <core_rv32.h>

#define MBOX_FUNC_CALL_PARM_NUM_MAX 10
#define MBOX_RETRY_MAX_CNT 100000

enum mbox_msg_id {
    MBOX_FUNC_CALL,
};

enum mbox_func_call_id {
    MBOX_FUNC_CALL_HAL_FLASH_READ_ID,
    MBOX_FUNC_CALL_HAL_FLASH_SECTOR_ERASE,
    MBOX_FUNC_CALL_HAL_FLASH_PAGE_PROGRAM,
    MBOX_FUNC_CALL_HAL_FLASH_MULTI_IO_READ,
    MBOX_FUNC_CALL_GO_IDLE,
};

typedef struct __packed {
    volatile uint32_t msg_id;
    volatile uint32_t api_id;
    volatile void *ret;
    volatile bool *done;
    volatile bool *ack;
    volatile uint32_t parm_num;
    volatile void *parm;
    // volatile uint8_t resvered[232];
} mbox_func_call_data_t;

int mbox_func_call(const struct mbox_dt_spec *tx_channel, enum mbox_func_call_id api_id, uint32_t parm_num, ...);
int mbox_acquire_cpu1_idle(const struct mbox_dt_spec *tx_channel);
void mbox_release_cpu1(void);

static inline bool is_cpu1_xip(void)
{
    return ((CONFIG_CPU1_BOOT_ADDR >= 0x8000000)
            && (CONFIG_CPU1_BOOT_ADDR < (0x8000000 + (16 * 1024 * 1024))));
}

static inline void nop_delay(uint64_t count)
{
    for (uint64_t i = 0; i < count; i ++) {
        __NOP();
    }
}

#endif /* __MBOX_LINKEDSEMI_H */
