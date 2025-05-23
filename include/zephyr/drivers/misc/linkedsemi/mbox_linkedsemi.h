#ifndef __MBOX_LINKEDSEMI_H
#define __MBOX_LINKEDSEMI_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <core_rv32.h>
#include "reg_sysc_sec_cpu.h"

#define MBOX_FUNC_CALL_PARM_NUM_MAX 10
#define MBOX_RETRY_MAX_CNT 0xffffffff

enum mbox_msg_id {
    MBOX_FUNC_CALL,
};

enum mbox_func_call_id {
    MBOX_FUNC_CALL_FLASH_READ_JEDEC_ID,
    MBOX_FUNC_CALL_FLASH_ERASE,
    MBOX_FUNC_CALL_FLASH_WRITE,
    MBOX_FUNC_CALL_FLASH_READ,
    MBOX_FUNC_CALL_DO_IDLE,
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
int mbox_acquire_cpu2_idle(const struct mbox_dt_spec *tx_channel);
void mbox_release_cpu2(void);
__ramfunc int mbox_linkedsemi_send_ramfunc(const struct device *dev, uint32_t channel, const struct mbox_msg *msg);

static inline bool is_cpu2_xip(void)
{
    return ((CONFIG_CPU2_BOOT_ADDR >= 0x8000000)
            && (CONFIG_CPU2_BOOT_ADDR < (0x8000000 + MB(16))));
}

static inline bool is_cpu2_running(void)
{
    return (SYSC_SEC_CPU->APP_CPU_SRST > 0);
}

static ALWAYS_INLINE void nop_delay(uint64_t count)
{
    for (uint64_t i = 0; i < count; i ++) {
        __NOP();
    }
}

#endif /* __MBOX_LINKEDSEMI_H */
