#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <cpu.h>

__nocache static volatile bool g_done = false;
__nocache static volatile bool g_ack = false;

__ramfunc static void mbox_func_call_send_and_wait(const struct mbox_dt_spec *tx_channel,
                                                   struct mbox_msg *msg,
                                                   uint32_t *retry_cnt)
{
    if (mbox_send_dt(tx_channel, msg) == -ENOSPC) {
        printk("mbox_send() full\n");
    }
    sys_cache_data_invd_all();
    *retry_cnt = 0;
    while ((!g_ack) && (*retry_cnt < MBOX_RETRY_MAX_CNT)) {
        (*retry_cnt)++;
        nop_delay(10);
    }
}

int mbox_acquire_cpu2_idle(const struct mbox_dt_spec *tx_channel)
{
    mbox_func_call_data_t mbox_func_call_data = {};
    struct mbox_msg msg = {};
    uint32_t retry_cnt = 0;
    int ret = 0;

    mbox_func_call_data.msg_id = MBOX_FUNC_CALL;
    mbox_func_call_data.api_id = MBOX_FUNC_CALL_GO_IDLE;
    mbox_func_call_data.done = &g_done;
    mbox_func_call_data.ack = &g_ack;
    mbox_func_call_data.parm_num = 0;
    mbox_func_call_data.parm = NULL;

    msg.data = &mbox_func_call_data;
    msg.size = sizeof(mbox_func_call_data_t);

    g_done = false;
    g_ack = false;
    sys_cache_data_flush_all();
    mbox_func_call_send_and_wait(tx_channel, &msg, &retry_cnt);
    if (retry_cnt > MBOX_RETRY_MAX_CNT) {
        ret = -1;
    }

    return ret;
}

void mbox_release_cpu2(void)
{
    g_done = true;
}
