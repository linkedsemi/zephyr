#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mbox_linkedsemi_app_cpu1);

#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <cpu.h>

BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);

static K_SEM_DEFINE(mbox_func_call_sem, 0, 1);
volatile bool *g_done;

void mbox_func_call_recv_do_idle_callback(mbox_func_call_data_t *mbox_func_call_data)
{
    if (NULL == mbox_func_call_data) {
        return;
    }

    g_done = mbox_func_call_data->done;
    k_sem_give(&mbox_func_call_sem);

    return;
}


void mbox_func_call_trx_register(const struct mbox_dt_spec *tx_channel,
                                    const struct mbox_dt_spec *rx_channel)
{
    __ASSERT_NO_MSG(tx_channel);
    __ASSERT_NO_MSG(rx_channel);
}

static int mbox_func_call_send_and_wait(const struct mbox_dt_spec *tx_channel,
                                                   struct mbox_msg *msg,
                                                   uint32_t *retry_cnt)
{
    __ASSERT_NO_MSG(tx_channel);
    if (mbox_send_dt(tx_channel, msg) == -ENOSPC) {
        printk("mbox_send() full\n");
        return -ENOSPC;
    }

    if (k_sem_take(&mbox_func_call_sem, K_MSEC(MBOX_RETRY_MAX_CNT)) != 0) {
        LOG_ERROR("no response!\n");
        return -ETIMEDOUT;
    }

    return 0;
}

int mbox_acquire_cpu2_idle(const struct mbox_dt_spec *tx_channel)
{
    __ASSERT_NO_MSG(tx_channel);
    mbox_func_call_data_t mbox_func_call_data = {};
    struct mbox_msg msg = {};
    uint32_t retry_cnt = 0;
    int ret = 0;

    mbox_func_call_data.msg_id = MBOX_FUNC_CALL;
    mbox_func_call_data.api_id = MBOX_FUNC_CALL_DO_IDLE;
    mbox_func_call_data.parm_num = 0;
    mbox_func_call_data.parm = NULL;

    msg.data = &mbox_func_call_data;
    msg.size = sizeof(mbox_func_call_data_t);

    int mbox_func_call_ret = mbox_func_call_send_and_wait(tx_channel, &msg, &retry_cnt);
    if (mbox_func_call_ret) {
        ret = -1;
    }

    return ret;
}

int mbox_send_cpu2_invalid(const struct mbox_dt_spec *tx_channel, enum mbox_func_call_id mbox_func_call_id)
{
    __ASSERT_NO_MSG(tx_channel);
    mbox_func_call_data_t mbox_func_call_data = {};
    struct mbox_msg msg = {};
    int ret = 0;

    mbox_func_call_data.msg_id = MBOX_FUNC_CALL;
    mbox_func_call_data.api_id = mbox_func_call_id;

    msg.data = &mbox_func_call_data;
    msg.size = sizeof(mbox_func_call_data_t);

    if (mbox_send_dt(tx_channel, &msg) == -ENOSPC) {
        printk("mbox_send() full\n");
        return -ENOSPC;
    }

    return ret;
}

void mbox_release_cpu2(void)
{
    if (NULL != g_done) {
        *g_done = true;
    }
}
