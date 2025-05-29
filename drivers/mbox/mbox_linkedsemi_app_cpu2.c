#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mbox_linkedsemi_app_cpu2);

#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <cpu.h>

BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);

__nocache static volatile bool g_done = false;
__nocache static volatile bool g_ack = false;
__nocache static volatile int g_ret = 0;
__nocache static void *parm[MBOX_FUNC_CALL_PARM_NUM_MAX] = {};
static K_SEM_DEFINE(mbox_func_call_sem, 0, 1);

static void mbox_func_call_recv_callback(const struct device *dev,
                                        mbox_channel_id_t channel_id,
                                        void *user_data,
                                        struct mbox_msg *data)
{
    uint32_t *mbox_data = (uint32_t *)data->data;
    if ((*mbox_data == MBOX_FUNC_CALL_FLASH_READ_JEDEC_ID)
        || (*mbox_data == MBOX_FUNC_CALL_FLASH_ERASE)
        || (*mbox_data == MBOX_FUNC_CALL_FLASH_WRITE)
        || (*mbox_data == MBOX_FUNC_CALL_FLASH_READ)) {
        k_sem_give(&mbox_func_call_sem);
    } else {
        LOG_ERR("invalid data\n");
    }
}

void mbox_func_call_recv_register(const struct mbox_dt_spec *rx_channel)
{
    if (mbox_register_callback_dt(rx_channel, mbox_func_call_recv_callback, NULL)) {
        LOG_ERR("mbox_register_callback() error\n");
        return;
    }

    if (mbox_set_enabled_dt(rx_channel, true)) {
        LOG_ERR("mbox_set_enable() error\n");
        return;
    }
}

__ramfunc static void mbox_func_call_wait(const struct mbox_dt_spec *tx_channel,
                                                   struct mbox_msg *msg,
                                                   uint32_t *retry_cnt)
{
    g_ack = true;
    *retry_cnt = 0;
    while ((!g_done) && (*retry_cnt < MBOX_RETRY_MAX_CNT)) {
        (*retry_cnt)++;
        nop_delay(10);
    }
}

int mbox_func_call(const struct mbox_dt_spec *tx_channel,
                   enum mbox_func_call_id api_id,
                   uint32_t parm_num,
                   ...)
{
    mbox_func_call_data_t mbox_func_call_data = {};
    struct mbox_msg msg = {};
    uint32_t retry_cnt = 0;
    int ret = 0;

    va_list args;
    va_start(args, parm_num);
    for (uint32_t i = 0; i < parm_num; i++) {
        parm[i] = va_arg(args, void *);
    }
    va_end(args);

    mbox_func_call_data.msg_id = MBOX_FUNC_CALL;
    mbox_func_call_data.api_id = api_id;
    mbox_func_call_data.done = &g_done;
    mbox_func_call_data.ack = &g_ack;
    // mbox_func_call_data.ret = &g_ret;
    mbox_func_call_data.parm_num = parm_num;
    mbox_func_call_data.parm = parm;

    msg.data = &mbox_func_call_data;
    msg.size = sizeof(mbox_func_call_data_t);

    g_done = false;
    g_ack = false;

    if (mbox_send_dt(tx_channel, &msg) == -ENOSPC) {
        while(1);
    }
    k_sem_take(&mbox_func_call_sem, K_FOREVER);
    disable_global_irq();
    mbox_func_call_wait(tx_channel, &msg, &retry_cnt);
    if (retry_cnt > MBOX_RETRY_MAX_CNT) {
        ret = -1;
        goto timeout;
    }

    if (mbox_func_call_data.ret) {
        ret = *((int *)mbox_func_call_data.ret);
    }

timeout:
    if ((retry_cnt == 0) || (retry_cnt > MBOX_RETRY_MAX_CNT)) {
        while(1);
    }
    enable_global_irq();

    return ret;
}
