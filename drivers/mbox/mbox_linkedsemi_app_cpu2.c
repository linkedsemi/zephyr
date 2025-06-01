#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/mbox.h>
#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mbox_linkedsemi_app_cpu2);

#include <zephyr/drivers/misc/linkedsemi/mbox_linkedsemi.h>
#include <cpu.h>

BUILD_ASSERT(CONFIG_NOCACHE_MEMORY);

static bool mbox_func_call_state = false;
__nocache static volatile bool g_done = false;
__nocache static volatile int g_ret = 0;
const struct mbox_dt_spec *g_tx_channel;
__nocache static void *parm[MBOX_FUNC_CALL_PARM_NUM_MAX] = {};
static K_SEM_DEFINE(mbox_func_call_sem, 0, 1);

__ramfunc static void mbox_func_call_wait(void)
{
    while (!g_done);
}

static void mbox_func_call_recv_callback(const struct device *dev,
                                        mbox_channel_id_t channel_id,
                                        void *user_data,
                                        struct mbox_msg *data)
{
    mbox_func_call_data_t *mbox_func_call_data = (mbox_func_call_data_t *)data->data;

    if (NULL == mbox_func_call_data) {
        return;
    }

    switch(mbox_func_call_data->api_id) {
    case MBOX_FUNC_CALL_DO_IDLE:
        disable_global_irq();
        struct mbox_msg msg = {};
        mbox_func_call_data_t mbox_func_call_data_ack = {};
        mbox_func_call_data_ack.msg_id = MBOX_FUNC_CALL;
        mbox_func_call_data_ack.api_id = MBOX_FUNC_CALL_DO_IDLE;
        mbox_func_call_data_ack.done = &g_done;

        msg.data = &mbox_func_call_data_ack;
        msg.size = sizeof(mbox_func_call_data_t);

        g_done = false;
        if (NULL != g_tx_channel) {
            if (mbox_send_dt(g_tx_channel, &msg) == -ENOSPC) {
                while(1);
            }
        } else {
            LOG_ERR("invalid g_tx_channel\n");
        }
        mbox_func_call_wait();
        enable_global_irq();
        if (mbox_func_call_state) {
            k_sem_give(&mbox_func_call_sem);
            mbox_func_call_state = false;
        }
        break;
    default:
        LOG_ERR("invalid data\n");
        break;
    };

    return;
}

void mbox_func_call_trx_register(const struct mbox_dt_spec *tx_channel,
                                    const struct mbox_dt_spec *rx_channel)
{
    __ASSERT_NO_MSG(tx_channel);
    __ASSERT_NO_MSG(rx_channel);
    g_tx_channel = tx_channel;

    if (mbox_register_callback_dt(rx_channel, mbox_func_call_recv_callback, NULL)) {
        LOG_ERR("mbox_register_callback() error\n");
        return;
    }

    if (mbox_set_enabled_dt(rx_channel, true)) {
        LOG_ERR("mbox_set_enable() error\n");
        return;
    }
}

int mbox_func_call(const struct mbox_dt_spec *tx_channel,
                   enum mbox_func_call_id api_id,
                   uint32_t parm_num,
                   ...)
{
    __ASSERT_NO_MSG(tx_channel);
    mbox_func_call_data_t mbox_func_call_data = {};
    struct mbox_msg msg = {};
    int ret = 0;

    va_list args;
    va_start(args, parm_num);
    for (uint32_t i = 0; i < parm_num; i++) {
        parm[i] = va_arg(args, void *);
    }
    va_end(args);

    mbox_func_call_data.msg_id = MBOX_FUNC_CALL;
    mbox_func_call_data.api_id = api_id;
    mbox_func_call_data.parm_num = parm_num;
    mbox_func_call_data.parm = parm;

    msg.data = &mbox_func_call_data;
    msg.size = sizeof(mbox_func_call_data_t);

    g_done = false;

    mbox_func_call_state = true;
    if (mbox_send_dt(tx_channel, &msg) == -ENOSPC) {
        while(1);
    }
    k_sem_take(&mbox_func_call_sem, K_FOREVER);

    if (mbox_func_call_data.ret) {
        ret = *((int *)mbox_func_call_data.ret);
    }

    return ret;
}
