#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mbox.h>
#include <string.h>
#include "espi_lpc_common.h"
#include "uart_agent.h"

#define DT_DRV_COMPAT linkedsemi_uart_agent
LOG_MODULE_REGISTER(uart_agent, LOG_LEVEL_INF);

#define UART_AGENT_RX_CHUNK 64

struct uart_agent_data;

struct uart_agent_backend {
    const struct device *uart;
    struct uart_agent_data *agent;
    uint8_t rx_buf[UART_AGENT_RX_CHUNK];
};

struct uart_agent_data {
    const struct device *dev;
    bool enabled;
    uart_agent_callback_t cb;
    void *user_data;
    struct k_work work;
    struct k_spinlock h2b_lock;
    struct k_spinlock mbox_lock;
    struct uart_agent_backend backends[CONFIG_UART_AGENT_MAX_BACKENDS];
};

struct uart_agent_cfg {
    struct host_vuart_fifo *vuart_fifo_base;
    const struct mbox_dt_spec mbox;
    const struct device *const *uarts;
    uint8_t num_uarts;
};

/*
 * mbox_send_dt() ends up in the shared memory FIFO helpers, which are not
 * reentrant. The senders run from thread context (B_TX_EMPTY, H_RX_AVAIL)
 * and from every backend RX interrupt (B_RX_AVAIL), so serialize them.
 */
static void uart_agent_status_send(struct uart_agent_data *data,
                                   enum vuart_hb_msg_type type)
{
    const struct uart_agent_cfg *cfg = data->dev->config;
    struct vuart_hb_msg hb_msg = { .type = type };
    struct mbox_msg msg = { .data = &hb_msg, .size = sizeof(hb_msg) };
    k_spinlock_key_t key = k_spin_lock(&data->mbox_lock);

    /*
     * -ENOSPC means the peer has not consumed the previous notification yet.
     * Dropping it is harmless: any queued notification makes the peer drain
     * the whole FIFO.
     */
    (void)mbox_send_dt(&cfg->mbox, &msg);
    k_spin_unlock(&data->mbox_lock, key);
}

/*
 * The h2b FIFO is written from the RX interrupt of every backend, so the
 * full check and the put must be done under the same lock.
 */
static void uart_agent_send_char_h2b(struct uart_agent_data *data, uint8_t c)
{
    const struct uart_agent_cfg *cfg = data->dev->config;
    k_spinlock_key_t key = k_spin_lock(&data->h2b_lock);
    bool full = sw_fifo_full(&cfg->vuart_fifo_base->h2b);

    if (full) {
        k_spin_unlock(&data->h2b_lock, key);
        LOG_WRN("h2b fifo full");
        return;
    }

    general_fifo_put(&cfg->vuart_fifo_base->h2b, &c);
    k_spin_unlock(&data->h2b_lock, key);
}

static void uart_callback(const struct device *uart, void *user_data)
{
    struct uart_agent_backend *backend = user_data;
    struct uart_agent_data *data = backend->agent;
    int recv_len;

    uart_irq_update(uart);

    if (!data->enabled) {
        return;
    }

    if (uart_irq_rx_ready(uart)) {
        recv_len = uart_fifo_read(uart, backend->rx_buf,
                                  sizeof(backend->rx_buf));
        if (recv_len > 0) {
            for (int i = 0; i < recv_len; i++) {
                uart_agent_send_char_h2b(data, backend->rx_buf[i]);
            }
            uart_agent_status_send(data, B_RX_AVAIL);
        } else {
            LOG_WRN("RX ready but fifo read returned 0");
        }
    }
}

static void uart_agent_work_handler(struct k_work *work)
{
    struct uart_agent_data *data =
        CONTAINER_OF(work, struct uart_agent_data, work);
    const struct device *agent = data->dev;
    const struct uart_agent_cfg *cfg = agent->config;

    if (!data->enabled) {
        return;
    }

    int count = sw_fifo_element_amount(&cfg->vuart_fifo_base->b2h);

    if (count) {
        for (int i = 0; i < count; i++) {
            char value;

            /*
             * The peer updates the write index while we read it, so the
             * count can still be off; never forward an unconsumed value.
             */
            if (!general_fifo_get(&cfg->vuart_fifo_base->b2h, &value)) {
                break;
            }

            for (int j = 0; j < cfg->num_uarts; j++) {
                uart_poll_out(data->backends[j].uart, value);
            }
        }
        uart_agent_status_send(data, B_TX_EMPTY);
    }
}

static void uart_agent_mbox_callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data, struct mbox_msg *msg)
{
    const struct device *agent = user_data;
    struct uart_agent_data *data = agent->data;
    const struct vuart_hb_msg *hb_msg = msg->data;

    if (hb_msg->type == VUART_MODE_SET) {
        if (data->cb) {
            data->cb(agent, data->user_data);
        }
        return;
    }

    if (!data->enabled) {
        return;
    }

    k_work_submit(&data->work);
}

int uart_agent_start(const struct device *dev)
{
    const struct uart_agent_cfg *cfg = dev->config;
    struct uart_agent_data *data = dev->data;

    if (data->enabled) {
        return -EALREADY;
    }

    data->enabled = true;
    for (int i = 0; i < cfg->num_uarts; i++) {
        const struct device *uart = data->backends[i].uart;

        uart_irq_callback_user_data_set(uart, uart_callback,
                                        &data->backends[i]);
        uart_irq_tx_disable(uart);
        uart_irq_rx_enable(uart);
    }

#ifdef CONFIG_SOC_LSQSH_CPU2
    uart_agent_status_send(data, H_RX_AVAIL);
#endif

    return 0;
}

int uart_agent_stop(const struct device *dev)
{
    const struct uart_agent_cfg *cfg = dev->config;
    struct uart_agent_data *data = dev->data;

    if (!data->enabled) {
        return -EALREADY;
    }

    data->enabled = false;
    k_work_cancel(&data->work);
    for (int i = 0; i < cfg->num_uarts; i++) {
        const struct device *uart = data->backends[i].uart;

        uart_irq_rx_disable(uart);
        uart_irq_tx_disable(uart);
    }

    return 0;
}

void uart_agent_callback_set(const struct device *dev, uart_agent_callback_t cb, void *user_data)
{
    struct uart_agent_data *data = dev->data;
    data->cb = cb;
    data->user_data = user_data;
}

static int ls_uart_agent_init(const struct device *dev)
{
    const struct uart_agent_cfg *cfg = dev->config;
    struct uart_agent_data *data = dev->data;

    data->dev = dev;
    k_work_init(&data->work, uart_agent_work_handler);

    for (int i = 0; i < cfg->num_uarts; i++) {
        if (!device_is_ready(cfg->uarts[i])) {
            LOG_ERR("uart backend %d not ready", i);
            return -ENODEV;
        }
        data->backends[i].uart = cfg->uarts[i];
        data->backends[i].agent = data;
    }

    if (cfg->vuart_fifo_base->init_magic != VUART_FIFO_MAGIC) {
        cfg->vuart_fifo_base->init_magic = VUART_FIFO_MAGIC;
        sw_fifo_init(&cfg->vuart_fifo_base->b2h, cfg->vuart_fifo_base->b2h_buf, VUART_FIFO_SIZE, 1);
        sw_fifo_init(&cfg->vuart_fifo_base->h2b, cfg->vuart_fifo_base->h2b_buf, VUART_FIFO_SIZE, 1);
        cfg->vuart_fifo_base->host_tx_to_vuart = false;
        cfg->vuart_fifo_base->host_rx_from_vuart = false;
    }

    mbox_register_callback_dt(&cfg->mbox, uart_agent_mbox_callback, (void *)dev);
    mbox_set_enabled_dt(&cfg->mbox, true);

    return 0;
}

#define UART_AGENT_DEV(node_id, prop, idx) \
        DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx))

#define LS_UART_AGENT_INIT(inst)                                                      \
    BUILD_ASSERT(DT_INST_PROP_LEN(inst, uart) > 0,                                    \
                 "uart_agent: no uart backend");                                       \
    BUILD_ASSERT(DT_INST_PROP_LEN(inst, uart) <= CONFIG_UART_AGENT_MAX_BACKENDS,      \
                 "uart_agent: too many uart backends, raise "                         \
                 "CONFIG_UART_AGENT_MAX_BACKENDS");                                   \
    static const struct device *const uart_agent_uarts##inst[] = {                    \
        DT_INST_FOREACH_PROP_ELEM_SEP(inst, uart, UART_AGENT_DEV, (,))                \
    };                                                                                \
    static struct uart_agent_data uart_agent_data##inst;                              \
    static const struct uart_agent_cfg uart_agent_cfg##inst = {                       \
        .vuart_fifo_base = (struct host_vuart_fifo *)DT_INST_PROP(inst, fifo_base),   \
        .mbox = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(inst, mbox), mbox),                  \
        .uarts = uart_agent_uarts##inst,                                              \
        .num_uarts = ARRAY_SIZE(uart_agent_uarts##inst),                              \
    };                                                                                \
    DEVICE_DT_INST_DEFINE(inst, ls_uart_agent_init, NULL, &uart_agent_data##inst,     \
                  &uart_agent_cfg##inst, POST_KERNEL,                                 \
                  CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(LS_UART_AGENT_INIT)
