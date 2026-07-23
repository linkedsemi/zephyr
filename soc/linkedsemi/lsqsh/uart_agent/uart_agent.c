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

struct uart_agent_data {
    const struct device *dev;
    bool enabled;
    uart_agent_callback_t cb;
    void *user_data;
    struct k_work work;
};

struct uart_agent_cfg {
    struct host_vuart_fifo *vuart_fifo_base;
    const struct mbox_dt_spec mbox_tx;
    const struct mbox_dt_spec mbox_rx;
    const struct device *uart;
};

static void uart_agent_status_send(const struct mbox_dt_spec *mbox,
                                   enum vuart_hb_msg_type type)
{
    struct vuart_hb_msg hb_msg = { .type = type };
    struct mbox_msg msg = { .data = &hb_msg, .size = sizeof(hb_msg) };
    mbox_send_dt(mbox, &msg);
}

static void uart_agent_send_char_h2b(const struct device *dev, uint8_t c)
{
    const struct uart_agent_cfg *cfg = dev->config;

    if (!sw_fifo_full(&cfg->vuart_fifo_base->h2b)) {
        general_fifo_put(&cfg->vuart_fifo_base->h2b, &c);
    } else {
        LOG_WRN("h2b fifo full");
    }
}

static void uart_callback(const struct device *dev, void *user_data)
{
    static uint8_t buffer[64];
    const struct device *agent = user_data;
    const struct uart_agent_cfg *cfg = agent->config;
    struct uart_agent_data *data = agent->data;
    int recv_len;

    uart_irq_update(dev);

    if (!data->enabled) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        recv_len = uart_fifo_read(dev, buffer, sizeof(buffer));
        if (recv_len > 0) {
            for (int i = 0; i < recv_len; i++) {
                uart_agent_send_char_h2b(agent, buffer[i]);
            }
            uart_agent_status_send(&cfg->mbox_tx, B_RX_AVAIL);
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

            general_fifo_get(&cfg->vuart_fifo_base->b2h, &value);
            uart_poll_out(cfg->uart, value);
        }
        uart_agent_status_send(&cfg->mbox_tx, B_TX_EMPTY);
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
    uart_irq_callback_user_data_set(cfg->uart, uart_callback, (void *)dev);
    uart_irq_tx_disable(cfg->uart);
    uart_irq_rx_enable(cfg->uart);

#ifdef CONFIG_SOC_LSQSH_CPU2
    uart_agent_status_send(&cfg->mbox_tx, H_RX_AVAIL);
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
    uart_irq_rx_disable(cfg->uart);
    uart_irq_tx_disable(cfg->uart);

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

    if (cfg->vuart_fifo_base->init_magic != VUART_FIFO_MAGIC) {
        cfg->vuart_fifo_base->init_magic = VUART_FIFO_MAGIC;
        sw_fifo_init(&cfg->vuart_fifo_base->b2h, cfg->vuart_fifo_base->b2h_buf, VUART_FIFO_SIZE, 1);
        sw_fifo_init(&cfg->vuart_fifo_base->h2b, cfg->vuart_fifo_base->h2b_buf, VUART_FIFO_SIZE, 1);
        cfg->vuart_fifo_base->host_tx_to_vuart = false;
        cfg->vuart_fifo_base->host_rx_from_vuart = false;
    }

    mbox_set_enabled_dt(&cfg->mbox_tx, true);
    mbox_register_callback_dt(&cfg->mbox_rx, uart_agent_mbox_callback, (void *)dev);
    mbox_set_enabled_dt(&cfg->mbox_rx, true);

    return 0;
}

#define LS_UART_AGENT_INIT(inst)                                                      \
    static struct uart_agent_data uart_agent_data##inst;                              \
    static const struct uart_agent_cfg uart_agent_cfg##inst = {                       \
        .vuart_fifo_base = (struct host_vuart_fifo *)DT_INST_PROP(inst, fifo_base),   \
        .mbox_tx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(inst, mbox), tx),                 \
        .mbox_rx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(inst, mbox), rx),                 \
        .uart = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart)),                           \
    };                                                                                \
    DEVICE_DT_INST_DEFINE(inst, ls_uart_agent_init, NULL, &uart_agent_data##inst,     \
                  &uart_agent_cfg##inst, POST_KERNEL,                                 \
                  CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(LS_UART_AGENT_INIT)
