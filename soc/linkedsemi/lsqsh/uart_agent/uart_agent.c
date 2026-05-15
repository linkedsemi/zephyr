#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include "espi_lpc_common.h"

#define DT_DRV_COMPAT linkedsemi_uart_agent
LOG_MODULE_REGISTER(uart_agent, LOG_LEVEL_INF);

struct uart_agent_data {
    atomic_t owner_is_peer;
    struct k_work switch_local;
    struct k_work switch_peer;
};

struct uart_agent_cfg {
    struct host_vuart_fifo *vuart_fifo_base;
    const struct mbox_dt_spec mbox_tx;
    const struct mbox_dt_spec mbox_rx;
    const struct device *uart;
};

static void uart_agent_status_send(const struct mbox_dt_spec *mbox, enum vuart_hb_msg_type type)
{
    struct vuart_hb_msg hb_msg = {
        .type = type
    };

    struct mbox_msg msg = {
        .data = &hb_msg,
        .size = sizeof(hb_msg)
    };

    mbox_send_dt(mbox, &msg);
}

static void uart_callback(const struct device *dev, void *user_data);

static int uart_switch_local(const struct device *dev, const struct shell *sh)
{
    struct uart_agent_data *data = dev->data;
    const struct uart_agent_cfg *cfg = dev->config;
    int err;

    if (!atomic_get(&data->owner_is_peer)) {
        return -EALREADY;
    }

    uart_irq_rx_disable(cfg->uart);
    uart_irq_tx_disable(cfg->uart);

    err = shell_backend_uart_resume(sh);
    if (err != 0) {
        atomic_set(&data->owner_is_peer, 1);
        uart_irq_callback_user_data_set(cfg->uart, uart_callback, (void *)dev);
        uart_irq_rx_enable(cfg->uart);
        return err;
    }
    err = shell_start(sh);
    atomic_set(&data->owner_is_peer, 0);

    return err;
}

static void switch_local_work_handler(struct k_work *work)
{
    const struct device *dev = DEVICE_DT_GET_ONE(DT_DRV_COMPAT);
    const struct shell *sh = shell_backend_uart_get_ptr();
    uart_switch_local(dev, sh);
}

static void uart_agent_mbox_callback(const struct device *dev, mbox_channel_id_t channel_id,
    void *user_data, struct mbox_msg *msg)
{
    const struct uart_agent_cfg *config = ((const struct device *)user_data)->config;
    struct uart_agent_data *data = ((const struct device *)user_data)->data;
    const struct vuart_hb_msg *hb_msg = msg->data;

    if (hb_msg->type != VUART_MODE_SET) {
        if (!atomic_get(&data->owner_is_peer)) {
            LOG_ERR("current uart owner is local core!!!\n");
            return;
        }
        int count = sw_fifo_element_amount(&config->vuart_fifo_base->b2h);
        if (count) {
            for (int i = 0; i < count; i++) {
                char value;
                general_fifo_get(&config->vuart_fifo_base->b2h, &value);
                uart_poll_out(config->uart, value);
            }
            uart_agent_status_send(&config->mbox_tx, B_TX_EMPTY);
        }
    } else {
        k_work_submit(&data->switch_local);
    }
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
    int recv_len;
    const struct device *uart_agent = user_data;
    const struct uart_agent_cfg *config = uart_agent->config;
    struct uart_agent_data *data = uart_agent->data;

    uart_irq_update(dev);

    if (!atomic_get(&data->owner_is_peer)) {
        LOG_ERR("current uart owneris app core!!!\n");
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        recv_len = uart_fifo_read(dev, buffer, 64);
        if (recv_len > 0) {
            for (int i = 0; i < recv_len; i++) {
                uart_agent_send_char_h2b(uart_agent, buffer[i]);
            }
            uart_agent_status_send(&config->mbox_tx, B_RX_AVAIL);
        } else {
            LOG_WRN("RX ready but fifo read returned 0");
        }
    }
}

static int uart_switch_peer(const struct device *dev, const struct shell *sh)
{
    const struct uart_agent_cfg *cfg = dev->config;
    struct uart_agent_data *data = dev->data;
    int err;

    if (atomic_get(&data->owner_is_peer)) {
        return -EALREADY;
    }

    shell_stop(sh);

    err = shell_backend_uart_suspend(sh);
    if (err != 0) {
        (void)shell_start(sh);
        return err;
    }
    atomic_set(&data->owner_is_peer, 1);

    uart_irq_callback_user_data_set(cfg->uart, uart_callback, (void *)dev);
    uart_irq_tx_disable(cfg->uart);
    uart_irq_rx_enable(cfg->uart);

#ifdef CONFIG_SOC_LSQSH_CPU2
    uart_agent_status_send(&cfg->mbox_tx, H_RX_AVAIL);
#endif

    return 0;
}

static void switch_peer_work_handler(struct k_work *work)
{
    const struct device *dev = DEVICE_DT_GET_ONE(DT_DRV_COMPAT);
    const struct shell *sh = shell_backend_uart_get_ptr();
    k_msleep(10);
    uart_switch_peer(dev, sh);
}

static int ls_uart_agent_init(const struct device *dev)
{
    const struct uart_agent_cfg *cfg = dev->config;
    struct uart_agent_data *data = dev->data;

    atomic_set(&data->owner_is_peer, 0);

    if (cfg->vuart_fifo_base->b2h.buf == NULL) {
        sw_fifo_init(&cfg->vuart_fifo_base->b2h, cfg->vuart_fifo_base->b2h_buf, VUART_FIFO_SIZE, 1);
        sw_fifo_init(&cfg->vuart_fifo_base->h2b, cfg->vuart_fifo_base->h2b_buf, VUART_FIFO_SIZE, 1);
        cfg->vuart_fifo_base->host_tx_to_vuart = false;
        cfg->vuart_fifo_base->host_rx_from_vuart = false;
    }

    mbox_set_enabled_dt(&cfg->mbox_tx, true);
    mbox_register_callback_dt(&cfg->mbox_rx, uart_agent_mbox_callback, (void *)dev);
    mbox_set_enabled_dt(&cfg->mbox_rx, true);

    k_work_init(&data->switch_local, switch_local_work_handler);
    k_work_init(&data->switch_peer, switch_peer_work_handler);

    return 0;
}

static int cmd_uart_switch_peer(const struct shell *sh, size_t argc, char **argv)
{
    if (sh != shell_backend_uart_get_ptr()) {
        return -EINVAL;
    }
    const struct device *dev = DEVICE_DT_GET_ONE(DT_DRV_COMPAT);
    struct uart_agent_data *data = dev->data;

    k_work_submit(&data->switch_peer);
    return 0;
}
SHELL_CMD_REGISTER(uart_switch_sec, NULL, "Switch uart to peer core", cmd_uart_switch_peer);

#define LS_UART_AGENT_INIT(inst)                                                         \
    static struct uart_agent_data uart_agent_data##inst;                                 \
    static const struct uart_agent_cfg uart_agent_cfg##inst = {                          \
        .vuart_fifo_base = (struct host_vuart_fifo *)DT_INST_PROP(inst, fifo_base),      \
        .mbox_tx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(inst, mbox), tx),                    \
        .mbox_rx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(inst, mbox), rx),                    \
        .uart = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart)),                              \
    };                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, ls_uart_agent_init, NULL, &uart_agent_data##inst,        \
                  &uart_agent_cfg##inst, POST_KERNEL,                                    \
                  CONFIG_APPLICATION_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(LS_UART_AGENT_INIT)
