#define DT_DRV_COMPAT linkedsemi_ls_vuart
#define VUART_DEV DEVICE_DT_GET_ONE(DT_DRV_COMPAT)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "espi_lpc_common.h"

#define VUART_RX_COUNT 7
#define VUART_RX_TIMEOUT_MS 4

#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>

LOG_MODULE_REGISTER(linkedsemi_ls_vuart, CONFIG_UART_LOG_LEVEL);

/* eSPI clock — needed to access shared fifo_base in eSPI register space */
#define ESPI_NODE DT_NODELABEL(espi1)


struct ls_vuart_cfg {
    struct host_bmc_msg_exch hb_exch;
	const char *vuart_irq_thread_name;
    void *irq_thread_stack;
    uint32_t irq_thread_stack_size;
    struct host_vuart_fifo *vuart_fifo_base;
};

struct ls_vuart_data {
    uart_irq_callback_user_data_t irq_cb;
    void *irq_user_data;
    struct k_thread irq_thread;
    struct k_sem irq_sem;
    struct k_timer rx_timer;
    bool rx_irq_enabled;
    bool tx_irq_enabled;
    bool peer_rx_valid;
};

static inline void vuart_local_wakeup_irq_thread(const struct device *dev)
{
    struct ls_vuart_data *data = dev->data;
    k_sem_give(&data->irq_sem);
}


static void vuart_rx_timer(struct k_timer *timer_id)
{
    const struct device *dev = k_timer_user_data_get(timer_id);
    const struct ls_vuart_cfg *cfg = dev->config;
    if (sw_fifo_element_amount(&cfg->vuart_fifo_base->h2b) > 0) {
        vuart_local_wakeup_irq_thread(dev);
    }
}



void bmc_vuart_rx_callback(const struct device *dev, void *msg)
{
    struct ls_vuart_data *data = dev->data;
    const struct ls_vuart_cfg *cfg = dev->config;
    struct vuart_hb_msg *vuart_msg = msg;

#ifdef CONFIG_SOC_LSQSH_CPU1
    struct ls_vuart_data *dev_data = dev->data;
    if (vuart_msg->type == H_RX_AVAIL) {
        dev_data->peer_rx_valid = true;
    }
#endif

    if(vuart_msg->type==B_TX_EMPTY)
    {
        vuart_local_wakeup_irq_thread(dev);
    }
    else 
    { 
        if (k_timer_remaining_get(&data->rx_timer) == 0) {
            k_timer_start(&data->rx_timer, K_MSEC(VUART_RX_TIMEOUT_MS), K_NO_WAIT);
        }    
        if (sw_fifo_element_amount(&cfg->vuart_fifo_base->h2b) > VUART_RX_COUNT) { 
            vuart_local_wakeup_irq_thread(dev);
            k_timer_stop(&data->rx_timer);
        }
    }

}

static void vuart_irq_thread(void *dev_ptr, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)dev_ptr;
	struct ls_vuart_data *ptr_data = dev->data;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
      if(k_sem_take(&ptr_data->irq_sem,K_FOREVER) == 0)
      {
        if(ptr_data->irq_cb)
        {
            ptr_data->irq_cb(dev,ptr_data->irq_user_data);
        }
      }
    }
}

static int vuart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct ls_vuart_cfg *cfg = dev->config;
    if(!general_fifo_get(&cfg->vuart_fifo_base->h2b,c))
    {
        vuart_status_send(&cfg->hb_exch,H_TX_EMPTY);
        return 0;
    }
    return -1;
}

static int vuart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct ls_vuart_cfg *cfg = dev->config;
    int i;
    for(i = 0;i<size;++i)
    {
        if(!general_fifo_get(&cfg->vuart_fifo_base->h2b,rx_data++))
        {
            vuart_status_send(&cfg->hb_exch,H_TX_EMPTY);
            break;
        }
    }
	return i;
}

static void vuart_poll_out(const struct device *dev, unsigned char c)
{
    const struct ls_vuart_cfg *cfg = dev->config;
    struct ls_vuart_data *data = dev->data;

    if (data->peer_rx_valid == false)
        return;

    while (1) {
        if (general_fifo_put(&cfg->vuart_fifo_base->b2h, &c) == true) {
            vuart_status_send(&cfg->hb_exch, H_RX_AVAIL);
            return;
        }
    }
}

static int vuart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	const struct ls_vuart_cfg *cfg = dev->config;
    struct ls_vuart_data *data = dev->data;

    int length = len;

    if (data->peer_rx_valid == false) {
        vuart_local_wakeup_irq_thread(dev);
        return length;
    }

    while(length--)
    {
        general_fifo_put(&cfg->vuart_fifo_base->b2h,(void *)tx_data++);
    }
    vuart_status_send(&cfg->hb_exch,H_RX_AVAIL);
	return len;
}

static void vuart_irq_tx_enable(const struct device *dev)
{
	struct ls_vuart_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = true;
	if (uart_irq_tx_ready(dev)) {
		vuart_local_wakeup_irq_thread(dev);
	}
}

static void vuart_irq_tx_disable(const struct device *dev)
{
	struct ls_vuart_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = false;
}

static void vuart_irq_rx_enable(const struct device *dev)
{
	struct ls_vuart_data *ptr_data = dev->data;
	ptr_data->rx_irq_enabled = true;

	if (uart_irq_rx_ready(dev)) {
		vuart_local_wakeup_irq_thread(dev);
	}
}

static int vuart_irq_tx_complete(const struct device *dev)
{
	const struct ls_vuart_cfg *cfg = dev->config;
	return sw_fifo_empty(&cfg->vuart_fifo_base->b2h);
}

static int vuart_irq_tx_ready(const struct device *dev)
{
	return vuart_irq_tx_complete(dev);
}

static int vuart_irq_rx_ready(const struct device *dev)
{
	const struct ls_vuart_cfg *cfg = dev->config;
	return !sw_fifo_empty(&cfg->vuart_fifo_base->h2b);

}

static void vuart_irq_rx_disable(const struct device *dev)
{
	struct ls_vuart_data *ptr_data = dev->data;
	ptr_data->rx_irq_enabled = false;
}

static int vuart_irq_is_pending(const struct device *dev)
{
	return (uart_irq_rx_ready(dev) || uart_irq_tx_ready(dev)) ? 1 : 0;
}

static int vuart_irq_update(const struct device *dev)
{
	return 1;
}

void vuart_ls_send_break(const struct device *dev)
{
    const struct ls_vuart_cfg *cfg = dev->config;
    vuart_status_send(&cfg->hb_exch, VUART_SEND_BREAK);
}

int get_host_vuart_mode_setting(const struct device *dev, uint8_t *rx_enable, uint8_t *tx_enable)
{
    if (rx_enable == NULL || tx_enable == NULL) {
        return -EFAULT; 
    }

    const struct ls_vuart_cfg *cfg = dev->config;
    if (cfg->vuart_fifo_base == NULL) {
        return -EIO; 
    }

    *rx_enable = (cfg->vuart_fifo_base->host_rx_from_vuart) ? 1 : 0;
    *tx_enable = (cfg->vuart_fifo_base->host_tx_to_vuart) ? 1 : 0;

    return 0;
}
static void vuart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				   void *user_data)
{
	struct ls_vuart_data *ptr_data = dev->data;
	ptr_data->irq_cb = cb;
	ptr_data->irq_user_data = user_data;
}

static const struct uart_driver_api ls_vuart_api = {
    .poll_in = vuart_poll_in,
    .poll_out = vuart_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = vuart_fifo_fill,
    .fifo_read = vuart_fifo_read,
    .irq_tx_enable = vuart_irq_tx_enable,
    .irq_tx_disable = vuart_irq_tx_disable,
    .irq_rx_enable = vuart_irq_rx_enable,
    .irq_rx_disable = vuart_irq_rx_disable,
    .irq_tx_ready = vuart_irq_tx_ready,
    .irq_rx_ready = vuart_irq_rx_ready,
    .irq_tx_complete = vuart_irq_tx_complete,
    .irq_is_pending = vuart_irq_is_pending,
    .irq_update = vuart_irq_update,
    .irq_callback_set = vuart_irq_callback_set,
#endif
};

static int ls_vuart_init(const struct device *dev)
{
    const struct ls_vuart_cfg *cfg = dev->config;
    struct ls_vuart_data *ptr_data = dev->data;

#ifdef CONFIG_SOC_LSQSH_CPU1

#if defined(CONFIG_CLOCK_CONTROL)
    const struct ls_clk_cfg espi_ccfg = {
        .cctl_dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(ESPI_NODE, clocks, 0)),
        .cctl_addr_offest = DT_PHA(ESPI_NODE, clocks, cctl_addr_offest),
        .set_bit  = DT_PHA(ESPI_NODE, clocks, set_bit),
        .clr_bit  = DT_PHA(ESPI_NODE, clocks, clr_bit),
    };

    if (espi_ccfg.cctl_dev && device_is_ready(espi_ccfg.cctl_dev)) {
        if (clock_control_get_status(espi_ccfg.cctl_dev, (clock_control_subsys_t)&espi_ccfg) == CLOCK_CONTROL_STATUS_OFF) {
            clock_control_on(espi_ccfg.cctl_dev, (clock_control_subsys_t)&espi_ccfg);
        }
    }
#endif

    if (cfg->vuart_fifo_base->init_magic != VUART_FIFO_MAGIC) {
        cfg->vuart_fifo_base->init_magic = VUART_FIFO_MAGIC;
        sw_fifo_init(&cfg->vuart_fifo_base->b2h, cfg->vuart_fifo_base->b2h_buf, VUART_FIFO_SIZE, 1);
        sw_fifo_init(&cfg->vuart_fifo_base->h2b, cfg->vuart_fifo_base->h2b_buf, VUART_FIFO_SIZE, 1);
        cfg->vuart_fifo_base->host_tx_to_vuart = false;
        cfg->vuart_fifo_base->host_rx_from_vuart = false;
    }

    ptr_data->peer_rx_valid = false;
#elif CONFIG_SOC_LSQSH_CPU2
    ptr_data->peer_rx_valid = true;
#endif

    k_timer_init(&ptr_data->rx_timer, vuart_rx_timer, NULL);
    k_timer_user_data_set(&ptr_data->rx_timer, (void *)dev);
    k_sem_init(&ptr_data->irq_sem, 0, 1);
    k_thread_create(&ptr_data->irq_thread, cfg->irq_thread_stack, cfg->irq_thread_stack_size,
      vuart_irq_thread, (void *)dev, NULL, NULL, CONFIG_VUART_IRQ_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ptr_data->irq_thread, cfg->vuart_irq_thread_name);
    host_bmc_msg_exch_init(&cfg->hb_exch);
    return 0;
}

#define LS_VUART_INIT(inst)                                                     \
	static struct ls_vuart_data ls_vuart_data_##inst;                                   \
    K_KERNEL_STACK_DEFINE(vuart_irq_thread_stack_##inst, CONFIG_VUART_IRQ_THREAD_STACK_SIZE);\
    static const struct ls_vuart_cfg ls_vuart_cfg_##inst = {                            \
		.hb_exch = HOST_BMC_MSG_EXCH_INIT(inst,bmc_vuart_rx_callback,host_vuart_rx_callback),\
		.vuart_irq_thread_name = "vuart_irq_thread_" #inst,                          \
        .irq_thread_stack = vuart_irq_thread_stack_##inst,                             \
        .irq_thread_stack_size = K_KERNEL_STACK_SIZEOF(vuart_irq_thread_stack_##inst),\
        .vuart_fifo_base = (struct host_vuart_fifo *)DT_INST_PROP(inst,fifo_base),      \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(inst,                                                         \
			      ls_vuart_init,                                           \
			      NULL,                                                        \
			      &ls_vuart_data_##inst,                                         \
			      &ls_vuart_cfg_##inst,                                          \
			      POST_KERNEL,                                                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                            \
			      &ls_vuart_api);

DT_INST_FOREACH_STATUS_OKAY(LS_VUART_INIT)

#if defined(CONFIG_ESPI_LPC_MBOX) && defined(CONFIG_SOC_LSQSH_CPU1)
static int cmd_uart_switch_app(const struct shell *sh, size_t argc, char **argv)
{
    const struct device *dev = VUART_DEV;
    const struct ls_vuart_cfg *cfg = dev->config;
    struct ls_vuart_data *data = dev->data;

    if (!device_is_ready(dev)) {
        shell_error(sh, "vuart not ready");
        return -ENODEV;
    }
    data->peer_rx_valid = false;
    vuart_b2h_mode_set(&cfg->hb_exch, true, true);
    return 0;
}
SHELL_CMD_REGISTER(uart_switch_app, NULL, "Switch uart to peer core", cmd_uart_switch_app);
#endif

#ifdef CONFIG_ESPI_LPC_MBOX
void host_vuart_mode_set(const struct device *dev,bool host_rx_from_vuart,bool host_tx_to_vuart)
{
    const struct ls_vuart_cfg *cfg = dev->config;
    vuart_b2h_mode_set(&cfg->hb_exch,host_rx_from_vuart,host_tx_to_vuart);
}

static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
    bool host_rx_from_vuart,host_tx_to_vuart;
    if (argc < 3) {
        shell_error(sh, "Usage: %s <host_rx_from_vuart:true|false> <host_tx_to_vuart:true|false>" ,argv[0]);
        return -EINVAL;
    }

    const struct device *dev = VUART_DEV;
    if (!device_is_ready(dev)) {
        shell_error(sh, "ls-host-vuart device not ready");
        return -ENODEV;
    }

    host_rx_from_vuart = (strcmp(argv[1], "true") == 0);
    host_tx_to_vuart   = (strcmp(argv[2], "true") == 0);

    host_vuart_mode_set(dev,host_rx_from_vuart,host_tx_to_vuart);

    shell_print(sh, "VUART mode set: host_rx_from_vuart=%s, host_tx_to_vuart=%s",
                host_rx_from_vuart ? "true" : "false",
                host_tx_to_vuart ? "true" : "false");
    return 0;
}

SHELL_CMD_ARG_REGISTER(chmode, NULL, "chmod usage: <host_rx_from_vuart:true|false> <host_tx_to_vuart:true|false>",  cmd_mode,  0, 3);
#endif
static int print_mode(const struct shell *sh, size_t argc, char **argv)
{
    const struct device *dev = VUART_DEV;
    if (!device_is_ready(dev)) {
        shell_error(sh, "ls-host-vuart device not ready");
        return -ENODEV;
    }
    
    uint8_t rx_enable , tx_enable = 0;
    int ret;
    ret = get_host_vuart_mode_setting(dev, &rx_enable, &tx_enable);
    if (ret != 0) {
        shell_error(sh, "Failed to get VUART mode, ret: %d", ret);
        return ret;
    }

    shell_print(sh, "VUART Mode: RX Enable = %d, TX Enable = %d", rx_enable, tx_enable);
    return 0;
}

SHELL_CMD_ARG_REGISTER(printmode, NULL, "printmode usage: show <host_rx_from_vuart:true|false> <host_tx_to_vuart:true|false>",  print_mode,  1, 0);