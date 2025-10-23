#define DT_DRV_COMPAT linkedsemi_vuart
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "vuart_ls.h"

LOG_MODULE_REGISTER(vuart_ls, CONFIG_UART_LOG_LEVEL);

#define FIFO_QUEUE_COUNT 16

#define IRQ_THREAD_STACK_SIZE 512
K_THREAD_STACK_DEFINE(irq_stack, IRQ_THREAD_STACK_SIZE);

struct vuart_ls_data {
	uart_irq_callback_user_data_t irq_cb;
	void *irq_user_data;
	bool rx_irq_enabled;
	bool tx_irq_enabled;
	ls_vuart_peer_evt_cb_t tx_start_cb;
	void *tx_start_user_data;
	struct k_thread irq_thread;
	struct k_sem irq_sem;
};

struct vuart_ls_config {
	struct k_pipe *h2b_pipe;
	struct k_pipe *b2h_pipe;
};
static void vuart_irq_thread(void *dev_ptr, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)dev_ptr;
	struct vuart_ls_data *ptr_data = dev->data;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		if (k_sem_take(&ptr_data->irq_sem, K_FOREVER) == 0) {
			if (ptr_data->irq_cb) {
				ptr_data->irq_cb(dev, ptr_data->irq_user_data);
			}
		}
	}
}

static void vuart_wakeup_irq_thread(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;
	k_sem_give(&ptr_data->irq_sem);
}

static int vuart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_read;
	int ret;

	ret = k_pipe_get(cfg->h2b_pipe, c, 1, &bytes_read, 1, K_FOREVER);
	if (ret < 0) {
		return -1;
	}

	return 0;
}

static void vuart_poll_out(const struct device *dev, unsigned char c)
{
	struct vuart_ls_data *ptr_data = dev->data;
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_written;

	k_pipe_put(cfg->b2h_pipe, &c, 1, &bytes_written, 1, K_FOREVER);

	if (ptr_data->tx_start_cb && bytes_written > 0) {
		ptr_data->tx_start_cb(ptr_data->tx_start_user_data);
	}
}

static int vuart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	struct vuart_ls_data *ptr_data = dev->data;
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_written;
	int ret;

	ret = k_pipe_put(cfg->b2h_pipe, tx_data, len, &bytes_written, 1, K_NO_WAIT);

	if (bytes_written < len) {
		LOG_WRN("TX pipe full, dropped %d bytes", len - bytes_written);
	}
	if (ptr_data->tx_start_cb && bytes_written > 0) {
		ptr_data->tx_start_cb(ptr_data->tx_start_user_data);
	}

	return bytes_written;
}
// 从中断函数中调用
static int vuart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_read;
	int ret;

	ret = k_pipe_get(cfg->h2b_pipe, rx_data, size, &bytes_read, 1, K_NO_WAIT);
	if (ret < 0) {
		return 0;
	}
	return bytes_read;
}
// rx发送块是否传输完成
static int vuart_ls_irq_tx_complete(const struct device *dev)
{
	const struct vuart_ls_config *cfg = dev->config;
	return k_pipe_read_avail(cfg->b2h_pipe) == 0 ? 1 : 0;
}
static int vuart_ls_irq_tx_ready(const struct device *dev)
{
	return vuart_ls_irq_tx_complete(dev);
}

static int vuart_ls_irq_rx_ready(const struct device *dev)
{
	const struct vuart_ls_config *cfg = dev->config;
	return k_pipe_read_avail(cfg->h2b_pipe) > 0 ? 1 : 0;
}

// 开启tx中断
static void vuart_irq_tx_enable(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = true;
	if (vuart_ls_irq_tx_ready(dev) && ptr_data->irq_cb) {
		vuart_wakeup_irq_thread(dev);
	}
}

static void vuart_irq_tx_disable(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = false;
}

// 开启rx中断
static void vuart_irq_rx_enable(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;
	ptr_data->rx_irq_enabled = true;

	if (vuart_ls_irq_rx_ready(dev) && ptr_data->irq_cb) {
		vuart_wakeup_irq_thread(dev);
	}
}

static void vuart_irq_rx_disable(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;
	ptr_data->rx_irq_enabled = false;
}

static int vuart_irq_is_pending(const struct device *dev)
{
	return (vuart_ls_irq_rx_ready(dev) || vuart_ls_irq_tx_ready(dev)) ? 1 : 0;
}

static int vuart_irq_update(const struct device *dev)
{
	return 1;
}

static void vuart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				   void *user_data)
{
	struct vuart_ls_data *ptr_data = dev->data;
	ptr_data->irq_cb = cb;
	ptr_data->irq_user_data = user_data;
}

static const struct uart_driver_api vuart_api = {
	.poll_in = vuart_poll_in,
	.poll_out = vuart_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = vuart_fifo_fill,
	.fifo_read = vuart_fifo_read,
	.irq_tx_enable = vuart_irq_tx_enable,
	.irq_tx_disable = vuart_irq_tx_disable,
	.irq_rx_enable = vuart_irq_rx_enable,
	.irq_rx_disable = vuart_irq_rx_disable,
	.irq_tx_ready = vuart_ls_irq_tx_ready,
	.irq_rx_ready = vuart_ls_irq_rx_ready,
	.irq_tx_complete = vuart_ls_irq_tx_complete,
	.irq_is_pending = vuart_irq_is_pending,
	.irq_update = vuart_irq_update,
	.irq_callback_set = vuart_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

int ls_vuart_get_tx_char(const struct device *dev, uint8_t *out_char)
{
	struct vuart_ls_data *ptr_data = dev->data;
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_read;
	int ret = k_pipe_get(cfg->b2h_pipe, out_char, 1, &bytes_read, 1, K_NO_WAIT);
	if (ret < 0) {
		return -1;
	}
	if (bytes_read > 0 && ptr_data->tx_irq_enabled) {
		vuart_wakeup_irq_thread(dev);
	}
	return 0;
}

bool host_vuart_rx_available(const struct device *dev)
{
	return !vuart_ls_irq_tx_complete(dev);
}

void ls_vuart_put_rx_char(const struct device *dev, uint8_t c)
{
	struct vuart_ls_data *ptr_data = dev->data;
	const struct vuart_ls_config *cfg = dev->config;
	size_t bytes_written;

	k_pipe_put(cfg->h2b_pipe, &c, 1, &bytes_written, 1, K_NO_WAIT);

	if (bytes_written > 0 && ptr_data->rx_irq_enabled) {
		vuart_wakeup_irq_thread(dev);
	}
}

static int vuart_init(const struct device *dev)
{
	struct vuart_ls_data *ptr_data = dev->data;

	ptr_data->irq_cb = NULL;
	ptr_data->irq_user_data = NULL;
	ptr_data->rx_irq_enabled = false;
	ptr_data->tx_irq_enabled = false;

	ptr_data->tx_start_cb = NULL;
	ptr_data->tx_start_user_data = NULL;

	k_sem_init(&ptr_data->irq_sem, 0, 1);

	k_thread_create(&ptr_data->irq_thread, irq_stack, K_KERNEL_STACK_SIZEOF(irq_stack),
			vuart_irq_thread, (void *)dev, NULL, NULL, -2, 0, K_NO_WAIT);

	return 0;
}

/* 注册 TX 开始回调函数 */
void ls_vuart_register_tx_start_callback(const struct device *vuart,
					 ls_vuart_peer_evt_cb_t callback, void *user_data)
{
	struct vuart_ls_data *ptr_data = vuart->data;
	ptr_data->tx_start_cb = callback;
	ptr_data->tx_start_user_data = user_data;
}

#define VUART_INIT(inst)                                                                           \
	K_PIPE_DEFINE(m_tx_pipe_##inst, FIFO_QUEUE_COUNT, 4);                                      \
	K_PIPE_DEFINE(m_rx_pipe_##inst, FIFO_QUEUE_COUNT, 4);                                      \
	static struct vuart_ls_data vuart_ls_data_##inst;                                          \
	static const struct vuart_ls_config vuart_ls_cfg_##inst = {                                \
		.h2b_pipe = &m_rx_pipe_##inst,                                                     \
		.b2h_pipe = &m_tx_pipe_##inst,                                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &vuart_init, NULL, &vuart_ls_data_##inst,                      \
			      &vuart_ls_cfg_##inst, POST_KERNEL,                                   \
			      CONFIG_VUART_INIT_PRIORITY, &vuart_api);

DT_INST_FOREACH_STATUS_OKAY(VUART_INIT)
