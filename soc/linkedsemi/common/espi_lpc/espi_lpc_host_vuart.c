#define DT_DRV_COMPAT linkedsemi_ls_host_vuart
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <espi_lpc_common.h>

#include "cpu.h"
#include "reg_dwuart_type.h"

typedef void (*irq_cfg_func_t)(const struct device *dev);

LOG_MODULE_REGISTER(ls_host_vuart, CONFIG_ESPI_LOG_LEVEL);

#define VUART_FIFO_SIZE 16
#define PORT_NUM   8
#define IER_RDA    0x01
#define IER_THRE   0x02
#define LCR_DLAB   0x80
#define LCR_DLS    0x3
#define IIR_NOPEND 0x01
#define IIR_THRE   0x02
#define IIR_RDA    0x04
#define FCR_FIFO    0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */
#define LSR_DR     0x01
#define LSR_THRE   0x20
#define LSR_TEMT   0x40
#define LSR_RESET_VAL 0x60

struct host_vuart_cfg {
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    const struct device *parent; /* eSPI/LPC 控制器 */
    const struct peri_ioport_content *ioport_content;
    const struct upstream_irq_type *up_irq;
    const char*  vuart_irq_thread_name;
    struct k_pipe *h2b_pipe;
    struct k_pipe *b2h_pipe;
    irq_cfg_func_t irq_config_func;
    reg_dwuart_t *reg;
    uint32_t clock_source;
    uint32_t baudrate;
    uint16_t local_irq;
};

struct host_vuart_reg {
  uint8_t dll;
  uint8_t dlh;
  uint8_t ier;
  uint8_t fcr;
  uint8_t lcr;
  uint8_t mcr;
  uint8_t scr;
};

struct host_vuart_data {
  struct peri_ioport ioport[PORT_NUM];
  uart_irq_callback_user_data_t irq_cb;
  void *irq_user_data;
  K_KERNEL_STACK_MEMBER(irq_thread_stack, CONFIG_VUART_IRQ_THREAD_STACK_SIZE);
  struct k_thread irq_thread;
  struct k_sem irq_sem;
  struct host_vuart_reg data_reg;
  struct k_spinlock lock;
  bool rx_irq_enabled;
  bool tx_irq_enabled;
  bool host_rx_from_vuart;
  bool host_tx_to_vuart;
};



static inline void vuart_wakeup_irq_thread(const struct device *dev)
{
	struct host_vuart_data *ptr_data = dev->data;
	k_sem_give(&ptr_data->irq_sem);
}

static inline bool host_vuart_rx_available(const struct device *dev)
{
	return !uart_irq_tx_complete(dev);
}

static inline bool host_vuart_tx_empty(const struct device *dev)
{
	return !uart_irq_rx_ready(dev);
}

static uint8_t host_vuart_calc_iir(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&ptr_data->lock);
    uint8_t iir = (ptr_data->data_reg.fcr & 0x1) ? 0xC0 : 0x00;
    if (ptr_data->data_reg.ier & 0x01) {
        if(ptr_data->host_rx_from_vuart) {
            if(host_vuart_rx_available(dev)) {
                iir |= IIR_RDA;
            }
        }else {
            if(cfg->reg->LSR & LSR_DR) {
                 iir |= IIR_RDA;
            }
        }
    } else if ((ptr_data->data_reg.ier & 0x02)) {
        iir |= IIR_THRE;
    } else {
        iir |= IIR_NOPEND;
    }
    k_spin_unlock(&ptr_data->lock,key);
    return iir;
}

static void host_vuart_report_active_edge_level_up_irq(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;

    if ((host_vuart_calc_iir(dev) & 0xf)  == IIR_NOPEND) {
        return;
    }

    if (cfg->up_irq) {
        espi_lpc_raise_edge_irq(cfg->parent, cfg->up_irq->idx);
    }
}

static void local_irq_state_update(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    if (cfg->up_irq == NULL) {
        return;
    }
    bool masked,pending;
    masked = irq_is_enabled(cfg->local_irq) ? true : false;
    if (!masked) {
        return;
    }
    clr_pending_irq(cfg->local_irq);
    pending = get_pending_irq(cfg->local_irq) ? true : false;
    if (!pending) {
        irq_enable(cfg->local_irq);
    }
}

static void host_vuart_local_isr(const void *arg)
{
    const struct device *dev = (const struct device *)arg;
    const struct host_vuart_cfg *cfg = dev->config;
    irq_disable(cfg->local_irq);
    if (!cfg->up_irq) {
        return;
    }
    espi_lpc_raise_edge_irq(cfg->parent, cfg->up_irq->idx);
}

int vuart_get_char_b2h(const struct device *dev, uint8_t *out_char)
{
	struct host_vuart_data *ptr_data = dev->data;
	const struct host_vuart_cfg *cfg = dev->config;
	size_t bytes_read;
	int ret = k_pipe_get(cfg->b2h_pipe, out_char, 1, &bytes_read, 1, K_NO_WAIT);
	if (ret < 0) {
		return -1;
	}
	if (bytes_read > 0 && ptr_data->tx_irq_enabled) {
        if (uart_irq_tx_ready(dev)) {
		    vuart_wakeup_irq_thread(dev);
    	}
	}
	return 0;
}


static void host_vuart_reg0_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        *val = ptr_data->data_reg.dll;
    } else{
        uint8_t val1,val2=0;
        val1 = cfg->reg->RBR_THR_DLL;
        if (host_vuart_rx_available(dev)) {
            vuart_get_char_b2h(dev, &val2);
        }
        *val = ptr_data->host_rx_from_vuart?val2:val1;
        local_irq_state_update(dev);
    }
}

static void vuart_send_char_h2b(const struct device *dev, uint8_t c)
{
	struct host_vuart_data *ptr_data = dev->data;
	const struct host_vuart_cfg *cfg = dev->config;
	size_t bytes_written;
    if(ptr_data->host_tx_to_vuart) {

        k_pipe_put(cfg->h2b_pipe, &c, 1, &bytes_written, 1, K_NO_WAIT);

        if (bytes_written > 0 && ptr_data->rx_irq_enabled) {
            vuart_wakeup_irq_thread(dev);
        }
    }
}

static void host_vuart_reg0_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        ptr_data->data_reg.dll = *val;
    } else {
        cfg->reg->RBR_THR_DLL = *val;
        vuart_send_char_h2b(dev,*val);
        local_irq_state_update(dev);
    }
}

static void host_vuart_reg1_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        *val = ptr_data->data_reg.dlh;
    } else {
        *val = ptr_data->data_reg.ier;
    }
}


static void host_vuart_reg1_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    const struct host_vuart_cfg *cfg = dev->config;
    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        ptr_data->data_reg.dlh = *val;
    } else {
        ptr_data->data_reg.ier = *val;
        if(ptr_data->host_rx_from_vuart) {
            cfg->reg->DLH_IER = *val & IER_THRE;
        }else {
            cfg->reg->DLH_IER = *val & (IER_THRE|IER_RDA);
        }
        host_vuart_report_active_edge_level_up_irq(dev);
        local_irq_state_update(dev);
    }
}

static void host_vuart_reg2_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    uint8_t *val = (uint8_t *)res;
    *val = host_vuart_calc_iir(dev);
}

static void host_vuart_reg2_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    ptr_data->data_reg.fcr = *val;
}

static void host_vuart_reg3_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    *val = ptr_data->data_reg.lcr;
}

static void host_vuart_reg3_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    ptr_data->data_reg.lcr = *val;
}

static void host_vuart_reg4_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    *val = ptr_data->data_reg.mcr;
}

static void host_vuart_reg4_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    ptr_data->data_reg.mcr = *val;
}

static void host_vuart_reg5_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    uint8_t lsr = cfg->reg->LSR;
    *val = 0;
    if ((lsr & (LSR_THRE | LSR_TEMT)) == (LSR_THRE | LSR_TEMT))
    {
        if((ptr_data->host_tx_to_vuart && host_vuart_tx_empty(dev) )|| !ptr_data->host_tx_to_vuart)
        {
            *val |= LSR_THRE|LSR_TEMT;
        }
    }
    if (ptr_data->host_rx_from_vuart) {
        if(host_vuart_rx_available(dev)) {
            *val |= LSR_DR;
        }
    }else {
        if(lsr & LSR_DR) {
            *val |= LSR_DR;
        }
    }
}
static void host_vuart_reg5_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
	__ASSERT_NO_MSG(false);
}

static void host_vuart_reg6_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    uint8_t *val = (uint8_t *)res;
    *val = 0x00; /* 调制解调器状态寄存器 */
}

static void host_vuart_reg6_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	__ASSERT_NO_MSG(false);
}

static void host_vuart_reg7_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    *val = ptr_data->data_reg.scr;
}

static void host_vuart_reg7_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    struct host_vuart_data *ptr_data = dev->data;
    ptr_data->data_reg.scr = *val;
}


static void vuart_irq_thread(void *dev_ptr, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)dev_ptr;
	struct host_vuart_data *ptr_data = dev->data;

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


static int host_vuart_init(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint32_t pclk = 75000000;
    if (!device_is_ready(cfg->parent) ) {
        LOG_ERR("parent not ready");
        return -ENODEV;
	}
    int ret = 0;
#if defined(CONFIG_CLOCK_CONTROL)
    if (cfg->ccfg.cctl_dev) {
        const struct device *clk_dev = cfg->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&cfg->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (cfg->reset.dev != NULL) {
        if (!device_is_ready(cfg->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (cfg->ccfg.cctl_dev) {
        const struct device *clk_dev = cfg->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->ccfg);
        clock_control_get_rate(clk_dev,(clock_control_subsys_t)&cfg->clock_source,&pclk);
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_DBG("%s: Could not configure pins", dev->name);
    }
#endif

    k_sem_init(&ptr_data->irq_sem, 0, 1);
    k_thread_create(&ptr_data->irq_thread, ptr_data->irq_thread_stack, K_KERNEL_STACK_SIZEOF(ptr_data->irq_thread_stack),
      vuart_irq_thread, (void *)dev, NULL, NULL, CONFIG_VUART_IRQ_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&ptr_data->irq_thread, cfg->vuart_irq_thread_name);

    uint16_t divisor = ((pclk + (cfg->baudrate << 3)) / cfg->baudrate ) >> 4;
    cfg->reg->IIR_FCR = FCR_FIFO|FCR_RCVRCLR|FCR_XMITCLR;
    cfg->reg->LCR |= LCR_DLAB|LCR_DLS;
    cfg->reg->RBR_THR_DLL = divisor;
    cfg->reg->DLH_IER = divisor>>8;
    cfg->reg->LCR &= ~LCR_DLAB;

    if (cfg->irq_config_func) {
        cfg->irq_config_func(dev);
    }

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        ptr_data->ioport[i].content = &cfg->ioport_content[i];
        espi_lpc_add_ioport(cfg->parent, (struct peri_ioport *)(&ptr_data->ioport[i]));
    }
    return 0;
}

void host_vuart_mode_set(const struct device *dev,bool host_rx_from_vuart,bool host_tx_to_vuart)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&ptr_data->lock);
    ptr_data->host_rx_from_vuart = host_rx_from_vuart;
    if(ptr_data->data_reg.ier & IER_RDA)
    {
        if(host_rx_from_vuart)
        {
            cfg->reg->DLH_IER &= ~IER_RDA;
        }else
        {
            cfg->reg->DLH_IER |= IER_RDA;
        }
    }
    k_spin_unlock(&ptr_data->lock, key);
    ptr_data->host_tx_to_vuart = host_tx_to_vuart;
}


static int vuart_ls_irq_tx_complete(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	return k_pipe_read_avail(cfg->b2h_pipe) == 0 ? 1 : 0;
}
static int vuart_ls_irq_tx_ready(const struct device *dev)
{
	return vuart_ls_irq_tx_complete(dev);
}

static int vuart_ls_irq_rx_ready(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	return k_pipe_read_avail(cfg->h2b_pipe) != 0;
}

static int vuart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct host_vuart_cfg *cfg = dev->config;
	size_t bytes_read;
	int ret;

	ret = k_pipe_get(cfg->h2b_pipe, c, 1, &bytes_read, 1, K_FOREVER);
	if (ret < 0) {
		return -1;
	}
    host_vuart_report_active_edge_level_up_irq(dev);
	return 0;
}

static int vuart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct host_vuart_cfg *cfg = dev->config;
	size_t bytes_read;
	int ret;

	ret = k_pipe_get(cfg->h2b_pipe, rx_data, size, &bytes_read, 1, K_NO_WAIT);
	if (ret < 0) {
		return 0;
	}
    host_vuart_report_active_edge_level_up_irq(dev);
	return bytes_read;
}

static void vuart_poll_out(const struct device *dev, unsigned char c)
{
	struct host_vuart_data *ptr_data = dev->data;
	const struct host_vuart_cfg *cfg = dev->config;
	size_t bytes_written;
    if(ptr_data->host_rx_from_vuart) {
        k_pipe_put(cfg->b2h_pipe, &c, 1, &bytes_written, 1, K_FOREVER);
        host_vuart_report_active_edge_level_up_irq(dev);
    }
}

static int vuart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int len)
{
	struct host_vuart_data *ptr_data = dev->data;
	const struct host_vuart_cfg *cfg = dev->config;
    if(ptr_data->host_rx_from_vuart){
    	size_t bytes_written;
        k_pipe_put(cfg->b2h_pipe, tx_data, len, &bytes_written, 1, K_NO_WAIT);
        if (bytes_written < len) {
            LOG_WRN("TX pipe full, dropped %d bytes", len - bytes_written);
        }
        host_vuart_report_active_edge_level_up_irq(dev);
        return bytes_written;
    }else{
        return 0;
    }
}

static void vuart_irq_tx_enable(const struct device *dev)
{
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = true;
	if (uart_irq_tx_ready(dev)) {
		vuart_wakeup_irq_thread(dev);
	}
}

static void vuart_irq_tx_disable(const struct device *dev)
{
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->tx_irq_enabled = false;
}

static void vuart_irq_rx_enable(const struct device *dev)
{
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->rx_irq_enabled = true;

	if (uart_irq_rx_ready(dev)) {
		vuart_wakeup_irq_thread(dev);
	}
}

static void vuart_irq_rx_disable(const struct device *dev)
{
	struct host_vuart_data *ptr_data = dev->data;
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

static void vuart_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				   void *user_data)
{
	struct host_vuart_data *ptr_data = dev->data;
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

#define HOST_VUART_INIT(inst)                                                                      \
  static void vuart_ls_irq_config_func_##inst(const struct device *dev)                                      \
    {                                                                                                         \
        IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), host_vuart_local_isr, DEVICE_DT_INST_GET(inst), 0); \
        irq_enable(DT_INST_IRQN(inst));                                                                        \
    }                                                                                                         \
  IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(inst)));                                                 \
  IF_ENABLED(DT_HAS_UP_IRQ(inst), (UPSTREAM_IRQ_DT_INST_DEFINE(inst)))                                     \
  const struct peri_ioport_content host_vuart_ioport_##inst[PORT_NUM] = {\
        [0] = {.io_read = host_vuart_reg0_read,                 \
              .io_write = host_vuart_reg0_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port)},              \
        [1] = {.io_read = host_vuart_reg1_read,                 \
              .io_write = host_vuart_reg1_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 1},          \
        [2] = {.io_read = host_vuart_reg2_read,                 \
              .io_write = host_vuart_reg2_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 2},\
        [3] = {.io_read = host_vuart_reg3_read,                 \
              .io_write = host_vuart_reg3_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 3},\
        [4] = {.io_read = host_vuart_reg4_read,                 \
              .io_write = host_vuart_reg4_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 4},\
        [5] = {.io_read = host_vuart_reg5_read,                 \
              .io_write = host_vuart_reg5_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 5},\
        [6] = {.io_read = host_vuart_reg6_read,                 \
              .io_write = host_vuart_reg6_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 6},\
        [7] = {.io_read = host_vuart_reg7_read,                         \
              .io_write = host_vuart_reg7_write,                       \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),                 \
              .addr = DT_INST_PROP(inst, port) + 7},\
  };\
  static struct host_vuart_data host_vuart_data_##inst; \
  K_PIPE_DEFINE(b2h_pipe_##inst, VUART_FIFO_SIZE, 4);                                      \
  K_PIPE_DEFINE(h2b_pipe_##inst, VUART_FIFO_SIZE, 4);                                      \
  static const struct host_vuart_cfg host_vuart_cfg_##inst = {                               \
    .vuart_irq_thread_name = "vuart_irq_thread_" #inst,                               \
    .local_irq = DT_INST_IRQN(inst),                                                                             \
    .irq_config_func = vuart_ls_irq_config_func_##inst,     \
    .ioport_content = host_vuart_ioport_##inst,             \
    .parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
    .reg = (reg_dwuart_t *)DT_INST_REG_ADDR(inst),                                                                         \
    .clock_source = DT_INST_PROP(inst, clock_source),      \
    .baudrate = DT_INST_PROP_OR(inst, current_speed,115200), \
    .b2h_pipe = &b2h_pipe_##inst,\
    .h2b_pipe = &h2b_pipe_##inst,\
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))                           \
    IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                                   \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))              \
    IF_ENABLED(DT_HAS_UP_IRQ(inst), (.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(inst))) }; \
  DEVICE_DT_INST_DEFINE(inst, &host_vuart_init, NULL, &host_vuart_data_##inst,               \
            &host_vuart_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &vuart_api);
DT_INST_FOREACH_STATUS_OKAY(HOST_VUART_INIT)
