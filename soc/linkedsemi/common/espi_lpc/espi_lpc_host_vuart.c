#define DT_DRV_COMPAT linkedsemi_ls_host_vuart
#define HOST_VUART_DEV DEVICE_DT_GET_ONE(DT_DRV_COMPAT)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
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

LOG_MODULE_REGISTER(ls_host_vuart, CONFIG_ESPI_LOG_LEVEL);

#define PORT_NUM   8
#define IER_RDA    0x01
#define IER_THRE   0x02
#define IER_ELSI   0x04    /* Enable Receiver Line Status Interrupt */
#define LCR_DLAB   0x80
#define LCR_DLS    0x3
#define IIR_NOPEND 0x01
#define IIR_THRE   0x02
#define IIR_RDA    0x04
#define IIR_RLS    0x06    /* Receiver Line Status Interrupt */
#define FCR_FIFO    0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */
#define LSR_DR     0x01
#define LSR_THRE   0x20
#define LSR_TEMT   0x40
#define LSR_BI     0x10    /* Break Interrupt */



struct host_vuart_reg {
  uint8_t dll;
  uint8_t dlh;
  uint8_t ier;
  uint8_t fcr;
  uint8_t lcr;
  uint8_t mcr;
  uint8_t scr;
};

struct retain_uart_var {
    struct host_vuart_reg data_reg;
};

struct host_vuart_cfg {
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    const struct device *espi_lpc; /* eSPI/LPC 控制器 */
    const struct peri_ioport_content *ioport_content;
    const struct upstream_irq_type *up_irq;
    struct host_bmc_msg_exch hb_exch;
    struct host_vuart_fifo *vuart_fifo_base;
    reg_dwuart_t *reg;
    struct retain_uart_var *retain;
    uint32_t clock_source;
    uint32_t baudrate;
    uint16_t local_irq;
    void (*irq_config_func)(const struct device *dev);
};

struct host_vuart_data {
  struct peri_ioport ioport[PORT_NUM];
  struct k_spinlock lock;
  bool break_pending;
};

void host_vuart_mode_set(const struct device *dev,bool host_rx_from_vuart,bool host_tx_to_vuart);
static void host_vuart_report_up_irq(const struct device *dev);

void host_vuart_rx_callback(const struct device *dev,void *msg)
{
    const struct device *vuart_dev = dev;
    struct vuart_hb_msg *vuart_msg = msg;
    struct host_vuart_data *ptr_data = dev->data;
    if(vuart_msg->type==VUART_MODE_SET)
    {
        host_vuart_mode_set(vuart_dev,vuart_msg->host_rx_from_vuart,vuart_msg->host_tx_to_vuart);
    }
    else if (vuart_msg->type == VUART_SEND_BREAK) 
    {
        ptr_data->break_pending = true; 
        host_vuart_report_up_irq(vuart_dev); 
    }
    {
	    host_vuart_report_up_irq(vuart_dev);
    }
}

static inline bool host_vuart_rx_available(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
	return !sw_fifo_empty(&cfg->vuart_fifo_base->b2h);
}

static inline bool host_vuart_tx_empty(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    return sw_fifo_empty(&cfg->vuart_fifo_base->h2b);
}

static inline bool lsr_tx_empty(uint8_t lsr)
{
    return lsr & LSR_THRE;
}

static inline bool lsr_rx_avail(uint8_t lsr)
{
    return lsr & LSR_DR;
}

static uint8_t host_vuart_calc_iir(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&ptr_data->lock);
    uint8_t iir = (cfg->retain->data_reg.fcr & 0x1) ? 0xC0 : 0x00;
    uint8_t lsr = cfg->reg->LSR;
    if (ptr_data->break_pending) {
        iir |= IIR_RLS;
    }
    if(cfg->vuart_fifo_base->host_rx_from_vuart) {
        if(host_vuart_rx_available(dev)) {
            iir |= IIR_RDA;
        }
    }else {
        if(lsr_rx_avail(lsr)) {
            iir |= IIR_RDA;
        }
    }
    if(!(iir&IIR_RDA) && lsr_tx_empty(lsr))
    {
        iir |= IIR_THRE;
    }
    if(!(iir&(IIR_RDA|IIR_THRE)))
    {
        iir |= IIR_NOPEND;
    }
    k_spin_unlock(&ptr_data->lock,key);
    return iir;
}

static void host_vuart_report_up_irq(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t iir = host_vuart_calc_iir(dev);

  if(((iir&IIR_RLS) && (cfg->retain->data_reg.ier&IER_ELSI))||
      ((iir&IIR_RDA) && (cfg->retain->data_reg.ier&IER_RDA)) ||
       ((iir&IIR_THRE) && (cfg->retain->data_reg.ier&IER_THRE))) {
        if (cfg->up_irq) {
            espi_lpc_raise_edge_irq(cfg->espi_lpc, cfg->up_irq->idx);
        }
    }
}

static void local_irq_state_update(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    if (cfg->up_irq == NULL) {
        return;
    }
    if (irq_is_enabled(cfg->local_irq)) {
        return;
    }
    clr_pending_irq(cfg->local_irq);
    bool pending = get_pending_irq(cfg->local_irq) ? true : false;
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
    espi_lpc_raise_edge_irq(cfg->espi_lpc, cfg->up_irq->idx);
}

void vuart_get_char_b2h(const struct device *dev, uint8_t *out_char)
{
	const struct host_vuart_cfg *cfg = dev->config;
    if(general_fifo_get(&cfg->vuart_fifo_base->b2h,out_char))
    {
        if(!host_vuart_rx_available(dev))
        {
            vuart_status_send(&cfg->hb_exch,B_TX_EMPTY);
        }
    }

}


static void host_vuart_reg0_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t *val = (uint8_t *)res;
    if (cfg->retain->data_reg.lcr & LCR_DLAB) {
        *val = cfg->retain->data_reg.dll;
    } else{
        uint8_t val1,val2=0;
        val1 = cfg->reg->RBR_THR_DLL;
        vuart_get_char_b2h(dev, &val2);
        *val = cfg->vuart_fifo_base->host_rx_from_vuart?val2:val1;
        local_irq_state_update(dev);
    }
}

static void vuart_send_char_h2b(const struct device *dev, uint8_t c)
{
    const struct host_vuart_cfg *cfg = dev->config;
    if(!sw_fifo_full(&cfg->vuart_fifo_base->h2b))
    {
        general_fifo_put(&cfg->vuart_fifo_base->h2b,&c);
        vuart_status_send(&cfg->hb_exch,B_RX_AVAIL);
    }
}

static void host_vuart_reg0_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    if (cfg->retain->data_reg.lcr & LCR_DLAB) {
        cfg->retain->data_reg.dll = *val;
    } else {
        cfg->reg->RBR_THR_DLL = *val;
        vuart_send_char_h2b(dev,*val);
        local_irq_state_update(dev);
    }
}

static void host_vuart_reg1_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t *val = (uint8_t *)res;
    if (cfg->retain->data_reg.lcr & LCR_DLAB) {
        *val = cfg->retain->data_reg.dlh;
    } else {
        *val = cfg->retain->data_reg.ier;
    }
}


static void host_vuart_reg1_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    if (cfg->retain->data_reg.lcr & LCR_DLAB) {
        cfg->retain->data_reg.dlh = *val;
    } else {
        cfg->retain->data_reg.ier = *val;
        if(cfg->vuart_fifo_base->host_rx_from_vuart) {
            cfg->reg->DLH_IER = *val & (IER_THRE|IER_ELSI);
            host_vuart_report_up_irq(dev);
        }else {
            cfg->reg->DLH_IER = *val & (IER_THRE|IER_RDA|IER_ELSI);
            local_irq_state_update(dev);
        }
    }
}

static void host_vuart_reg2_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    uint8_t *val = (uint8_t *)res;
    *val = host_vuart_calc_iir(dev);
    local_irq_state_update(dev);
}

static void host_vuart_reg2_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    cfg->retain->data_reg.fcr = *val;
}

static void host_vuart_reg3_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t *val = (uint8_t *)res;
    *val = cfg->retain->data_reg.lcr;
}

static void host_vuart_reg3_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    cfg->retain->data_reg.lcr = *val;
}

static void host_vuart_reg4_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t *val = (uint8_t *)res;
    *val = cfg->retain->data_reg.mcr;
}

static void host_vuart_reg4_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    cfg->retain->data_reg.mcr = *val;
}

static void host_vuart_reg5_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;
    uint8_t lsr = cfg->reg->LSR;
    *val = 0;
    if (ptr_data->break_pending) {
        *val |= LSR_BI;
        ptr_data->break_pending = false; 
    }
    if (lsr_tx_empty(lsr))
    {
        *val |= LSR_THRE|LSR_TEMT;
    }
    if (cfg->vuart_fifo_base->host_rx_from_vuart) {
        if(host_vuart_rx_available(dev)) {
            *val |= LSR_DR;
        }
    }else {
        if(lsr_rx_avail(lsr)) {
            *val |= LSR_DR;
        }
    }
    local_irq_state_update(dev);
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
    const struct host_vuart_cfg *cfg = dev->config;
    uint8_t *val = (uint8_t *)res;
    *val = cfg->retain->data_reg.scr;
}

static void host_vuart_reg7_write(const struct peri_ioport_content *ioport, uint8_t size,
                uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    cfg->retain->data_reg.scr = *val;
}




static int host_vuart_init(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint32_t pclk = 75000000;
    if (!device_is_ready(cfg->espi_lpc) ) {
        LOG_ERR("espi_lpc not ready");
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
    host_bmc_msg_exch_init(&cfg->hb_exch);
    sw_fifo_init(&cfg->vuart_fifo_base->b2h,cfg->vuart_fifo_base->b2h_buf,VUART_FIFO_SIZE,1);
    sw_fifo_init(&cfg->vuart_fifo_base->h2b,cfg->vuart_fifo_base->h2b_buf,VUART_FIFO_SIZE,1);
    cfg->vuart_fifo_base->host_tx_to_vuart = false;
    cfg->vuart_fifo_base->host_rx_from_vuart = false;
    memset(&ptr_data->lock,0,sizeof(ptr_data->lock));

    uint16_t divisor = ((pclk + (cfg->baudrate << 3)) / cfg->baudrate ) >> 4;
    cfg->reg->IIR_FCR = FCR_FIFO|FCR_RCVRCLR|FCR_XMITCLR;
    cfg->reg->LCR |= LCR_DLAB|LCR_DLS;
    cfg->reg->RBR_THR_DLL = divisor;
    cfg->reg->DLH_IER = divisor>>8;
    cfg->reg->LCR &= ~LCR_DLAB;


    k_spinlock_key_t key = k_spin_lock(&ptr_data->lock);
    for (uint32_t i = 0; i < PORT_NUM; i++) {
        ptr_data->ioport[i].content = &cfg->ioport_content[i];
        espi_lpc_add_ioport(cfg->espi_lpc, (struct peri_ioport *)(&ptr_data->ioport[i]));
    }
    k_spin_unlock(&ptr_data->lock, key);

    cfg->irq_config_func(dev);
    return 0;
}

void host_vuart_mode_set(const struct device *dev,bool host_rx_from_vuart,bool host_tx_to_vuart)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&ptr_data->lock);
    cfg->vuart_fifo_base->host_rx_from_vuart = host_rx_from_vuart;
    if(cfg->retain->data_reg.ier & IER_RDA)
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
    cfg->vuart_fifo_base->host_tx_to_vuart = host_tx_to_vuart;
    local_irq_state_update(dev);
}

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
  static struct retain_uart_var host_vuart_retain_var_##inst __attribute__((section(".var_retain.98."#inst)));  \
  static struct host_vuart_data host_vuart_data_##inst __noinit; \
  static const struct host_vuart_cfg host_vuart_cfg_##inst = {                               \
    .retain = &host_vuart_retain_var_##inst,                                            \
    .local_irq = DT_INST_IRQN(inst),                                                                             \
    .ioport_content = host_vuart_ioport_##inst,             \
    .espi_lpc = DEVICE_DT_GET(DT_INST_PHANDLE(inst,espi_lpc)),\
    .reg = (reg_dwuart_t *)DT_INST_REG_ADDR(inst),                                                                         \
    .clock_source = DT_INST_PROP(inst, clock_source),      \
    .baudrate = DT_INST_PROP_OR(inst, current_speed,115200), \
    .vuart_fifo_base = (struct host_vuart_fifo *)DT_INST_PROP(inst,fifo_base),\
    .hb_exch = HOST_BMC_MSG_EXCH_INIT(inst,host_vuart_rx_callback,bmc_vuart_rx_callback),            \
    .irq_config_func = vuart_ls_irq_config_func_##inst,\
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))                           \
    IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                                   \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))              \
    IF_ENABLED(DT_HAS_UP_IRQ(inst), (.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(inst))) }; \
  DEVICE_DT_INST_DEFINE(inst, &host_vuart_init, NULL, &host_vuart_data_##inst,               \
            &host_vuart_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);
DT_INST_FOREACH_STATUS_OKAY(HOST_VUART_INIT)


static int cmd_mode(const struct shell *sh, size_t argc, char **argv)
{
    bool host_rx_from_vuart,host_tx_to_vuart;
    if (argc < 3) {
        shell_error(sh, "Usage: %s <host_rx_from_vuart:true|false> <host_tx_to_vuart:true|false>" ,argv[0]);
        return -EINVAL;
    }

    const struct device *dev = HOST_VUART_DEV;
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

