#define DT_DRV_COMPAT linkedsemi_ls_host_vuart
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <espi_lpc_common.h>
#include "cpu.h"
#include "vuart_ls.h"

typedef void (*irq_cfg_func_t)(const struct device *dev);

LOG_MODULE_REGISTER(ls_host_vuart, CONFIG_ESPI_LOG_LEVEL);

#define IRQ_TYPE_NONE         0
#define IRQ_TYPE_EDGE_RISING  1
#define IRQ_TYPE_EDGE_FALLING 2
#define IRQ_TYPE_EDGE_BOTH    (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH   4
#define IRQ_TYPE_LEVEL_LOW    8

#define PORT_NUM   8
#define LCR_DLAB   0x80
#define IIR_NOPEND 0x01
#define IIR_THRE   0x02
#define IIR_RDA    0x04
#define LSR_DR     0x01
#define LSR_THRE   0x20
#define LSR_TEMT   0x40
#define LSR_RESET_VAL 0x60
#ifndef VUART_MMIO_STRIDE
#define VUART_MMIO_STRIDE 1
#endif

struct host_vuart_cfg {
  IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
  IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
  IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
  const struct device *parent; /* eSPI 控制器 */
  const struct device *vuart;
  const struct upstream_irq_type *up_irq;
  irq_cfg_func_t irq_config_func;
  mem_addr_t reg;
  uint16_t host_vuart_reg;
  uint16_t phy_irq;
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
  struct host_vuart_reg data_reg;
  struct k_spinlock lock;
  bool is_lcr_avoid;
  bool use_virtual;
};

static uint8_t host_vuart_calc_iir(const struct device *dev)
{
  const struct host_vuart_cfg *cfg = dev->config;
  struct host_vuart_data *ptr_data = dev->data;
  k_spinlock_key_t key;
  uint8_t iir = 0;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t val = sys_read8(cfg->reg + (2 * 4));
        return val;
    }

    key = k_spin_lock(&ptr_data->lock);

    iir = (ptr_data->data_reg.fcr & 0x1) ? 0xC0 : 0x00;

    if ((ptr_data->data_reg.ier & 0x01) && host_vuart_rx_available(cfg->vuart)) {
        iir |= IIR_RDA;
    } else if ((ptr_data->data_reg.ier & 0x02)) {
        iir |= IIR_THRE;
    } else {
        iir |= IIR_NOPEND;
    }

    k_spin_unlock(&ptr_data->lock, key);

    return iir;
}

static void host_vuart_report_active_edge_level_up_irq(const struct device *dev)
{
  const struct host_vuart_cfg *cfg = dev->config;
  struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual) {
        return;
    }

    if ((host_vuart_calc_iir(dev) & 0xf)  == IIR_NOPEND) {
        return;
    }

    if (cfg->up_irq) {
        espi_lpc_raise_edge_irq(cfg->parent, cfg->up_irq->idx);
    }
}


static void host_vuart_rx_callback(void *dev)
{
    const struct device *d = (const struct device *)dev;
    struct host_vuart_data *ptr_data = d->data;
    if (!ptr_data->use_virtual) {
        return;
    }
    host_vuart_report_active_edge_level_up_irq(d);
}



static void host_phy_irq_state_update(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    if (!cfg->reg || cfg->phy_irq == 0U || cfg->up_irq == NULL) {
        return;
    }
    bool masked,pending;

    masked = irq_is_enabled(cfg->phy_irq) ? true : false;
    if (!masked) {
        return;
    }

    clr_pending_irq(cfg->phy_irq);

    pending = get_pending_irq(cfg->phy_irq) ? true : false;
    if (!pending) {

        irq_enable(cfg->phy_irq);
    }
}



static void host_phy_isr(const void *arg)
{
    const struct device *dev = (const struct device *)arg;
    const struct host_vuart_cfg *cfg = dev->config;
    irq_disable(cfg->phy_irq);
    if (!cfg->up_irq) {
        return;
    }
    espi_lpc_raise_edge_irq(cfg->parent, cfg->up_irq->idx);
}



int host_vuart_set_mode(const struct device *dev, bool virtual)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *d = dev->data;
    int ret = 0;

    k_spinlock_key_t key = k_spin_lock(&d->lock);

    if (virtual) {
        d->use_virtual = true;
        if (cfg->phy_irq) {
            irq_disable(cfg->phy_irq);
        }
    } else {
        if (!cfg->reg || !cfg->phy_irq) {
            ret = -ENODEV;
            goto out;
        }

        d->use_virtual = false;
        uint8_t lcr = d->data_reg.lcr;
        sys_write8(lcr | LCR_DLAB,   cfg->reg + 3 * 4);
        sys_write8(d->data_reg.dll,  cfg->reg + 0 * 4);          // DLL
        sys_write8(d->data_reg.dlh,  cfg->reg + 1 * 4);          // DLH
        sys_write8(lcr,              cfg->reg + 3 * 4);         //恢复正常的寄存器映射
        sys_write8(d->data_reg.ier,  cfg->reg + 1 * 4);         // 中断使能寄存器
        sys_write8(d->data_reg.fcr,  cfg->reg + 2 * 4);         // FIFO控制寄存器
        sys_write8(d->data_reg.lcr,  cfg->reg + 3 * 4);         // 线路控制寄存器（传输特性）
        sys_write8(d->data_reg.mcr,  cfg->reg + 4 * 4);         // 调制解调器控制寄存器
        sys_write8(d->data_reg.scr,  cfg->reg + 7 * 4);         // 暂存寄存器

        d->is_lcr_avoid = (lcr & LCR_DLAB) != 0;
        irq_enable(cfg->phy_irq);
    }

out:
    k_spin_unlock(&d->lock, key);
    return ret;
}

static void host_vuart_reg0_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
  struct device *dev = ioport->ctx;
  const struct host_vuart_cfg *cfg = dev->config;
  struct host_vuart_data *ptr_data = dev->data;
  uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        *val = ptr_data->data_reg.dll;
    } else if (host_vuart_rx_available(cfg->vuart)) {
        ls_vuart_get_tx_char(cfg->vuart, val);
    }
}

static void host_vuart_reg0_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
  struct device *dev = ioport->ctx;
  const struct host_vuart_cfg *cfg = dev->config;
  struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        if (ptr_data->data_reg.lcr & LCR_DLAB) {
            ptr_data->data_reg.dll = *val;
        }
        if (!ptr_data->is_lcr_avoid) {
            sys_write8(*val, cfg->reg + offset);
        } else {
            if (offset == 0) {
                sys_write8(0x29, cfg->reg + offset);
            } else if (offset == 4) {
                sys_write8(0x0, cfg->reg + offset);
            }
        }
        host_phy_irq_state_update(dev);
        return;
    }

    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        ptr_data->data_reg.dll = *val;
    } else {
        ls_vuart_put_rx_char(cfg->vuart, *val);
    }
}

static void host_vuart_reg1_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
  const struct device *dev = ioport->ctx;
  struct host_vuart_data *ptr_data = dev->data;
  const struct host_vuart_cfg *cfg = dev->config;
  uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr -  cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

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
    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        if (ptr_data->data_reg.lcr & LCR_DLAB) {
            ptr_data->data_reg.dlh = *val;
        } else {
            ptr_data->data_reg.ier = *val;
        }
        if (!ptr_data->is_lcr_avoid) {
            sys_write8(*val, cfg->reg + offset);
        } else {
            if (offset == 0) {
                sys_write8(0x29, cfg->reg + offset);
            } else if (offset == 4) {
                sys_write8(0x0, cfg->reg + offset);
            }
        }
        host_phy_irq_state_update(dev);
        return;
    }

    if (ptr_data->data_reg.lcr & LCR_DLAB) {
        ptr_data->data_reg.dlh = *val;
    } else {
        ptr_data->data_reg.ier = *val;
        host_vuart_report_active_edge_level_up_irq(dev);
    }
}

static void host_vuart_reg2_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    *val = host_vuart_calc_iir(dev);
}

static void host_vuart_reg2_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        ptr_data->data_reg.fcr = *val;
        sys_write8(*val, cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    ptr_data->data_reg.fcr = *val;
}

static void host_vuart_reg3_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;


    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        ptr_data->data_reg.lcr = *val;
        host_phy_irq_state_update(dev);
        return;
    }

    *val = ptr_data->data_reg.lcr;
}

static void host_vuart_reg3_write(const struct peri_ioport_content *ioport, uint8_t size, uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        ptr_data->data_reg.lcr = *val;
        if (*val & LCR_DLAB) {
            ptr_data->is_lcr_avoid = true;
        } else {
            ptr_data->is_lcr_avoid = false;
        }
        sys_write8(*val, cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    ptr_data->data_reg.lcr = *val;
}

static void host_vuart_reg4_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        ptr_data->data_reg.mcr = *val;
        host_phy_irq_state_update(dev);
        return;
    }

    *val = ptr_data->data_reg.mcr;
}

static void host_vuart_reg4_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        ptr_data->data_reg.mcr = *val;
        sys_write8(*val, cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    ptr_data->data_reg.mcr = *val;
}

static void host_vuart_reg5_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    *val = 0x60;
    if (host_vuart_rx_available(cfg->vuart)) {
        *val |= 0x01;
    }
}
static void host_vuart_reg5_write(const struct peri_ioport_content *ioport, uint8_t size,uint8_t *val)
{

    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        sys_write8(*val, cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

	__ASSERT(ptr_data->use_virtual, "Unreachable code path");
}

static void host_vuart_reg6_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

    *val = 0x00; /* 调制解调器状态寄存器 */
}

static void host_vuart_reg6_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        sys_write8(*val, cfg->reg + offset);
        host_phy_irq_state_update(dev);
        return;
    }

	__ASSERT(ptr_data->use_virtual, "Unreachable code path");
}

static void host_vuart_reg7_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    uint8_t *val = (uint8_t *)res;

    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        *val = sys_read8(cfg->reg + offset);
        ptr_data->data_reg.scr = *val;
        host_phy_irq_state_update(dev);
        return;
    }

    *val = ptr_data->data_reg.scr;
}

static void host_vuart_reg7_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
    const struct device *dev = ioport->ctx;
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;
    if (!ptr_data->use_virtual && cfg->reg) {
        uint8_t offset = (ioport->addr - cfg->host_vuart_reg) * 4;
        sys_write8(*val, cfg->reg + offset);
        ptr_data->data_reg.scr = *val;
        host_phy_irq_state_update(dev);
        return;
    }

    ptr_data->data_reg.scr = *val;
}

static int host_vuart_init(const struct device *dev)
{
    const struct host_vuart_cfg *cfg = dev->config;
    struct host_vuart_data *ptr_data = dev->data;

    if (!device_is_ready(cfg->parent) || !device_is_ready(cfg->vuart)) {
        LOG_ERR("parent or vuart not ready");
        return -ENODEV;
	}
    int ret = 0;

    ptr_data->use_virtual = true;
    ptr_data->is_lcr_avoid = false;
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
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_DBG("%s: Could not configure pins", dev->name);
    }
#endif

    ls_vuart_register_tx_start_callback(cfg->vuart, host_vuart_rx_callback, (void *)dev);


    if (cfg->reg && cfg->phy_irq) {
        if (cfg->irq_config_func) {
            cfg->irq_config_func(dev);
        }
    }

    for (uint32_t i = 0; i < PORT_NUM; i++) {
        espi_lpc_add_ioport(cfg->parent, (struct peri_ioport *)(&ptr_data->ioport[i]));
    }
    return 0;
}



#define HOST_VUART_INIT(inst)                                                                      \
  static void vuart_ls_irq_config_func_##inst(const struct device *dev)                                      \
    {                                                                                                         \
        IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), host_phy_isr, DEVICE_DT_INST_GET(inst), 0); \
        irq_enable(DT_INST_IRQN(inst));                                                                        \
    }                                                                                                         \
  IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(inst)));                                                 \
  IF_ENABLED(DT_HAS_UP_IRQ(inst), (UPSTREAM_IRQ_DT_INST_DEFINE(inst)))                                     \
  static struct host_vuart_data host_vuart_data_##inst = {                                   \
    .ioport = {                                                                 \
         [0] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg0_read,                 \
              .io_write = host_vuart_reg0_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port)}},              \
         [1] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg1_read,                 \
              .io_write = host_vuart_reg1_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 1}},          \
         [2] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg2_read,                 \
              .io_write = host_vuart_reg2_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 2  }},       \
         [3] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg3_read,                 \
              .io_write = host_vuart_reg3_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 3}},          \
         [4] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg4_read,                 \
              .io_write = host_vuart_reg4_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 4}},          \
         [5] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg5_read,                 \
              .io_write = host_vuart_reg5_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 5}},          \
         [6] = {.content =                                                       \
            &(struct peri_ioport_content){                           \
              .io_read = host_vuart_reg6_read,                 \
              .io_write = host_vuart_reg6_write,               \
              .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
              .addr = DT_INST_PROP(inst, port) + 6}},          \
         [7] = {.content = &(struct peri_ioport_content){                        \
            .io_read = host_vuart_reg7_read,                         \
            .io_write = host_vuart_reg7_write,                       \
            .ctx = (void *)DEVICE_DT_INST_GET(inst),                 \
            .addr = DT_INST_PROP(inst, port) + 7}}}};                \
  static const struct host_vuart_cfg host_vuart_cfg_##inst = {                               \
    .phy_irq = DT_INST_IRQN(inst),                                                                             \
    .irq_config_func = vuart_ls_irq_config_func_##inst,     \
    .parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
    .reg = DT_INST_REG_ADDR(inst),                                                                         \
    .host_vuart_reg = DT_INST_PROP(inst, port),                                                            \
    .vuart = DEVICE_DT_GET(DT_INST_PHANDLE(inst, target)),                             \
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))                           \
    IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                                   \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))              \
    IF_ENABLED(DT_HAS_UP_IRQ(inst), (.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(inst))) }; \
  DEVICE_DT_INST_DEFINE(inst, &host_vuart_init, NULL, &host_vuart_data_##inst,               \
            &host_vuart_cfg_##inst, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, 0);
DT_INST_FOREACH_STATUS_OKAY(HOST_VUART_INIT)
