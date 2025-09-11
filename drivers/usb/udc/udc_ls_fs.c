#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include "zephyr/irq.h"
#include "zephyr/kernel/thread.h"
#include "zephyr/net_buf.h"
#include "zephyr/sys/util.h"

#include <soc_clock.h>
#include "udc_common.h"
#include "musb_type.h"
#include "linked_async_framework.h"
#include "reg_usb_type.h"
#include "ls_soc_gpio.h"

#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif

#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif

LOG_MODULE_REGISTER(udc_ls_fs, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define USB_EP0_SIZE            (64)
#define USB_TX_BUF_ADDR         (8)
#define USB_TX_FIFO_SZ          (2)
#define USB_TX_BUF_SIZE         (1<<(3+USB_TX_FIFO_SZ))
#define USB_RX_BUF_ADDR         (USB_TX_BUF_ADDR + USB_TX_BUF_SIZE / 8)

#define USB_DIR_OUT             0       /* to device */
#define USB_DIR_IN              0x80    /* to host */

#define USB_MSGQ_LENGTH         (16)

typedef enum {
    USB_EP0_STAGE_IDLE,         /* idle, waiting for SETUP */
    USB_EP0_STAGE_SETUP,        /* received SETUP */
    USB_EP0_STAGE_TX,           /* IN data */
    USB_EP0_STAGE_RX,           /* OUT data */
    USB_EP0_STAGE_STATUSIN,     /* (after OUT data) */
    USB_EP0_STAGE_STATUSOUT     /* (after IN data) */
} ep0_state_t;

struct udc_ep_item
{
    struct co_list_hdr tlist;
    struct net_buf *buf;
    uint8_t ep_index;
    const void *priv;
};
struct udc_ls_config
{
    uint32_t num_endpoints;
    uint16_t ep0_mps;
    uint16_t ep_mps;
    void (*make_thread)(const struct device *dev);
    void (*irq_connect)(const struct device *dev);
    void (*irq_disconnect)(const struct device *dev);
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

typedef union {
    uint8_t   u8;
    uint16_t  u16;
    uint32_t  u32;
} hw_fifo_t;

typedef enum {
    LS_EVT_EP0_XFER,
    LS_EVT_SETUP,
    LS_EVT_IN_COMPLETE,
    LS_EVT_IN_ZLP,
} ls_event_t;

struct ls_event
{
    ls_event_t type;
    uint8_t ep;
};

struct udc_ls_data
{
    reg_usb_t *usb_instance;
    struct udc_ep_config *ep_cfg_in;
    struct udc_ep_config *ep_cfg_out;
    struct udc_ep_item *ep_tx;
    linked_async_inst_t async_list;
    ep0_state_t ep0_state;
    struct k_thread thread;
    struct usb_setup_packet setup;
    bool is_set_addr;
    uint16_t rx_fifo_addr;
    struct k_msgq msgq;
    struct ls_event msgq_buf[USB_MSGQ_LENGTH];
};

static void fifo_write(volatile void *fifo, void *buf, unsigned len)
{
    volatile hw_fifo_t *reg = (volatile hw_fifo_t*)fifo;
    uintptr_t addr = (uintptr_t)buf;

    while (len >= 4)
    {
        reg->u32 = *(uint32_t const *)addr;
        addr += 4;
        len  -= 4;
    }
    if (len >= 2) 
    {
        reg->u16 = *(uint16_t const *)addr;
        addr += 2;
        len  -= 2;
    }
    if (len)
    {
        reg->u8 = *(uint8_t const *)addr;
    }
}

static void fifo_read(void *buf, volatile void *fifo, unsigned len)
{
    volatile hw_fifo_t *reg = (volatile hw_fifo_t*)fifo;
    uintptr_t addr = (uintptr_t)buf;
    while (len >= 4)
    {
        *(uint32_t *)addr = reg->u32;
        addr += 4;
        len  -= 4;
    }
    if (len >= 2)
    {
        *(uint16_t *)addr = reg->u16;
        addr += 2;
        len  -= 2;
    }
    if (len)
    {
        *(uint8_t *)addr = reg->u8;
    }
}

static void musb_set_active_ep(reg_usb_t *usb_instance, uint8_t ep_index)
{
    usb_instance->EPIDX = ep_index;
}

static uint8_t musb_get_active_ep(reg_usb_t *usb_instance)
{
    return usb_instance->EPIDX;
}

static int udc_ls_lock(const struct device *dev)
{
    return udc_lock_internal(dev, K_FOREVER);
}

static int udc_ls_unlock(const struct device *dev)
{
    return udc_unlock_internal(dev);
}

static int udc_ls_init(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    const struct udc_ls_config *usb_cfg = dev->config;
    reg_usb_t *usb_reg = usb_data->usb_instance;

#ifdef CONFIG_SOC_LSQSH
    /* If the dp is externally pulled up, a low level will be output here to make the host initiate a reset. */
    io_cfg_output(PH14);
    io_write_pin(PH14, 0);
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (usb_cfg->ccfg.cctl_dev)
    {
        const struct device *clk_dev = usb_cfg->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev))
        {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&usb_cfg->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (usb_cfg->reset.dev != NULL)
    {
        if (!device_is_ready(usb_cfg->reset.dev))
        {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        int ret = reset_line_toggle(usb_cfg->reset.dev, usb_cfg->reset.id);
        if (ret != 0)
        {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (usb_cfg->ccfg.cctl_dev)
    {
        const struct device *clk_dev = usb_cfg->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&usb_cfg->ccfg);
    }
#endif

#if defined(CONFIG_PINCTRL)
    if (pinctrl_apply_state(usb_cfg->pcfg, PINCTRL_STATE_DEFAULT) < 0) {
        LOG_DBG("%s: Could not configure pins", dev->name);
    }
#ifdef CONFIG_SOC_LS1010
    PD00_PD01_PowerOn();
#endif
#endif

    for (uint8_t i = 1; i < usb_cfg->num_endpoints; ++i)
    {
        usb_reg->EPIDX = i;
        usb_reg->TXFIFO_SIZE[0] = 0;
        usb_reg->TXFIFO_SIZE[1] = 0;
        usb_reg->RXFIFO_SIZE[0] = 0;
        usb_reg->RXFIFO_SIZE[1] = 0;
    }

    k_msgq_init(&usb_data->msgq, (char *)usb_data->msgq_buf, sizeof(struct ls_event), USB_MSGQ_LENGTH);
    usb_cfg->make_thread(dev);

    return 0;
}

static int udc_ls_enable(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    const struct udc_ls_config *usb_cfg = dev->config;
    reg_usb_t *usb_reg = usb_data->usb_instance;

    if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0) || \
                udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0))
    {
        LOG_ERR("Failed to enable control endpoint");
        return -EIO;
    }

    usb_reg->IE |= USB_IE_SUSPND;
    usb_reg->POWER |= USB_POWER_SOFTCONN;
    usb_reg->FADDR = 0;
    usb_cfg->irq_connect(dev);

    return 0;
}

static int udc_ls_disable(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    const struct udc_ls_config *usb_cfg = dev->config;
    reg_usb_t *usb_reg = usb_data->usb_instance;

    if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT) || udc_ep_disable_internal(dev, USB_CONTROL_EP_IN))
    {
        LOG_ERR("Failed to disable control endpoint");
        return -EIO;
    }
    usb_reg->POWER &= ~USB_POWER_SOFTCONN;
    usb_cfg->irq_disconnect(dev);

    return 0;
}

static int udc_ls_tx(const struct device *dev, uint8_t ep_index,
        struct net_buf *buf)

{
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_index | USB_EP_DIR_IN);
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    uint16_t len = MIN(buf->len, ep_cfg->mps);

    musb_set_active_ep(usb_reg, ep_index);
    usb_reg->TXCSRH |= USB_TXCSRH1_MODE;
    if (len)
    {
        fifo_write(&usb_reg->FIFO0_WORD + ep_index, buf->data, len);
        net_buf_pull(buf, len);
    }
    /* trigger irq */
    usb_reg->TXCSRL = USB_TXCSRL1_TXRDY;

    return 0;
}

static int udc_ls_rx(const struct device *dev, uint8_t ep_index,
        struct net_buf *buf)
{
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_index | USB_EP_DIR_OUT);
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    uint16_t rx_len, read_count;

    musb_set_active_ep(usb_reg, ep_index);
    read_count = usb_reg->RXCOUNT;
    rx_len = MIN(MIN(net_buf_tailroom(buf), ep_cfg->mps), read_count);
    fifo_read(net_buf_add(buf, rx_len), &usb_reg->FIFO0_WORD + ep_index, rx_len);

    if (!net_buf_tailroom(buf) || rx_len < ep_cfg->mps)
    {
        /* If the space of buf is greater than the endpoint max size, 
         * the last packet (equal to ep_cfg->mps) of the host must be sent zlp, if it is not sent, the upper layer will not be notified.
        */
        buf = udc_buf_get(dev, ep_cfg->addr);
        udc_submit_ep_event(dev, buf, 0);
    }
    usb_reg->RXCSRL &= ~USB_RXCSRL1_RXRDY;

    return 0;
}

static void async_tx_process(linked_async_inst_t *async, struct co_list_hdr *item)
{
    struct udc_ep_item *ep_item = (struct udc_ep_item *)item;
    udc_ls_tx(ep_item->priv, ep_item->ep_index, ep_item->buf);
}

static bool async_tx_end(struct linked_async_inst_s *inst, struct co_list_hdr *hdr, void *dummy, uint8_t status)
{
    return false;
}

static int udc_ls_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
        struct net_buf *const buf)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    LOG_DBG("%p enqueue %x %p, len = %d.\n", dev, cfg->addr, buf, buf->len);

    if (USB_EP_GET_IDX(cfg->addr))
    {
        if (USB_EP_DIR_IS_IN(cfg->addr))
        {
            if (udc_ep_is_busy(dev, cfg->addr))
                return 0;
            udc_buf_put(cfg, buf);
            udc_ep_set_busy(dev, cfg->addr, true);
            usb_data->ep_tx[USB_EP_GET_IDX(cfg->addr)].buf = buf;
            usb_data->ep_tx[USB_EP_GET_IDX(cfg->addr)].priv = dev;
            usb_data->ep_tx[USB_EP_GET_IDX(cfg->addr)].ep_index = USB_EP_GET_IDX(cfg->addr);
            linked_async_start(&usb_data->async_list, &usb_data->ep_tx[USB_EP_GET_IDX(cfg->addr)].tlist);
        }
        else
        {
            reg_usb_t *usb_instance = usb_data->usb_instance;
            musb_set_active_ep(usb_instance, USB_EP_GET_IDX(cfg->addr));

            /* disable rx irq */
            usb_instance->RXIE &= ~BIT(USB_EP_GET_IDX(cfg->addr));
            udc_buf_put(cfg, buf);
            if (usb_instance->RXCOUNT)
                udc_ls_rx(dev, USB_EP_GET_IDX(cfg->addr), buf);
            usb_instance->RXIE |= BIT(USB_EP_GET_IDX(cfg->addr));
        }
    }
    else
    {
        struct ls_event evt = {
            .type = LS_EVT_EP0_XFER,
            .ep = cfg->addr
        };
        /* post event */
        udc_buf_put(cfg, buf);
        k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);
    }
    return 0;
}

static int udc_ls_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
    struct net_buf *buf;
    buf = udc_buf_get_all(dev, cfg->addr);

    if (buf)
    {
        udc_submit_ep_event(dev, buf, -ECONNABORTED);
    }
    udc_ep_set_busy(dev, cfg->addr, false);

    return 0;
}

static int udc_ls_ep_set_halt(const struct device *dev,struct udc_ep_config *const cfg)
{
    /* set stall */
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    uint8_t ep_index = USB_EP_GET_IDX(cfg->addr);
    musb_set_active_ep(usb_reg, ep_index);

    if (ep_index)
    {
        if (USB_EP_DIR_IS_IN(cfg->addr))
            usb_reg->TXCSRL |= USB_TXCSRL1_STALL;
        else
            usb_reg->RXCSRL |= USB_RXCSRL1_STALL;
    }
    else
    {
        usb_data->ep0_state = USB_EP0_STAGE_IDLE;
        usb_reg->TXCSRL |= (USB_CSRL0_STALL | USB_CSRL0_RXRDYC);
    }

    if (ep_index)
        cfg->stat.halted = true;

    return 0;
}

static int udc_ls_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    uint8_t ep_index = USB_EP_GET_IDX(cfg->addr);
    musb_set_active_ep(usb_reg, ep_index);

    if (ep_index)
    {
        if (USB_EP_DIR_IS_IN(cfg->addr))
            usb_reg->TXCSRL = USB_TXCSRL1_CLRDT;
        else
            usb_reg->RXCSRL = USB_RXCSRL1_CLRDT;
    }
    else
    {
        usb_reg->TXCSRL &= ~USB_CSRL0_STALL;
    }
    cfg->stat.halted = false;

    return 0;
}


static int udc_ls_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    uint8_t ep_index = USB_EP_GET_IDX(cfg->addr);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    musb_set_active_ep(usb_reg, ep_index);

    if (USB_EP_DIR_IS_IN(cfg->addr))
    {
        usb_reg->TXIE  &= ~BIT(ep_index);
        usb_reg->TXMAXP = 0;
        usb_reg->TXCSRH = 0;
        if (usb_reg->TXCSRL & USB_TXCSRL1_TXRDY)
            usb_reg->TXCSRL = USB_TXCSRL1_CLRDT | USB_TXCSRL1_FLUSH;
        else
            usb_reg->TXCSRL = USB_TXCSRL1_CLRDT;
        usb_reg->TXFIFO_SIZE[0]  = 0;
        usb_reg->TXFIFO_SIZE[1]  = 0;
        usb_data->rx_fifo_addr = USB_TX_BUF_ADDR;
    }
    else
    {
        usb_reg->RXIE  &= ~BIT(ep_index);
        usb_reg->RXMAXP = 0;
        usb_reg->RXCSRH = 0;
        if (usb_reg->RXCSRL & USB_RXCSRL1_RXRDY)
            usb_reg->RXCSRL = USB_RXCSRL1_CLRDT | USB_RXCSRL1_FLUSH;
        else
            usb_reg->RXCSRL = USB_RXCSRL1_CLRDT;
        usb_reg->RXFIFO_SIZE[0]  = 0;
        usb_reg->RXFIFO_SIZE[1]  = 0;
    }

    if (cfg->addr == USB_CONTROL_EP_OUT)
    {
        struct net_buf *buf = udc_buf_get_all(dev, cfg->addr);
        if (buf)
        {
            net_buf_unref(buf);
        }
    }

    return 0;
}

static int udc_ls_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    uint8_t ep_index = USB_EP_GET_IDX(cfg->addr);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    musb_set_active_ep(usb_reg, ep_index);

    if (USB_EP_DIR_IS_IN(cfg->addr))
    {
        usb_reg->TXMAXP = cfg->mps;
        usb_reg->TXCSRH = (cfg->attributes == USB_EP_TYPE_ISO) ? USB_TXCSRH1_ISO : 0;
        if (usb_reg->TXCSRL & USB_TXCSRL1_TXRDY)
            usb_reg->TXCSRL = USB_TXCSRL1_CLRDT | USB_TXCSRL1_FLUSH;
        else
            usb_reg->TXCSRL = USB_TXCSRL1_CLRDT;
        usb_reg->TXIE |= BIT(ep_index);
    }
    else
    {
        usb_reg->RXMAXP = cfg->mps;
        usb_reg->RXCSRH = (cfg->attributes == USB_EP_TYPE_ISO) ? USB_RXCSRH1_ISO : 0;
        if (usb_reg->RXCSRL & USB_RXCSRL1_RXRDY)
            usb_reg->RXCSRL = USB_RXCSRL1_CLRDT | USB_RXCSRL1_FLUSH;
        else
            usb_reg->RXCSRL = USB_RXCSRL1_CLRDT;
        usb_reg->RXIE |= BIT(ep_index);
    }

    if (ep_index)
    {
        if (USB_EP_DIR_IS_IN(cfg->addr)) 
        {
            usb_reg->TXFIFO_SIZE[0] = USB_TX_BUF_ADDR;
            usb_reg->TXFIFO_SIZE[1] = USB_TX_FIFO_SZ << USB_TX_FIFO_SIZE_POS; // 32bytes
        }
        else
        {
            __ASSERT(usb_data->rx_fifo_addr < 16, "no enough fifo");
            usb_reg->RXFIFO_SIZE[0] = usb_data->rx_fifo_addr;
            switch(cfg->mps)
            {
            case 8:
                usb_reg->RXFIFO_SIZE[1] = 0 << USB_RX_FIFO_SIZE_POS;
                usb_data->rx_fifo_addr += 1;
            break;
            case 16:
                usb_reg->RXFIFO_SIZE[1] = 1 << USB_RX_FIFO_SIZE_POS;
                usb_data->rx_fifo_addr += 2;
            break;
            case 32:
                usb_reg->RXFIFO_SIZE[1] = 2 << USB_RX_FIFO_SIZE_POS;
                usb_data->rx_fifo_addr += 4;
            break;
            case 64:
                usb_reg->RXFIFO_SIZE[1] = 3 << USB_RX_FIFO_SIZE_POS;
                usb_data->rx_fifo_addr += 8;
            break;
            default:
                __ASSERT(0, "no enough fifo");
            break;
            }
        }
    }

    return 0;
}

static int udc_ls_shutdown(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    const struct udc_ls_config *usb_cfg = dev->config;

    usb_data->usb_instance->POWER |= USB_POWER_PWRDNPHY;
    usb_data->usb_instance->TXIE = 1;
    usb_data->usb_instance->RXIE = 0; 

    for (uint8_t i = 1; i < usb_cfg->num_endpoints; ++i)
    {
        usb_data->usb_instance->EPIDX = i;
        usb_data->usb_instance->TXFIFO_SIZE[0] = 0;
        usb_data->usb_instance->TXFIFO_SIZE[1] = 0;
        usb_data->usb_instance->RXFIFO_SIZE[0] = 0;
        usb_data->usb_instance->RXFIFO_SIZE[1] = 0;
    }

    return 0;
}

static int udc_ls_set_address(const struct device *dev, const uint8_t addr)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    usb_data->usb_instance->FADDR = addr;
    return 0;
}

static int udc_ls_host_wakeup(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    usb_data->usb_instance->POWER |= USB_POWER_RESUME;
    k_sleep(K_MSEC(10)); // max 15ms
    usb_data->usb_instance->POWER &= ~USB_POWER_RESUME;
    return 0;
}

static int usbd_ctrl_feed_dout(const struct device *dev, const size_t length)
{
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    struct net_buf *buf;
	struct udc_buf_info *bi;

    buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
    if (buf == NULL)
        return -ENOMEM;
    bi = udc_get_buf_info(buf);
    bi->data = true;

    udc_buf_put(ep_cfg, buf);
    return 0;
}

static void ls_handle_evt_setup(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_instance = usb_data->usb_instance;
    struct net_buf *buf;
    musb_set_active_ep(usb_instance, 0);

    buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, 8);
    if (buf == NULL)
    {
        udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
        return;
    }

    net_buf_add_mem(buf, &usb_data->setup, 8);
    udc_ep_buf_set_setup(buf);
    LOG_HEXDUMP_DBG(buf->data, buf->len, "setup token");

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_data_out(dev))
    {
        /* setup packet: host -> device, alloc and feed buffer for data out stage */
        if (usbd_ctrl_feed_dout(dev, udc_data_stage_length(buf)) == -ENOMEM)
        {
            udc_submit_ep_event(dev, buf, -ENOMEM);
        }
        /* We have prepared a new buffer and are now ready to receive data */
        usb_instance->CSRL0 = USB_CSRL0_RXRDYC;
    }
    else if (udc_ctrl_stage_is_data_in(dev))
    {
        /* setup packet: device -> host, notify upper layer */
        udc_ctrl_submit_s_in_status(dev);
    }
    else
    {
        udc_ctrl_submit_s_status(dev);
    }
}

static int udc_ctrl_out_state_update(const struct device *dev, struct net_buf *const buf)
{
    int err = 0;

    if (udc_ctrl_stage_is_status_out(dev)) {
        /* Status stage finished, notify upper layer */
        err = udc_ctrl_submit_status(dev, buf);
    }

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_status_in(dev)) {
        return udc_ctrl_submit_s_out_status(dev, buf);
    }

    return err;
}

static int udc_ctrl_in_state_update(const struct device *dev, struct net_buf *const buf)
{
    int err = 0;

    if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
        /* Status stage finished, notify upper layer */
        err = udc_ctrl_submit_status(dev, buf);
    }

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_status_out(dev)) {
        /* IN transfer finished, release buffer */
        net_buf_unref(buf);
    }

    return err;
}

static void _usb_ep0_txstate(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    struct net_buf *buf = udc_buf_peek(dev, USB_CONTROL_EP_IN);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
    uint16_t trans = MIN(buf->len, ep_cfg->mps);
    uint8_t csr = USB_CSRL0_TXRDY;

    fifo_write(&usb_reg->FIFO0_WORD, buf->data, trans);
    net_buf_pull(buf, trans);

    if (trans < ep_cfg->mps)
    {
        buf = udc_buf_get(dev, USB_CONTROL_EP_IN);
        udc_ctrl_in_state_update(dev, buf);
        usb_data->ep0_state = USB_EP0_STAGE_STATUSOUT;
        csr |= USB_CSRL0_DATAEND;
    }

    usb_reg->CSRL0= csr;
}

static void _usb_ep0_rxstate(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    struct net_buf *buf = udc_buf_peek(dev, USB_CONTROL_EP_OUT);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    uint8_t csr = 0;
    uint8_t ep0_count = usb_reg->COUNT0;
    size_t len = MIN(ep0_count, net_buf_tailroom(buf));

    if (len)
    {
        fifo_read(buf->data, (void *)&usb_reg->FIFO0_WORD, len);
        net_buf_add(buf, len);

        if (len < ep_cfg->mps || !net_buf_tailroom(buf))
        {
            usb_data->ep0_state = USB_EP0_STAGE_STATUSIN;
            csr |= USB_CSRL0_DATAEND;
            /* update stage */
            buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
            udc_ctrl_out_state_update(dev, buf);
        }
    }
    usb_reg->CSRL0 = csr;
}

static void ls_handle_evt_xfer(const struct device *dev, uint8_t ep)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_reg = usb_data->usb_instance;
    struct net_buf *buf = udc_buf_peek(dev, ep);
    struct udc_buf_info *bi = udc_get_buf_info(buf);

    musb_set_active_ep(usb_reg, 0);

    if (bi->data)
    {
        if (usb_data->ep0_state == USB_EP0_STAGE_TX)
        {
            /* usb thread ctx post event */
            _usb_ep0_txstate(dev);
        }
        else if (usb_data->ep0_state == USB_EP0_STAGE_RX)
        {
            /* irq post event */
            _usb_ep0_rxstate(dev);
        }
    }
    else if (bi->status)
    {
        /* usb thread ctx post event */
        buf = udc_buf_get(dev, USB_CONTROL_EP_IN);
        udc_ctrl_in_state_update(dev, buf);
        usb_reg->CSRL0 = USB_CSRL0_DATAEND | USB_CSRL0_RXRDYC | USB_CSRL0_TXRDY;
    }
}

static void ls_handle_evt_in_complete(const struct device *dev, uint8_t ep)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    linked_async_end(&usb_data->async_list, NULL, 0);
}

static void ls_handle_evt_in_zlp(const struct device *dev, uint8_t ep)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_instance = usb_data->usb_instance;

    musb_set_active_ep(usb_instance, USB_EP_GET_IDX(ep));
    usb_instance->TXCSRH |= USB_TXCSRH1_MODE;
    /* trigger irq, finish tx trans */
    usb_instance->TXCSRL = USB_TXCSRL1_TXRDY;
}

static void udc_ls_thread_handler(void *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    struct ls_event evt;

    while (1)
    {
        k_msgq_get(&usb_data->msgq, &evt, K_FOREVER);
        switch (evt.type)
        {
            case LS_EVT_EP0_XFER:
                ls_handle_evt_xfer(dev, evt.ep);
                break;
            case LS_EVT_SETUP:
                ls_handle_evt_setup(dev);
                break;
            case LS_EVT_IN_COMPLETE:
                ls_handle_evt_in_complete(dev, evt.ep);
                break;
            case LS_EVT_IN_ZLP:
                ls_handle_evt_in_zlp(dev, evt.ep);
                break;
            default:
                __ASSERT(0, "error");
        }
    }
}

static void _usbd_process_ep0(const struct device *dev)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_instance = usb_data->usb_instance;
    uint16_t len = usb_instance->COUNT0;
    uint16_t csr = usb_instance->CSRL0;

    if (csr & USB_CSRL0_DATAEND) {
        /*
         * If DATAEND is set we should not call the callback,
         * hence the status stage is not complete.
         */
        return;
    }

    if (csr & USB_CSRL0_STALLED) {
        /* Returned STALL packet to HOST. */
        usb_instance->CSRL0 = csr & ~USB_CSRL0_STALLED;
        usb_data->ep0_state         = USB_EP0_STAGE_IDLE;
        csr = usb_instance->CSRL0;
    }

    if (csr & USB_CSRL0_SETEND) {
        usb_instance->CSRL0 = USB_CSRL0_SETEND;
        switch (usb_data->ep0_state) {
        case USB_EP0_STAGE_TX:
            usb_data->ep0_state = USB_EP0_STAGE_STATUSOUT;
            break;
        case USB_EP0_STAGE_RX:
            usb_data->ep0_state = USB_EP0_STAGE_STATUSIN;
            break;
        default:
            __ASSERT(0, "SetupEnd came in a wrong ep0stage %d\n", usb_data->ep0_state);
            break;
        }
        csr = usb_instance->CSRL0;
        if (!(csr & USB_CSRL0_RXRDY)) {
            return;
        }
    }

    switch (usb_data->ep0_state) {
    case USB_EP0_STAGE_TX:
        /* irq on clearing txpktrdy */
        if ((csr & USB_CSRL0_TXRDY) == 0) {
            struct ls_event evt = {
                .type = LS_EVT_EP0_XFER,
                .ep = USB_CONTROL_EP_IN
            };
            /* send remain data */
            k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);
        }
        break;

    case USB_EP0_STAGE_RX:
        /* irq on set rxpktrdy */
        if (csr & USB_CSRL0_RXRDY) {
            struct ls_event evt = {
                .type = LS_EVT_EP0_XFER,
                .ep = USB_CONTROL_EP_OUT
            };
            /* receive data */
            k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);
        }
        break;
    case USB_EP0_STAGE_STATUSIN:
        /* end of sequence #2 or #3 (no data), host move status stage, another Endpoint 0 interrupt will be generated to indicate that the
            request has completed */
        if (usb_data->is_set_addr) {
            usb_data->is_set_addr = false;
            if (usb_data->setup.bRequest == USB_SREQ_SET_ADDRESS && usb_data->setup.bmRequestType == 0x00) {
                usb_instance->FADDR = usb_data->setup.wValue;
            }
        }
    case USB_EP0_STAGE_STATUSOUT:
        /* end of sequence #1, host move status stage, the interrupt is just a confirmation that the request
            completed successfully.*/
        if (csr & USB_CSRL0_RXRDY)
            goto setup;
        usb_data->ep0_state = USB_EP0_STAGE_IDLE;
        break;
    case USB_EP0_STAGE_IDLE:
        usb_data->ep0_state = USB_EP0_STAGE_SETUP;
    case USB_EP0_STAGE_SETUP:
    setup:
        // setup begin
        if (csr & USB_CSRL0_RXRDY) {
            if (len != 8) {
                __ASSERT(0,"SETUP packet len %d != 8 ?\n", len);
                break;
            }
            usb_data->is_set_addr = false;
            fifo_read(&usb_data->setup, (void *)&usb_instance->FIFO0_WORD, sizeof(usb_data->setup));

            struct ls_event evt = {
                .type = LS_EVT_SETUP,
                .ep = USB_CONTROL_EP_OUT
            };
            k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);

            if (usb_data->setup.wLength == 0) {
                /* No data stage, must send zlp to host */
                usb_data->ep0_state = USB_EP0_STAGE_STATUSIN;
                if (usb_data->setup.bRequest == USB_SREQ_SET_ADDRESS) {
                    usb_data->is_set_addr = true;
                }
            } else if (usb_data->setup.bmRequestType & USB_DIR_IN) {
                /* IN stage, send data to host */
                usb_data->ep0_state = USB_EP0_STAGE_TX;
                usb_instance->CSRL0 = USB_CSRL0_RXRDYC;
                while ((usb_instance->CSRL0 & USB_CSRL0_RXRDY) != 0);
            } else {
                /* OUT stage, recive data from host */
                usb_data->ep0_state = USB_EP0_STAGE_RX;
            }
        }
        break;
    default:
        /* "can't happen" */
        usb_data->ep0_state = USB_EP0_STAGE_IDLE;
        break;
    }
}

static void endpoint_tx_handler(const struct device *dev, reg_usb_t *reg, uint8_t ep_num)
{
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    struct net_buf *buf = udc_buf_peek(dev, ep_num | USB_EP_DIR_IN);
    struct ls_event evt = {
        .type = LS_EVT_IN_COMPLETE,
        .ep = ep_num | USB_EP_DIR_IN
    };

    musb_set_active_ep(reg, ep_num);
    if (reg->TXCSRL & USB_TXCSRL1_STALLED)
    {
        reg->TXCSRL &= ~(USB_TXCSRL1_STALLED | USB_TXCSRL1_UNDRN);
        return;
    }

    if (!buf->len && udc_ep_buf_has_zlp(buf))
    {
        udc_ep_buf_clear_zlp(buf);
        evt.type = LS_EVT_IN_ZLP;
        k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);
        return;
    }

    if (buf->len)
    {
        /* send remain data */
        udc_ls_tx(dev, ep_num, buf);
    }
    else
    {
        /* current data send complete */
        k_msgq_put(&usb_data->msgq, &evt, K_NO_WAIT);
        /* release buffer */
        buf = udc_buf_get(dev, ep_num | USB_EP_DIR_IN);
        udc_ep_set_busy(dev, ep_num | USB_EP_DIR_IN, false);
        /* send finish, notify upper layer */
        udc_submit_ep_event(dev, buf, 0);
    }
}

static void endpoint_rx_handler(const struct device *dev, reg_usb_t *reg, uint8_t ep_num)
{
    struct net_buf *buf = udc_buf_peek(dev, ep_num);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_num | USB_EP_DIR_OUT);
    uint16_t rx_len, read_count;

    musb_set_active_ep(reg, ep_num);
    read_count = reg->RXCOUNT;

    /* The USB protocol stack will definitely prepare the buffer in the future, so just return directly here. */
    if (buf == NULL)
        return;

    rx_len = MIN(MIN(net_buf_tailroom(buf), ep_cfg->mps), read_count);
    fifo_read(net_buf_add(buf, rx_len), &reg->FIFO0_WORD + ep_num, rx_len);

    if (!net_buf_tailroom(buf) || rx_len < ep_cfg->mps)
    {
        /* Buffer is ready, and post event */
        buf = udc_buf_get(dev, ep_cfg->addr);
        udc_submit_ep_event(dev, buf, 0);
    }
    reg->RXCSRL &= ~USB_RXCSRL1_RXRDY;
}

static void udc_ls_irq(const struct device *dev)
{
    uint_fast8_t is, txis, rxis;
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    reg_usb_t *usb_instance = usb_data->usb_instance;
    uint8_t old_ep = musb_get_active_ep(usb_data->usb_instance);

    is   = usb_instance->IS;
    txis = usb_instance->TXIS;
    rxis = usb_instance->RXIS;

    is &= usb_instance->IE;

    if (is & USB_IS_DISCON) {
    }

    if (is & USB_IS_SOF) {
        udc_submit_event(dev, UDC_EVT_SOF, 0);
    }

    if (is & USB_IS_RESET) {
        usb_instance->POWER |= USB_POWER_PWRDNPHY;
        usb_data->ep0_state = USB_EP0_STAGE_IDLE;
        udc_submit_event(dev, UDC_EVT_RESET, 0);
    }

    if (is & USB_IS_RESUME) {
        udc_submit_event(dev, UDC_EVT_RESUME, 0);
    }

    if (is & USB_IS_SUSPEND) {
        udc_set_suspended(dev, true);
        udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
    }

    txis &= usb_instance->TXIE;
    if (txis & USB_TXIE_EP0) {
        musb_set_active_ep(usb_instance, 0);
        _usbd_process_ep0(dev); 
        txis &= ~BIT(0);
    }

    while (txis) {
        unsigned const num = __builtin_ctz(txis);
        endpoint_tx_handler(dev, usb_instance, num);
        txis &= ~BIT(num);
    }

    rxis &= usb_instance->RXIE;
    while (rxis) {
        unsigned const num = __builtin_ctz(rxis);
        endpoint_rx_handler(dev, usb_instance, num);
        rxis &= ~BIT(num);
    }

    /* Resume the old endpoint. */
    musb_set_active_ep(usb_instance, old_ep);
}

static int udc_ls_driver_preinit(const struct device *dev)
{
    struct udc_data *data = dev->data;
    struct udc_ls_data *usb_data = (struct udc_ls_data *)udc_get_private(dev);
    const struct udc_ls_config *usb_cfg = dev->config;
    struct udc_ep_config *ep_cfg_out = usb_data->ep_cfg_out;
    struct udc_ep_config *ep_cfg_in = usb_data->ep_cfg_in;
    int err;

    /* init udc data */
    data->caps.hs = 0;
    data->caps.rwup = 1;
    data->caps.mps0 = UDC_MPS0_64;
    data->caps.addr_before_status = false;

    for (unsigned int i = 0; i < usb_cfg->num_endpoints; i++)
    {
        ep_cfg_out[i].caps.out = 1;

        if (i == 0)
        {
            ep_cfg_out[i].caps.control = 1;
            ep_cfg_out[i].caps.mps = usb_cfg->ep0_mps;
        }
        else
        {
            ep_cfg_out[i].caps.bulk = 1;
            ep_cfg_out[i].caps.interrupt = 1;
            ep_cfg_out[i].caps.iso = 1;
            ep_cfg_out[i].caps.mps = usb_cfg->ep_mps;
        }

        ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
        err = udc_register_ep(dev, &ep_cfg_out[i]);
        if (err != 0)
        {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }

    for (unsigned int i = 0; i < usb_cfg->num_endpoints; i++)
    {
        ep_cfg_in[i].caps.in = 1;

        if (i == 0)
        {
            ep_cfg_in[i].caps.control = 1;
            ep_cfg_in[i].caps.mps = usb_cfg->ep0_mps;
        }
        else
        {
            ep_cfg_in[i].caps.bulk = 1;
            ep_cfg_in[i].caps.interrupt = 1;
            ep_cfg_in[i].caps.iso = 1;
            ep_cfg_in[i].caps.mps = usb_cfg->ep_mps;
        }

        ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
        err = udc_register_ep(dev, &ep_cfg_in[i]);
        if (err != 0)
        {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }
    linked_async_init(&usb_data->async_list, async_tx_process, async_tx_end);

    return 0;
}

static const struct udc_api udc_ls_api = {
    .lock = udc_ls_lock,
    .unlock = udc_ls_unlock,
    .init = udc_ls_init,
    .enable = udc_ls_enable,
    .disable = udc_ls_disable,
    .shutdown = udc_ls_shutdown,
    .set_address = udc_ls_set_address,
    .host_wakeup = udc_ls_host_wakeup,
    .ep_enable = udc_ls_ep_enable,
    .ep_disable = udc_ls_ep_disable,
    .ep_set_halt = udc_ls_ep_set_halt,
    .ep_clear_halt = udc_ls_ep_clear_halt,
    .ep_enqueue = udc_ls_enqueue,
    .ep_dequeue = udc_ls_dequeue,
};

#define DT_DRV_COMPAT       linkedsemi_ls_fs_usb

#define UDC_LS_DEVICE_DEFINE(n)       \
    K_THREAD_STACK_DEFINE(udc_ls_stack_##n, CONFIG_UDC_LS_FS_THREAD_STACK_SIZE); \
                                            \
    static void udc_ls_thread_##n(void *dev, void *arg1, void *arg2)  \
    {          \
        while (true) {        \
            udc_ls_thread_handler(dev);     \
        }         \
    }          \
                                            \
    static void udc_ls_make_thread_##n(const struct device *dev)  \
    {          \
        struct udc_ls_data *priv = udc_get_private(dev);   \
                                            \
        k_thread_create(&priv->thread,     \
                udc_ls_stack_##n,     \
                K_THREAD_STACK_SIZEOF(udc_ls_stack_##n),  \
                udc_ls_thread_##n,    \
                (void *)dev, NULL, NULL,    \
                K_PRIO_COOP(CONFIG_UDC_LS_FS_THREAD_PRIORITY), \
                K_ESSENTIAL,      \
                K_NO_WAIT);      \
        k_thread_name_set(&priv->thread, dev->name);   \
    }      \
    static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];   \
    static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];   \
    static struct udc_ep_item ep_tx_##n[DT_INST_PROP(n, num_bidir_endpoints)];\
                                            \
    static void udc_ls_irq_connect##n(const struct device *dev)     \
    {                                   \
        IRQ_CONNECT(DT_INST_IRQN(n),                    \
                DT_INST_IRQ(n, priority),               \
                udc_ls_irq,                   \
                DEVICE_DT_INST_GET(n),              \
                0);                             \
                                            \
        irq_enable(DT_INST_IRQN(n));                    \
    }                                   \
    static void udc_ls_irq_disconnect##n(const struct device *dev)      \
    {                                       \
        irq_disable(DT_INST_IRQN(n));                       \
    }               \
    IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(n))); \
    static const struct udc_ls_config udc_ls_config_##n = {   \
        .num_endpoints = DT_INST_PROP(n, num_bidir_endpoints),   \
        .ep0_mps = USB_EP0_SIZE,      \
        .ep_mps = DT_INST_PROP(n, ram_size),        \
        .make_thread = udc_ls_make_thread_##n,      \
        .irq_connect = udc_ls_irq_connect##n,          \
        .irq_disconnect = udc_ls_irq_disconnect##n,    \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), )) \
        IF_ENABLED(DT_HAS_CLOCKS(n), (.ccfg = LS_DT_CLK_CFG_ITEM(n), )) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n), )) \
    };          \
                                            \
    static struct udc_ls_data udc_priv_##n = {     \
        .usb_instance = (reg_usb_t *)DT_INST_REG_ADDR(n),        \
        .ep_cfg_in = ep_cfg_in_##n,                 \
        .ep_cfg_out = ep_cfg_out_##n,               \
        .ep_tx = ep_tx_##n,                         \
        .rx_fifo_addr = USB_TX_BUF_ADDR,        \
        .ep0_state = USB_EP0_STAGE_SETUP,        \
        .is_set_addr = false                    \
    };          \
                                                \
    static struct udc_data udc_data_##n = {      \
        .mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),   \
        .priv = &udc_priv_##n,       \
    };          \
                                            \
    DEVICE_DT_INST_DEFINE(n, udc_ls_driver_preinit, NULL,   \
                    &udc_data_##n, &udc_ls_config_##n,   \
                    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,  \
                    &udc_ls_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_LS_DEVICE_DEFINE)
