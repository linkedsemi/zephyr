#include "xhci.h"
#include <string.h>
#include <stdio.h>

#define	XHCI_PORT_RO    ((1<<0) | (1<<3) | (0xf<<10) | (1<<30))
#define XHCI_PORT_RWS   ((0xf<<5) | (1<<9) | (0x3<<14) | (0x7<<25))

#define EP_CTX_MAX_PACKET(n)         ((n) << 16)
#define EP_CTX_TYPE(n)               ((n) << 3)
/* Note: CErr does not apply to Isoch endpoints and shall be set to ‘0’ if EP Type = Isoch Out ('1') or Isoch In ('5'). */
#define EP_CTX_ERR_CNT(n)            ((n & 0x3) << 1)

#define SLOT_CTX_ENTRIES(n)                     ((n) << 27)
#define SLOT_CTX_ROOT_STR(n)                    ((n) << 0)
#define SLOT_CTX_SPEED(n)                       ((n) << 20)
#define SLOT_CTX_ROOTHUB_PORT_NUM(n)            ((n) << 16)
#define EP_DIR_IN(ep)                           ((ep & 0x80) ? 1 : 0)

enum XHCI_PLS
{
    PLS_U0_STATE,
    PLS_U1_STATE,
    PLS_U2_STATE,
    PLS_U3_STATE,
    PLS_DISABLE_STATE,
    PLS_RXDETECT_STATE,
    PLS_INACTIVE_STATE,
    PLS_POLLING_STATE,
};

static const char *speed_map[] = {
    "invalid",
    "full-speed",
    "low-speed",
    "high-speed",
    "supper-speed",
    "supper-speed-plus"
};

static void command_complete_handle(struct xhci_hcd *hcd, struct xhci_trb *evt_trb)
{
    struct xhci_trb *cmd_trb = (struct xhci_trb *)((uint32_t)evt_trb->paramater);

    switch (cmd_trb->control & TRB_TYPE_BITMASK)
    {
    case TRB_TYPE(TRB_ENABLE_SLOT):
        XHCI_LOG_DBG("TRB_ENABLE_SLOT\n");
        break;
    case TRB_TYPE(TRB_DISABLE_SLOT):
        XHCI_LOG_DBG("TRB_DISABLE_SLOT\n");
        break;
    case TRB_TYPE(TRB_ADDR_DEV):
        XHCI_LOG_DBG("TRB_ADDR_DEV\n");
        break;
    case TRB_TYPE(TRB_CONFIG_EP):
        break;
    case TRB_TYPE(TRB_CMD_NOOP):
        XHCI_LOG_DBG("TRB_CMD_NOOP\n");
        break;
    default:
        break;
    }
}

static uint32_t xhci_port_state_to_neutral(uint32_t state)
{
    return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

static void port_status_handle(struct xhci_hcd *hcd, struct xhci_trb *evt_trb)
{
    uint16_t port_id = GET_PORT_ID(evt_trb->paramater) - 1;
    uint32_t port_sc = hcd->op_reg->port[port_id].portsc;
    uint8_t dev_speed;

    switch (XHCI_OP_PORTSC_LINK_STATE(port_sc))
    {
        case PLS_U0_STATE:
            /*  usb3.0 --> u0 state or usb2.0 --> enable */
            /* Now, we can get device speed */
            dev_speed = XHCI_OP_PORTSC_SPEED(port_sc);
            /* clear connect status change bit */
            port_sc = xhci_port_state_to_neutral(port_sc);
            port_sc |= XHCI_OP_PORTSC_PRC;
            hcd->op_reg->port[port_id].portsc = port_sc;
            XHCI_LOG_DBG("portid: %d, %s device connected.\n", port_id+1, speed_map[dev_speed]);
            return;
        break;
        case PLS_POLLING_STATE:
            /* usb2.0 device, reset port */
            port_sc = xhci_port_state_to_neutral(port_sc);
            port_sc |= XHCI_OP_PORTSC_RESET;
            hcd->op_reg->port[port_id].portsc |= port_sc;
            /* wait port reset change bit */
            while (!(hcd->op_reg->port[port_id].portsc & XHCI_OP_PORTSC_PRC));
            XHCI_LOG_DBG("port reset status = 0x%x.\n", hcd->op_reg->port[port_id].portsc);
        break;
        case PLS_RXDETECT_STATE:
            /*  usb3.0 --> RxDetect or usb2.0 --> Disconnected */
            /* clear connect status change bit */
            port_sc = xhci_port_state_to_neutral(port_sc);
            port_sc |= XHCI_OP_PORTSC_CSC;
            hcd->op_reg->port[port_id].portsc = port_sc;
            XHCI_LOG_DBG("portid: %d disconnected, port_sc = %x.\n", port_id+1, port_sc);
            break;
        default:
            XHCI_LOG_DBG("portsc link state = 0x%x.\n", XHCI_OP_PORTSC_LINK_STATE(port_sc));
            break;
    }
}

void xhci_isr(struct xhci_hcd *hcd)
{
    uint32_t cur_rd_index = hcd->event_ring.read_index;
    struct xhci_trb *event_trb = &hcd->event_ring.trb[cur_rd_index];
    uint64_t erdp = hcd->runtime_reg->ir_set[0].erdp;

    hcd->op_reg->status |= XHCI_USBSTS_EINT;
    hcd->runtime_reg->ir_set[0].iman |= XHCI_IMAIN_IP;

    /* Invalidate all TRBs to prevent any single TRB from being misaligned with a cache line */
    xhci_cache_invalid(hcd->event_ring.trb, hcd->event_ring.num_trb * sizeof(struct xhci_trb));

    while (1)
    {
        if ((event_trb->control & TRB_CYCLE) != hcd->event_ring.cycle_bit)
            break;

        /* handle event */
        switch (event_trb->control & TRB_TYPE_BITMASK)
        {
        case TRB_TYPE(TRB_COMPLETION):
            command_complete_handle(hcd, event_trb);
            hcd->evt_trb = *event_trb;
            xhci_event_complete();
            break;
        case TRB_TYPE(TRB_PORT_STATUS):
            port_status_handle(hcd, event_trb);
            break;
        case TRB_TYPE(TRB_TRANSFER):
            hcd->evt_trb = *event_trb;
            xhci_event_complete();
            break;
        case TRB_TYPE(TRB_DEV_NOTE):
            break;
        default:
            break;
        }

        hcd->event_ring.read_index = (hcd->event_ring.read_index + 1) % hcd->event_ring.num_trb;
        if (hcd->event_ring.read_index == 0)
            hcd->event_ring.cycle_bit ^=1;
        event_trb = &hcd->event_ring.trb[hcd->event_ring.read_index];
    }

    if (cur_rd_index != hcd->event_ring.read_index)
    {
        erdp &= ERST_PTR_MASK;
        erdp |= (size_t)event_trb;
    }
    erdp |= ERST_EHB;
    /* update a new trb for xhci */
    hcd->runtime_reg->ir_set[0].erdp = erdp;
}

int xhci_port_reset(struct xhci_hcd *hcd, int port)
{
    uint32_t temp = xhci_port_state_to_neutral(hcd->op_reg->port[port].portsc);
    temp |= XHCI_OP_PORTSC_RESET;
    hcd->op_reg->port[port].portsc = temp;
    return 0;
}

uint32_t xhci_get_portsc(struct xhci_hcd *hcd, int port)
{
    return hcd->op_reg->port[port].portsc;
}

int xhci_ring_init(struct xhci_ring *ring, struct xhci_trb *trb, uint16_t num_trb, ring_type_t type)
{
    memset(trb, 0, sizeof(struct xhci_trb) * num_trb);

    ring->cycle_bit = 1;
    ring->trb = trb;
    ring->num_trb = num_trb;
    ring->read_index = 0;
    ring->write_index = 0;
    ring->type = type;

    if (type != EVENT_RING)
    {
        ring->trb[num_trb - 1].control = TRB_TYPE(TRB_LINK) | LINK_TOGGLE;
        ring->trb[num_trb - 1].paramater = (size_t)trb;
    }

    return 0;
}

void xhci_ring_cmd_doorbell(struct xhci_hcd *xhci)
{
    xhci->db_reg->doorbell[0] = 0x0;
}

void xhci_ring_ep_doorbell(struct xhci_hcd *xhci, size_t slot, size_t ep_num, size_t stream_id)
{
    xhci->db_reg->doorbell[slot] = ep_num | stream_id << 16;
}

static void xhci_enqueue_trb(struct xhci_hcd *xhci, struct xhci_ring *ring, struct xhci_trb *trb)
{
    struct xhci_trb *ring_trb = &ring->trb[ring->write_index];
    /* enqueue ring trb */
    *ring_trb = *trb;
    /* update write point */
    ring->write_index++;
    /* Flush all TRBs to prevent any single TRB from being misaligned with a cache line */
    xhci_cache_flush(ring->trb, ring->num_trb * sizeof(struct xhci_trb));

    if (ring->type != EVENT_RING && ring->write_index == ring->num_trb - 1)
    {
        ring_trb = &ring->trb[ring->write_index];
        ring_trb->control ^= TRB_CYCLE;
        ring->cycle_bit ^= 1;
        ring->write_index = 0;
    }
}

static int xhci_command_submit(struct xhci_hcd *xhci, struct xhci_trb *trb)
{
    xhci_enqueue_lock();
    xhci_enqueue_trb(xhci, &xhci->cmd_ring, trb);
    xhci_ring_cmd_doorbell(xhci);
    xhci_event_wait();
    *trb = xhci->evt_trb;
    xhci_enqueue_unlock();

    return 0;
}

int xhci_cmd_disable_slot(struct xhci_hcd *xhci, int slot)
{
    struct xhci_trb trb = {
        .paramater = 0,
        .status = 0,
        .control = TRB_TYPE(TRB_DISABLE_SLOT) | SLOT_ID_FOR_TRB(slot) | xhci->cmd_ring.cycle_bit
    };
    xhci_command_submit(xhci, &trb);

    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? 0 : -1;
}

int xhci_cmd_enable_slot(struct xhci_hcd *xhci)
{
    struct xhci_trb trb = {
        .paramater = 0,
        .status = 0,
        .control = TRB_TYPE(TRB_ENABLE_SLOT) | xhci->cmd_ring.cycle_bit
    };
    xhci_command_submit(xhci, &trb);

    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? TRB_TO_SLOT_ID(trb.control) : -1;
}

int xhci_cmd_noop(struct xhci_hcd *xhci)
{
    struct xhci_trb trb = {
        .paramater = 0,
        .status = 0,
        .control = TRB_TYPE(TRB_CMD_NOOP) | xhci->cmd_ring.cycle_bit
    };
    xhci_command_submit(xhci, &trb);

    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? 0 : -1;
}

struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_hcd *hcd, struct xhci_ctx *ctx)
{
    if (ctx->type == XHCI_CTX_TYPE_DEVICE)
        return (struct xhci_slot_ctx *)ctx->ctx;
    return (struct xhci_slot_ctx *)(ctx->ctx + CTX_SIZE(hcd->cap_reg->hccparams1));
}

struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_hcd *hcd, struct xhci_ctx *ctx, int ep_index)
{
    ep_index++;
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        ep_index++; // input ctx has input_control_ctx

    return (struct xhci_ep_ctx *)(ctx->ctx + ep_index * CTX_SIZE(hcd->cap_reg->hccparams1));
}

int xhci_alloc_device(struct xhci_hcd *xhci)
{
    /* enable slot */
    int slot = xhci_cmd_enable_slot(xhci);

    uint8_t *out_ctx = xhci_mem_alloc(XHCI_ALIGN, 2048);
    uint8_t *in_ctx = xhci_mem_alloc(XHCI_ALIGN, 2112);
    struct xhci_trb *ep0_trb = xhci_mem_alloc(XHCI_ALIGN, XHCI_RING_TRB_MAX_NUM * sizeof (struct xhci_trb));

    if (!out_ctx || !in_ctx || !ep0_trb)
    {
        xhci_mem_free(out_ctx);
        xhci_mem_free(in_ctx);
        xhci_mem_free(ep0_trb);
        XHCI_LOG_DBG("alloc device fail, can't alloc memory!!\n");
        return -1;
    }
    xhci->device[slot].device_ctx.ctx = out_ctx;
    xhci->device[slot].device_ctx.type = XHCI_CTX_TYPE_DEVICE;
    xhci->device[slot].device_ctx.size = HCC_64BYTE_CONTEXT(xhci->cap_reg->hccparams1) ? 2048 : 1024;

    /* pust device contex table */
    xhci->dev_ctx_tab[slot] = (size_t)out_ctx;

    xhci->device[slot].input_ctx.ctx = in_ctx;
    xhci->device[slot].input_ctx.size = HCC_64BYTE_CONTEXT(xhci->cap_reg->hccparams1) ? 2112 : 1056;
    xhci->device[slot].input_ctx.type = XHCI_CTX_TYPE_INPUT;

    memset(xhci->device[slot].device_ctx.ctx, 0, xhci->device[slot].device_ctx.size);
    memset(xhci->device[slot].input_ctx.ctx, 0, xhci->device[slot].input_ctx.size);

    /* ep0 transfer ring init */
    xhci_ring_init(&xhci->device[slot].ep[0].ep_ring, ep0_trb, XHCI_RING_TRB_MAX_NUM, CTRL_RING);

    /* prepare input ctx for address device command */
    xhci->device[slot].slot = slot;

    return slot;
}

int xhci_free_device(struct xhci_hcd *xhci, int slot)
{
    return 0;
}

int xhci_cmd_address_device(struct xhci_hcd *xhci, int slot, xhci_addr_dev_type_t type)
{
    struct xhci_trb trb;
    struct xhci_input_control_ctx *control_ctx;
    struct xhci_slot_ctx *slot_ctx;
    struct xhci_ep_ctx *ep0_ctx;
    uint16_t max_packet;

    control_ctx = (struct xhci_input_control_ctx *)xhci->device[slot].input_ctx.ctx;
    control_ctx->add_flags = SLOT_FLAG | EP0_FLAG;
    control_ctx->drop_flags = 0;

    slot_ctx = xhci_get_slot_ctx(xhci, &xhci->device[slot].input_ctx);
    /*
        Root Hub
        ├─Port1─DeviceA
        │  RouteString=0x0
        ├─Port2─Hub1
        │        ├─Port3─DeviceB (根→Hub1的3号口)
        │        │  RouteString=0x3
        │        └─Port5─Hub2
        │               └─Port2─DeviceC (根→Hub1 5号→Hub2 2号)
        │                  RouteString= (2<<4)|5 = 0x52
        └─Port3─DeviceD
            RouteString=0x0
    */

    /* Only the control endpoint is valid - one endpoint context */
    slot_ctx->dev_info = SLOT_CTX_ENTRIES(1) | SLOT_CTX_ROOT_STR(0) | SLOT_CTX_SPEED(xhci->device[slot].speed);
    slot_ctx->dev_info2 = SLOT_CTX_ROOTHUB_PORT_NUM(1);
    ep0_ctx = xhci_get_ep_ctx(xhci, &xhci->device[slot].input_ctx, 0);

    if (type == DEFAULT_TYPE)
    {
        switch (xhci->device[slot].speed)
        {
            case USB_SPEED_LOW:
                max_packet = 8;
                break;
            case USB_SPEED_FULL:
            case USB_SPEED_HIGH:
                max_packet = 64;
                break;
            default:
                max_packet = -1;
                break;
        }
        ep0_ctx->ep_info[0] = 0;
        ep0_ctx->ep_info[1] = EP_CTX_TYPE(EP_CONTROL_BIDIR) | EP_CTX_MAX_PACKET(max_packet) | EP_CTX_ERR_CNT(3); // err_cnt not apply to Isoch endpoints
        /* ep ctx update, Dcs and TR Dequeue Pointer */
        ep0_ctx->deq = (size_t)xhci->device[slot].ep[0].ep_ring.trb | xhci->device[slot].ep[0].ep_ring.cycle_bit;
    }
    else
    {
        /* update input ep0 ctx deq */
        uint32_t write_index = xhci->device[slot].ep[0].ep_ring.write_index;
        ep0_ctx->deq = (size_t)(xhci->device[slot].ep[0].ep_ring.trb + write_index) | xhci->device[slot].ep[0].ep_ring.cycle_bit;
    }
    xhci_cache_flush(control_ctx, xhci->device[slot].input_ctx.size);

    /* prepare command trb */
    trb.paramater = (size_t)control_ctx;
    trb.status = 0;
    trb.control = SLOT_ID_FOR_TRB(slot) | TRB_TYPE(TRB_ADDR_DEV) | (type == DEFAULT_TYPE ? TRB_BSR : 0) | xhci->cmd_ring.cycle_bit;
    xhci_command_submit(xhci, &trb);
    /* check command exec resule */
    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? 0 : -1;
}

void xhci_set_dev_speed(struct xhci_hcd *xhci, int slot, enum xhci_usb_speed speed)
{
    xhci->device[slot].speed = speed;
}

enum xhci_usb_speed xhci_get_port_speed(struct xhci_hcd *xhci, int port_id)
{
    return XHCI_OP_PORTSC_SPEED(xhci->op_reg->port[port_id].portsc);
}

static int xhci_ep_2_index(uint8_t ep, xhci_ep_type_t type)
{
    /* index of endpoint contex */
    if (type == EP_CONTROL_BIDIR)
        ep = (ep & 0xf) * 2;
    else
        ep = (ep & 0xf) * 2 - (ep & 0x80 ? 0 : 1);
    return ep;
}

int xhci_xfer_control(struct xhci_hcd *xhci, int slot, int ep, struct usb_setup_packet *req, void *data, uint32_t length)
{
    int xfer_len = -1;
    uint8_t ep_index = xhci_ep_2_index(ep, EP_CONTROL_BIDIR);
    struct xhci_ring *ep_ring = &xhci->device[slot].ep[ep_index].ep_ring;
    struct xhci_generic_trb trb = {0};

    xhci_enqueue_lock();
    trb.field[0] = req->bmRequestType | req->bRequest << 8 | req->wValue << 16;
    trb.field[1] = req->wIndex | req->wLength << 16;
    trb.field[2] = 8;
    trb.field[3] = TRB_TYPE(TRB_SETUP) | TRB_IDT | ep_ring->cycle_bit;

    if (data && length)
    {
        if (req->bmRequestType & 0x80)
            trb.field[3] |= TRB_TRT(TRB_DATA_IN);
        else
            trb.field[3] |= TRB_TRT(TRB_DATA_OUT);
    }
    xhci_enqueue_trb(xhci, ep_ring, (struct xhci_trb *)&trb);

    if (data && length)
    {
        /* Data Stage TRB */
        trb.field[0] = (size_t)data;
        trb.field[1] = 0;
        trb.field[2] = length;
        trb.field[3] = TRB_TYPE(TRB_DATA) | ep_ring->cycle_bit;

        if (req->bmRequestType & 0x80)
            trb.field[3] |= (TRB_ISP | TRB_DIR_IN);
        xhci_enqueue_trb(xhci, ep_ring, (struct xhci_trb *)&trb);
    }

    /* Status Stage TRB */
    trb.field[0] = 0;
    trb.field[1] = 0;
    trb.field[2] = 0;
    trb.field[3] = TRB_TYPE(TRB_STATUS) | ep_ring->cycle_bit;

    if (data && length && req->bmRequestType & 0x80)
        trb.field[3] |= TRB_ISP;
    else
        trb.field[3] |= TRB_ISP | TRB_DIR_IN;

    xhci_enqueue_trb(xhci, ep_ring, (struct xhci_trb *)&trb);
    /* ring ep0 doorbell */
    xhci_ring_ep_doorbell(xhci, slot, ep_index + 1, 0);
    /* wait data stage transfer event */
    xhci_event_wait();
    xhci_enqueue_unlock();

    trb = *(struct xhci_generic_trb *)&xhci->evt_trb;

    if (GET_COMP_CODE(trb.field[2]) == COMP_SHORT_PACKET)
        xfer_len = trb.field[2] & 0xffffff;
    else if (GET_COMP_CODE(trb.field[2]) == COMP_SUCCESS)
        xfer_len = 0;

    return xfer_len;
}

int xhci_xfer_noop(struct xhci_hcd *xhci, int slot, uint8_t ep, xhci_ep_type_t type)
{
    struct xhci_trb trb = {0};
    uint8_t ep_index = xhci_ep_2_index(ep, type);
    struct xhci_ring *ep_ring = &xhci->device[slot].ep[ep_index].ep_ring;
    trb.control = TRB_TYPE(TRB_TR_NOOP) | 1 << 5 | ep_ring->cycle_bit;

    xhci_enqueue_lock();
    xhci_enqueue_trb(xhci, ep_ring, &trb);
    /* ring doorbell */
    xhci_ring_ep_doorbell(xhci, slot, ep_index + 1, 0);
    xhci_event_wait();
    trb = xhci->evt_trb;
    xhci_enqueue_unlock();

    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? 0 : -1;
}

/* 
* After the xHCI retrieves this TRB, it can know how much data remains to be transferred 
* based on the TD_SIZE field, which helps improve data transfer efficiency inside the xHCI controller.
* 
* What is TD_SIZE? How to calculate it?
*     TD_SIZE: After processing the current TRB, the entire TD (Transfer Descriptor) still has 
*              TD_SIZE number of packets remaining to be transferred.
*
*     Three elements are needed:
*         total_data_len --> Total data size
*         maxpacket      --> Maximum data per transfer for this endpoint
*         number of trb  --> Number of TRBs required to form the TD (these TRBs will share the total_data_len)
*
*     Example:
*         total_data_len --> 8192
*         maxpacket      --> 512
*         number of trb  --> 4
*
*         TD Packet count = ROUNDUP(8192 / 512) = 16
*
*         If each TRB equally shares 8192, then one TRB transfers: 8192 / 4 = 2048
*
*         Calculation for each TRB:
*         First TRB TD_SIZE:  16 - ROUNDDOWN(2048 / 512) = 16 - 4 = 12
*         Second TRB TD_SIZE: 16 - ROUNDDOWN((2048 + 2048) / 512) = 16 - 8 = 8
*         Third TRB TD_SIZE:  16 - ROUNDDOWN((2048 + 2048 + 2048) / 512) = 16 - 12 = 4
*         Fourth TRB TD_SIZE: 0 (always 0 for the last TRB in TD)
*
* For simplicity, this function uses only one TRB to transfer data, so the TD_SIZE field does not need to be considered.
*/
int xhci_xfer_bulk(struct xhci_hcd *xhci, int slot, int ep, void *data, int length)
{
    int xfer_len = -1;
    struct xhci_generic_trb trb = {0};
    uint8_t ep_index = xhci_ep_2_index(ep, EP_BULK_IN);
    struct xhci_ring *ep_ring = &xhci->device[slot].ep[ep_index].ep_ring;

    trb.field[0] = (size_t)data;
    trb.field[1] = 0;
    trb.field[2] = length;
    trb.field[3] = TRB_IOC | TRB_TYPE(TRB_NORMAL) | ep_ring->cycle_bit;

    /* Set the TRB_ISP only for IN direction. */
    if (EP_DIR_IN(ep))
        trb.field[3] |= TRB_ISP;

    xhci_enqueue_lock();
    xhci_enqueue_trb(xhci, ep_ring, (struct xhci_trb *)&trb);
    xhci_ring_ep_doorbell(xhci, slot, ep_index + 1, 0);
    xhci_event_wait();
    xhci_enqueue_unlock();

    trb = *(struct xhci_generic_trb *)&xhci->evt_trb;

    if (GET_COMP_CODE(trb.field[2]) == COMP_SHORT_PACKET)
        xfer_len = trb.field[2] & 0xffffff;
    else if (GET_COMP_CODE(trb.field[2]) == COMP_SUCCESS)
        xfer_len = 0;

    return xfer_len;
}

int xhci_xfer_interrupt(struct xhci_hcd *xhci, int slot, int ep, void *data, int legth)
{
    return 0;
}

void xhci_slot_ctx_copy(struct xhci_hcd *xhci, int slot)
{
    struct xhci_slot_ctx *in, *out;

    in = xhci_get_slot_ctx(xhci, &xhci->device[slot].input_ctx);
    out = xhci_get_slot_ctx(xhci, &xhci->device[slot].device_ctx);
    in->dev_info = out->dev_info;
    in->dev_info2 = out->dev_info2;
    in->tt_info = out->tt_info;
    in->dev_state = out->dev_state;
}

int xhci_cmd_add_endpoint(struct xhci_hcd *xhci, int slot, struct xhci_ep_config *ep_config)
{
    struct xhci_trb trb;
    struct xhci_input_control_ctx *control_ctx;
    struct xhci_ep_ctx *ep_ctx;
    struct xhci_slot_ctx *in;
    uint8_t ep_index =  xhci_ep_2_index(ep_config->ep_addr, ep_config->ep_type);
    struct xhci_trb *ep_trb = xhci_mem_alloc(XHCI_ALIGN, XHCI_RING_TRB_MAX_NUM * sizeof (struct xhci_trb));

    control_ctx = (struct xhci_input_control_ctx *)xhci->device[slot].input_ctx.ctx;
    control_ctx->add_flags = SLOT_FLAG;
    control_ctx->drop_flags = 0;
    /* slot ctx no update */
    xhci_slot_ctx_copy(xhci, slot);

    in = xhci_get_slot_ctx(xhci, &xhci->device[slot].input_ctx);
    in->dev_info &= ~SLOT_CTX_ENTRIES(0x1f);
    // This field indicates the size of the Device Context structure.
    // For example, ((Context Entries + 1) * 32 bytes) = Total bytes for this structure.
    in->dev_info |= SLOT_CTX_ENTRIES(ep_index + 1);
    control_ctx->add_flags |= 1 << (ep_index + 1);

    ep_ctx = xhci_get_ep_ctx(xhci, &xhci->device[slot].input_ctx, ep_index);
    xhci_ring_init(&xhci->device[slot].ep[ep_index].ep_ring, ep_trb, XHCI_RING_TRB_MAX_NUM, CTRL_RING);

    ep_ctx->ep_info[0] = ep_config->ep_interval << 16;
    ep_ctx->ep_info[1] = EP_CTX_MAX_PACKET(ep_config->ep_mps) | EP_CTX_TYPE(ep_config->ep_type) | EP_CTX_ERR_CNT(3); // retry 3 times
    ep_ctx->deq = (size_t)xhci->device[slot].ep[ep_index].ep_ring.trb | xhci->device[slot].ep[ep_index].ep_ring.cycle_bit;
    ep_ctx->tx_info = 0;
    xhci_cache_flush(control_ctx, xhci->device[slot].input_ctx.size);

    trb.paramater = (size_t)control_ctx;
    trb.status = 0;
    trb.control = SLOT_ID_FOR_TRB(slot) | TRB_TYPE(TRB_CONFIG_EP) | xhci->cmd_ring.cycle_bit;

    /*
        With reference to Section 4.5.3 "Slot States" in the xHCI specification, 
        the slot must be in the Addressed state to use the Configure Endpoint command.
    */
    xhci_command_submit(xhci, &trb);
    /* check command exec resule */
    return GET_COMP_CODE(trb.status) == COMP_SUCCESS ? 0 : -1;
}

int xhci_cmd_drop_endpoint(struct xhci_hcd *xhci, int slot, uint8_t ep)
{
    return 0;
}

int xhci_cmd_reset_device(struct xhci_hcd *xhci, int slot)
{
    struct xhci_trb trb = {
        .paramater = 0,
        .control = 0,
        .status = TRB_TYPE(TRB_RESET_DEV) | SLOT_ID_FOR_TRB(slot) | xhci->cmd_ring.cycle_bit
    };
    xhci_command_submit(xhci, &trb);

    return 0;
}

static int xhci_command_ring_init(struct xhci_hcd *hcd)
{
    xhci_ring_init(&hcd->cmd_ring, hcd->command_trb, XHCI_RING_TRB_MAX_NUM, COMMAND_RING);
    hcd->op_reg->cmd_ring = (size_t)hcd->cmd_ring.trb | hcd->cmd_ring.cycle_bit;

    return 0;
}

static int xhci_event_ring_init(struct xhci_hcd *hcd)
{
    static XHCI_MEM_ALIGN(64) struct xhci_erst_entry erst_table[1];

    xhci_ring_init(&hcd->event_ring, hcd->event_trb, XHCI_RING_TRB_MAX_NUM, EVENT_RING);
    hcd->erst.entry = erst_table;
    hcd->erst.entry[0].seg_addr = (size_t)hcd->event_ring.trb;
    hcd->erst.entry[0].seg_size = hcd->event_ring.num_trb;
    xhci_cache_flush(hcd->erst.entry, sizeof (struct xhci_erst_entry));

    hcd->runtime_reg->ir_set[0].erstba = (size_t)hcd->erst.entry;
    hcd->runtime_reg->ir_set[0].erstsz = 1;
    hcd->runtime_reg->ir_set[0].erdp = (size_t)hcd->event_ring.trb;

    return 0;
}

int xhci_device_ctx_show(struct xhci_hcd *hcd, size_t slot)
{
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(hcd, &hcd->device[slot].device_ctx);
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(hcd, &hcd->device[slot].device_ctx, 0);
    xhci_cache_invalid(slot_ctx, sizeof(struct xhci_slot_ctx));
    xhci_cache_invalid(ep_ctx, sizeof(struct xhci_ep_ctx));

    XHCI_LOG_DBG("slot_ctx: %x %x %x %x\n", slot_ctx->dev_info, slot_ctx->dev_info2, slot_ctx->tt_info, slot_ctx->dev_state);
    XHCI_LOG_DBG("ep0_bidir_ctx: %x %x %x %x\n", ep_ctx->ep_info[0], ep_ctx->ep_info[1], (size_t)ep_ctx->deq, ep_ctx->tx_info);
    for (uint8_t i = 0; i < 30; i++)
    {
        ep_ctx = xhci_get_ep_ctx(hcd, &hcd->device[slot].device_ctx, i+1);
        XHCI_LOG_DBG("ep%d_%s_ctx: %x %x %x %x\n", (i/2)+1, i%2 ? "in":"out", ep_ctx->ep_info[0], ep_ctx->ep_info[1], (size_t)ep_ctx->deq, ep_ctx->tx_info);
    }

    return 0;
}

int xhci_run(struct xhci_hcd *hcd)
{
    /* Event Interrupt Enable */
    hcd->op_reg->command |= CMD_EIE;

    /* Interrupter Management Register */
    hcd->runtime_reg->ir_set[0].iman = 1 << 1;
    hcd->runtime_reg->ir_set[0].imod = 500U;
    /* start xhci */
    hcd->op_reg->command |= CMD_RUN;
    while (hcd->op_reg->status & STS_HALT);

    return 0;
}

static int xhci_reset(struct xhci_hcd *hcd)
{
    /* make sure xhci is halt */
    if (!(hcd->op_reg->status & STS_HALT))
    {
        hcd->op_reg->command &= ~CMD_RUN;
        /* wait xhci stop  */
        while(!(hcd->op_reg->status & STS_HALT));
    }

    /* reset */
    hcd->op_reg->command |= CMD_RESET;
    /* This bit is cleared to ‘0’ by the Host Controller when the reset process is complete  */
    while(hcd->op_reg->command & CMD_RESET);
    /* wait xhci ready */
    while(hcd->op_reg->status & STS_CNR);

    return 0;
}

static int xhci_ext_cap_dump(struct xhci_hcd *hcd)
{
    size_t ext_cap_addr = (((hcd->cap_reg->hccparams1 >> 16) & 0xffff) << 2) + hcd->base_addr;
    uint16_t next_point = 0;
    uint8_t minor, major;
    size_t name_str, port_info;

    do {
        size_t cap = *(size_t *)ext_cap_addr;
        switch (cap & 0xff) {
        case 1:
            break;
        case 2:
            minor = cap >> 16 & 0xff;
            major = cap >> 24 & 0xff;
            name_str = *(size_t *)(ext_cap_addr+4);
            port_info = *(size_t *)(ext_cap_addr+8);
            XHCI_LOG_DBG("xHCI Supported %c%c%c%c%d.%d \n", name_str & 0xff, (name_str >> 8) & 0xff, (name_str >> 16) & 0xff, (name_str >> 24) & 0xff, major, minor);
            XHCI_LOG_DBG("Compatible Port Offset %x, Compatible Port Count %x, Protocol Defined = %x.\n", port_info & 0xff, (port_info >> 8) & 0xff, (port_info >> 16) & 0xff);
            break;
        default:
            break;
        }
        next_point = (cap >> 8) & 0xff;
        ext_cap_addr += next_point << 2;
    } while (next_point);

    return 0;
}

int xhci_init(struct xhci_hcd *hcd, uint32_t xhci_base_addr)
{
    hcd->base_addr = xhci_base_addr;

    hcd->cap_reg = (struct xhci_cap_regs *)hcd->base_addr;
    hcd->op_reg = (struct xhci_op_regs *)((hcd->cap_reg->caplength & 0xff) + hcd->base_addr);
    hcd->runtime_reg = (struct xhci_runtime_regs *)((hcd->cap_reg->rtsoff & ~0x1f) + hcd->base_addr);
    hcd->db_reg = (struct xhci_doorbell_arry *)((hcd->cap_reg->dboff & ~0x3) + hcd->base_addr);
    hcd->version = (hcd->cap_reg->caplength >> 16) & 0xfffff;
    hcd->max_slots = hcd->cap_reg->hcsparams1 & 0xff;
    hcd->max_port = hcd->cap_reg->hcsparams1 >> 24;
    hcd->page_size = 1 << (hcd->op_reg->page_size + 11);

    hcd->dev_ctx_tab = xhci_mem_alloc(XHCI_ALIGN, sizeof(uint64_t) * hcd->max_slots);
    hcd->scratchpad_tab = xhci_mem_alloc(XHCI_ALIGN, sizeof(uint64_t) * HCSPARAMS2_SCRATPAD_NUM(hcd->cap_reg->hcsparams2));
    hcd->scratchpad_buf = xhci_mem_alloc(hcd->page_size, hcd->page_size * HCSPARAMS2_SCRATPAD_NUM(hcd->cap_reg->hcsparams2));
    hcd->command_trb = xhci_mem_alloc(XHCI_ALIGN, XHCI_RING_TRB_MAX_NUM * sizeof (struct xhci_trb));
    hcd->event_trb = xhci_mem_alloc(XHCI_ALIGN, XHCI_RING_TRB_MAX_NUM * sizeof (struct xhci_trb));

    if (!hcd->dev_ctx_tab || !hcd->scratchpad_tab || !hcd->scratchpad_buf || !hcd->command_trb || !hcd->event_trb)
    {
        XHCI_LOG_DBG("xhci init fail, can't alloc memory!!");
        xhci_mem_free(hcd->dev_ctx_tab);
        xhci_mem_free(hcd->scratchpad_tab);
        xhci_mem_free(hcd->scratchpad_buf);
        xhci_mem_free(hcd->command_trb);
        xhci_mem_free(hcd->event_trb);
        return -1;
    }

    /* xhci reset */
    xhci_reset(hcd);
    xhci_ext_cap_dump(hcd);

    hcd->op_reg->config_reg = hcd->max_slots;
    hcd->dev_ctx_tab[0] = (size_t)hcd->scratchpad_tab;
    hcd->op_reg->dcbaa_ptr = (size_t)hcd->dev_ctx_tab;

    for (int i = 0; i < HCSPARAMS2_SCRATPAD_NUM(hcd->cap_reg->hcsparams2); i++)
        hcd->scratchpad_tab[i] = (size_t)hcd->scratchpad_buf[i * hcd->page_size];

    xhci_command_ring_init(hcd);
    xhci_event_ring_init(hcd);
    xhci_run(hcd);

    return 0;
}
