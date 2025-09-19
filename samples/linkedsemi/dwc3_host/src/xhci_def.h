#ifndef __XHCI_DEF_H__
#define __XHCI_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef XHCI_LOG_DBG
#define XHCI_LOG_DBG(...)           printf("XHCI: ");printf(__VA_ARGS__)
#endif 

#if defined(__GNUC__) || defined(__clang__)
#define XHCI_MEM_ALIGN(n)           __attribute__((aligned(n)))
#else
#define XHCI_MEM_ALIGN(n)
#endif

#define XHCI_RING_TRB_MAX_NUM       32
#define XHCI_ALIGN                  64
#define XHCI_SLOT_MAX               16

/* TRB bit mask */
#define TRB_TYPE_BITMASK     (0xfc00)
#define TRB_TYPE(p)        ((p) << 10)
#define TRB_FIELD_TO_TYPE(p)     (((p) & TRB_TYPE_BITMASK) >> 10)

#define TRB_IDT             (1 << 6)
#define TRB_TRT(p)          ((p) << 16)
#define TRB_ISP             (1 << 2)
#define TRB_DIR_IN          (1 << 16)
#define TRB_IOC             (1 << 5)

#define TRB_DATA_IN         3
#define TRB_DATA_OUT        2
#define TRB_DATA_NO         0

#define TRB_BSR              (1<<9)
#define TRB_CYCLE            (1<<0)
#define LINK_TOGGLE          (0x1<<1)

#define STS_HALT        (1 << 0)
#define STS_CNR         (1 << 11)

#define CMD_RUN        (1 << 0)
#define CMD_RESET     (1 << 1)
#define CMD_EIE        (1 << 2)

#define ERST_EHB        (1 << 3)
#define ERST_PTR_MASK        (0xf)

#define GET_PORT_ID(p)        (((p) & (0xff << 24)) >> 24)
#define SLOT_FLAG             (1 << 0)
#define EP0_FLAG              (1 << 1)

#define COMP_CODE_MASK        (0xff << 24)
#define GET_COMP_CODE(p)     (((p) & COMP_CODE_MASK) >> 24)

#define TRB_TO_SLOT_ID(p)      (((p) & (0xff<<24)) >> 24)
#define SLOT_ID_FOR_TRB(p)     (((p) & 0xff) << 24)

#define HCC_64BYTE_CONTEXT(p)  ((p) & (1 << 2))
#define CTX_SIZE(_hcc)         (HCC_64BYTE_CONTEXT(_hcc) ? 64 : 32)

/* portsc */
#define XHCI_OP_PORTSC_PRC                  (1 << 21)
#define XHCI_OP_PORTSC_RESET                (1 << 4)
#define XHCI_OP_PORTSC_CSC                  (1 << 17)
#define XHCI_OP_PORTSC_PED                  (1 << 18)

#define XHCI_OP_PORTSC_SPEED(psc)           ((psc >> 10) & 0xf)
#define XHCI_OP_PORTSC_LINK_STATE(psc)           ((psc >> 5) & 0xf)

/* usbsts */
#define XHCI_USBSTS_EINT            (1 << 3)
/* imain */
#define XHCI_IMAIN_IP               (1 << 0)

/* hscratpd num */
#define HCSPARAMS2_SCRATPAD_NUM(hcsparams2)       ((((hcsparams2) >> 16) & 0x3e0) | (((hcsparams2) >> 27) & 0x1f))

#define PORTSC        0
#define PORTPMSC     1
#define PORTLI        2
#define PORTHLPMC     3

enum XHCI_TRB_TYPE
{
    TRB_NORMAL = 1,
    TRB_SETUP,
    TRB_DATA,
    TRB_STATUS,
    TRB_ISOC,
    TRB_LINK,
    TRB_EVENT_DATA,
    TRB_TR_NOOP,
    TRB_ENABLE_SLOT,
    TRB_DISABLE_SLOT,
    TRB_ADDR_DEV,
    TRB_CONFIG_EP,
    TRB_EVAL_CONTEXT,
    TRB_RESET_EP,
    TRB_STOP_RING,
    TRB_SET_DEQ,
    TRB_RESET_DEV,
    TRB_FORCE_EVENT,
    TRB_NEG_BANDWIDTH,
    TRB_SET_LT,
    TRB_GET_BW,
    TRB_FORCE_HEADER,
    TRB_CMD_NOOP,
    TRB_TRANSFER = 32,
    TRB_COMPLETION,
    TRB_PORT_STATUS,
    TRB_BANDWIDTH_EVENT,
    TRB_DOORBELL,
    TRB_HC_EVENT,
    TRB_DEV_NOTE,
    TRB_MFINDEX_WRAP,
    TRB_TYPE_MAX
};

enum XHCI_COMP_CODE
{
    COMP_INVALID,
    COMP_SUCCESS,
    COMP_DATA_BUFFER_ERROR,
    COMP_BABBLE_DETECTED_ERROR,
    COMP_USB_TRANSACTION_ERROR,
    COMP_TRB_ERROR,
    COMP_STALL_ERROR,
    COMP_RESOURCE_ERROR,
    COMP_BANDWIDTH_ERROR,
    COMP_NO_SLOTS_AVAILABLE_ERROR,
    COMP_INVALID_STREAM_TYPE_ERROR,
    COMP_SLOT_NOT_ENABLED_ERROR,
    COMP_ENDPOINT_NOT_ENABLED_ERROR,
    COMP_SHORT_PACKET,
    COMP_RING_UNDERRUN,
    COMP_RING_OVERRUN,
    COMP_VF_EVENT_RING_FULL_ERROR,
    COMP_PARAMETER_ERROR,
    COMP_BANDWIDTH_OVERRUN_ERROR,
    COMP_CONTEXT_STATE_ERROR,
    COMP_NO_PING_RESPONSE_ERROR,
    COMP_EVENT_RING_FULL_ERROR,
    COMP_INCOMPATIBLE_DEVICE_ERROR,
    COMP_MISSED_SERVICE_ERROR,
    COMP_COMMAND_RING_STOPPED,
    COMP_COMMAND_ABORTED,
    COMP_STOPPED,
    COMP_STOPPED_LENGTH_INVALID,
    COMP_STOPPED_SHORT_PACKET,
    COMP_MAX_EXIT_LATENCY_TOO_LARGE_ERROR,
    COMP_ISOCH_BUFFER_OVERRUN = 31,
    COMP_EVENT_LOST_ERROR,
    COMP_UNDEFINED_ERROR,
    COMP_INVALID_STREAM_ID_ERROR,
    COMP_SECONDARY_BANDWIDTH_ERROR,
    COMP_SPLIT_TRANSACTION_ERROR
};

enum xhci_usb_speed
{
    USB_SPEED_UNKNOWN = 0,
    USB_SPEED_FULL,
    USB_SPEED_LOW,
    USB_SPEED_HIGH,
};

struct xhci_cap_regs
{
    volatile uint32_t caplength;
    volatile uint32_t hcsparams1;
    volatile uint32_t hcsparams2;
    volatile uint32_t hcsparams3;
    volatile uint32_t hccparams1;
    volatile uint32_t dboff;
    volatile uint32_t rtsoff;
    volatile uint32_t hccparams2;
};

struct xhci_port_regs
{
    volatile uint32_t portsc;
    volatile uint32_t portpmsc;
    volatile uint32_t portli;
    volatile uint32_t porthlpmc; // usb2.0：If LPM is not supported (HLC = '0') then this register is reserved
}; 

struct xhci_op_regs
{
    volatile uint32_t command;
    volatile uint32_t status;
    volatile uint32_t page_size;
    volatile uint32_t reserved1;
    volatile uint32_t reserved2;
    volatile uint32_t dev_notification;
    volatile uint64_t cmd_ring;
    /* rsvd: offset 0x20-2F */
    volatile uint32_t reserved3[4];
    volatile uint64_t dcbaa_ptr;
    volatile uint32_t config_reg;
    /* rsvd: offset 0x3C-3FF */
    volatile uint32_t reserved4[241];
    volatile struct xhci_port_regs port[1]; // 400-13FFh
};

struct interrupter_regs
{
    volatile uint32_t iman;
    volatile uint32_t imod;
    volatile uint32_t erstsz;
    volatile uint32_t rsvd;
    volatile uint64_t erstba;
    volatile uint64_t erdp;
};
struct xhci_doorbell_arry
{
    volatile uint32_t doorbell[256];
};

struct xhci_erst_entry
{
    uint64_t seg_addr;
    uint32_t seg_size;
    uint32_t rsvd;
};

struct xhci_erst
{
    struct xhci_erst_entry *entry;
};

struct xhci_runtime_regs
{
    volatile uint32_t micro_frame;
    volatile uint32_t rsvd[7];
    volatile struct interrupter_regs ir_set[1];
};

typedef enum
{
    EP_ISO_OUT = 1,
    EP_BULK_OUT,
    EP_INT_OUT,
    EP_CONTROL_BIDIR,
    EP_ISO_IN,
    EP_BULK_IN,
    EP_INT_IN
} xhci_ep_type_t;

struct xhci_ep_config
{
    xhci_ep_type_t ep_type;
    uint8_t ep_addr;
    uint8_t ep_interval;
    uint16_t ep_mps;
};

struct xhci_trb
{
    uint64_t paramater;
    uint32_t status;
    uint32_t control;
};

struct xhci_generic_trb
{
    uint32_t field[4];
};

struct xhci_slot_ctx
{
    uint32_t dev_info;
    uint32_t dev_info2;
    uint32_t tt_info;
    uint32_t dev_state;
    /* offset 0x10 to 0x1f reserved for HC internal use */
    uint32_t reserved[4];
};

struct xhci_ep_ctx
{
    uint32_t ep_info[2];
    uint64_t deq;
    uint32_t tx_info;
    /* offset 0x14 - 0x1f reserved for HC internal use */
    uint32_t reserved[3];
};

struct xhci_input_control_ctx
{
    uint32_t drop_flags;
    uint32_t add_flags;
    uint32_t rsvd2[6];
};

struct xhci_device_ctx
{
    struct xhci_slot_ctx slot_ctx;
    struct xhci_ep_ctx ep_ctx[31];
};

struct xhci_input_ctx
{
    struct xhci_input_control_ctx control_ctx;
    struct xhci_slot_ctx slot_ctx;
    struct xhci_ep_ctx ep_ctx[31];
};

typedef enum
{
    EVENT_RING,
    COMMAND_RING,
    CTRL_RING,
    BULK_RING,
    INTR_RING
} ring_type_t;

typedef enum 
{
    DEFAULT_TYPE,
    ADDRESSED_TYPE,
} xhci_addr_dev_type_t;

struct xhci_ring
{
    struct xhci_trb *trb;
    uint32_t num_trb;
    ring_type_t type;
    uint32_t write_index;
    uint32_t read_index;
    uint32_t cycle_bit;
};

struct xhci_hcd;

struct xhci_ep
{
    struct xhci_ring ep_ring;
    uint8_t state;
};

struct usb_setup_packet
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};
struct xhci_ctx
{
#define XHCI_CTX_TYPE_DEVICE  0x1
#define XHCI_CTX_TYPE_INPUT   0x2
    int type;
    int size;
    uint8_t *ctx;
};

struct xhci_device
{
    uint32_t slot;
    struct xhci_ctx device_ctx;
    struct xhci_ctx input_ctx;
    struct xhci_hcd *hcd;
    struct xhci_ep ep[31];
    enum xhci_usb_speed speed;
};

struct xhci_hcd
{
    /* regs */
    uint32_t base_addr;
    struct xhci_cap_regs *cap_reg;
    struct xhci_op_regs *op_reg;
    struct xhci_runtime_regs *runtime_reg;
    struct xhci_doorbell_arry *db_reg;

    /* ring struct */
    struct xhci_erst erst;
    struct xhci_ring event_ring;
    struct xhci_ring cmd_ring;

    /* device struct */
    struct xhci_device device[XHCI_SLOT_MAX];
    uint64_t *dev_ctx_tab;
    uint64_t *scratchpad_tab;
    uint8_t *scratchpad_buf;
    struct xhci_trb *command_trb;
    struct xhci_trb *event_trb;
    struct xhci_trb evt_trb;

    uint32_t version;
    uint16_t page_size;
    uint8_t max_slots;
    uint8_t max_port;
};

#ifdef __cplusplus
}
#endif

#endif //__XHCI_DEF_H__