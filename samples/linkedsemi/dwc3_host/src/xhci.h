#ifndef __XHCI_H__
#define __XHCI_H__

#include <stdint.h>
#include <stddef.h>
#include "xhci_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* init */
int xhci_init(struct xhci_hcd *hcd, uint32_t xhci_base_addr);

/* command */
int xhci_cmd_disable_slot(struct xhci_hcd *xhci, int slot);
int xhci_cmd_enable_slot(struct xhci_hcd *xhci);
int xhci_cmd_address_device(struct xhci_hcd *xhci, int slot, xhci_addr_dev_type_t type);
int xhci_cmd_reset_device(struct xhci_hcd *xhci, int slot);
int xhci_cmd_add_endpoint(struct xhci_hcd *xhci, int slot, struct xhci_ep_config *ep_config);
int xhci_cmd_drop_endpoint(struct xhci_hcd *xhci, int slot, uint8_t ep);
int xhci_cmd_noop(struct xhci_hcd *xhci);

/* function */
int xhci_alloc_device(struct xhci_hcd *xhci);
int xhci_free_device(struct xhci_hcd *xhci, int slot);
int xhci_address_device(struct xhci_hcd *xhci, int slot, int type);
struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_hcd *hcd, struct xhci_ctx *ctx, int ep_index);
struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_hcd *hcd, struct xhci_ctx *ctx);
int xhci_device_ctx_show(struct xhci_hcd *hcd, size_t slot);
int xhci_port_reset(struct xhci_hcd *hcd, int port);
uint32_t xhci_get_portsc(struct xhci_hcd *hcd, int port);
enum xhci_usb_speed xhci_get_port_speed(struct xhci_hcd *xhci, int port_id);
void xhci_set_dev_speed(struct xhci_hcd *xhci, int slot, enum xhci_usb_speed speed);

/* transfer */
int xhci_xfer_control(struct xhci_hcd *xhci, int slot, int ep, struct usb_setup_packet *req, void *data, uint32_t length);
int xhci_xfer_bulk(struct xhci_hcd *xhci, int slot, int ep, void *data, int length);
int xhci_xfer_interrupt(struct xhci_hcd *xhci, int slot, int ep, void *data, int length);
int xhci_xfer_noop(struct xhci_hcd *xhci, int slot, uint8_t ep, xhci_ep_type_t type);

/* isr */
void xhci_isr(struct xhci_hcd *hcd);

/* port */
void xhci_enqueue_lock();
void xhci_enqueue_unlock();
void xhci_event_complete();
void xhci_event_wait();
void *xhci_mem_alloc(size_t align, size_t size);
void xhci_mem_free(void *ptr);
void xhci_cache_flush(void *buf, size_t size);
void xhci_cache_invalid(void *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif // __XHCI_H__