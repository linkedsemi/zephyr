#ifndef USB_DEVICE_INIT_H
#define USB_DEVICE_INIT_H

#include <zephyr/usb/usbd.h>

struct usbd_context *usbd_init_device(usbd_msg_cb_t msg_cb);

#endif /* USB_DEVICE_INIT_H */
