#ifndef __HID_BRIDGE_H__
#define __HID_BRIDGE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct hid_bridge
{
    int (*send_data_2_host)(struct hid_bridge *hid, void *data, uint8_t length);
    void *user_data;
};

typedef int (*send_data_2_host)(struct hid_bridge *hid, void *data, uint8_t length);

int hid_bridge_init(struct hid_bridge *hid, send_data_2_host send_data, void *user_data);
int hid_bridge_raw_event(struct hid_bridge *hid, const uint8_t raw_data[64]);
int hid_bridge_raw_send(struct hid_bridge *hid, uint8_t raw_data[64]);

#ifdef __cplusplus
}
#endif

#endif //__HID_BRIDGE_H__