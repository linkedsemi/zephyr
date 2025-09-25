#ifndef LS_VUART_PEER_API_H
#define LS_VUART_PEER_API_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

typedef void (*ls_vuart_peer_evt_cb_t)(void *dev);

int ls_vuart_get_tx_char(const struct device *dev, uint8_t *out_char);
bool host_vuart_rx_available(const struct device *dev);
void ls_vuart_put_rx_char(const struct device *dev, uint8_t c);
void ls_vuart_register_tx_start_callback(const struct device *dev, ls_vuart_peer_evt_cb_t callback,
					 void *user_data);

#endif /* LS_VUART_PEER_API_H */
