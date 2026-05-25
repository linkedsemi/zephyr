#ifndef __MBOX_LINKEDSEMI_H
#define __MBOX_LINKEDSEMI_H

int mbox_linkedsemi_tx_mtu_get(const struct device *dev, uint32_t channel);
int mbox_linkedsemi_rx_mtu_get(const struct device *dev, uint32_t channel);

#endif /* __MBOX_LINKEDSEMI_H */
