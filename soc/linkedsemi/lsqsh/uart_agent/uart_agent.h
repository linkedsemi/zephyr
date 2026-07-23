#ifndef UART_AGENT_H_
#define UART_AGENT_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*uart_agent_callback_t)(const struct device *dev, void *user_data);

int uart_agent_start(const struct device *dev);
int uart_agent_stop(const struct device *dev);
void uart_agent_callback_set(const struct device *dev, uart_agent_callback_t cb, void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* UART_AGENT_H_ */
