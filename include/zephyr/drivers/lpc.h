#ifndef ZEPHYR_INCLUDE_DRIVERS_LPC_H_
#define ZEPHYR_INCLUDE_DRIVERS_LPC_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum lpc_peripheral_opcode {

};

typedef int (*lpc_read_request_t)(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data);

typedef int (*lpc_write_request_t)(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data);

__subsystem struct lpc_driver_api {
    lpc_read_request_t read_request;
    lpc_write_request_t write_request;
};

__syscall int lpc_read_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data);

static inline int z_impl_lpc_read_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    const struct lpc_driver_api *api = (const struct lpc_driver_api *)dev->api;
    return api->read_request(dev,op,data);
}

__syscall int lpc_write_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data);

static inline int z_impl_lpc_write_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    const struct lpc_driver_api *api = (const struct lpc_driver_api *)dev->api;
    return api->write_request(dev,op,data);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/lpc.h>
#endif
