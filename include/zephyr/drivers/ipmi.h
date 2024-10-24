#ifndef ZEPHYR_INCLUDE_DRIVERS_IPMI_H_
#define ZEPHYR_INCLUDE_DRIVERS_IPMI_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*impi_read_t)(const struct device *,uint8_t *,uint32_t);

typedef int (*impi_write_t)(const struct device *,uint8_t *,uint32_t);

__subsystem struct ipmi_driver_api {
    impi_read_t read;
    impi_write_t write;
};

__syscall int ipmi_read(const struct device *dev,uint8_t *data,uint32_t size);

static inline int z_impl_ipmi_read(const struct device *dev,uint8_t *data,uint32_t size)
{
    const struct ipmi_driver_api *api = (const struct ipmi_driver_api *)dev->api;
    if(api->read == NULL) {
        return -ENOSYS;
    }
    return api->read(dev,data,size);
}

__syscall int ipmi_write(const struct device *dev,uint8_t *data,uint32_t size);

static inline int z_impl_ipmi_write(const struct device *dev,uint8_t *data,uint32_t size)
{
    const struct ipmi_driver_api *api = (const struct ipmi_driver_api *)dev->api;
    if(api->write == NULL) {
        return -ENOSYS;
    }
    return api->write(dev,data,size);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/ipmi.h>

#endif