#ifndef ZEPHYR_INCLUDE_DRIVERS_IPMI_H_
#define ZEPHYR_INCLUDE_DRIVERS_IPMI_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IPMI_RX_EVENT,
} ipmi_event_type_t;

typedef int (*ipmi_read_t)(const struct device *,uint8_t *,uint32_t);

typedef int (*ipmi_write_t)(const struct device *,uint8_t *,uint32_t);

typedef int (*ipmi_update_status_t)(const struct device *,uint8_t,uint8_t);

typedef int (*ipmi_force_abort_t)(const struct device *);

typedef int (*ipmi_callback_t)(const struct device *dev, void *param, ipmi_event_type_t type);

typedef int (*ipmi_set_callback_t)(const struct device *dev, ipmi_callback_t callback, void *param);

__subsystem struct ipmi_driver_api {
    ipmi_read_t read;
    ipmi_write_t write;
    ipmi_update_status_t update_status;
    ipmi_force_abort_t force_abort;
    ipmi_set_callback_t callback;
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

__syscall int ipmi_update_status(const struct device *dev,uint8_t mask,uint8_t val);

static inline int z_impl_ipmi_update_status(const struct device *dev,uint8_t mask,uint8_t val)
{
    const struct ipmi_driver_api *api = (const struct ipmi_driver_api *)dev->api;
    if(api->update_status == NULL) {
        return -ENOSYS;
    }
    return api->update_status(dev,mask,val);
}

__syscall int ipmi_force_abort(const struct device *dev);

static inline int z_impl_ipmi_force_abort(const struct device *dev)
{
    const struct ipmi_driver_api *api = (const struct ipmi_driver_api *)dev->api;
    if(api->force_abort == NULL) {
        return -ENOSYS;
    }
    return api->force_abort(dev);
}

__syscall int ipmi_set_callback(const struct device *dev, ipmi_callback_t callback, void *param);

static inline int z_impl_ipmi_set_callback(const struct device *dev, ipmi_callback_t callback, void *param)
{
    const struct ipmi_driver_api *api = (const struct ipmi_driver_api *)dev->api;
    if(api->callback == NULL) {
        return -ENOSYS;
    }
    return api->callback(dev, callback, param);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/ipmi.h>

#endif