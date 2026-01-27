#ifndef ZEPHYR_INCLUDE_DRIVERS_IPMB_H_
#define ZEPHYR_INCLUDE_DRIVERS_IPMB_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>
#include <errno.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef int (*ipmb_register_t)(const struct device *dev);

typedef int (*ipmb_unregister_t)(const struct device *dev);

typedef int (*ipmb_write_t)(const struct device *,uint8_t *,uint32_t);

typedef void (*ipmb_rx_cb_t)(void *,uint8_t **);

typedef int (*ipmb_set_callback_t)(const struct device *,ipmb_rx_cb_t,void *,uint8_t *,uint16_t);

__subsystem struct ipmb_driver_api {
	ipmb_register_t drv_register;
	ipmb_unregister_t drv_unregister;
    ipmb_write_t write;
	ipmb_set_callback_t set_rx_callback;
};

__syscall int ipmb_register(const struct device *dev);

static inline int z_impl_ipmb_register(const struct device *dev)
{
    const struct ipmb_driver_api *api = (const struct ipmb_driver_api *)dev->api;
    if(api->drv_register == NULL) {
        return -ENOSYS;
    }
    return api->drv_register(dev);
}

__syscall int ipmb_unregister(const struct device *dev);

static inline int z_impl_ipmb_unregister(const struct device *dev)
{
    const struct ipmb_driver_api *api = (const struct ipmb_driver_api *)dev->api;
    if(api->drv_unregister == NULL) {
        return -ENOSYS;
    }
    return api->drv_unregister(dev);
}

__syscall int ipmb_set_rx_callback(const struct device *dev,ipmb_rx_cb_t cb,void *param,uint8_t *rx_buf,uint16_t buf_size);

static inline int z_impl_ipmb_set_rx_callback(const struct device *dev,ipmb_rx_cb_t cb,void *param,uint8_t *rx_buf,uint16_t buf_size)
{
	const struct ipmb_driver_api *api = (const struct ipmb_driver_api *)dev->api;
	if(api->set_rx_callback == NULL)
	{
		return -ENOSYS;
	}
	return api->set_rx_callback(dev,cb,param,rx_buf,buf_size);
}

__syscall int ipmb_write(const struct device *dev,uint8_t *data,uint32_t size);

static inline int z_impl_ipmb_write(const struct device *dev,uint8_t *data,uint32_t size)
{
    const struct ipmb_driver_api *api = (const struct ipmb_driver_api *)dev->api;
    if(api->write == NULL) {
        return -ENOSYS;
    }
    return api->write(dev,data,size);
}


#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/ipmb.h>

#endif