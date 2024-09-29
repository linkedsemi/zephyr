#ifndef ZEPHYR_INCLUDE_DRIVERS_KCS_H_
#define ZEPHYR_INCLUDE_DRIVERS_KCS_H_

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stddef.h>


#ifdef __cplusplus
extern "C" {
#endif

#define KCS_OBF		BIT(0)
#define KCS_IBF		BIT(1)
#define KCS_CMD_DAT	BIT(3)

typedef int (*kcs_read_data_t)(const struct device *dev,uint8_t *data);

typedef int (*kcs_write_data_t)(const struct device *dev,uint8_t data);

typedef int (*kcs_read_status_t)(const struct device *dev,uint8_t *status);

typedef int (*kcs_update_status_t)(const struct device *dev,uint8_t mask,uint8_t val);

typedef void (*ibf_callback_t)(const struct device *);

typedef int (*kcs_set_ibf_callback_t)(const struct device *dev,ibf_callback_t callback);

__subsystem struct kcs_driver_api {
    kcs_read_data_t read_data;
    kcs_write_data_t write_data;
    kcs_read_status_t read_status;
    kcs_update_status_t update_status;
    kcs_set_ibf_callback_t set_ibf_callback;
};

__syscall int kcs_read_data(const struct device *dev,uint8_t *data);

static inline int z_impl_kcs_read_data(const struct device *dev,uint8_t *data)
{
    const struct kcs_driver_api *api = (const struct kcs_driver_api *)dev->api;
    if(api->read_data == NULL) {
        return -ENOSYS;
    }
    return api->read_data(dev,data);
}

__syscall int kcs_write_data(const struct device *dev,uint8_t data);

static inline int z_impl_kcs_write_data(const struct device *dev,uint8_t data)
{
    const struct kcs_driver_api *api = (const struct kcs_driver_api *)dev->api;
    if(api->write_data == NULL) {
        return -ENOSYS;
    }
    return api->write_data(dev,data);
}

__syscall int kcs_read_status(const struct device *dev,uint8_t *status);

static inline int z_impl_kcs_read_status(const struct device *dev,uint8_t *status)
{
    const struct kcs_driver_api *api = (const struct kcs_driver_api *)dev->api;
    if(api->read_status == NULL) {
        return -ENOSYS;
    }
    return api->read_status(dev,status);
}

__syscall int kcs_update_status(const struct device *dev,uint8_t mask,uint8_t val);

static inline int z_impl_kcs_update_status(const struct device *dev,uint8_t mask,uint8_t val)
{
    const struct kcs_driver_api *api = (const struct kcs_driver_api *)dev->api;
    if(api->update_status == NULL) {
        return -ENOSYS;
    }
    return api->update_status(dev,mask,val);
}

__syscall int kcs_set_ibf_callback(const struct device *dev,ibf_callback_t callback);

static inline int z_impl_kcs_set_ibf_callback(const struct device *dev,ibf_callback_t callback)
{
    const struct kcs_driver_api *api = (const struct kcs_driver_api *)dev->api;
    if(api->set_ibf_callback == NULL) {
        return -ENOSYS;
    }
    return api->set_ibf_callback(dev,callback);
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/kcs.h>

#endif