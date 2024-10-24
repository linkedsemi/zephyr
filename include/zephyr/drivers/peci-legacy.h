/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018-2019 Intel Corporation */

#ifndef __LINUX_PECI_H
#define __LINUX_PECI_H


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/peci-ioctl.h>


#define PECI_NAME_SIZE   32

typedef unsigned int uint;
typedef unsigned long ulong;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint16_t __le16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t __le64;

/* CRC Head Section */
/* required table size for crc8 algorithm */
#define CRC8_TABLE_SIZE			256

/* helper macro assuring right table size is used */
#define DECLARE_CRC8_TABLE(_table) \
	static u8 _table[CRC8_TABLE_SIZE]

struct peci_board_info {
	char			type[PECI_NAME_SIZE];
	u8			addr;	/* CPU client address */
	u8			domain_id;
	struct device_node	*of_node;
};

/**
 * struct peci_adapter - represent a PECI adapter
 * @owner: owner module of the PECI adpater
 * @bus_lock: k_mutex for exclusion of multiple callers
 * @dev: device interface to this driver
 * @nr: the bus number to map
 * @name: name of the adapter
 * @userspace_clients_lock: k_mutex for exclusion of clients handling
 * @userspace_clients: list of registered clients
 * @xfer: low-level transfer function pointer of the adapter
 * @cmd_mask: mask for supportable PECI commands
 * @use_dma: flag for indicating that adapter uses DMA
 *
 * Each PECI adapter can communicate with one or more PECI client children.
 * These make a small bus, sharing a single wired PECI connection.
 */
struct peci_adapter {
	struct module		*owner;
	struct k_mutex		bus_lock; /* k_mutex for bus locking */
	struct device		dev;
	int			nr;
	char			name[PECI_NAME_SIZE];
	struct k_mutex		userspace_clients_lock; /* clients list k_mutex */
	sys_dnode_t			userspace_clients;
	int			(*xfer)(struct peci_adapter *adapter,
					struct peci_xfer_msg *msg);
	u32			cmd_mask;
	bool			use_dma;
	u8			peci_revision;
};

static inline struct peci_adapter *to_peci_adapter(void *d)
{
	return CONTAINER_OF(d, struct peci_adapter, dev);
}

static inline void *peci_get_adapdata(const struct peci_adapter *adapter)
{
	return adapter->dev.data;
}

static inline void peci_set_adapdata(struct peci_adapter *adapter, void *data)
{
	adapter->dev.data = data;
}

/**
 * struct peci_client - represent a PECI client device
 * @dev: driver model device node for the client
 * @adapter: manages the bus segment hosting this PECI device
 * @addr: address used on the PECI bus connected to the parent adapter
 * @name: indicates the type of the device
 * @detected: detected PECI clients list
 *
 * A peci_client identifies a single device (i.e. CPU) connected to a peci bus.
 * The behaviour exposed to Linux is defined by the driver managing the device.
 */
struct peci_client {
	struct device		dev;
	struct peci_adapter	*adapter;
	u8			addr;
	u8			domain_id;
	char			name[PECI_NAME_SIZE];
	sys_dnode_t		detected;
};

static inline struct peci_client *to_peci_client(void *d)
{
	return CONTAINER_OF(d, struct peci_client, dev);
}

struct peci_device_id {
	char	name[PECI_NAME_SIZE];
	ulong	driver_data;	/* Data private to the driver */
};

// /**
//  * struct peci_driver - represent a PECI device driver
//  * @probe: callback for device binding
//  * @remove: callback for device unbinding
//  * @shutdown: callback for device shutdown
//  * @driver: device driver model driver
//  * @id_table: list of PECI devices supported by this driver
//  *
//  * The driver.owner field should be set to the module owner of this driver.
//  * The driver.name field should be set to the name of this driver.
//  */
// struct peci_driver {
// 	int				(*probe)(struct peci_client *client);
// 	int				(*remove)(struct peci_client *client);
// 	void				(*shutdown)(struct peci_client *client);
// 	struct device_driver		driver;
// 	const struct peci_device_id	*id_table;
// };

// static inline struct peci_driver *to_peci_driver(void *d)
// {
// 	return CONTAINER_OF(d, struct peci_driver, driver);
// }

// /**
//  * module_peci_driver - Helper macro for registering a modular PECI driver
//  * @__peci_driver: peci_driver struct
//  *
//  * Helper macro for PECI drivers which do not do anything special in module
//  * init/exit. This eliminates a lot of boilerplate. Each module may only
//  * use this macro once, and calling it replaces module_init() and module_exit()
//  */
// #define module_peci_driver(__peci_driver) \
// 	module_driver(__peci_driver, peci_add_driver, peci_del_driver)

// /* use a define to avoid include chaining to get THIS_MODULE */
// #define peci_add_driver(driver) peci_register_driver(THIS_MODULE, driver)

// extern struct bus_type peci_bus_type;
// extern struct device_type peci_adapter_type;
// extern struct device_type peci_client_type;

// int  peci_register_driver(struct module *owner, struct peci_driver *drv);
// void peci_del_driver(struct peci_driver *driver);
// struct peci_client *peci_verify_client(struct device *dev);
struct peci_adapter *peci_alloc_adapter(struct device *dev, uint size);
// struct peci_adapter *peci_get_adapter(int nr);
// void peci_put_adapter(struct peci_adapter *adapter);
int  peci_add_adapter(struct peci_adapter *adapter);
// void peci_del_adapter(struct peci_adapter *adapter);
// struct peci_adapter *peci_verify_adapter(struct device *dev);
// int  peci_for_each_dev(void *data, int (*fn)(struct device *, void *));
int peci_core_init(void);
struct peci_xfer_msg *peci_get_xfer_msg(u8 tx_len, u8 rx_len);
void peci_put_xfer_msg(struct peci_xfer_msg *msg);
int  peci_command(struct peci_adapter *adpater, enum peci_cmd cmd, uint msg_len, void *vmsg);
long peci_dev_ioctl(struct device* dev, uint iocmd, char* umsg);
// int  peci_get_cpu_id(struct peci_adapter *adapter, u8 addr, u8 domain_id, u32 *cpu_id);

#endif /* __LINUX_PECI_H */
