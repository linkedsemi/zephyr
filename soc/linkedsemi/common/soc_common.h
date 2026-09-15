#ifndef SOC_LINKEDSEMI_COMMON_SOC_COMMON_H_
#define SOC_LINKEDSEMI_COMMON_SOC_COMMON_H_
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include "ls_hal_flash.h"

#define IRQ_NESTED_MAX 10

#define IRQ_TYPE_NONE         0
#define IRQ_TYPE_EDGE_RISING  1
#define IRQ_TYPE_EDGE_FALLING 2
#define IRQ_TYPE_EDGE_BOTH    (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH   4
#define IRQ_TYPE_LEVEL_LOW    8

#define DEV_ERR(dev, fmt, ...) LOG_ERR("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_WRN(dev, fmt, ...) LOG_WRN("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_INF(dev, fmt, ...) LOG_INF("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_DBG(dev, fmt, ...) LOG_DBG("%s: " fmt, (dev)->name, ##__VA_ARGS__)

enum delegate_server_op
{
	FLASH_DELEGATE_SERVER_READ,
	FLASH_DELEGATE_SERVER_WRITE,
	FLASH_DELEGATE_SERVER_WRITE_ALIGN,
	FLASH_DELEGATE_SERVER_READ_ALIGN,
	FLASH_DELEGATE_SERVER_ERASE,
	FLASH_DELEGATE_SERVER_GET_PARAMS,
	FLASH_DELEGATE_SERVER_READ_JEDEC_ID,
	FLASH_DELEGATE_SERVER_SFDP_READ,
	FLASH_DELEGATE_SERVER_SUSPEND,
	FLASH_DELEGATE_SERVER_READ_EAR,
	FLASH_DELEGATE_SERVER_MAX
};

struct delegate_c2s_params
{
	void *reg;
	off_t offset;
	void *data;
	size_t size;
	enum delegate_server_op op;
};

struct flash_ls_shared_data
{
	void *reg;
	volatile bool busy;
	volatile bool suspend_request;
	volatile bool hold_ack;
};

enum delegate_client_op
{
	FLASH_DELEGATE_CLIENT_HOLD,
	FLASH_DELEGATE_CLIENT_OP_RETURN,
	FLASH_DELEGATE_CLIENT_MAX
};

struct flash_op_return
{
	struct flash_parameters flash_params;
	int value;
};

struct delegate_s2c_params
{
	struct flash_ls_shared_data *shared;
	struct flash_op_return ret;
	enum delegate_client_op op;
};

struct flash_xfer_buf {
	off_t offset;
	void *buf;
	size_t len;
};

enum {
	FLASH_XFER_BUF_IDX_HEAD = 0,
	FLASH_XFER_BUF_IDX_MIDDLE = 1,
	FLASH_XFER_BUF_IDX_TAIL = 2,
	FLASH_XFER_BUF_IDX_MAX = 3,
};

typedef struct __aligned(CONFIG_DCACHE_LINE_SIZE) flash_op_align_buf {
	struct flash_xfer_buf buf[FLASH_XFER_BUF_IDX_MAX];
} flash_op_align_buf_t;

#define FLASH_DRIVER_SUSPEND_OPCODE 0x8001
#define FLASH_DRIVER_RESUME_OPCODE 0x8002
#define FLASH_DRIVER_CLIENT_XIP_ACTIVE 0x8003
#define FLASH_DRIVER_CLIENT_XIP_INACTIVE 0x8004

int busy_poll(bool (*poll_fn)(void *),void *param,uint32_t usec_to_wait);
#endif
