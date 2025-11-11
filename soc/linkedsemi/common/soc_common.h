#ifndef SOC_LINKEDSEMI_COMMON_SOC_COMMON_H_
#define SOC_LINKEDSEMI_COMMON_SOC_COMMON_H_
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include "ls_hal_flash.h"

enum delegate_server_op
{
	FLASH_DELEGATE_SERVER_READ,
	FLASH_DELEGATE_SERVER_WRITE,
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

#define FLASH_DRIVER_SUSPEND_OPCODE 0x8001
#define FLASH_DRIVER_RESUME_OPCODE 0x8002
#define FLASH_DRIVER_CLIENT_XIP_ACTIVE 0x8003

int busy_poll(bool (*poll_fn)(void *),void *param,uint32_t usec_to_wait);
#endif
