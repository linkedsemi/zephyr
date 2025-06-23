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
	FLASH_DELEGATE_SERVER_LAYOUT,
	FLASH_DELEGATE_SERVER_READ_JEDEC_ID,
	FLASH_DELEGATE_SERVER_SFDP_READ,
	FLASH_DELEGATE_SERVER_SUSPEND,
	FLASH_DELEGATE_SERVER_HOLD_ACK,
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

struct flash_ls_data {
	struct hal_flash_env env;
	struct k_sem sem;
	#ifdef CONFIG_FLASH_OP_DELEGATION_SERVER
    struct k_sem delegate_sem;
	struct k_work worker;
	const struct device *dev;
	struct delegate_c2s_params req_param;
	bool requested_suspending;
	#endif
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
	struct flash_pages_layout flash_layout;
	int value;
};

struct delegate_s2c_params
{
	struct flash_ls_data *server;
	struct flash_op_return ret;
	enum delegate_client_op op;
};

#define FLASH_DRIVER_SUSPEND_OPCODE 0x8001
#define FLASH_DRIVER_RESUME_OPCODE 0x8002
#endif
