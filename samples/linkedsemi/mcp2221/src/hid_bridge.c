#include "hid_bridge.h"
#include <string.h>

/* Commands codes in a raw output report */
enum {
    MCP2221_I2C_WR_DATA = 0x90,
    MCP2221_I2C_WR_NO_STOP = 0x94,
    MCP2221_I2C_RD_DATA = 0x91,
    MCP2221_I2C_RD_RPT_START = 0x93,
    MCP2221_I2C_GET_DATA = 0x40,
    MCP2221_I2C_PARAM_OR_STATUS	= 0x10,
    MCP2221_I2C_SET_SPEED = 0x20,
    MCP2221_I2C_CANCEL = 0x10,
    MCP2221_GPIO_SET = 0x50,
    MCP2221_GPIO_GET = 0x51,
    MCP2221_SET_SRAM_SETTINGS = 0x60,
    MCP2221_GET_SRAM_SETTINGS = 0x61,
    MCP2221_READ_FLASH_DATA = 0xb0,
};

/* Response codes in a raw input report */
enum {
    MCP2221_SUCCESS = 0x00,
    MCP2221_I2C_ENG_BUSY = 0x01,
    MCP2221_I2C_START_TOUT = 0x12,
    MCP2221_I2C_STOP_TOUT = 0x62,
    MCP2221_I2C_WRADDRL_TOUT = 0x23,
    MCP2221_I2C_WRDATA_TOUT = 0x44,
    MCP2221_I2C_WRADDRL_NACK = 0x25,
    MCP2221_I2C_MASK_ADDR_NACK = 0x40,
    MCP2221_I2C_WRADDRL_SEND = 0x21,
    MCP2221_I2C_ADDR_NACK = 0x25,
    MCP2221_I2C_READ_PARTIAL = 0x54,
    MCP2221_I2C_READ_COMPL = 0x55,
    MCP2221_ALT_F_NOT_GPIOV = 0xEE,
    MCP2221_ALT_F_NOT_GPIOD = 0xEF,
};

typedef union
{
    struct
    {
        uint8_t command;
        uint8_t byte_l;
        uint8_t byte_h;
        uint8_t address;
        uint8_t payload[0];
    } mcp2221_i2c;
    struct
    {
        uint8_t command;
        uint8_t reserved;
        uint8_t cancle_transfer;
        uint8_t set_speed;
        uint8_t payload[0];
    } mcp2221_param;
    uint8_t response_raw[64];
} mcp2221_command;

typedef union
{
    struct
    {
        uint8_t response_echo;
        uint8_t is_busy;
        uint8_t response_code; // busy 时，设置这个告诉 host 为啥 busy 啦
    } mcp2221_i2c;

    struct
    {
        uint8_t response_echo;
        uint8_t cmd_complete;
        uint8_t reserved; // busy 时，设置这个告诉 host 为啥 busy 啦
        uint8_t data_len;
        uint8_t data[0];
    } mcp2221_i2c_data;

    struct
    {
        uint8_t response_echo;
        uint8_t cmd_complete;
        uint8_t cancle_transfer;
        uint8_t reserved[5];
        uint8_t i2c_commu_state;
    } mcp2221_param;

    uint8_t response_raw[64];
} mcp2221_response;

static uint8_t i2c_current_state = 0;

static int mcp2221_i2c_write_handle(struct hid_bridge *hid, mcp2221_command *command)
{
    static __attribute__((aligned(32))) mcp2221_response response = {0};
    /* TODO: 解 command，然后执行相关动作 */

    response.mcp2221_i2c.response_echo = command->mcp2221_i2c.command;
    response.mcp2221_i2c.is_busy = 0;
    hid_bridge_raw_send(hid, response.response_raw);
    i2c_current_state = MCP2221_SUCCESS;

    return 0;
}

static int mcp2221_i2c_param_handle(struct hid_bridge *hid, mcp2221_command *command)
{
    /* hid class 层会拿着这个 buffer 直接到驱动层使用，如果这里返回了，驱动层还没将数据发送出去，buffer 中的数据将会失效，因此这里将 buffer 开在 data 区 */
    static __attribute__((aligned(32))) mcp2221_response response = {0};
    /* TODO: 解 command，然后执行相关动作 */

    /* 构造 response 包 */
    response.mcp2221_param.response_echo = command->mcp2221_param.command;
    response.mcp2221_param.cmd_complete = 0;
    /* 返回上次 i2c 通信的状态, 这里写为 i2c_current_state，这可能得根据义务去改 */
    response.mcp2221_param.i2c_commu_state = i2c_current_state;
    hid_bridge_raw_send(hid, response.response_raw);

    return 0;
}

static int mcp2221_i2c_read_handle(struct hid_bridge *hid, mcp2221_command *command)
{
    static __attribute__((aligned(32))) mcp2221_response response = {0};
    /* TODO: 解 command，然后执行相关动作 */

    i2c_current_state = MCP2221_I2C_READ_COMPL; // 设置为读完成

    response.mcp2221_i2c.response_echo = command->mcp2221_i2c.command;
    response.mcp2221_i2c.is_busy = 0;
    hid_bridge_raw_send(hid, response.response_raw);
    return 0;
}

static int mcp2221_i2c_get_data_handle(struct hid_bridge *hid, mcp2221_command *command)
{
    static __attribute__((aligned(32))) mcp2221_response response = {0};
    /* TODO: 解 command，然后执行相关动作 */
    response.mcp2221_i2c_data.response_echo = command->mcp2221_i2c.command;
    response.mcp2221_i2c_data.cmd_complete = 0;
    response.mcp2221_i2c_data.data_len = strlen("hello world!!");
    strcpy(response.mcp2221_i2c_data.data, "hello world!!");
    hid_bridge_raw_send(hid, response.response_raw);

    return 0;
}

int hid_bridge_init(struct hid_bridge *hid, send_data_2_host send_data, void *user_data)
{
    hid->send_data_2_host = send_data;
    hid->user_data = user_data;
    return 0;
}

int hid_bridge_raw_event(struct hid_bridge *hid, const uint8_t raw_data[64])
{
    /* 这里走 mcp2221 的协议 */
    switch (raw_data[0])
    {
        case MCP2221_I2C_WR_DATA:
            mcp2221_i2c_write_handle(hid, (mcp2221_command *)raw_data);
            break;
        case MCP2221_I2C_PARAM_OR_STATUS:
            mcp2221_i2c_param_handle(hid, (mcp2221_command *)raw_data);
            break;
        case MCP2221_I2C_RD_DATA:
            mcp2221_i2c_read_handle(hid, (mcp2221_command *)raw_data);
            break;
        case MCP2221_I2C_GET_DATA:
            mcp2221_i2c_get_data_handle(hid, (mcp2221_command *)raw_data);
            break;
        default:
            break;
    }

    return 0;
}

int hid_bridge_raw_send(struct hid_bridge *hid, uint8_t raw_data[64])
{
    int res = 0;
    if (hid->send_data_2_host)
    {
        res = hid->send_data_2_host(hid, raw_data, 64);
    }

    return res;
}