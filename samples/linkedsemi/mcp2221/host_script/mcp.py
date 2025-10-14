import re
import time
import datetime
from collections import namedtuple
import hid
import platform
from sys import exit
from common import read_config
import subprocess
import serial.tools.list_ports

vendor_id = 0x0000
product_id = 0x0000

vendor_id_manuf = 0x0088  # 供应商ID
product_id_manuf = 0x0068  # 产品ID
vendor_id_def = 0x04d8  # 供应商ID
product_id_def = 0x00dd  # 产品ID


ENOENT = 2
ENXIO = 6
ETIMEDOUT = 60
EAGAIN = 35
EIO = 5
EINVAL = 22
EOPNOTSUPP = 102

DEFAULT_I2C_CLK_FREQ = 400
INTERFACE_NUMBER = 2  # the interface number of the HID device
ENDPOINT_OUT = 0x03  # the output endpoint address of the HID device
ENDPOINT_IN = 0x83  # the input endpoint address of the HID device
TIMEOUT = 2  # read and write device timeout

# i2c_smbus_xfer read or write markers
I2C_SMBUS_READ = 1
I2C_SMBUS_WRITE = 0

# SMBus transaction types (size parameter in functions)
I2C_SMBUS_BYTE_DATA = 2
I2C_SMBUS_WORD_DATA = 3
I2C_SMBUS_BLOCK_DATA = 5

# Other parameter definitions
MCP_NGPIO = 4
POLY = (0x1070 << 3)  # Assuming this is intended as a constant value, not a bit shift operation in Python
MCP_READ_MAX_LEN = 60
MCP_STATUS_TIMEOUT = 4000

MCP2221_I2C_WR_DATA = 0x90
MCP2221_I2C_WR_NO_STOP = 0x94
MCP2221_I2C_RD_DATA = 0x91
MCP2221_I2C_RD_RPT_START = 0x93
MCP2221_I2C_GET_DATA = 0x40
MCP2221_I2C_PARAM_OR_STATUS = 0x10
MCP2221_I2C_SET_SPEED = 0x20
MCP2221_I2C_CANCEL = 0x10
MCP2221_GPIO_SET = 0x50
MCP2221_GPIO_GET = 0x51
MCP2221_SET_SRAM_SETTINGS = 0x60
MCP2221_GET_SRAM_SETTINGS = 0x61
MCP2221_READ_FLASH_DATA = 0xb0
MCP2221_WRITE_FLASH_DATA = 0xb1
MCP2221_SEND_FLASH_PASSWORD = 0xb2
MCP2221_CHIP_SETTING = 0x00
MCP2221_GP_SETTING = 0x01
MCP2221_USB_MANU_DESC = 0x02
MCP2221_USB_PRODUCT_DESC = 0x03
MCP2221_USB_SERIAL_NUMBER_DESC = 0x04

MCP2221_SUCCESS = 0x00
MCP2221_I2C_ENG_BUSY = 0x01
MCP2221_I2C_START_TOUT = 0x12
MCP2221_I2C_STOP_TOUT = 0x62
MCP2221_I2C_WRADDRL_TOUT = 0x23
MCP2221_I2C_WAITING_DATA = 0x41
MCP2221_I2C_WRDATA_TOUT = 0x44
MCP2221_I2C_WRDATA_END_NOSTOP = 0x45
MCP2221_I2C_READ_DATA = 0x50
MCP2221_I2C_RDDATA_ACK = 0x53
MCP2221_I2C_WRADDRL_NACK = 0x25
MCP2221_I2C_MASK_ADDR_NACK = 0x40
MCP2221_I2C_WRADDRL_SEND = 0x21
MCP2221_I2C_ADDR_NACK = 0x25
MCP2221_I2C_READ_COMPL_WAIT = 0x54
MCP2221_I2C_READ_COMPL = 0x55
MCP2221_ALT_F_NOT_GPIOV = 0xEE
MCP2221_ALT_F_NOT_GPIOD = 0xEF

MCP2221_DIR_OUT = 0x00
MCP2221_DIR_IN = 0x01

hid_dev = None


def hid_dev_write(data):
    if platform.system() == 'Windows':
        data = b'\x00' + data
    global hid_dev
    hid_dev.write(data)


# # 使用方式：
# print(MCP2221GPIODirection.MCP2221_DIR_OUT)
# print(MCP2221GPIODirection.MCP2221_DIR_IN)

MCP_GPIO_SET = namedtuple('MCP_GPIO_SET', ['change_value', 'value', 'change_direction', 'direction'])
MCP_GPIO_SET_SIZE = 4


class MCPSetGPIO:
    def __init__(self, cmd=None, dummy=None):
        self.cmd = cmd if cmd is not None else 0x50  # 假设默认值为0x50
        self.dummy = dummy if dummy is not None else 0x00  # 假设默认值为0x00
        self.gpio = [MCP_GPIO_SET(0, 0, 0, 0) for _ in range(MCP_NGPIO)]


# # 示例用法：
# mcp_gpio_set = MCPSetGPIO(cmd=0x51, dummy=0xFF)
# mcp_gpio_set.gpio[0] = MCP_GPIO(change_value=1, value=1, change_direction=1, direction=MCP2221GPIODirection.MCP2221_DIR_OUT.value)

MCP_GPIO_GET = namedtuple('MCP_GPIO_GET', ['value', 'direction'])
MCP_GPIO_GET_SIZE = 2

class MCPGetGPIO:
    def __init__(self, cmd=None, dummy=None):
        self.cmd = cmd if cmd is not None else 0x51
        self.dummy = dummy if dummy is not None else 0x00
        self.gpio = [MCP_GPIO_GET(0, 0) for _ in range(MCP_NGPIO)]


def crc8(data: int) -> int:
    poly = POLY  # 引用之前定义的POLY常量

    for _ in range(8):
        if data & 0x8000:
            data ^= poly
        data <<= 1

    return (data >> 8) & 0xFF


def i2c_smbus_pec(crc: int, data: bytes) -> int:
    for byte in data:
        crc = crc8((crc ^ byte) << 8)
    return crc & 0xFF  # 返回的是一个字节（uint8_t），所以需要做位掩码操作


# # 示例：
# initial_crc = 0
# data_array = bytearray([0x11, 0x22, 0x33])
# crc_value = i2c_smbus_pec(initial_crc, data_array)
# print(crc_value)

class MCP2221:
    def __init__(self):
        self.rxbuf = bytearray(64)
        self.txbuf = bytearray(64)
        self.i2c_rxbuf = bytearray(64)
        self.i2c_rxbuf_len = 0
        self.status = 0
        self.cur_i2c_clk_div = 0
        self.gc = None  # 对应gpio_chip，这里假设为一个GPIO控制器对象
        self.gp_idx = 0
        self.gpio_dir = 0
        self.mode = bytearray(4)
        self.desire_state = MCP2221_SUCCESS

    def mcp_send_report(self):
        try:
            hid_dev_write(self.txbuf)
            self.rxbuf = hid_dev.read(64)

            #调试时查看每次发送和接收的数据情况
            # ten_bytes = self.txbuf[:10]
            # ints = [byte for byte in ten_bytes]
            # array1 = [format(number, '02x') for number in ints]
            # array2 = [format(number, '02x') for number in self.rxbuf]
            # print(array1)
            # print(array2[0:10])
            # print("---------------------------")

        except IOError as ex:
            print(ex)
            print("You probably don't have the hard-coded device.")
            print("Update the h.open() line in this script with the one")
            print("from the enumeration list output above and try again.")
            return -1
        return 0

    def mcp_get_i2c_eng_state(self, status: int) -> int:
        i2c_state_map = {
            MCP2221_I2C_WRADDRL_NACK: -ENXIO,
            MCP2221_I2C_WRADDRL_SEND: -ENXIO,
            MCP2221_I2C_START_TOUT: -ETIMEDOUT,
            MCP2221_I2C_STOP_TOUT: -ETIMEDOUT,
            MCP2221_I2C_WRADDRL_TOUT: -ETIMEDOUT,
            MCP2221_I2C_WRDATA_TOUT: -ETIMEDOUT,
            MCP2221_I2C_ENG_BUSY: -EAGAIN,
            MCP2221_I2C_READ_DATA: 0x00,
            MCP2221_I2C_RDDATA_ACK: 0x00,
            MCP2221_I2C_WRDATA_END_NOSTOP: 0x00,
            MCP2221_I2C_READ_COMPL_WAIT: 0x00,
            MCP2221_I2C_READ_COMPL: 0x00,
            MCP2221_SUCCESS: 0x00,
        }

        # 直接使用传入的状态码查找对应的返回值
        return i2c_state_map.get(status, -EIO)

    def mcp2221_raw_event(self):
        data = self.rxbuf

        if data[0] in [MCP2221_I2C_WR_DATA, MCP2221_I2C_WR_NO_STOP, MCP2221_I2C_RD_DATA, MCP2221_I2C_RD_RPT_START]:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
            else:
                self.status = self.mcp_get_i2c_eng_state(data[2])

        if data[0] == MCP2221_I2C_PARAM_OR_STATUS:
            if data[1] == MCP2221_SUCCESS:
                if (self.txbuf[3] == MCP2221_I2C_SET_SPEED) and (data[3] != MCP2221_I2C_SET_SPEED):
                    self.status = -EAGAIN
                elif data[20] & MCP2221_I2C_MASK_ADDR_NACK:
                    self.status = -ENXIO
                else:
                    self.status = self.mcp_get_i2c_eng_state(data[8])
            else:
                self.status = -EIO

        if data[0] == MCP2221_I2C_GET_DATA:
            if data[1] == MCP2221_SUCCESS:
                if data[2] == MCP2221_I2C_ADDR_NACK:
                    self.status = -ENXIO
                elif data[3] == 127:
                    self.status = -EIO
                else:
                    self.status = self.mcp_get_i2c_eng_state(data[2])
            else:
                self.status = -EIO

        if data[0] == MCP2221_GPIO_SET:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
            else:
                self.status = -EAGAIN

        if data[0] == MCP2221_GPIO_GET:
            if data[1] == MCP2221_SUCCESS:
                if (data[self.gp_idx] == MCP2221_ALT_F_NOT_GPIOV) or \
                        (data[self.gp_idx + 1] == MCP2221_ALT_F_NOT_GPIOD):
                    self.status = -ENOENT
                else:
                    self.status = bool(data[self.gp_idx])
                    self.gpio_dir = data[self.gp_idx + 1]
            else:
                self.status = -EAGAIN

        if data[0] == MCP2221_SET_SRAM_SETTINGS:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
            else:
                self.status = -EAGAIN

        if data[0] == MCP2221_READ_FLASH_DATA:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
                if self.txbuf[1] == MCP2221_CHIP_SETTING:
                    if self.txbuf[1] != 0:
                        self.status = -EIO
                if self.txbuf[1] == MCP2221_GP_SETTING:
                    if data[1] != 0:
                        self.status = -EIO
            else:
                self.status = -EAGAIN

        if data[0] == MCP2221_WRITE_FLASH_DATA:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
            else:
                self.status = -EAGAIN

        if data[0] == MCP2221_SEND_FLASH_PASSWORD:
            if data[1] == MCP2221_SUCCESS:
                self.status = 0
            else:
                self.status = -EAGAIN

        if data[0] not in [MCP2221_I2C_WR_DATA, MCP2221_I2C_WR_NO_STOP, MCP2221_I2C_RD_DATA, MCP2221_I2C_RD_RPT_START,
                           MCP2221_I2C_PARAM_OR_STATUS, MCP2221_I2C_GET_DATA, MCP2221_SET_SRAM_SETTINGS,
                           MCP2221_READ_FLASH_DATA, MCP2221_SEND_FLASH_PASSWORD, MCP2221_WRITE_FLASH_DATA]:
            self.status = -EIO

    def mcp_send_data_req_status(self):
        ret = self.mcp_send_report()
        if ret:
            return ret
        time.sleep(0.002)
        self.mcp2221_raw_event()
        return self.status

    def mcp_chk_last_cmd_status(self):
        """
        Status/Set Parameters – command code
        :return:
        """
        self.txbuf[:64] = bytearray(64)  # 使用Python方式清零txbuf前64个字节
        self.txbuf[0] = MCP2221_I2C_PARAM_OR_STATUS  # 0x10

        ret = self.mcp_send_data_req_status()
        if ret:
            return ret
        
        if self.rxbuf[8] != self.desire_state:
            time.sleep(0.005)
            ret = self.mcp_send_data_req_status()
        
        if self.rxbuf[8] != self.desire_state:
            return -EAGAIN

        return ret

    def mcp_clear_buffer(self, smbus_addr):
        """
        清除mcp buffer
        """
        retry = 0
        while retry < 5:
            self.txbuf[0] = MCP2221_I2C_PARAM_OR_STATUS  # 0x10 检查状态
            ret = self.mcp_send_report()
            if ret:
                return ret
            # 根据返回状态处理
            error_status_code = [0x25, 0x17, 0x45, 0x44, 0x62]
            if self.rxbuf[8] == 0x55:
                print(f'[warning] mcp status is {self.rxbuf[8]:02x}, clear buffer...')
                self.txbuf = bytearray(64)
                self.txbuf[0] = MCP2221_I2C_GET_DATA # 0x40
                ret = self.mcp_send_data_req_status()
                if ret:
                    return ret
            elif self.rxbuf[8] in error_status_code:
                print(f'[warning] mcp status is {self.rxbuf[8]:02x}, clear buffer...')
                self.txbuf = bytearray(64)
                self.txbuf[0] = MCP2221_I2C_RD_RPT_START # 0x93
                self.txbuf[1] = 0x01
                self.txbuf[3] = (smbus_addr << 1)| I2C_SMBUS_READ
                self.txbuf[4] = 0x01
                ret = self.mcp_send_report()
                if ret:
                    return ret
            elif self.rxbuf[8] == 0x00:
                break
            else:
                print(f"Error: Unknown status code, {self.rxbuf[8]:02x}")

            retry += 1

        return ret

    def mcp_cancel_last_cmd(self):
        """
        取消当前I2C/SMBus传输
        :return: 发送状态
        """
        self.txbuf[:64] = bytearray(64)  # 使用Python方式清零txbuf前64个字节
        self.txbuf[0] = MCP2221_I2C_PARAM_OR_STATUS
        self.txbuf[2] = MCP2221_I2C_CANCEL

        return self.mcp_send_data_req_status()

    def mcp_set_i2c_speed(self):
        """
        设置I2C/SMBus通信速度
        :return:
        """
        ret = 0
        self.txbuf[:64] = bytearray(64)  # 清零txbuf前64个字节
        self.txbuf[0] = MCP2221_I2C_PARAM_OR_STATUS
        self.txbuf[3] = MCP2221_I2C_SET_SPEED
        self.txbuf[4] = self.cur_i2c_clk_div
        ret = self.mcp_send_data_req_status()

        if ret:
            time.sleep(0.001)  # usleep(1000) 转换为 Python 的 time.sleep()，单位为秒
            self.mcp_cancel_last_cmd()

        return ret

    def mcp_smbus_write(self, flag, smbus_addr, smb_cmd, buf):
        """
        :param smbus_addr: 7bit设备地址0x68
        :param flag: pec enable
        :param smb_cmd: smbus smb_cmd字段
        :param buf: buf内容
        :return:
        """
        data_len = 0
        ret = 0
        buf_len = len(buf)

        self.txbuf[0] = MCP2221_I2C_WR_DATA

        if flag:
            self.txbuf[1] = buf_len + 3  # 包含命令字节、长度字节和PEC字节
        else:
            self.txbuf[1] = buf_len + 2
        self.txbuf[2] = 0
        self.txbuf[3] = (smbus_addr << 1)
        self.txbuf[4] = smb_cmd
        self.txbuf[5] = buf_len

        if buf_len > MCP_READ_MAX_LEN:
            return -EINVAL
        else:
            if buf is not None:
                self.txbuf[6:6 + buf_len] = buf  # 使用切片赋值将buf内容复制到txbuf指定位置
                data_len = buf_len + 6
            else:
                data_len = 6

        if flag:
            addrbyte = (smbus_addr << 1) | I2C_SMBUS_WRITE
            pec = i2c_smbus_pec(0, bytes([addrbyte]))
            pec = i2c_smbus_pec(pec, self.txbuf[4:4 + buf_len + 2])
            self.txbuf[data_len] = pec
            ret = self.mcp_send_data_req_status()
        else:
            ret = self.mcp_send_data_req_status()

        if ret != 0:
            return ret

        retry_count = 0
        self.desire_state = MCP2221_SUCCESS
        while retry_count <= 10:
            ret = self.mcp_chk_last_cmd_status()
            if ret == 0:
                break
            retry_count += 1
        if retry_count > 10:
            return -ETIMEDOUT

        return ret

    def i2c_raw_send(self, mcp_cmd, smbus_addr, rw_flag, smbus_buf):
        ret = 0
        self.txbuf[0] = mcp_cmd
        self.txbuf[1] = len(smbus_buf)
        self.txbuf[2] = 0
        self.txbuf[3] = (smbus_addr << 1) | rw_flag
        self.txbuf[4:] = smbus_buf
        ret = self.mcp_send_data_req_status()

        return ret

    def i2c_raw_buf_read(self):
        self.txbuf[:4] = bytearray(4)
        self.txbuf[0] = MCP2221_I2C_GET_DATA
        ret = self.mcp_send_data_req_status()
        if ret:
            return ret
        # 0x40读取I2C的data, 返回数据长度
        self.i2c_rxbuf_len = self.rxbuf[3]
        if not (self.i2c_rxbuf_len > 0 and self.i2c_rxbuf_len <= 60):
            print("Error:rxbuf_len < 0 or > 60,", self.rxbuf[0:10])
            return -1
        self.i2c_rxbuf = self.rxbuf[4:64]

        return 0

    def mcp_smbus_read(self, flag, smbus_addr, smb_cmd):
        ret = 0
        read_cnt = 0

        while read_cnt < 5:
            read_cnt += 1
            # I2C读取数据前发送write指令
            smb_buf_tmp = bytearray(1)
            smb_buf_tmp[0] = smb_cmd
            ret = self.i2c_raw_send(MCP2221_I2C_WR_DATA, smbus_addr, 0, smb_buf_tmp)
            if ret:
                return ret
            # I2C读取数据
            smb_buf_tmp = bytearray(1)
            smb_buf_tmp[0] = 0x01  # 1byte length
            # 发送mcp指令
            ret = self.i2c_raw_send(MCP2221_I2C_RD_DATA, smbus_addr, 1, smb_buf_tmp)
            if ret:
                return ret
            
            self.desire_state = MCP2221_I2C_READ_COMPL
            ret = self.mcp_chk_last_cmd_status()
            if ret:
                return ret
            # 把数据读回来
            ret = self.i2c_raw_buf_read()
            if ret:
                return ret
            if self.i2c_rxbuf[0] != 0:
                break
            
        
        txlen = self.i2c_rxbuf[0] + 1 + flag
        smb_buf_tmp = bytearray(txlen)
        ret = self.i2c_raw_send(MCP2221_I2C_RD_DATA, smbus_addr, 1, smb_buf_tmp)
        if ret:
            return ret
        
        ret = self.mcp_chk_last_cmd_status()
        if ret:
            return ret

        ret = self.i2c_raw_buf_read()  # 0x40
        if ret:
            return ret

        if flag:
            addrbyte = (smbus_addr << 1) | I2C_SMBUS_WRITE
            pec = i2c_smbus_pec(0, bytes([addrbyte]))
            pec = i2c_smbus_pec(pec, bytes([smb_cmd]))
            addrbyte = (smbus_addr << 1) | I2C_SMBUS_READ
            pec = i2c_smbus_pec(pec, bytes([addrbyte]))
            pec = i2c_smbus_pec(pec, self.i2c_rxbuf[:self.i2c_rxbuf[0] + 1])

            if pec != self.i2c_rxbuf[self.i2c_rxbuf[0] + 1]:
                print(f"Calculated pec is {pec:02x}, diff from {self.i2c_rxbuf[self.i2c_rxbuf[0] + 1]:02x}")
                return -EIO

        return ret

    def mcp_smbus_xfer(self, flag, addr, read_write, command, data):
        """
        :param flag: enable or disable pec
        :param addr: 设备地址
        :param read_write: read-->1, write-->0
        :param command: smbus cmd
        :param data: 传输的数据
        :return:
        """
        ret = 0
        if read_write == I2C_SMBUS_READ:
            ret = self.mcp_smbus_read(flag, addr, command)
            if ret != 0:
                print(f"read ret is {ret}.")
            return ret
        else:
            if len(data) == 0:
                ret = -EINVAL
                return ret
            ret = self.mcp_smbus_write(flag, addr, command, data)
            if ret != 0:
                print(f"write ret is {ret}.")
            return ret

    def mcp_gpio_dir_set(self, offset, val):
        self.txbuf = bytearray(18)
        self.txbuf[0] = MCP2221_GPIO_SET

        mcp_gpio_set = MCPSetGPIO(cmd=MCP2221_GPIO_SET, dummy=0x00)
        # 通过offset定位到特定引脚的方向设置位置
        self.gp_idx = 4 + offset * MCP_GPIO_SET_SIZE
        self.txbuf[self.gp_idx] = 1
        if val == 'out':
            dir_val = MCP2221_DIR_OUT
        elif val == 'in':
            dir_val = MCP2221_DIR_IN
        else:
            print("【Error】gpio direction val inval")
            return -EINVAL
        self.txbuf[self.gp_idx + 1] = dir_val  # 将1映射为输入，0映射为输出

        return self.mcp_send_data_req_status()

    def mcp_gpio_set(self, offset, val):
        self.txbuf = bytearray(18)
        self.txbuf[0] = MCP2221_GPIO_SET

        mcp_gpio_set = MCPSetGPIO(cmd=MCP2221_GPIO_SET, dummy=0x00)
        # 通过offset定位到特定引脚的方向设置位置
        self.gp_idx = 2 + offset * MCP_GPIO_SET_SIZE
        self.txbuf[self.gp_idx] = 1
        self.txbuf[self.gp_idx + 1] = val  # 将1映射为高电平，0映射为低电平

        return self.mcp_send_data_req_status()

    def mcp_gpio_get(self, offset):
        self.txbuf[0] = MCP2221_GPIO_GET
        self.gp_idx = 2 + offset * MCP_GPIO_GET_SIZE
        self.mcp_send_data_req_status()
        return self.status

    def set_gpio_direction(self, pinindex, direction):
        self.txbuf = bytearray(18)
        self.txbuf[0] = MCP2221_GPIO_SET
        self.txbuf[pinindex * 4 + 4] = 0xff
        if direction == 'in':
            self.txbuf[pinindex * 4 + 5] = 0xff
        print(f"【set_gpio_direction】: {self.txbuf}")
        self.mcp_send_data_req_status()

    def set_gpio_value(self, pinindex, value):
        self.txbuf = bytearray(18)
        self.txbuf[0] = MCP2221_GPIO_SET
        self.txbuf[pinindex * 4 + 2] = 0xff
        self.txbuf[pinindex * 4 + 3] = value
        print(f"【set_gpio_value】: {self.txbuf}")
        self.mcp_send_data_req_status()

    def mcp_send_flash_password(self, password):
        """
        发送 Flash 访问密码
        :param password: 密码
        :return: 返回执行状态
        """
        self.txbuf = bytearray(9)
        self.txbuf[0] = MCP2221_SEND_FLASH_PASSWORD
        self.txbuf[1] = 0x00
        self.txbuf[2:10] = password.encode('utf-8')
        self.mcp_send_data_req_status()

        return self.status

    def mcp_read_chip_setting(self):
        """
        读取闪存数据
        :return: 返回执行状态
        """
        self.txbuf = bytearray(2)
        self.txbuf[0] = MCP2221_READ_FLASH_DATA
        self.txbuf[1] = MCP2221_CHIP_SETTING
        ret = self.mcp_send_data_req_status()
        if ret == 0:
            return bytearray(self.rxbuf)
        else:
            ret = -EIO
            return ret

    def mcp_chip_security_opt(self, opt, password):
        """
        :param opt: 安全配置选项，"00"-->未加密；“01”-->密码保护
        :param password: 密码，传入字符串
        :return:
        """
        self.txbuf = bytearray(64)
        self.txbuf = self.mcp_read_chip_setting()
        if self.txbuf == bytearray(self.rxbuf):
            self.txbuf[:2] = bytearray(2)  # 清空前2个字节
            self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
            self.txbuf[1] = MCP2221_CHIP_SETTING
            self.txbuf[2:] = self.rxbuf[4:]

            pw_binary = bin(self.txbuf[2])[2:].zfill(8)[:-2] + opt
            hex_pw_opt = hex(int(pw_binary, 2))[2:]
            self.txbuf[2] = int(hex_pw_opt, 16)  # 将16进制字符串转换为整数，并赋值给txbuf的第三个字节
            # 如果opt == "00",未加密，无需传入密码，如果是”01“，byte12-19传入密码
            if opt == "01":
                if len(password) != 8:
                    print("[Error]: Password must be 8 bytes.")
                else:
                    data = [ord(char) for char in password]
                    index = 12
                    for byte in data:
                        self.txbuf[index] = int("{:02x}".format(byte), 16)
                        index += 1

            return self.mcp_send_data_req_status()
        else:
            ret = -EIO
            return ret

    def mcp_read_gp_setting(self):
        """
        读取GP设置
        :return:
        """
        self.txbuf = bytearray(2)
        self.txbuf[0] = MCP2221_READ_FLASH_DATA
        self.txbuf[1] = MCP2221_GP_SETTING
        ret = self.mcp_send_data_req_status()
        if ret == 0:
            return bytearray(self.rxbuf)
        else:
            ret = -EIO
            return ret

    def mcp_write_gp_setting(self):
        """
        写入GP设置
        :param gp_setting: GP设置，传入格式如：0x01
        :return:
        """
        ret = 0
        gpio_cfg = [0x08, 0x08, 0x08, 0x08]
        self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
        self.txbuf[1] = MCP2221_GP_SETTING
        self.txbuf[2:] = gpio_cfg
        self.mcp_send_data_req_status()
        read_gp_data = self.mcp_read_gp_setting()
        if not all(byte == 0x08 for byte in read_gp_data[4:8]):
            print("[Error]: Write GP setting failed.")
            ret = -EIO
            return ret
        print("GP-SETTING initialized successfully")
        return ret

    def mcp_write_vid_pid(self, vid, pid):
        """
        修改设备的VID,PID
        :param vid: VID,传入格式如：0x04d8
        :param pid: PID,传入格式如：0x00dd
        :return:
        """
        self.txbuf = self.mcp_read_chip_setting()
        self.rxbuf = bytearray(self.rxbuf)
        if self.txbuf == self.rxbuf:
            self.txbuf[:4] = bytearray(4)  # 清空前4个字节
            # 重新构造指令
            self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
            self.txbuf[1] = MCP2221_CHIP_SETTING
            self.txbuf[2:] = self.rxbuf[4:]
            self.txbuf[6] = vid & 0xFF
            self.txbuf[7] = vid >> 8
            self.txbuf[8] = pid & 0xFF
            self.txbuf[9] = pid >> 8
            ret = self.mcp_send_data_req_status()
            if ret:
                print("[Error]: Change VID,PID failed.")
                return ret
            print("VID PID initialized successfully")
            return ret
        else:
            ret = -EIO
            print("读取chip setting和写入chip setting不对应")
            return ret

    def mcp_read_usb_manu_dec(self):
        """
        读取设备制造商描述
        :return:
        """
        self.txbuf = bytearray(2)
        self.txbuf[0] = MCP2221_READ_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_MANU_DESC
        self.mcp_send_data_req_status()
        rsp = bytearray(self.rxbuf)
        print(f"manu : {rsp}")

        return rsp
    
    def mcp_write_usb_manu_dec(self, desc_str):
        """
        修改设备制造商描述
        :param desc_str: 描述字符串
        :return:
        """
        self.txbuf = bytearray(4 + 2 * len(desc_str))
        self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_MANU_DESC
        self.txbuf[2] = 2 * len(desc_str) + 2  # 2 + 2 x (number of Unicode characters in the string)
        self.txbuf[3] = 0x03  # 固定值
        # 从byte index 4开始，获取描述字符串的高低位并按顺序存储在txbuf
        self.lower_and_hight_unicode_char(desc_str, index=4)

        ret = self.mcp_send_data_req_status()
        if ret:
            print("[Error]: Write USB Manufacturer Descriptor failed.")
            return ret
        print("USB Manufacturer Descriptor initialized successfully")

        return ret

    def mcp_read_usb_product_dec(self):
        """
        读取产品描述字符串
        :return:
        """
        self.txbuf = bytearray(2)
        self.txbuf[0] = MCP2221_READ_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_PRODUCT_DESC
        self.mcp_send_data_req_status()
        rsp = bytearray(self.rxbuf)

        return rsp

    def mcp_write_usb_product_dec(self, desc_str):
        """
        修改产品描述字符串
        :param desc_str: 描述字符串
        :return:
        """
        self.txbuf = bytearray(4 + 2 * len(desc_str))
        self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_PRODUCT_DESC
        self.txbuf[2] = 2 * len(desc_str) + 2  # 2 + 2 x (number of Unicode characters in the string)
        self.txbuf[3] = 0x03  # 固定值
        # 从byte index 4开始，获取描述字符串的高低位并按顺序存储在txbuf
        self.lower_and_hight_unicode_char(desc_str, index=4)

        ret = self.mcp_send_data_req_status()
        if ret:
            print("[Error]: Write USB Product Descriptor failed.")
            return ret
        print("USB Product Descriptor initialized successfully")

        return ret

    def mcp_read_usb_serial_num_dec(self):
        """
        读取设备序列号描述字符串
        :return:
        """
        self.txbuf = bytearray(2)
        self.txbuf[0] = MCP2221_READ_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_SERIAL_NUMBER_DESC
        self.mcp_send_data_req_status()
        rsp = bytearray(self.rxbuf)

        return rsp

    def mcp_write_usb_serial_num_dec(self):
        """
        修改设备序列号描述字符串
        :param desc_str: 描述字符串
        :return:
        """
        product_dec = "AliPROT"
        product_dec_byte = []
        for char in product_dec:
            low_byte = (ord(char) & 0xFF)
            high_byte = ((ord(char) >> 8) & 0xFF)
            product_dec_byte.append(low_byte)
            product_dec_byte.append(high_byte)
        product_data = bytearray(product_dec_byte)
        data = self.mcp_read_usb_product_dec()[4:2 * len(product_dec) + 4]
        if product_data == data:
            print("USB Serial Number Descriptor already initialized.")
            ret = 0
            return ret
        # desc_str取当前时间，精确到微秒
        current_time = datetime.datetime.now()
        desc_str = current_time.strftime("%Y%m%d%H%M%S%f")
        self.txbuf = bytearray(4 + 2 * len(desc_str))
        self.txbuf[0] = MCP2221_WRITE_FLASH_DATA
        self.txbuf[1] = MCP2221_USB_SERIAL_NUMBER_DESC
        self.txbuf[2] = 2 * len(desc_str) + 2  # 2 + 2 x (number of Unicode characters in the string)
        self.txbuf[3] = 0x03  # 固定值
        # 从byte index 4开始，获取描述字符串的高低位并按顺序存储在txbuf
        self.lower_and_hight_unicode_char(desc_str, index=4)
        write_data = self.txbuf[4:2 * len(desc_str) + 2]
        ret = self.mcp_send_data_req_status()
        read_rsp = self.mcp_read_usb_serial_num_dec()[4:2 * len(desc_str) + 2]
        if write_data[4:] != read_rsp[4:]:
            print("[Error]: Write USB serial number failed.")
            ret = -EIO
            return ret
        print("USB Serial Number Descriptor initialized successfully")

        return ret

    def lower_and_hight_unicode_char(self, desc_str, index):
        """
        获取字符的高低字节并按顺序写入byte index
        :param desc_str: 字符
        :param index: byte index
        :return:
        """
        for char in desc_str:
            low_byte = ord(char) & 0xFF
            high_byte = (ord(char) >> 8) & 0xFF
            self.txbuf[index] = low_byte
            self.txbuf[index + 1] = high_byte
            index += 2

    def reset_chip(self):
        """
        retset chip
        :return: 返回执行状态
        """
        self.txbuf = bytearray(4)
        self.txbuf[0:4] = [0x70, 0xAB, 0xCD, 0xEF]
        hid_dev_write(self.txbuf)

        return 0
    
    def get_port_device(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port is None:
                print("Error: No serial port found")
                exit(1)
            else:
                uart_port =  read_config('common', 'uart_port')
                if uart_port in port.device:
                    # print(port.device, port.hwid)
                    location = re.findall(r'LOCATION=(.*):', port.hwid)[0]
                    vid_pid = re.findall(r'VID:PID=(\w+:\w+)', port.hwid)[0]
                    vid = vid_pid.split(':')[0]
                    pid = vid_pid.split(':')[1]
                    prot = port.device
                    return (prot, vid, pid, location)
                
        print("No AliPROT device found, please check physical port in config.ini")
        print("Check physical port:\n  Linux: send lsusb or stty -F /dev/ttyACM0\n  Windows: Open PowerShell, send cmd: Get-PnpDevice -Class Ports")
        exit(1)

    def init_device(self):
        global hid_dev
        hid_dev = hid.device()

        # physical_port = '1-4' # windows: 7&2616b5b5&0&0000
        # uart_port:eg windows:COM9, linux:/dev/ttyACM0，mac:/dev/tty.usbmodem11101
        physical_port =  read_config('common', 'physical_port')
        uart_port = read_config('common', 'uart_port')
        vid = None
        pid = None
        if physical_port == '' and uart_port == '':
            config_vid = read_config("common", "vid")
            config_pid = read_config("common", "pid")
            if config_vid and config_pid:
                try:
                    for port in serial.tools.list_ports.comports():
                        int_vid = int(config_vid, 16)
                        int_pid = int(config_pid, 16)
                        vid_pid_str = f"VID:PID={int_vid:04X}:{int_pid:04X}"
                        vid_pid_str_def = f"VID:PID={vendor_id_def:04X}:{product_id_def:04X}"
                        if vid_pid_str in port.hwid:
                            vid = int_vid
                            pid = int_pid 
                        elif vid_pid_str_def in port.hwid:
                            vid = vendor_id_def
                            pid = product_id_def
                    if vid == None or pid == None:
                        print(f"not found hid device with config:|vid={int_vid:04x}, pid={int_pid:04x}| or default:|vid={vendor_id_def:04x}, pid={product_id_def:04x}|")
                        exit(1)
                    print(f"Use vendor_id:{vid:04x}, product_id:{pid:04x}")
                    hid_dev.open(vid, pid)
                    return hid_dev
                except Exception as e:
                    print(e)
                    print("Please check if the device is connected or not.")
                    exit(1)
            else:
                print("Please set physical_port or uart_port or vid pid in config.ini")
                print("Get physical_port: <python3 arottool.py|./arottool> usb_port [--port e.g /dev/ttyACM0]")
                exit(1)

        try:
            target_device_path = None
            for device in hid.enumerate():
                device_path = device['path'].decode()

                # 检查设备是否包含 "hid", 区分系统
                if "hidraw" in device_path:  # linux
                    # 获取设备信息的命令
                    cmd = f"udevadm info --name={device_path} --query=all"
                    result = subprocess.run(cmd.split(), stdout=subprocess.PIPE, universal_newlines=True)
                    stdout = result.stdout

                    # 检查物理端口
                    if physical_port != '':
                        if physical_port in stdout:
                            target_device_path = device_path
                            # print(f"Found matching HID device at {device_path}")
                            break
                    elif physical_port == '' and uart_port != '':
                        prot_info = self.get_port_device()
                        if prot_info[3] in stdout:
                            target_device_path = device_path
                            break
                    else:
                        print("Please set physical_port or uart_port in config.ini")
                        exit(1)
                else:
                    if physical_port != '':
                        if physical_port in device_path:
                            target_device_path = device_path
                            # print(f"Found matching device at {device_path}")
                            break
                    elif physical_port == '' and uart_port != '':
                        prot_info = self.get_port_device()
                        if prot_info[3] in device_path:
                            target_device_path = device_path
                            break
                        if prot_info[1] in device_path and prot_info[2] in device_path:
                            target_device_path = device_path
                            break
                    else:
                        print("Please set physical_port or uart_port in config.ini")

            if target_device_path is None:
                print(f"No device found with physical port {physical_port}")
                return

            # 打开设备
            hid_dev.open_path(target_device_path.encode())
            return hid_dev
        
        except Exception as e:
            print(e)
            print("Please check if the device is connected or not.")
            exit(1)


    def free_device(self):

        if hid_dev is not None:
            hid_dev.close()
        else:
            return -EIO
        
        return 0
