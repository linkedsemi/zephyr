from mcp import *

I2C_SMBUS_READ = 1
I2C_SMBUS_WRITE = 0

mcp2221 = MCP2221()

if __name__ == "__main__":
    mcp2221.init_device()
    # write i2c slave
    mcp2221.mcp_smbus_xfer(True, 0x68, I2C_SMBUS_WRITE, 2, bytes([0xb8, 0x04, 0x00, 0x4c, 0xa5, 0x00]))
    # read i2c slave
    mcp2221.mcp_smbus_xfer(False, 0x68, I2C_SMBUS_READ, 2, None)
    print(bytes(mcp2221.i2c_rxbuf))
