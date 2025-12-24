

config I2C_LS
	bool "Linkedsemi I2C driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_I2C_ENABLED
	select I2C_BITBANG
	help
	  Enable support for linkedsemi I2C driver

config LS_I2C_BUS_TIMEOUT_MS
	int "The longest bus deadlock time: After the timeout, the Master resets the I2C and releases the bus."
	default 200

if I2C_LS

config I2C_SHOW_STATE
	bool "I2C_SHOW_STATE"
	default y
	depends on SOC_LSQSH
	help
	  i2c show internal state

endif
