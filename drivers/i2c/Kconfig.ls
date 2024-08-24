

config I2C_LS
	bool "Linkedsemi I2C driver"
	default y
	depends on DT_HAS_LINKEDSEMI_LS_I2C_ENABLED
	help
	  Enable support for linkedsemi I2C driver
