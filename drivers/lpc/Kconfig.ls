
config LPC_LS
    bool "Linkedsemi KCS driver"
	depends on DT_HAS_LINKEDSEMI_LS_LPC_ENABLED
    help
      Enable support for linkedsemi LPC driver