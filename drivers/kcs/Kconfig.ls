
config KCS_LS
    bool "Linkedsemi KCS driver"
    default y
	depends on DT_HAS_LINKEDSEMI_LS_KCS_ENABLED
    help
      Enable support for linkedsemi KCS driver