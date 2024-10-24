
config IPMI_KCS_LS
    bool "Linkedsemi IPMI over KCS driver"
    default y
	depends on DT_HAS_LINKEDSEMI_LS_IPMI_KCS_ENABLED
    help
      Enable support for linkedsemi IPMI over KCS driver