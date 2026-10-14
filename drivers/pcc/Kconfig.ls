
config PCC_LS
    bool "Linkedsemi PCC driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_PCC_ENABLED
    help
      Enable support for linkedsemi PCC driver
