
menuconfig  WDT_LS
    bool "Linkedsemi Watchdog driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_WATCHDOG_ENABLED                                 
    help
      Enable driver for Linkedsemi's hardware watchdog timer.