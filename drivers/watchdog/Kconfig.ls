
menuconfig  WDT_LS
    bool "Linkedsemi Watchdog driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_WATCHDOG_ENABLED     
    select HAS_WDT_DISABLE_AT_BOOT
    help
      Enable driver for Linkedsemi's hardware watchdog timer.

      If CONFIG_WDT_DISABLE_AT_BOOT is also enabled, driver init
      stops the IWDG hardware and gates off its clock. 