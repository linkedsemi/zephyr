config RTC_LS
    bool "LS101x RTC driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_RTC_ENABLED
    help
      Enable support for LS101x RTC peripheral.

config RTC_TIMER_LS
    bool "Linkedsemi BSTIM/GPTIM software RTC driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_RTC_TIMER_ENABLED
    depends on NEWLIB_LIBC || PICOLIBC
    help
      Software RTC backed by the BSTIM + PIS + GPTIMA timer chain in
      platform.c. Only set_time and get_time are supported.
