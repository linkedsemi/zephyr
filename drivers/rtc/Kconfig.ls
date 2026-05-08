config RTC_LS
    bool "LS101x RTC driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_RTC_ENABLED
    help
      Enable support for LS101x RTC peripheral.