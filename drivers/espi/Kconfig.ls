config ESPI_LS
    bool "Linkedsemi ESPI slave driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_ESPI_ENABLED
    help
        Enable Linkedsemi ESPI slave driver.