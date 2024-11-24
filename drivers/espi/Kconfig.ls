config ESPI_LS
    bool "Linkedsemi ESPI slave driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_ESPI_ENABLED
    help
        Enable Linkedsemi ESPI slave driver.

if ESPI_LS

config ESPI_LS_VUART
    bool "Linkedsemi ESPI slave to uart driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_VUART_ENABLED
    help
        Enable Linkedsemi ESPI slave to uart driver.

config ESPI_LS_SIO
    bool "Linkedsemi ESPI slave to sio driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_SIO_ENABLED
    help
        Enable Linkedsemi ESPI slave to sio driver.

config ESPI_LS_MMBI
    bool "Linkedsemi ESPI slave to mmbi driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_MMBI_ENABLED
    help
        Enable Linkedsemi ESPI slave to mmbi driver.

endif
