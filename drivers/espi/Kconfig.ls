config ESPI_LS
    bool "Linkedsemi ESPI slave driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_ESPI_ENABLED
    help
        Enable Linkedsemi ESPI slave driver.

if ESPI_LS

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

config ESPI_VW_TX_MSG_MAX
    int "Maximum pending virtual wire TX messages"
    range 1 32
    default 4
    help
      Maximum number of virtual wire TX messages that can be queued
      before they are handled by the dedicated TX thread. Messages are
      dropped when the queue is full.

config ESPI_VW_TX_THREAD_STACK_SIZE
    int "Virtual wire TX thread stack size"
    default 1024
    help
      Stack size for the dedicated virtual wire TX thread that drains
      the TX message queue.

config ESPI_VW_TX_THREAD_PRIORITY
    int "Virtual wire TX thread priority"
    default -2
    help
      Priority of the dedicated virtual wire TX thread.

endif

config ESPI_BMC_LS
    bool "Linkedsemi ESPI slave driver"
    default y
    depends on DT_HAS_LINKEDSEMI_LS_ESPI_BMC_ENABLED
    help
        Enable Linkedsemi ESPI slave driver.
