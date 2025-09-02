.. _spi_flash_communication:

NOTICE
********************
The SPI and external flash pins need to be consistent with the hardware.

Sample Output(if board is ls101x)
=======================================
Serial port output:
    Serial port output
    jedec-id = [85 20 18];//flash:PY25Q
    Flash sector erased
    Data written to Flash
    Data read from Flash:0x9F
Connect the logic analyzer:
    the observation function return the device ID
                             and the process related to the commands for reading and writing

Sample Output(if board is qsh610)
=======================================
    Serial port output
    jedec-id = [85 63 19];//flash:PY25R
    Flash sector erased
    Data written to Flash
    Data read from Flash:0x9F
Connect the logic analyzer:
    the observation function return the device ID
                             and the process related to the commands for reading and writing