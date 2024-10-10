
#ifndef ZEPHYR_DRIVERS_MISC_SPID_REG_H_
#define ZEPHYR_DRIVERS_MISC_SPID_REG_H_

/*     |name                    offset     | length   |   description                                  */
#define SPID_INTR_STATE          0x0     //|        4 | Interrupt State Register                        |
#define SPID_INTR_ENABLE         0x4     //|        4 | Interrupt Enable Register                       |
#define SPID_INTR_TEST           0x8     //|        4 | Interrupt Test Register                         |
#define SPID_ALERT_TEST          0xc     //|        4 | Alert Test Register                             |
#define SPID_CONTROL             0x10    //|        4 | Control register                                |
#define SPID_CFG                 0x14    //|        4 | Configuration Register                          |
#define SPID_STATUS              0x18    //|        4 | SPI Device status register                      |
#define SPID_INTERCEPT_EN        0x1c    //|        4 | Intercept Passthrough datapath.                 |
#define SPID_ADDR_MODE           0x20    //|        4 | Flash address mode configuration                |
#define SPID_LAST_READ_ADDR      0x24    //|        4 | Last Read Address                               |
#define SPID_FLASH_STATUS        0x28    //|        4 | SPI Flash Status register.                      |
#define SPID_JEDEC_CC            0x2c    //|        4 | JEDEC Continuation Code configuration register. |
#define SPID_JEDEC_ID            0x30    //|        4 | JEDEC ID register.                              |
#define SPID_READ_THRESHOLD      0x34    //|        4 | Read Buffer threshold register.                 |
#define SPID_MAILBOX_ADDR        0x38    //|        4 | Mailbox Base address register.                  |
#define SPID_UPLOAD_STATUS       0x3c    //|        4 | Upload module status register.                  |
#define SPID_UPLOAD_STATUS2      0x40    //|        4 | Upload module status 2 register.                |
#define SPID_UPLOAD_CMDFIFO      0x44    //|        4 | Command Fifo Read Port.                         |
#define SPID_UPLOAD_ADDRFIFO     0x48    //|        4 | Address Fifo Read Port.                         |
#define SPID_CMD_FILTER_0        0x4c    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_1        0x50    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_2        0x54    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_3        0x58    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_4        0x5c    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_5        0x60    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_6        0x64    //|        4 | Command Filter                                  |
#define SPID_CMD_FILTER_7        0x68    //|        4 | Command Filter                                  |
#define SPID_ADDR_SWAP_MASK      0x6c    //|        4 | Address Swap Mask register.                     |
#define SPID_ADDR_SWAP_DATA      0x70    //|        4 | The address value for the address swap feature. |
#define SPID_PAYLOAD_SWAP_MASK   0x74    //|        4 | Write Data Swap in the passthrough mode.        |
#define SPID_PAYLOAD_SWAP_DATA   0x78    //|        4 | Write Data Swap in the passthrough mode.        |
#define SPID_CMD_INFO_0          0x7c    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_1          0x80    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_2          0x84    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_3          0x88    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_4          0x8c    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_5          0x90    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_6          0x94    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_7          0x98    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_8          0x9c    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_9          0xa0    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_10         0xa4    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_11         0xa8    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_12         0xac    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_13         0xb0    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_14         0xb4    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_15         0xb8    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_16         0xbc    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_17         0xc0    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_18         0xc4    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_19         0xc8    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_20         0xcc    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_21         0xd0    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_22         0xd4    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_23         0xd8    //|        4 | Command Info register.                          |
#define SPID_CMD_INFO_EN4B       0xdc    //|        4 | Opcode for EN4B.                                |
#define SPID_CMD_INFO_EX4B       0xe0    //|        4 | Opcode for EX4B                                 |
#define SPID_CMD_INFO_WREN       0xe4    //|        4 | Opcode for Write Enable (WREN)                  |
#define SPID_CMD_INFO_WRDI       0xe8    //|        4 | Opcode for Write Disable (WRDI)                 |
#define SPID_TPM_CAP             0x800   //|        4 | TPM HWIP Capability register.                   |
#define SPID_TPM_CFG             0x804   //|        4 | TPM Configuration register.                     |
#define SPID_TPM_STATUS          0x808   //|        4 | TPM submodule state register.                   |
#define SPID_TPM_ACCESS_0        0x80c   //|        4 | TPM_ACCESS_x register.                          |
#define SPID_TPM_ACCESS_1        0x810   //|        4 | TPM_ACCESS_x register.                          |
#define SPID_TPM_STS             0x814   //|        4 | TPM_STS_x register.                             |
#define SPID_TPM_INTF_CAPABILITY 0x818   //|        4 | TPM_INTF_CAPABILITY                             |
#define SPID_TPM_INT_ENABLE      0x81c   //|        4 | TPM_INT_ENABLE                                  |
#define SPID_TPM_INT_VECTOR      0x820   //|        4 | TPM_INT_VECTOR                                  |
#define SPID_TPM_INT_STATUS      0x824   //|        4 | TPM_INT_STATUS                                  |
#define SPID_TPM_DID_VID         0x828   //|        4 | TPM_DID/ TPM_VID register                       |
#define SPID_TPM_RID             0x82c   //|        4 | TPM_RID                                         |
#define SPID_TPM_CMD_ADDR        0x830   //|        4 | TPM Command and Address buffer                  |
#define SPID_TPM_READ_FIFO       0x834   //|        4 | TPM Read command return data FIFO.              |
#define SPID_EGRESS_BUFFER       0x1000  //|     3392 | SPI internal egress buffer.                     |
#define SPID_INGRESS_BUFFER      0x1e00  //|      448 | SPI internal ingress buffer.                    |

#endif /* ZEPHYR_DRIVERS_MISC_SPID_REG_H_ */
