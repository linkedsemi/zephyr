.. _jtag_master:

jtag_Master
###########

Overview
********
###
 jtag shell命令举例
###

```
jtag                /*查看JTAG device*/

jtag jtag@40092400  /*查看支持的shell命令*/

jtag jtag@40092400 tap_set RESET

jtag jtag@40092400 ir_scan 0x5 0x1

jtag jtag@40092400 dr_scan 0x20 0x0

jtag jtag@40092400 tck_run 9

jtag jtag@40092400 tap_get

jtag jtag@40092400 sw_xfer high TCK

jtag jtag@40092400 sw_xfer low TCK

jtag jtag@40092400 sw_xfer high TMS

jtag jtag@40092400 sw_xfer low TMS

jtag jtag@40092400 sw_xfer high TDI

jtag jtag@40092400 sw_xfer low TDI

jtag jtag@40092400 tdo_get
```

