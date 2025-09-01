#include "udc_common.h"
#include <zephyr/cache.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#if defined(CONFIG_RESET)
#include <zephyr/drivers/reset.h>
#endif

#if defined(CONFIG_CLOCK_CONTROL)
#include <soc_clock.h>
#include <zephyr/drivers/clock_control.h>
#endif

#include "field_manipulate.h"

LOG_MODULE_REGISTER(udc_dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT        snps_dwc3
#define DWC3_MSGQ_LENGTH     16
#define EVT_BUF_LENGTH_WORDS 16

#define DWC3_XHCI_REGS_START    0x0
#define DWC3_XHCI_REGS_END      0x7fff
#define DWC3_GLOBALS_REGS_START 0xc100
#define DWC3_GLOBALS_REGS_END   0xc6ff
#define DWC3_DEVICE_REGS_START  0xc700
#define DWC3_DEVICE_REGS_END    0xcbff
#define DWC3_OTG_REGS_START     0xcc00
#define DWC3_OTG_REGS_END       0xccff

/* Global SoC Bus Configuration INCRx Register 0 */
#define DWC3_GSBUSCFG0_INCR256BRSTENA (1 << 7) /* INCR256 burst */
#define DWC3_GSBUSCFG0_INCR128BRSTENA (1 << 6) /* INCR128 burst */
#define DWC3_GSBUSCFG0_INCR64BRSTENA  (1 << 5) /* INCR64 burst */
#define DWC3_GSBUSCFG0_INCR32BRSTENA  (1 << 4) /* INCR32 burst */
#define DWC3_GSBUSCFG0_INCR16BRSTENA  (1 << 3) /* INCR16 burst */
#define DWC3_GSBUSCFG0_INCR8BRSTENA   (1 << 2) /* INCR8 burst */
#define DWC3_GSBUSCFG0_INCR4BRSTENA   (1 << 1) /* INCR4 burst */
#define DWC3_GSBUSCFG0_INCRBRSTENA    (1 << 0) /* undefined length enable */
#define DWC3_GSBUSCFG0_INCRBRST_MASK  0xff

/* Global Debug LSP MUX Select */
#define DWC3_GDBGLSPMUX_ENDBC         BIT(15) /* Host only */
#define DWC3_GDBGLSPMUX_HOSTSELECT(n) ((n) & 0x3fff)
#define DWC3_GDBGLSPMUX_DEVSELECT(n)  (((n) & 0xf) << 4)
#define DWC3_GDBGLSPMUX_EPSELECT(n)   ((n) & 0xf)

/* Global Debug Queue/FIFO Space Available Register */
#define DWC3_GDBGFIFOSPACE_NUM(n)             ((n) & 0x1f)
#define DWC3_GDBGFIFOSPACE_TYPE(n)            (((n) << 5) & 0x1e0)
#define DWC3_GDBGFIFOSPACE_SPACE_AVAILABLE(n) (((n) >> 16) & 0xffff)

#define DWC3_TXFIFO     0
#define DWC3_RXFIFO     1
#define DWC3_TXREQQ     2
#define DWC3_RXREQQ     3
#define DWC3_RXINFOQ    4
#define DWC3_PSTATQ     5
#define DWC3_DESCFETCHQ 6
#define DWC3_EVENTQ     7
#define DWC3_AUXEVENTQ  8

/* Global RX Threshold Configuration Register */
#define DWC3_GRXTHRCFG_MAXRXBURSTSIZE(n) (((n) & 0x1f) << 19)
#define DWC3_GRXTHRCFG_RXPKTCNT(n)       (((n) & 0xf) << 24)
#define DWC3_GRXTHRCFG_PKTCNTSEL         BIT(29)

/* Global TX Threshold Configuration Register */
#define DWC3_GTXTHRCFG_MAXTXBURSTSIZE(n) (((n) & 0xff) << 16)
#define DWC3_GTXTHRCFG_TXPKTCNT(n)       (((n) & 0xf) << 24)
#define DWC3_GTXTHRCFG_PKTCNTSEL         BIT(29)

/* Global RX Threshold Configuration Register for DWC_usb31 only */
#define DWC31_GRXTHRCFG_MAXRXBURSTSIZE(n) (((n) & 0x1f) << 16)
#define DWC31_GRXTHRCFG_RXPKTCNT(n)       (((n) & 0x1f) << 21)
#define DWC31_GRXTHRCFG_PKTCNTSEL         BIT(26)
#define DWC31_RXTHRNUMPKTSEL_HS_PRD       BIT(15)
#define DWC31_RXTHRNUMPKT_HS_PRD(n)       (((n) & 0x3) << 13)
#define DWC31_RXTHRNUMPKTSEL_PRD          BIT(10)
#define DWC31_RXTHRNUMPKT_PRD(n)          (((n) & 0x1f) << 5)
#define DWC31_MAXRXBURSTSIZE_PRD(n)       ((n) & 0x1f)

/* Global TX Threshold Configuration Register for DWC_usb31 only */
#define DWC31_GTXTHRCFG_MAXTXBURSTSIZE(n) (((n) & 0x1f) << 16)
#define DWC31_GTXTHRCFG_TXPKTCNT(n)       (((n) & 0x1f) << 21)
#define DWC31_GTXTHRCFG_PKTCNTSEL         BIT(26)
#define DWC31_TXTHRNUMPKTSEL_HS_PRD       BIT(15)
#define DWC31_TXTHRNUMPKT_HS_PRD(n)       (((n) & 0x3) << 13)
#define DWC31_TXTHRNUMPKTSEL_PRD          BIT(10)
#define DWC31_TXTHRNUMPKT_PRD(n)          (((n) & 0x1f) << 5)
#define DWC31_MAXTXBURSTSIZE_PRD(n)       ((n) & 0x1f)

/* Global Configuration Register */
#define DWC3_GCTL_PWRDNSCALE(n)   ((n) << 19)
#define DWC3_GCTL_PWRDNSCALE_MASK GENMASK(31, 19)
#define DWC3_GCTL_U2RSTECN        BIT(16)
#define DWC3_GCTL_RAMCLKSEL(x)    (((x) & DWC3_GCTL_CLK_MASK) << 6)
#define DWC3_GCTL_CLK_BUS         (0)
#define DWC3_GCTL_CLK_PIPE        (1)
#define DWC3_GCTL_CLK_PIPEHALF    (2)
#define DWC3_GCTL_CLK_MASK        (3)

#define DWC3_GCTL_PRTCAP(n)     (((n) & (3 << 12)) >> 12)
#define DWC3_GCTL_PRTCAPDIR(n)  ((n) << 12)
#define DWC3_GCTL_PRTCAP_HOST   1
#define DWC3_GCTL_PRTCAP_DEVICE 2
#define DWC3_GCTL_PRTCAP_OTG    3

#define DWC3_GCTL_CORESOFTRESET    BIT(11)
#define DWC3_GCTL_SOFITPSYNC       BIT(10)
#define DWC3_GCTL_SCALEDOWN(n)     ((n) << 4)
#define DWC3_GCTL_SCALEDOWN_MASK   DWC3_GCTL_SCALEDOWN(3)
#define DWC3_GCTL_DISSCRAMBLE      BIT(3)
#define DWC3_GCTL_U2EXIT_LFPS      BIT(2)
#define DWC3_GCTL_GBLHIBERNATIONEN BIT(1)
#define DWC3_GCTL_DSBLCLKGTNG      BIT(0)

/* Global User Control 1 Register */
#define DWC3_GUCTL1_DEV_DECOUPLE_L1L2_EVT       BIT(31)
#define DWC3_GUCTL1_TX_IPGAP_LINECHECK_DIS      BIT(28)
#define DWC3_GUCTL1_DEV_FORCE_20_CLK_FOR_30_CLK BIT(26)
#define DWC3_GUCTL1_DEV_L1_EXIT_BY_HW           BIT(24)
#define DWC3_GUCTL1_PARKMODE_DISABLE_SS         BIT(17)
#define DWC3_GUCTL1_PARKMODE_DISABLE_HS         BIT(16)
#define DWC3_GUCTL1_RESUME_OPMODE_HS_HOST       BIT(10)

/* Global Status Register */
#define DWC3_GSTS_OTG_IP           BIT(10)
#define DWC3_GSTS_BC_IP            BIT(9)
#define DWC3_GSTS_ADP_IP           BIT(8)
#define DWC3_GSTS_HOST_IP          BIT(7)
#define DWC3_GSTS_DEVICE_IP        BIT(6)
#define DWC3_GSTS_CSR_TIMEOUT      BIT(5)
#define DWC3_GSTS_BUS_ERR_ADDR_VLD BIT(4)
#define DWC3_GSTS_CURMOD(n)        ((n) & 0x3)
#define DWC3_GSTS_CURMOD_DEVICE    0
#define DWC3_GSTS_CURMOD_HOST      1

/* Global USB2 PHY Configuration Register */
#define DWC3_GUSB2PHYCFG_PHYSOFTRST        BIT(31)
#define DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS BIT(30)
#define DWC3_GUSB2PHYCFG_ULPIEXTVBUSDRV    BIT(17)
#define DWC3_GUSB2PHYCFG_SUSPHY            BIT(6)
#define DWC3_GUSB2PHYCFG_ULPI_UTMI         BIT(4)
#define DWC3_GUSB2PHYCFG_ENBLSLPM          BIT(8)
#define DWC3_GUSB2PHYCFG_PHYIF(n)          (n << 3)
#define DWC3_GUSB2PHYCFG_PHYIF_MASK        DWC3_GUSB2PHYCFG_PHYIF(1)
#define DWC3_GUSB2PHYCFG_USBTRDTIM(n)      (n << 10)
#define DWC3_GUSB2PHYCFG_USBTRDTIM_MASK    DWC3_GUSB2PHYCFG_USBTRDTIM(0xf)
#define USBTRDTIM_UTMI_8_BIT               9
#define USBTRDTIM_UTMI_16_BIT              5
#define UTMI_PHYIF_16_BIT                  1
#define UTMI_PHYIF_8_BIT                   0

/* Global USB2 PHY Vendor Control Register */
#define DWC3_GUSB2PHYACC_NEWREGREQ      BIT(25)
#define DWC3_GUSB2PHYACC_DONE           BIT(24)
#define DWC3_GUSB2PHYACC_BUSY           BIT(23)
#define DWC3_GUSB2PHYACC_WRITE          BIT(22)
#define DWC3_GUSB2PHYACC_ADDR(n)        (n << 16)
#define DWC3_GUSB2PHYACC_EXTEND_ADDR(n) (n << 8)
#define DWC3_GUSB2PHYACC_DATA(n)        (n & 0xff)

/* Global USB3 PIPE Control Register */
#define DWC3_GUSB3PIPECTL_PHYSOFTRST    BIT(31)
#define DWC3_GUSB3PIPECTL_U2SSINP3OK    BIT(29)
#define DWC3_GUSB3PIPECTL_DISRXDETINP3  BIT(28)
#define DWC3_GUSB3PIPECTL_UX_EXIT_PX    BIT(27)
#define DWC3_GUSB3PIPECTL_REQP1P2P3     BIT(24)
#define DWC3_GUSB3PIPECTL_DEP1P2P3(n)   ((n) << 19)
#define DWC3_GUSB3PIPECTL_DEP1P2P3_MASK DWC3_GUSB3PIPECTL_DEP1P2P3(7)
#define DWC3_GUSB3PIPECTL_DEP1P2P3_EN   DWC3_GUSB3PIPECTL_DEP1P2P3(1)
#define DWC3_GUSB3PIPECTL_DEPOCHANGE    BIT(18)
#define DWC3_GUSB3PIPECTL_SUSPHY        BIT(17)
#define DWC3_GUSB3PIPECTL_LFPSFILT      BIT(9)
#define DWC3_GUSB3PIPECTL_RX_DETOPOLL   BIT(8)
#define DWC3_GUSB3PIPECTL_TX_DEEPH_MASK DWC3_GUSB3PIPECTL_TX_DEEPH(3)
#define DWC3_GUSB3PIPECTL_TX_DEEPH(n)   ((n) << 1)

/* Global TX Fifo Size Register */
#define DWC31_GTXFIFOSIZ_TXFRAMNUM   BIT(15)        /* DWC_usb31 only */
#define DWC31_GTXFIFOSIZ_TXFDEP(n)   ((n) & 0x7fff) /* DWC_usb31 only */
#define DWC3_GTXFIFOSIZ_TXFDEP(n)    ((n) & 0xffff)
#define DWC3_GTXFIFOSIZ_TXFSTADDR(n) ((n) & 0xffff0000)

/* Global RX Fifo Size Register */
#define DWC31_GRXFIFOSIZ_RXFDEP(n) ((n) & 0x7fff) /* DWC_usb31 only */
#define DWC3_GRXFIFOSIZ_RXFDEP(n)  ((n) & 0xffff)

/* Global Event Size Registers */
#define DWC3_GEVNTSIZ_INTMASK BIT(31)
#define DWC3_GEVNTSIZ_SIZE(n) ((n) & 0xffff)

/* Global HWPARAMS0 Register */
#define DWC3_GHWPARAMS0_MODE(n)      ((n) & 0x3)
#define DWC3_GHWPARAMS0_MODE_GADGET  0
#define DWC3_GHWPARAMS0_MODE_HOST    1
#define DWC3_GHWPARAMS0_MODE_DRD     2
#define DWC3_GHWPARAMS0_MBUS_TYPE(n) (((n) >> 3) & 0x7)
#define DWC3_GHWPARAMS0_SBUS_TYPE(n) (((n) >> 6) & 0x3)
#define DWC3_GHWPARAMS0_MDWIDTH(n)   (((n) >> 8) & 0xff)
#define DWC3_GHWPARAMS0_SDWIDTH(n)   (((n) >> 16) & 0xff)
#define DWC3_GHWPARAMS0_AWIDTH(n)    (((n) >> 24) & 0xff)

/* Global HWPARAMS1 Register */
#define DWC3_GHWPARAMS1_EN_PWROPT(n)  (((n) & (3 << 24)) >> 24)
#define DWC3_GHWPARAMS1_EN_PWROPT_NO  0
#define DWC3_GHWPARAMS1_EN_PWROPT_CLK 1
#define DWC3_GHWPARAMS1_EN_PWROPT_HIB 2
#define DWC3_GHWPARAMS1_PWROPT(n)     ((n) << 24)
#define DWC3_GHWPARAMS1_PWROPT_MASK   DWC3_GHWPARAMS1_PWROPT(3)
#define DWC3_GHWPARAMS1_ENDBC         BIT(31)

/* Global HWPARAMS3 Register */
#define DWC3_GHWPARAMS3_SSPHY_IFC(n)        ((n) & 3)
#define DWC3_GHWPARAMS3_SSPHY_IFC_DIS       0
#define DWC3_GHWPARAMS3_SSPHY_IFC_GEN1      1
#define DWC3_GHWPARAMS3_SSPHY_IFC_GEN2      2 /* DWC_usb31 only */
#define DWC3_GHWPARAMS3_HSPHY_IFC(n)        (((n) & (3 << 2)) >> 2)
#define DWC3_GHWPARAMS3_HSPHY_IFC_DIS       0
#define DWC3_GHWPARAMS3_HSPHY_IFC_UTMI      1
#define DWC3_GHWPARAMS3_HSPHY_IFC_ULPI      2
#define DWC3_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI 3
#define DWC3_GHWPARAMS3_FSPHY_IFC(n)        (((n) & (3 << 4)) >> 4)
#define DWC3_GHWPARAMS3_FSPHY_IFC_DIS       0
#define DWC3_GHWPARAMS3_FSPHY_IFC_ENA       1

/* Global HWPARAMS4 Register */
#define DWC3_GHWPARAMS4_HIBER_SCRATCHBUFS(n) (((n) & (0x0f << 13)) >> 13)
#define DWC3_MAX_HIBER_SCRATCHBUFS           15

/* Global HWPARAMS6 Register */
#define DWC3_GHWPARAMS6_BCSUPPORT   BIT(14)
#define DWC3_GHWPARAMS6_OTG3SUPPORT BIT(13)
#define DWC3_GHWPARAMS6_ADPSUPPORT  BIT(12)
#define DWC3_GHWPARAMS6_HNPSUPPORT  BIT(11)
#define DWC3_GHWPARAMS6_SRPSUPPORT  BIT(10)
#define DWC3_GHWPARAMS6_EN_FPGA     BIT(7)

/* DWC_usb32 only */
#define DWC3_GHWPARAMS6_MDWIDTH(n) ((n) & (0x3 << 8))

/* Global HWPARAMS7 Register */
#define DWC3_GHWPARAMS7_RAM1_DEPTH(n) ((n) & 0xffff)
#define DWC3_GHWPARAMS7_RAM2_DEPTH(n) (((n) >> 16) & 0xffff)

/* Global HWPARAMS9 Register */
#define DWC3_GHWPARAMS9_DEV_TXF_FLUSH_BYPASS BIT(0)
#define DWC3_GHWPARAMS9_DEV_MST              BIT(1)

/* Global Frame Length Adjustment Register */
#define DWC3_GFLADJ_30MHZ_SDBND_SEL   BIT(7)
#define DWC3_GFLADJ_30MHZ_MASK        0x3f
#define DWC3_GFLADJ_REFCLK_FLADJ_MASK GENMASK(21, 8)
#define DWC3_GFLADJ_REFCLK_LPM_SEL    BIT(23)
#define DWC3_GFLADJ_240MHZDECR        GENMASK(30, 24)
#define DWC3_GFLADJ_240MHZDECR_PLS1   BIT(31)

/* Global User Control Register*/
#define DWC3_GUCTL_REFCLKPER_MASK 0xffc00000
#define DWC3_GUCTL_REFCLKPER_SEL  22

/* Global User Control Register 2 */
#define DWC3_GUCTL2_RST_ACTBITLATER BIT(14)

/* Global User Control Register 3 */
#define DWC3_GUCTL3_SPLITDISABLE BIT(14)

/* Device Configuration Register */
#define DWC3_DCFG_NUMLANES(n) (((n) & 0x3) << 30) /* DWC_usb32 only */

#define DWC3_DCFG_DEVADDR(addr) ((addr) << 3)
#define DWC3_DCFG_DEVADDR_MASK  DWC3_DCFG_DEVADDR(0x7f)

#define DWC3_DCFG_SPEED_MASK      (7 << 0)
#define DWC3_DCFG_SUPERSPEED_PLUS (5 << 0) /* DWC_usb31 only */
#define DWC3_DCFG_SUPERSPEED      (4 << 0)
#define DWC3_DCFG_HIGHSPEED       (0 << 0)
#define DWC3_DCFG_FULLSPEED       BIT(0)

#define DWC3_DCFG_NUMP_SHIFT 17
#define DWC3_DCFG_NUMP(n)    (((n) >> DWC3_DCFG_NUMP_SHIFT) & 0x1f)
#define DWC3_DCFG_NUMP_MASK  (0x1f << DWC3_DCFG_NUMP_SHIFT)
#define DWC3_DCFG_LPM_CAP    BIT(22)
#define DWC3_DCFG_IGNSTRMPP  BIT(23)

/* Device Control Register */
#define DWC3_DCTL_RUN_STOP BIT(31)
#define DWC3_DCTL_CSFTRST  BIT(30)
#define DWC3_DCTL_LSFTRST  BIT(29)

#define DWC3_DCTL_HIRD_THRES_MASK (0x1f << 24)
#define DWC3_DCTL_HIRD_THRES(n)   ((n) << 24)

#define DWC3_DCTL_APPL1RES BIT(23)

/* These apply for core versions 1.87a and earlier */
#define DWC3_DCTL_TRGTULST_MASK     (0x0f << 17)
#define DWC3_DCTL_TRGTULST(n)       ((n) << 17)
#define DWC3_DCTL_TRGTULST_U2       (DWC3_DCTL_TRGTULST(2))
#define DWC3_DCTL_TRGTULST_U3       (DWC3_DCTL_TRGTULST(3))
#define DWC3_DCTL_TRGTULST_SS_DIS   (DWC3_DCTL_TRGTULST(4))
#define DWC3_DCTL_TRGTULST_RX_DET   (DWC3_DCTL_TRGTULST(5))
#define DWC3_DCTL_TRGTULST_SS_INACT (DWC3_DCTL_TRGTULST(6))

/* These apply for core versions 1.94a and later */
#define DWC3_DCTL_NYET_THRES(n) (((n) & 0xf) << 20)

#define DWC3_DCTL_KEEP_CONNECT BIT(19)
#define DWC3_DCTL_L1_HIBER_EN  BIT(18)
#define DWC3_DCTL_CRS          BIT(17)
#define DWC3_DCTL_CSS          BIT(16)

#define DWC3_DCTL_INITU2ENA    BIT(12)
#define DWC3_DCTL_ACCEPTU2ENA  BIT(11)
#define DWC3_DCTL_INITU1ENA    BIT(10)
#define DWC3_DCTL_ACCEPTU1ENA  BIT(9)
#define DWC3_DCTL_TSTCTRL_MASK (0xf << 1)

#define DWC3_DCTL_ULSTCHNGREQ_MASK (0x0f << 5)
#define DWC3_DCTL_ULSTCHNGREQ(n)   (((n) << 5) & DWC3_DCTL_ULSTCHNGREQ_MASK)

#define DWC3_DCTL_ULSTCHNG_NO_ACTION   (DWC3_DCTL_ULSTCHNGREQ(0))
#define DWC3_DCTL_ULSTCHNG_SS_DISABLED (DWC3_DCTL_ULSTCHNGREQ(4))
#define DWC3_DCTL_ULSTCHNG_RX_DETECT   (DWC3_DCTL_ULSTCHNGREQ(5))
#define DWC3_DCTL_ULSTCHNG_SS_INACTIVE (DWC3_DCTL_ULSTCHNGREQ(6))
#define DWC3_DCTL_ULSTCHNG_RECOVERY    (DWC3_DCTL_ULSTCHNGREQ(8))
#define DWC3_DCTL_ULSTCHNG_COMPLIANCE  (DWC3_DCTL_ULSTCHNGREQ(10))
#define DWC3_DCTL_ULSTCHNG_LOOPBACK    (DWC3_DCTL_ULSTCHNGREQ(11))

/* Device Event Enable Register */
#define DWC3_DEVTEN_ECCERREN            BIT(16)
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN   BIT(12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN      BIT(11)
#define DWC3_DEVTEN_CMDCMPLTEN          BIT(10)
#define DWC3_DEVTEN_ERRTICERREN         BIT(9)
#define DWC3_DEVTEN_SOFEN               BIT(7)
#define DWC3_DEVTEN_U3L2L1SUSPEN        BIT(6)
#define DWC3_DEVTEN_HIBERNATIONREQEVTEN BIT(5)
#define DWC3_DEVTEN_WKUPEVTEN           BIT(4)
#define DWC3_DEVTEN_ULSTCNGEN           BIT(3)
#define DWC3_DEVTEN_CONNECTDONEEN       BIT(2)
#define DWC3_DEVTEN_USBRSTEN            BIT(1)
#define DWC3_DEVTEN_DISCONNEVTEN        BIT(0)

#define DWC3_DSTS_CONNLANES(n) (((n) >> 30) & 0x3) /* DWC_usb32 only */

/* Device Status Register */
#define DWC3_DSTS_DCNRD BIT(29)

/* This applies for core versions 1.87a and earlier */
#define DWC3_DSTS_PWRUPREQ BIT(24)

/* These apply for core versions 1.94a and later */
#define DWC3_DSTS_RSS BIT(25)
#define DWC3_DSTS_SSS BIT(24)

#define DWC3_DSTS_COREIDLE   BIT(23)
#define DWC3_DSTS_DEVCTRLHLT BIT(22)

#define DWC3_DSTS_USBLNKST_MASK (0x0f << 18)
#define DWC3_DSTS_USBLNKST(n)   (((n) & DWC3_DSTS_USBLNKST_MASK) >> 18)

#define DWC3_DSTS_RXFIFOEMPTY BIT(17)

#define DWC3_DSTS_SOFFN_MASK (0x3fff << 3)
#define DWC3_DSTS_SOFFN(n)   (((n) & DWC3_DSTS_SOFFN_MASK) >> 3)

#define DWC3_DSTS_CONNECTSPD (7 << 0)

#define DWC3_DSTS_SUPERSPEED_PLUS (5 << 0) /* DWC_usb31 only */
#define DWC3_DSTS_SUPERSPEED      (4 << 0)
#define DWC3_DSTS_HIGHSPEED       (0 << 0)
#define DWC3_DSTS_FULLSPEED       BIT(0)

/* Device Generic Command Register */
#define DWC3_DGCMD_SET_LMP          0x01
#define DWC3_DGCMD_SET_PERIODIC_PAR 0x02
#define DWC3_DGCMD_XMIT_FUNCTION    0x03

/* These apply for core versions 1.94a and later */
#define DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO 0x04
#define DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI 0x05

#define DWC3_DGCMD_SELECTED_FIFO_FLUSH  0x09
#define DWC3_DGCMD_ALL_FIFO_FLUSH       0x0a
#define DWC3_DGCMD_SET_ENDPOINT_NRDY    0x0c
#define DWC3_DGCMD_SET_ENDPOINT_PRIME   0x0d
#define DWC3_DGCMD_RUN_SOC_BUS_LOOPBACK 0x10
#define DWC3_DGCMD_DEV_NOTIFICATION     0x07

#define DWC3_DGCMD_STATUS(n) (((n) >> 12) & 0x0F)
#define DWC3_DGCMD_CMDACT    BIT(10)
#define DWC3_DGCMD_CMDIOC    BIT(8)

/* Device Generic Command Parameter Register */
#define DWC3_DGCMDPAR_FORCE_LINKPM_ACCEPT BIT(0)
#define DWC3_DGCMDPAR_FIFO_NUM(n)         ((n) << 0)
#define DWC3_DGCMDPAR_RX_FIFO             (0 << 5)
#define DWC3_DGCMDPAR_TX_FIFO             BIT(5)
#define DWC3_DGCMDPAR_LOOPBACK_DIS        (0 << 0)
#define DWC3_DGCMDPAR_LOOPBACK_ENA        BIT(0)
#define DWC3_DGCMDPAR_DN_FUNC_WAKE        BIT(0)
#define DWC3_DGCMDPAR_INTF_SEL(n)         ((n) << 4)

/* Device Endpoint Command Register */
#define DWC3_DEPCMD_PARAM_SHIFT    16
#define DWC3_DEPCMD_PARAM(x)       ((x) << DWC3_DEPCMD_PARAM_SHIFT)
#define DWC3_DEPCMD_GET_RSC_IDX(x) (((x) >> DWC3_DEPCMD_PARAM_SHIFT) & 0x7f)
#define DWC3_DEPCMD_STATUS(x)      (((x) >> 12) & 0x0F)
#define DWC3_DEPCMD_HIPRI_FORCERM  BIT(11)
#define DWC3_DEPCMD_CLEARPENDIN    BIT(11)
#define DWC3_DEPCMD_CMDACT         BIT(10)
#define DWC3_DEPCMD_CMDIOC         BIT(8)

#define DWC3_DEPCMD_DEPSTARTCFG    (0x09 << 0)
#define DWC3_DEPCMD_ENDTRANSFER    (0x08 << 0)
#define DWC3_DEPCMD_UPDATETRANSFER (0x07 << 0)
#define DWC3_DEPCMD_STARTTRANSFER  (0x06 << 0)
#define DWC3_DEPCMD_CLEARSTALL     (0x05 << 0)
#define DWC3_DEPCMD_SETSTALL       (0x04 << 0)
/* This applies for core versions 1.90a and earlier */
#define DWC3_DEPCMD_GETSEQNUMBER (0x03 << 0)
/* This applies for core versions 1.94a and later */
#define DWC3_DEPCMD_GETEPSTATE        (0x03 << 0)
#define DWC3_DEPCMD_SETTRANSFRESOURCE (0x02 << 0)
#define DWC3_DEPCMD_SETEPCONFIG       (0x01 << 0)

#define DEPEVT_PARAMETER_CMD(n) (((n) & (0xf << 8)) >> 8)
#define DWC3_DEPCMD_CMD(x)      ((x) & 0xf)

/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define DWC3_DALEPENA_EP(n) BIT(n)

/* DWC_usb32 DCFG1 config */
#define DWC3_DCFG1_DIS_MST_ENH BIT(1)

#define DWC3_DEPCMD_TYPE_CONTROL 0
#define DWC3_DEPCMD_TYPE_ISOC    1
#define DWC3_DEPCMD_TYPE_BULK    2
#define DWC3_DEPCMD_TYPE_INTR    3

#define DWC3_DEV_IMOD_COUNT_SHIFT    16
#define DWC3_DEV_IMOD_COUNT_MASK     (0xffff << 16)
#define DWC3_DEV_IMOD_INTERVAL_SHIFT 0
#define DWC3_DEV_IMOD_INTERVAL_MASK  (0xffff << 0)

/* OTG Configuration Register */
#define DWC3_OCFG_DISPWRCUTTOFF BIT(5)
#define DWC3_OCFG_HIBDISMASK    BIT(4)
#define DWC3_OCFG_SFTRSTMASK    BIT(3)
#define DWC3_OCFG_OTGVERSION    BIT(2)
#define DWC3_OCFG_HNPCAP        BIT(1)
#define DWC3_OCFG_SRPCAP        BIT(0)

/* OTG CTL Register */
#define DWC3_OCTL_OTG3GOERR      BIT(7)
#define DWC3_OCTL_PERIMODE       BIT(6)
#define DWC3_OCTL_PRTPWRCTL      BIT(5)
#define DWC3_OCTL_HNPREQ         BIT(4)
#define DWC3_OCTL_SESREQ         BIT(3)
#define DWC3_OCTL_TERMSELIDPULSE BIT(2)
#define DWC3_OCTL_DEVSETHNPEN    BIT(1)
#define DWC3_OCTL_HSTSETHNPEN    BIT(0)

/* OTG Event Register */
#define DWC3_OEVT_DEVICEMODE     BIT(31)
#define DWC3_OEVT_XHCIRUNSTPSET  BIT(27)
#define DWC3_OEVT_DEVRUNSTPSET   BIT(26)
#define DWC3_OEVT_HIBENTRY       BIT(25)
#define DWC3_OEVT_CONIDSTSCHNG   BIT(24)
#define DWC3_OEVT_HRRCONFNOTIF   BIT(23)
#define DWC3_OEVT_HRRINITNOTIF   BIT(22)
#define DWC3_OEVT_ADEVIDLE       BIT(21)
#define DWC3_OEVT_ADEVBHOSTEND   BIT(20)
#define DWC3_OEVT_ADEVHOST       BIT(19)
#define DWC3_OEVT_ADEVHNPCHNG    BIT(18)
#define DWC3_OEVT_ADEVSRPDET     BIT(17)
#define DWC3_OEVT_ADEVSESSENDDET BIT(16)
#define DWC3_OEVT_BDEVBHOSTEND   BIT(11)
#define DWC3_OEVT_BDEVHNPCHNG    BIT(10)
#define DWC3_OEVT_BDEVSESSVLDDET BIT(9)
#define DWC3_OEVT_BDEVVBUSCHNG   BIT(8)
#define DWC3_OEVT_BSESSVLD       BIT(3)
#define DWC3_OEVT_HSTNEGSTS      BIT(2)
#define DWC3_OEVT_SESREQSTS      BIT(1)
#define DWC3_OEVT_ERROR          BIT(0)

/* OTG Event Enable Register */
#define DWC3_OEVTEN_XHCIRUNSTPSETEN  BIT(27)
#define DWC3_OEVTEN_DEVRUNSTPSETEN   BIT(26)
#define DWC3_OEVTEN_HIBENTRYEN       BIT(25)
#define DWC3_OEVTEN_CONIDSTSCHNGEN   BIT(24)
#define DWC3_OEVTEN_HRRCONFNOTIFEN   BIT(23)
#define DWC3_OEVTEN_HRRINITNOTIFEN   BIT(22)
#define DWC3_OEVTEN_ADEVIDLEEN       BIT(21)
#define DWC3_OEVTEN_ADEVBHOSTENDEN   BIT(20)
#define DWC3_OEVTEN_ADEVHOSTEN       BIT(19)
#define DWC3_OEVTEN_ADEVHNPCHNGEN    BIT(18)
#define DWC3_OEVTEN_ADEVSRPDETEN     BIT(17)
#define DWC3_OEVTEN_ADEVSESSENDDETEN BIT(16)
#define DWC3_OEVTEN_BDEVBHOSTENDEN   BIT(11)
#define DWC3_OEVTEN_BDEVHNPCHNGEN    BIT(10)
#define DWC3_OEVTEN_BDEVSESSVLDDETEN BIT(9)
#define DWC3_OEVTEN_BDEVVBUSCHNGEN   BIT(8)

/* OTG Status Register */
#define DWC3_OSTS_DEVRUNSTP       BIT(13)
#define DWC3_OSTS_XHCIRUNSTP      BIT(12)
#define DWC3_OSTS_PERIPHERALSTATE BIT(4)
#define DWC3_OSTS_XHCIPRTPOWER    BIT(3)
#define DWC3_OSTS_BSESVLD         BIT(2)
#define DWC3_OSTS_VBUSVLD         BIT(1)
#define DWC3_OSTS_CONIDSTS        BIT(0)

/*
 * Test Mode Selectors
 * See USB 2.0 spec Table 9-7
 */
#define	TEST_J          1
#define	TEST_K          2
#define	TEST_SE0_NAK    3
#define	TEST_PACKET     4
#define	TEST_FORCE_EN   5

/* Force Gen1 speed on Gen2 link */
#define DWC3_LLUCTL_FORCE_GEN1 BIT(10)

#define EP_ADDR_2_PHY_EP_IDX(addr) ((((addr & 0x0F) << 1)) + !!(addr & 0x80))
#define PHY_EP_IDX_2_EP_ADDR(idx)  ((idx & 0x1) << 7 | idx >> 1)

/* TRB Length */
#define DWC3_TRB_SIZE_MASK      (0x00ffffff)
#define DWC3_TRB_SIZE_LENGTH(n) ((n) & DWC3_TRB_SIZE_MASK)

/* high speed mps */
#define BOUNCE_ADDR_SIZE (512)

/* Control-only Status */
#define DEPEVT_STATUS_CONTROL_DATA   1
#define DEPEVT_STATUS_CONTROL_STATUS 2

#define DWC3_GEVNTCOUNT_MASK    0xfffc

struct dwc3_xhci_reg {
    volatile uint32_t CAPLENGTH;
    volatile uint32_t HCSPARAMS1;
    volatile uint32_t HCSPARAMS2;
    volatile uint32_t HCSPARAMS3;
    volatile uint32_t HCCPARAMS1;
    volatile uint32_t DBOFF;
    volatile uint32_t RTSOFF;
    volatile uint32_t HCCPARAMS2;
};

#define DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX 16
#define DWC_USB3_HOST_NUM_U2_ROOT_PORTS     1
#define DWC_USB3_HOST_NUM_U3_ROOT_PORTS_MAX 16
#define DWC_USB3_HOST_NUM_U3_ROOT_PORTS     1
#define DWC_USB3_DEVICE_NUM_INT_MAX         32
#define DWC_USB3_DEVICE_NUM_INT             1

struct dwc3_gevnt_reg {
    volatile uint32_t ADRLO; // 0x300
    volatile uint32_t ADRHI; // 0x304
    volatile uint32_t SIZ;   // 0x308
    volatile uint32_t COUNT; // 0x30c
};

#define DWC_USB3_NUM_EPS_MAX 32
#define DWC_USB3_NUM_EPS     8
struct dwc3_dev_ep_reg {
    volatile uint32_t CMDPAR2;
    volatile uint32_t CMDPAR1;
    volatile uint32_t CMDPAR0;
    volatile uint32_t CMD;
};

struct dwc3_global_reg {
    volatile uint32_t GSBUSCFG0;      // 0x00
    volatile uint32_t GSBUSCFG1;      // 0x04
    volatile uint32_t GTXTHRCFG;      // 0x08
    volatile uint32_t GRXTHRCFG;      // 0x0c
    volatile uint32_t GCTL;           // 0x10
    volatile uint32_t GPMSTS;         // 0x14
    volatile uint32_t GSTS;           // 0x18
    volatile uint32_t GUCTL1;         // 0x1c
    volatile uint32_t GSNPSID;        // 0x20
    volatile uint32_t GGPIO;          // 0x24
    volatile uint32_t GUID;           // 0x28
    volatile uint32_t GUCTL;          // 0x2c
    volatile uint32_t GBUSERRADDRLO;  // 0x30
    volatile uint32_t GBUSERRADDRHI;  // 0x34
    volatile uint32_t GPRTBIMAPLO;    // 0x38
    volatile uint32_t GPRTBIMAPHI;    // 0x3c
    volatile uint32_t GHWPARAMS0;     // 0x40
    volatile uint32_t GHWPARAMS1;     // 0x44
    volatile uint32_t GHWPARAMS2;     // 0x48
    volatile uint32_t GHWPARAMS3;     // 0x4c
    volatile uint32_t GHWPARAMS4;     // 0x50
    volatile uint32_t GHWPARAMS5;     // 0x54
    volatile uint32_t GHWPARAMS6;     // 0x58
    volatile uint32_t GHWPARAMS7;     // 0x5c
    volatile uint32_t GDBGFIFOSPACE;  // 0x60
    volatile uint32_t GDBGLTSSM;      // 0x64
    volatile uint32_t GDBGLNMCC;      // 0x68
    volatile uint32_t GDBGBMU;        // 0x6c
    volatile uint32_t GDBGLSPMUX;     // 0x70
    volatile uint32_t GDBGLSP;        // 0x74
    volatile uint32_t GDBGEPINFO0;    // 0x78
    volatile uint32_t GDBGEPINFO1;    // 0x7c
    volatile uint32_t GPRTBIMAP_HSLO; // 0x80
    volatile uint32_t GPRTBIMAP_HSHI; // 0x84
    volatile uint32_t GPRTBIMAP_FSLO; // 0x88
    volatile uint32_t GPRTBIMAP_FSHI; // 0x8c
    volatile uint32_t RESERVED1;
    volatile uint32_t GERRINJCTL_1; // 0x94
    volatile uint32_t GERRINJCTL_2; // 0x98
    volatile uint32_t GUCTL2;       // 0x9c
    volatile uint32_t RESERVED[24];
    volatile uint32_t GUSB2PHYCFG[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; // 0x100
    volatile uint32_t RESERVED2[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX - DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
    volatile uint32_t GUSB2I2CCTL[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; // 0x140
    volatile uint32_t RESERVED3[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX - DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
    volatile uint32_t GUSB2PHYACC[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; // 0x180
    volatile uint32_t RESERVED4[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX - DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
    volatile uint32_t GUSB3PIPECTL[DWC_USB3_HOST_NUM_U3_ROOT_PORTS]; // 0x1c0
    volatile uint32_t RESERVED5[DWC_USB3_HOST_NUM_U3_ROOT_PORTS_MAX - DWC_USB3_HOST_NUM_U3_ROOT_PORTS];
    volatile uint32_t GTXFIFOSIZ[32];                     // 0x200
    volatile uint32_t GRXFIFOSIZ[32];                     // 0x280
    struct dwc3_gevnt_reg GEVNT[DWC_USB3_DEVICE_NUM_INT]; // 0x300
    volatile uint32_t RESERVED6[(DWC_USB3_DEVICE_NUM_INT_MAX - DWC_USB3_DEVICE_NUM_INT) * sizeof(struct dwc3_gevnt_reg) / sizeof(uint32_t)];
    volatile uint32_t GHWPARAMS8; // 0x500
    volatile uint32_t RESERVED7[2];
    volatile uint32_t GUCTL3;        // 0x50c
    volatile uint32_t GTXFIFOPRIDEV; // 0x510
    volatile uint32_t RESERVED8;
    volatile uint32_t GTXFIFOPRIHST; // 0x518
    volatile uint32_t GRXFIFOPRIHST; // 0x51c
    volatile uint32_t GFIFOPRIDBC;   // 0x520
    volatile uint32_t GDMAHLRATIO;   // 0x524
    volatile uint32_t RESERVED9[2];
    volatile uint32_t GFLADJ; // 0x530
    volatile uint32_t RESERVED10[3];
    volatile uint32_t GUSB2RHBCTL[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; // 0x540
};

struct dwc3_dev_reg {
    volatile uint32_t DCFG;     // 0x00
    volatile uint32_t DCTL;     // 0x04
    volatile uint32_t DEVTEN;   // 0x08
    volatile uint32_t DSTS;     // 0x0c
    volatile uint32_t DGCMDPAR; // 0x10
    volatile uint32_t DGCMD;    // 0x14
    volatile uint32_t RESERVED1[2];
    volatile uint32_t DALEPENA; // 0x20
    volatile uint32_t RESERVED2[55];
    struct dwc3_dev_ep_reg DEP[DWC_USB3_NUM_EPS]; // 0x100
    volatile uint32_t RESERVED3[(DWC_USB3_NUM_EPS_MAX - DWC_USB3_NUM_EPS) * sizeof(struct dwc3_dev_ep_reg) / sizeof(uint32_t)];
    volatile uint32_t DEV_IMOD[DWC_USB3_DEVICE_NUM_INT]; // 0x300
};

union dep_command_param {
    struct {
        uint32_t Reserved4 : 1,
            EPType : 2,
            MPS : 11,
            Reserved5 : 3,
            FIFONum : 5,
            BrstSiz : 4,
            Reserved6 : 4,
            ConfigAction : 2;
        uint32_t IntrNum : 5,
            Reserved1 : 3,
            XferCmplEn : 1,
            XferInProgEn : 1,
            XferNRdyEn : 1,
            Reserved2 : 2,
            StreamEvtEn : 1,
            TRBWrBack : 1,
            EBCMode : 1,
            bInterval_m1 : 8,
            StrmCap : 1,
            EpDir : 1,
            EpNum : 4,
            Reserved3 : 1,
            FIFOBased : 1;
        uint32_t ep_state;
    } cfg;
    struct {
        uint32_t NumXferRes : 16;
    } xfercfg;
    struct {
        uint32_t td_addr_high;
        uint32_t td_addr_low;
    } startxfer;
    uint32_t param[3];
};

union dep_command {
    struct {
        uint32_t cmdtyp : 4,
            reserved1 : 4,
            cmdioc : 1,
            reserved2 : 1,
            cmdact : 1,
            hipri_forcerm : 1,
            cmdstatus : 4,
            commandparam : 16;
    } cmd;
    uint32_t val;
};

enum TRB_Control {
    TRB_Normal = 1,
    TRB_Control_Setup,
    TRB_Control_Status_2,
    TRB_Control_Status_3,
    TRB_Control_Data,
    TRB_Isochronous_First,
    TRB_Isochronous,
    TRB_Link,
    TRB_Normal_ZLP
};

enum dwc3_link_state {
    /* In SuperSpeed */
    DWC3_LINK_STATE_U0 = 0x00, /* in HS, means ON */
    DWC3_LINK_STATE_U1 = 0x01,
    DWC3_LINK_STATE_U2 = 0x02, /* in HS, means SLEEP */
    DWC3_LINK_STATE_U3 = 0x03, /* in HS, means SUSPEND */
    DWC3_LINK_STATE_SS_DIS = 0x04,
    DWC3_LINK_STATE_RX_DET = 0x05, /* in HS, means Early Suspend */
    DWC3_LINK_STATE_SS_INACT = 0x06,
    DWC3_LINK_STATE_POLL = 0x07,
    DWC3_LINK_STATE_RECOV = 0x08,
    DWC3_LINK_STATE_HRESET = 0x09,
    DWC3_LINK_STATE_CMPLY = 0x0a,
    DWC3_LINK_STATE_LPBK = 0x0b,
    DWC3_LINK_STATE_RESET = 0x0e,
    DWC3_LINK_STATE_RESUME = 0x0f,
    DWC3_LINK_STATE_MASK = 0x0f,
};

struct dwc3_trb {
    uint32_t bptrl;
    uint32_t bptrh;
    uint32_t bufsiz : 24,
        pcm1 : 2,
        spr : 1,
        reserved1 : 1,
        trbsts : 4;
    uint32_t hwo : 1,
        lst : 1,
        chn : 1,
        csp : 1,
        trbctl : 6,
        isp_imi : 1,
        ioc : 1,
        reserved2 : 2,
        sid_sofn : 16,
        reserved3 : 2;
};

enum ep_config_action {
    Initialize_EP_State,
    Restore_EP_State,
    Modify_EP_State
};

enum ep_evt {
    DEPEVT_XferComplete = 1,
    DEPEVT_XferInProgress = 2,
    DEPEVT_XferNotReady = 3,
    DEPEVT_StreamEvt = 6,
    DEPEVT_EPCmdCmplt = 7,
};

enum dev_evt {
    DEVT_DisconnEvt = 0,
    DEVT_USBRst = 1,
    DEVT_ConnectDone = 2,
    DEVT_ULStChng = 3,
    DEVT_WkUpEvt = 4,
    DEVT_HibernationRequestEvt = 5,
    DEVT_USBSuspendEntryEvt = 6,
    DEVT_Sof = 7,
    DEVT_L1SUSP = 8,
    DEVT_ErrticErr = 9,
    DEVT_CmdCmplt = 10,
    DEVT_EvntOverflow = 11,
    DEVT_VndrDevTstRcved = 12,
    DEVT_L1Resume_RemoteWake_Evt = 14,
    DEVT_ECC_Err = 16,
};

union evt_buf_u {
    struct {
        uint32_t : 1,
            phy_ep_num : 5,
            ep_evt_type : 4, : 2,
            evt_status : 4,
            evt_param : 16;
    } depevt;
    struct {
        uint32_t : 8,
            dev_evt_type : 5, : 3,
            evt_info : 9, : 7;
    } devt;
    uint32_t dev_ep : 1;
};

struct dwc3_ep_trb {
    struct dwc3_trb trb[2];
};

struct udc_dwc3_config {
    size_t num_in_eps;
    size_t num_out_eps;
    struct udc_ep_config *ep_cfg_in;
    struct udc_ep_config *ep_cfg_out;
    struct dwc3_ep_trb *in_trb;
    struct dwc3_ep_trb *out_trb;
    uint32_t base;
    void (*make_thread)(const struct device *dev);
    void (*irq_enable_func)(const struct device *dev);
    void (*irq_disable_func)(const struct device *dev);
    union evt_buf_u *evt_buf;
    uint8_t *bounce_addr;
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct dwc3_msgq_type {
    void (*handler)(const struct device *, void *);
    void *param;
};

enum dwc3_ep0_state {
    EP0_UNCONNECTED = 0,
    EP0_SETUP_PHASE,
    EP0_DATA_PHASE,
    EP0_STATUS_PHASE,
};

struct udc_dwc3_data {
    struct k_msgq msgq;
    const struct device *dev;
    struct dwc3_msgq_type msgq_buf[DWC3_MSGQ_LENGTH];
    enum dwc3_ep0_state ep0_state;
    uint8_t ep0_expect_in;
    uint8_t three_stage_setup;
    uint16_t evt_buf_rd_idx;
    uint32_t evt_count;
    enum udc_bus_speed speed;
    struct k_thread thread;
    uint8_t ep_res_index[DWC_USB3_NUM_EPS];
    union evt_buf_u evt_buf[EVT_BUF_LENGTH_WORDS];
    struct k_sem ep0_sync;
};

static int dwc3_ctrl_feed_dout(const struct device *dev, const size_t length, enum TRB_Control trbctl);

#define DWC3_GEVNT_HANDLER_BUSY BIT(31)

static enum udc_bus_speed udc_dwc3_device_speed(const struct device *dev)
{
    LOG_DBG("%s udc device speed", dev->name);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    enum udc_bus_speed speed = UDC_BUS_UNKNOWN;
    switch (dwc3_dev->DCFG & DWC3_DCFG_SPEED_MASK) {
    case DWC3_DCFG_FULLSPEED:
        speed = UDC_BUS_SPEED_FS;
        break;
    case DWC3_DCFG_HIGHSPEED:
        speed = UDC_BUS_SPEED_HS;
        break;
    case DWC3_DCFG_SUPERSPEED:
        speed = UDC_BUS_SPEED_SS;
        break;
    }
    return speed;
}

static void dwc3_prepare_one_trb(struct dwc3_trb *trb, void *buf_ptr, uint32_t size, enum TRB_Control trbctl, bool chain)
{
    trb->bptrl = (uint32_t)buf_ptr;
    trb->bptrh = 0;
    trb->hwo = 1;
    trb->lst = chain ? 0 : 1;
    trb->chn = chain;
    trb->csp = 0;
    trb->trbctl = trbctl;
    trb->isp_imi = 0;
    trb->ioc = chain ? 0 : 1;
    trb->reserved2 = 0;
    trb->sid_sofn = 0;
    trb->reserved3 = 0;
    trb->bufsiz = size;
    trb->pcm1 = 0;
    trb->spr = 0;
    trb->reserved1 = 0;
    trb->trbsts = 0;
}

static void dwc3_dep_command(struct dwc3_dev_reg *reg, uint8_t phy_ep_idx, union dep_command *cmd, const union dep_command_param *param)
{
    reg->DEP[phy_ep_idx].CMDPAR0 = param->param[0];
    reg->DEP[phy_ep_idx].CMDPAR1 = param->param[1];
    reg->DEP[phy_ep_idx].CMDPAR2 = param->param[2];
    reg->DEP[phy_ep_idx].CMD = cmd->val;
    while (reg->DEP[phy_ep_idx].CMD & DWC3_DEPCMD_CMDACT);
    cmd->val = reg->DEP[phy_ep_idx].CMD;
}

static void dwc3_dep_start_config(struct dwc3_dev_reg *reg, uint8_t XferRscIdx)
{
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_DEPSTARTCFG,
            .cmdioc = 0,
            .commandparam = XferRscIdx,
        },
    };
    union dep_command_param param = {
        .param = {
            [0] = 0,
            [1] = 0,
            [2] = 0,
        },
    };
    dwc3_dep_command(reg, 0, &cmd, &param);
}

static void dwc3_dep_config(struct dwc3_dev_reg *reg, uint8_t phy_ep_idx, const union dep_command_param *param)
{
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_SETEPCONFIG,
        },
    };
    dwc3_dep_command(reg, phy_ep_idx, &cmd, param);
}

static void dwc3_dep_xfer_resource(struct dwc3_dev_reg *reg, uint8_t phy_ep_idx, const union dep_command_param *param)
{
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_SETTRANSFRESOURCE,
        },
    };
    dwc3_dep_command(reg, phy_ep_idx, &cmd, param);
}

static void dwc3_dep_start_transfer(const struct device *dev, uint8_t phy_ep_idx, void *td_addr, uint16_t stream_id)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *reg = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);

    union dep_command cmd = {
        .cmd = {
            .commandparam = stream_id,
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_STARTTRANSFER,
        },
    };
    union dep_command_param param = {
        .startxfer = {
            .td_addr_low = (uint32_t)td_addr,
            .td_addr_high = 0,
        },
    };
    dwc3_dep_command(reg, phy_ep_idx, &cmd, &param);
    dwc3_data->ep_res_index[phy_ep_idx] = DWC3_DEPCMD_GET_RSC_IDX(cmd.val);
}

#define DEPCFG_POR(EPDir)                        \
    (union dep_command_param)                    \
    {                                            \
        .cfg = {                                 \
            .FIFOBased = 0,                      \
            .EpDir = EPDir,                      \
            .EpNum = 0,                          \
            .StrmCap = 0,                        \
            .bInterval_m1 = 0,                   \
            .EBCMode = 0,                        \
            .TRBWrBack = 0,                      \
            .StreamEvtEn = 1,                    \
            .XferCmplEn = 1,                     \
            .XferInProgEn = 1,                   \
            .XferNRdyEn = 1,                     \
            .IntrNum = 0,                        \
            .ConfigAction = Initialize_EP_State, \
            .BrstSiz = 0,                        \
            .FIFONum = 0,                        \
            .MPS = 64,                           \
            .EPType = 0,                         \
        },                                       \
    }

static int dwc3_ctrl_feed_dout(const struct device *dev, const size_t length, enum TRB_Control trbctl)
{
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
    const struct udc_dwc3_config *config = dev->config;
    struct net_buf *buf;

    buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
    __ASSERT(buf, "dout udc_ctrl_alloc fail");
    udc_buf_put(ep_cfg, buf);
    dwc3_prepare_one_trb(config->out_trb[0].trb, buf->data, length, trbctl, false);
    dwc3_dep_start_transfer(dev, 0, config->out_trb[0].trb, 0);

    return 0;
}

static void dwc3_ep0_setup_phase(const struct device *dev, const union evt_buf_u *evt)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    struct net_buf *buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);

    udc_ep_buf_set_setup(buf);
    net_buf_add(buf, 8);

    sys_cache_data_invd_range(buf->data, buf->len);
    LOG_HEXDUMP_DBG(buf->data, buf->len, "setup token");
    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_data_out(dev)) {
        dwc3_data->ep0_state = EP0_DATA_PHASE;
        dwc3_data->three_stage_setup = 1;
        dwc3_data->ep0_expect_in = 0;
        if (dwc3_ctrl_feed_dout(dev, 64, TRB_Control_Data) == -ENOMEM) {
            udc_submit_ep_event(dev, buf, -ENOMEM);
        }
    } else if (udc_ctrl_stage_is_data_in(dev)) {
        dwc3_data->ep0_state = EP0_DATA_PHASE;
        dwc3_data->three_stage_setup = 1;
        dwc3_data->ep0_expect_in = 1;
        udc_ctrl_submit_s_in_status(dev);
    } else {
        if (((struct usb_setup_packet *)buf->data)->bRequest == USB_SREQ_SET_ADDRESS) {
            uint16_t addr = ((struct usb_setup_packet *)buf->data)->wValue;
            uint32_t reg = dwc3_dev->DCFG;
            reg &= ~(DWC3_DCFG_DEVADDR_MASK);
            reg |= DWC3_DCFG_DEVADDR(addr);
            dwc3_dev->DCFG = reg;
        }

        dwc3_data->three_stage_setup = 0;
        dwc3_data->ep0_expect_in = 0;
        /* two stage */
        udc_ctrl_submit_s_status(dev);
    }
}

static int dwc3_ep0_in_state_update(const struct device *dev, struct net_buf *const buf)
{
    int err = 0;

    if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
        /* Status stage finished, notify upper layer */
        err = udc_ctrl_submit_status(dev, buf);
    }

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_status_out(dev)) {
        net_buf_unref(buf);
    }

    return err;
}

static int dwc3_ep0_out_state_update(const struct device *dev, struct net_buf *const buf)
{
    int err = 0;

    if (udc_ctrl_stage_is_status_out(dev)) {
        /* Status stage finished, notify upper layer */
        err = udc_ctrl_submit_status(dev, buf);
    }

    /* Update to next stage of control transfer */
    udc_ctrl_update_stage(dev, buf);

    if (udc_ctrl_stage_is_status_in(dev)) {
        /* move status in, send zlp */
        return udc_ctrl_submit_s_out_status(dev, buf);
    }

    return err;
}

static void dwc3_ep0_complete_data(const struct device *dev, const union evt_buf_u *evt)
{
    uint8_t ep_num = evt->depevt.phy_ep_num;
    const struct udc_dwc3_config *config = dev->config;
    struct net_buf *buf;
    if (!ep_num) {
        /* out complete */
        buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
        udc_ep_set_busy(dev, USB_CONTROL_EP_OUT, false);
        net_buf_add(buf, 64 - config->out_trb[0].trb[0].bufsiz);
        sys_cache_data_invd_range(buf->data, buf->len);
        __ASSERT(!dwc3_ep0_out_state_update(dev, buf), "out state update fail");
    } else {
        /* in complete */
        buf = udc_buf_get(dev, USB_CONTROL_EP_IN);
        udc_ep_set_busy(dev, USB_CONTROL_EP_IN, false);
        __ASSERT(!dwc3_ep0_in_state_update(dev, buf), "in state update fail");
    }
}

static void dwc3_ep0_status_stage(const struct device *dev, const union evt_buf_u *evt)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);

    dwc3_data->ep0_state = EP0_SETUP_PHASE;
    /* alloc trb for ep0 out */
    dwc3_ctrl_feed_dout(dev, 8, TRB_Control_Setup);
}

static void dwc3_ep0_xfer_complete(const struct device *dev, const union evt_buf_u *evt)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);

    switch (dwc3_data->ep0_state) {
    case EP0_SETUP_PHASE:
        LOG_DBG("EP0_SETUP_PHASE!!");
        dwc3_ep0_setup_phase(dev, evt);
        break;
    case EP0_DATA_PHASE:
        LOG_DBG("EP0_DATA_PHASE!!");
        dwc3_ep0_complete_data(dev, evt);
        break;
    case EP0_STATUS_PHASE:
        LOG_DBG("EP0_STATUS_PHASE!!");
        dwc3_ep0_status_stage(dev, evt);
        break;
    default:
        break;
    }
}

static void dwc3_ep0_end_control_data(const struct device *dev)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    union dep_command_param param = {0};
    union dep_command cmd = {
        .cmd = {
            .commandparam = DWC3_DEPCMD_PARAM(dwc3_data->ep_res_index[0]),
            .cmdioc = 1, // trigger EPCmdCmplt irq
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_ENDTRANSFER,
        },
    };

    dwc3_dep_command(dwc3_dev, 0, &cmd, &param);
    dwc3_data->ep_res_index[0] = 0;
}

static void dwc3_ep0_stall_and_restart(const struct device *dev)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    union dep_command_param param = {0};
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_SETSTALL,
        },
    };
    dwc3_dep_command(dwc3_dev, 0, &cmd, &param);

    dwc3_data->ep0_state = EP0_SETUP_PHASE;
    /* alloc trb for ep0 out */
    dwc3_ctrl_feed_dout(dev, 8, TRB_Control_Setup);
}

static void dwc3_ep0_xfer_notready(const struct device *dev, const union evt_buf_u *evt)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    uint8_t ep_num = evt->depevt.phy_ep_num;

    switch (evt->depevt.evt_status) {
    case DEPEVT_STATUS_CONTROL_DATA: {
        LOG_DBG("DEPEVT_STATUS_CONTROL_DATA, ep_num = %d", ep_num);
        if (dwc3_data->ep0_expect_in != ep_num) {
            LOG_DBG("dwc3_data->ep0_expect_in != ep_num!!\n");
            dwc3_ep0_end_control_data(dev);
            dwc3_ep0_stall_and_restart(dev);
        }
    } break;
    case DEPEVT_STATUS_CONTROL_STATUS: {
        LOG_DBG("DEPEVT_STATUS_CONTROL_STATUS, ep_num = %d", ep_num);
        dwc3_data->ep0_state = EP0_STATUS_PHASE;
        if (dwc3_data->three_stage_setup && !ep_num) {
            /* Three stage, receive zlp. */
            dwc3_prepare_one_trb(config->out_trb[0].trb, NULL, 0, TRB_Control_Status_3, false);
            dwc3_dep_start_transfer(dev, ep_num, config->out_trb[0].trb, 0);
        } else {
            k_sem_give(&dwc3_data->ep0_sync);
        }
    } break;
    }
}

static void dwc3_ep0_event(const struct device *dev, const union evt_buf_u *evt)
{
    enum ep_evt evt_type = evt->depevt.ep_evt_type;
    switch (evt_type) {
    /* Applies to IN and OUT endpoints. Indicates that a EP/Stream transfer completed and the controller stopped the transfer. */
    case DEPEVT_XferComplete:
        dwc3_ep0_xfer_complete(dev, evt);
        break;
    /* Control-Data Start Transfe  */
    case DEPEVT_XferNotReady:
        dwc3_ep0_xfer_notready(dev, evt);
        break;
    case DEPEVT_EPCmdCmplt:
        break;
    default:
        break;
    }
}

static void dwc3_endpoint_event(const struct device *dev, const union evt_buf_u *evt)
{
    /* Device Endpoint-Specific Events  */
    enum ep_evt evt_type = evt->depevt.ep_evt_type;
    uint8_t ep_num = evt->depevt.phy_ep_num;
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, PHY_EP_IDX_2_EP_ADDR(ep_num));
    const struct udc_dwc3_config *config = dev->config;
    struct net_buf *buf;

    if (ep_num == 0 || ep_num == 1) {
        dwc3_ep0_event(dev, evt);
        return;
    }

    /* other endpoint */
    switch (evt_type) {
    case DEPEVT_XferComplete:
        LOG_DBG("phy_ep %d, ep_addr= %x DEPEVT_XferComplete.", ep_num, ep_cfg->addr);
        buf = udc_buf_get(dev, ep_cfg->addr);
        if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
            net_buf_add(buf, buf->size - config->out_trb[USB_EP_GET_IDX(ep_cfg->addr)].trb[0].bufsiz);
            sys_cache_data_invd_range(buf->data, buf->len);
        }
        udc_ep_set_busy(dev, ep_cfg->addr, false);
        udc_submit_ep_event(dev, buf, 0);
        break;
    case DEPEVT_XferNotReady:
        /* stream-capable ep (usb3.0) or iso ep */
        LOG_DBG("phy_ep %d, ep_addr= %x DEPEVT_XferNotReady.", ep_num, ep_cfg->addr);
        break;
    case DEPEVT_XferInProgress:
        LOG_DBG("phy_ep %d DEPEVT_XferInProgress.", ep_num);
        break;
    case DEPEVT_StreamEvt:
        LOG_DBG("phy_ep %d DEPEVT_StreamEvt.", ep_num);
        break;
    case DEPEVT_EPCmdCmplt:
        LOG_DBG("phy_ep %d DEPEVT_EPCmdCmplt.", ep_num);
        break;
    }
}

static void dwc3_dev_reset(const struct device *dev, const union evt_buf_u *evt)
{
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);

    dwc3_dev->DCTL &= ~DWC3_DCTL_TSTCTRL_MASK;
    /* Set DevAddr to 0 */
    MODIFY_REG(dwc3_dev->DCFG, DWC3_DCFG_DEVADDR_MASK, DWC3_DCFG_DEVADDR(0));
    udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static void dwc3_dev_disconnect(const struct device *dev, const union evt_buf_u *evt)
{
    /* This event is generated only if Vbus is off */
}

static void dwc3_dev_connect_done(const struct device *dev, const union evt_buf_u *evt)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    uint8_t speed = dwc3_dev->DSTS & DWC3_DSTS_CONNECTSPD;
    union dep_command_param cmd_param = DEPCFG_POR(USB_EP_DIR_OUT);

    switch (speed) {
    case DWC3_DSTS_HIGHSPEED:
        dwc3_data->speed = UDC_BUS_SPEED_HS;
        LOG_DBG("DWC3_DSTS_HIGHSPEED.");
        break;
    case DWC3_DSTS_FULLSPEED:
        dwc3_data->speed = UDC_BUS_SPEED_FS;
        LOG_DBG("DWC3_DSTS_FULLSPEED.");
        break;
    case DWC3_DSTS_SUPERSPEED:
    case DWC3_DSTS_SUPERSPEED_PLUS:
        dwc3_data->speed = UDC_BUS_SPEED_SS;
        break;
    default:
        __ASSERT(0, "not support %d speed", dwc3_data->speed);
        break;
    }

    cmd_param.cfg.ConfigAction = Modify_EP_State;
    dwc3_dep_config(dwc3_dev, 0, &cmd_param);
    cmd_param.cfg.EpDir = 1;
    dwc3_dep_config(dwc3_dev, 1, &cmd_param);

    dwc3_dev->DEVTEN |= DWC3_DEVTEN_ULSTCNGEN;
}

static void dwc3_dev_suspend_entry_event(const struct device *dev, const union evt_buf_u *evt)
{
    /* This event is generated when hibernation mode is disabled and host into sleep mode. */
    udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
}

static void dwc3_dev_wakeup_detected_event(const struct device *dev, const union evt_buf_u *evt)
{
    /* This event is generated when the host initiates a resume condition on the USB bus */
    /* This event is not generated by the device initiating a remote wakeup to the host. */
    udc_submit_event(dev, UDC_EVT_RESUME, 0);
}

static void dwc3_dev_link_change_event(const struct device *dev, const union evt_buf_u *evt)
{
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);

    switch (evt->devt.evt_info) {
    case DWC3_LINK_STATE_U3:
        dwc3_dev->DEVTEN &= ~DWC3_DEVTEN_ULSTCNGEN;
        udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
        break;
    default:
        break;
    }
}

static void dwc3_dev_event(const struct device *dev, const union evt_buf_u *evt)
{
    /* Device-Specific Events */
    enum dev_evt evt_type = evt->devt.dev_evt_type;
    switch (evt_type) {
    case DEVT_DisconnEvt:
        dwc3_dev_disconnect(dev, evt);
        break;
    case DEVT_USBRst:
        dwc3_dev_reset(dev, evt);
        break;
    case DEVT_ConnectDone:
        dwc3_dev_connect_done(dev, evt);
        break;
    case DEVT_WkUpEvt:
        dwc3_dev_wakeup_detected_event(dev, evt);
        break;
    case DEVT_USBSuspendEntryEvt:
        dwc3_dev_suspend_entry_event(dev, evt);
        break;
    case DEVT_ULStChng:
        dwc3_dev_link_change_event(dev, evt);
        break;
    case DEVT_HibernationRequestEvt:
        break;
    case DEVT_CmdCmplt:
        break;
    case DEVT_EvntOverflow:
        LOG_ERR("DEVT_EvntOverflow.\n");
        break;
    default:
        break;
    }
}

static void event_handler(const struct device *dev, void *param)
{
    const struct udc_dwc3_config *config = dev->config;
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    struct dwc3_global_reg *dwc3_gbl = (struct dwc3_global_reg *)(config->base + DWC3_GLOBALS_REGS_START);

    for (uint32_t i = 0; i < dwc3_data->evt_count; i += sizeof(union evt_buf_u)) {
        union evt_buf_u evt = dwc3_data->evt_buf[dwc3_data->evt_buf_rd_idx];
        LOG_DBG("evt = 0x%x, count = %d, dwc3_data->evt_buf_rd_idx = %d.\n", *(uint32_t *)&evt, dwc3_data->evt_count, dwc3_data->evt_buf_rd_idx);

        if (!evt.dev_ep)
            dwc3_endpoint_event(dev, &evt);
        else
            dwc3_dev_event(dev, &evt);
        dwc3_data->evt_buf_rd_idx = (dwc3_data->evt_buf_rd_idx + 1) % EVT_BUF_LENGTH_WORDS;
    }

    dwc3_gbl->GEVNT[0].SIZ &= ~DWC3_GEVNTSIZ_INTMASK;
}

static void udc_dwc3_isr_handler(const struct device *dev)
{
    uint32_t count, remain;
    const struct udc_dwc3_config *config = dev->config;
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    struct dwc3_global_reg *dwc3_gbl = (struct dwc3_global_reg *)(config->base + DWC3_GLOBALS_REGS_START);
    struct dwc3_msgq_type msg = {
        .handler = event_handler,
        .param = NULL,
    };

    count = dwc3_gbl->GEVNT[0].COUNT;
    count &= DWC3_GEVNTCOUNT_MASK;
    if (!count)
        return;

    dwc3_gbl->GEVNT[0].SIZ |= DWC3_GEVNTSIZ_INTMASK;
    dwc3_data->evt_count = count;
    remain = MIN(count, (EVT_BUF_LENGTH_WORDS - dwc3_data->evt_buf_rd_idx) * 4);

    sys_cache_data_invd_range(config->evt_buf, sizeof(union evt_buf_u) * EVT_BUF_LENGTH_WORDS);
    memcpy(dwc3_data->evt_buf + dwc3_data->evt_buf_rd_idx, config->evt_buf + dwc3_data->evt_buf_rd_idx, remain);
    if (remain < count)
        memcpy(dwc3_data->evt_buf, config->evt_buf, count - remain);

    /* update count */
    dwc3_gbl->GEVNT[0].COUNT = count;
    __ASSERT(!k_msgq_put(&dwc3_data->msgq, &msg, K_NO_WAIT), "dwc3 udc isr msgq error");
}

static int dwc3_driver_preinit(const struct device *dev)
{
    const struct udc_dwc3_config *config = dev->config;
    struct udc_data *data = dev->data;
    uint16_t mps = 512;
    int err;

    k_mutex_init(&data->mutex);
    data->caps.rwup = true;
    data->caps.mps0 = UDC_MPS0_64;
    data->caps.hs = 1;

    for (int i = 0; i < config->num_out_eps; i++) {
        config->ep_cfg_out[i].caps.out = 1;
        if (i == 0) {
            config->ep_cfg_out[i].caps.control = 1;
            config->ep_cfg_out[i].caps.mps = 64;
        } else {
            config->ep_cfg_out[i].caps.bulk = 1;
            config->ep_cfg_out[i].caps.interrupt = 1;
            config->ep_cfg_out[i].caps.iso = 1;
            config->ep_cfg_out[i].caps.mps = mps;
        }

        config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
        err = udc_register_ep(dev, &config->ep_cfg_out[i]);
        if (err != 0) {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }

    for (int i = 0; i < config->num_in_eps; i++) {
        config->ep_cfg_in[i].caps.in = 1;
        if (i == 0) {
            config->ep_cfg_in[i].caps.control = 1;
            config->ep_cfg_in[i].caps.mps = 64;
        } else {
            config->ep_cfg_in[i].caps.bulk = 1;
            config->ep_cfg_in[i].caps.interrupt = 1;
            config->ep_cfg_in[i].caps.iso = 1;
            config->ep_cfg_in[i].caps.mps = mps;
        }

        config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
        err = udc_register_ep(dev, &config->ep_cfg_in[i]);
        if (err != 0) {
            LOG_ERR("Failed to register endpoint");
            return err;
        }
    }

    return 0;
}

static int udc_dwc3_lock(const struct device *dev)
{
    return udc_lock_internal(dev, K_FOREVER);
}

static int udc_dwc3_unlock(const struct device *dev)
{
    return udc_unlock_internal(dev);
}

#if 0

#include <zephyr/shell/shell.h>
#include <stdlib.h>

static uint8_t dwc3_phy_cfg_read(uint8_t reg_addr)
{
    uint32_t res;

    /* rst en */
    *(uint32_t *)0x40058010 |= BIT(25);
    k_usleep(1);

    /* set reg_addr and write_data*/
    *(uint32_t *)0x40058010 &= ~(0xff << 2);
    *(uint32_t *)0x40058010 |= (reg_addr << 2);
    k_usleep(1);
    /* enable read */
    *(uint32_t *)0x40058010 |= (0x1 << 0);
    k_usleep(1);
    *(uint32_t *)0x40058010 &= ~(0x1 << 0);
    k_usleep(1);
    res = *(uint32_t *)0x40058010;
    k_usleep(1);

    return (res >> 8) & 0xff;
}

static void dwc3_phy_cfg_write(uint8_t reg_addr, uint8_t write_data)
{
    /* rst en */
    *(uint32_t *)0x40058010 |= BIT(25);
    k_usleep(1);

    /* set reg_addr and write_data*/
    *(uint32_t *)0x40058010 &= ~(0xff << 2);
    *(uint32_t *)0x40058010 |= (reg_addr << 2);
    *(uint32_t *)0x40058010 &= ~(0xff << 16);
    *(uint32_t *)0x40058010 |= (write_data << 16);

    k_usleep(1);
    /* enable write */
    *(uint32_t *)0x40058010 |= (0x1 << 1);
    k_usleep(1);
    *(uint32_t *)0x40058010 &= ~(0x1 << 1);
}

static int usb_phy_reg_write(const struct shell *sh, size_t argc, char **argv) 
{
    if (argc < 3) {
        printk("%s <address> <data>\n", argv[0]);
        return 0;
    }
    dwc3_phy_cfg_write(atoi(argv[1]), atoi(argv[2]));
    return 0;
}
SHELL_CMD_REGISTER(usb_phy_reg_write, NULL, "naneng phy write reg", usb_phy_reg_write);

static int usb_phy_reg_read(const struct shell *sh, size_t argc, char **argv) 
{
    uint8_t res = 0;
    if (argc < 2) {
        printk("%s <address>\n", argv[0]);
        return 0;
    }
    res = dwc3_phy_cfg_read(atoi(argv[1]));
    printk("reg_%x value: 0x%x\n", atoi(argv[1]), res);
    return 0;
}
SHELL_CMD_REGISTER(usb_phy_reg_read, NULL, "naneng phy read reg", usb_phy_reg_read);

#endif

static int dwc3_phy_setup(const struct device *dev)
{
    *(uint32_t *)0x40058014 = 0x131; // pll_en
    return 0;
}

static int udc_dwc3_init(const struct device *dev)
{
    LOG_DBG("%s udc init", dev->name);
    const struct udc_dwc3_config *config = dev->config;
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    struct dwc3_global_reg *dwc3_gbl = (struct dwc3_global_reg *)(config->base + DWC3_GLOBALS_REGS_START);

    k_msgq_init(&dwc3_data->msgq, (char *)dwc3_data->msgq_buf, sizeof(struct dwc3_msgq_type), DWC3_MSGQ_LENGTH);
    k_sem_init(&dwc3_data->ep0_sync, 0, 1);

    dwc3_data->dev = dev;
    dwc3_data->ep0_state = EP0_SETUP_PHASE;
    config->make_thread(dev);

#if defined(CONFIG_CLOCK_CONTROL)
    if (config->ccfg.cctl_dev) {
        const struct device *clk_dev = config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (config->reset.dev != NULL) {
        if (!device_is_ready(config->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        reset_line_toggle(config->reset.dev, config->reset.id);
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (config->ccfg.cctl_dev) {
        const struct device *clk_dev = config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&config->ccfg);
    }
#endif

    /* phy init */
    dwc3_phy_setup(dev);

    /* soft reset */
    MODIFY_REG(dwc3_dev->DCTL, DWC3_DCTL_RUN_STOP, DWC3_DCTL_CSFTRST);
    while (dwc3_dev->DCTL & DWC3_DCTL_CSFTRST);

    dwc3_gbl->GUSB2PHYCFG[0] = 0x40002407;
    dwc3_dev->DCFG = 0x480800; 
    dwc3_gbl->GUCTL = 0xa400010;

    dwc3_gbl->GEVNT[0].ADRLO = (uint32_t)config->evt_buf;
    dwc3_gbl->GEVNT[0].ADRHI = 0;
    dwc3_gbl->GEVNT[0].SIZ = EVT_BUF_LENGTH_WORDS * sizeof(union evt_buf_u);
    dwc3_gbl->GEVNT[0].COUNT = 0;
    dwc3_gbl->GCTL = 0x30C12204;

    dwc3_dev->DEVTEN = DWC3_DEVTEN_DISCONNEVTEN | DWC3_DEVTEN_USBRSTEN | DWC3_DEVTEN_CONNECTDONEEN 
                    | DWC3_DEVTEN_WKUPEVTEN | DWC3_DEVTEN_HIBERNATIONREQEVTEN
                    | DWC3_DEVTEN_ERRTICERREN | DWC3_DEVTEN_CMDCMPLTEN | DWC3_DEVTEN_EVNTOVERFLOWEN | DWC3_DEVTEN_ECCERREN;
    return 0;
}

static int udc_dwc3_enable(const struct device *dev)
{
    LOG_DBG("%s udc enable", dev->name);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    union dep_command_param cmd_param = {
        .xfercfg.NumXferRes = 1
    };

    config->irq_enable_func(dev);
    dwc3_dep_start_config(dwc3_dev, 0);

    /* 
       There must be only one transfer resource allocated per endpoint.
       Start Transfer causes the use of the transfer resource.
       End Transfer or an XferComplete event releases the transfer resource.
    */
    for (int i = 0; i < DWC_USB3_NUM_EPS; i++) {
        dwc3_dep_xfer_resource(dwc3_dev, i, &cmd_param);
    }

    if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
                               USB_EP_TYPE_CONTROL, 64, 0)) {
        LOG_ERR("Failed to enable control endpoint");
        return -EIO;
    }

    if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
                               USB_EP_TYPE_CONTROL, 64, 0)) {
        LOG_ERR("Failed to enable control endpoint");
        return -EIO;
    }
    /* allow the device to attach to the host. */
    MODIFY_REG(dwc3_dev->DCTL, 0, DWC3_DCTL_RUN_STOP);

    return 0;
}

static int udc_dwc3_shutdown(const struct device *dev)
{
    LOG_DBG("%s udc shutdown", dev->name);
    return 0;
}

static int udc_dwc3_disable(const struct device *dev)
{
    LOG_DBG("%s udc disable", dev->name);
    return 0;
}

static int udc_dwc3_set_address(const struct device *dev, const uint8_t addr)
{
    LOG_DBG("%s udc set address %d", dev->name, addr);
    return 0;
}

static int udc_dwc3_test_mode(const struct device *dev, const uint8_t mode, const bool dryrun)
{
    uint32_t reg = 0;
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    LOG_INF("%s udc test mode,%d,%d", dev->name, mode, dryrun);

    if (mode == 0U || mode > TEST_FORCE_EN) {
        return -EINVAL;
    }

    if (dryrun) {
        return 0;
    }

    reg = dwc3_dev->DCTL;
    reg &= ~DWC3_DCTL_TSTCTRL_MASK;

    switch (mode) {
    case TEST_J:
    case TEST_K:
    case TEST_SE0_NAK:
    case TEST_PACKET:
    case TEST_FORCE_EN:
        reg |= mode << 1;
        break;
    default:
        return -EINVAL;
    }
    dwc3_dev->DCTL = reg;

    return 0;
}

static int udc_dwc3_host_wakeup(const struct device *dev)
{
    LOG_DBG("%s udc host wakeup", dev->name);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    uint32_t reg = dwc3_dev->DCTL;

    reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
    reg |= DWC3_DCTL_ULSTCHNGREQ(DWC3_LINK_STATE_RECOV);
    dwc3_dev->DCTL = reg;

    return 0;
}

static int udc_dwc3_ep_activate(const struct device *dev, struct udc_ep_config *const cfg)
{
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
    uint8_t ep_dir = USB_EP_GET_DIR(cfg->addr);
    union dep_command_param cmd_param;

    LOG_DBG("%s udc ep activate,cfg->addr = %x", dev->name, cfg->addr);
    if (ep_idx) {
        uint8_t ep_in = ep_dir == USB_EP_DIR_IN;
        uint8_t phy_ep_idx = EP_ADDR_2_PHY_EP_IDX(cfg->addr);
        cmd_param.cfg.FIFOBased = 0;
        cmd_param.cfg.StrmCap = 0;
        cmd_param.cfg.EBCMode = 0;
        cmd_param.cfg.TRBWrBack = 0;
        cmd_param.cfg.IntrNum = 0;
        cmd_param.cfg.BrstSiz = 0;
        cmd_param.cfg.XferCmplEn = 1;
        cmd_param.cfg.XferInProgEn = 1;
        cmd_param.cfg.XferNRdyEn = 0;
        cmd_param.cfg.StreamEvtEn = 1;
        cmd_param.cfg.ConfigAction = Initialize_EP_State;
        cmd_param.cfg.bInterval_m1 = cfg->interval;
        cmd_param.cfg.MPS = cfg->mps; // USB 3.0 supports up to 1024 bytes
        cmd_param.cfg.FIFONum = ep_in ? ep_idx : 0;
        cmd_param.cfg.EpNum = ep_idx;
        cmd_param.cfg.EpDir = ep_in;

        switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
        case USB_EP_TYPE_BULK:
            LOG_DBG("USB_EP_TYPE_BULK %d %d", phy_ep_idx, cfg->interval);
            cmd_param.cfg.EPType = DWC3_DEPCMD_TYPE_BULK;
            break;
        case USB_EP_TYPE_INTERRUPT:
            LOG_DBG("USB_EP_TYPE_INTERRUPT %d %d", phy_ep_idx, cfg->interval);
            cmd_param.cfg.EPType = DWC3_DEPCMD_TYPE_INTR;
            break;
        case USB_EP_TYPE_ISO:
            LOG_DBG("USB_EP_TYPE_ISO %d %d", phy_ep_idx, cfg->interval);
            cmd_param.cfg.EPType = DWC3_DEPCMD_TYPE_ISOC;
            break;
        default:
            return -EINVAL;
        }
        dwc3_dep_config(dwc3_dev, phy_ep_idx, &cmd_param);
        dwc3_dev->DALEPENA |= 1 << phy_ep_idx;
    } else {
        cmd_param = DEPCFG_POR(USB_EP_DIR_OUT);
        if (ep_dir == USB_EP_DIR_OUT) {
            dwc3_dep_config(dwc3_dev, 0, &cmd_param);
            dwc3_dev->DALEPENA |= 0x1;
            /* begin to receive SETUP packets */
            dwc3_ctrl_feed_dout(dev, 8, TRB_Control_Setup);
        } else {
            cmd_param.cfg.EpDir = !USB_EP_DIR_OUT;
            dwc3_dep_config(dwc3_dev, 1, &cmd_param);
            dwc3_dev->DALEPENA |= 0x2;
        }
    }
    return 0;
}

static int udc_dwc3_ep_deactivate(const struct device *dev, struct udc_ep_config *const cfg)
{
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    uint8_t phy_ep_idx = EP_ADDR_2_PHY_EP_IDX(cfg->addr);

    LOG_DBG("%s udc ep deactivate, ep : %x, phy_ep_idx : %d", dev->name, cfg->addr, phy_ep_idx);

    union dep_command_param param = {0};
    union dep_command end_trans = {
        .cmd = {
            .commandparam = dwc3_data->ep_res_index[phy_ep_idx],
            .cmdact = 1,
            .cmdioc = 1,
            .hipri_forcerm = 1,
            .cmdtyp = DWC3_DEPCMD_ENDTRANSFER,
        },
    };

    if (phy_ep_idx && dwc3_data->ep_res_index[phy_ep_idx]) {
        dwc3_dep_command(dwc3_dev, phy_ep_idx, &end_trans, &param);
        dwc3_data->ep_res_index[phy_ep_idx] = 0;
    }

    if (cfg->stat.halted) {
        union dep_command clear_halt = {
            .cmd = {
                .cmdact = 1,
                .cmdtyp = DWC3_DEPCMD_CLEARSTALL,
            },
        };
        dwc3_dep_command(dwc3_dev, phy_ep_idx, &clear_halt, &param);
        cfg->stat.halted = false;
    }
    dwc3_dev->DALEPENA &= ~(1 << phy_ep_idx);

    return 0;
}

static int udc_dwc3_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
    LOG_DBG("%s udc ep set halt, ep 0x%x", dev->name, cfg->addr);
    const struct udc_dwc3_config *config = dev->config;
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);
    union dep_command_param param = {0};
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_SETSTALL,
        },
    };
    /* For control endpoints, the application issues only the Set Stall command, and only on the OUT direction of the control endpoint. */
    /* For non-control endpoints, the application is responsible for both setting and clearing STALL via the Set
       Stall/Clear Stall commands. When the application clears the STALL, the endpoint's data sequence number
       is reset to zero. */

    if (!USB_EP_GET_IDX(cfg->addr)) {
        dwc3_dep_command(dwc3_dev, 0, &cmd, &param);
        /* update ep0 state */
        dwc3_data->ep0_state = EP0_SETUP_PHASE;
        /* prepare trb for ep0 and start transfer */
        dwc3_ctrl_feed_dout(dev, 8, TRB_Control_Setup);
    } else {
        dwc3_dep_command(dwc3_dev, EP_ADDR_2_PHY_EP_IDX(cfg->addr), &cmd, &param);
    }

    cfg->stat.halted = true;
    return 0;
}

static int udc_dwc3_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
    LOG_DBG("%s udc ep clear halt", dev->name);
    /* The controller automatically clears the STALL when it receives a SETUP token for the endpoint */
    /* The application must not issue the Clear Stall command on a control endpoint. */
    const struct udc_dwc3_config *config = dev->config;
    struct dwc3_dev_reg *dwc3_dev = (struct dwc3_dev_reg *)(config->base + DWC3_DEVICE_REGS_START);

    union dep_command_param param = {0};
    union dep_command cmd = {
        .cmd = {
            .cmdact = 1,
            .cmdtyp = DWC3_DEPCMD_CLEARSTALL,
        },
    };

    if (USB_EP_GET_IDX(cfg->addr)) {
        dwc3_dep_command(dwc3_dev, EP_ADDR_2_PHY_EP_IDX(cfg->addr), &cmd, &param);
    }

    cfg->stat.halted = false;
    return 0;
}

static int udc_dwc3_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg, struct net_buf *const buf)
{
    __ASSERT(!udc_ep_is_busy(dev, cfg->addr), "ep %x is busy", cfg->addr);
    const struct udc_dwc3_config *config = dev->config;
    uint8_t addr = cfg->addr;
    struct udc_dwc3_data *dwc3_data = udc_get_private(dev);
    uint8_t ep_idx = USB_EP_GET_IDX(addr);
    struct udc_buf_info *bi = udc_get_buf_info(buf);
    struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, addr);
    struct dwc3_trb *trb;

    LOG_DBG("ep: %x buf:%p, buf-len: %d, buf->size = %d, bi->data:%d bi->status %d", addr, buf, buf->len, buf->size, bi->data, bi->status);

    udc_buf_put(cfg, buf);
    if (ep_idx) {
        /*
            For OUT endpoints, the following rules apply:
            ■ The BUFSIZ field must be ≥ 1 byte.
            ■ The total size of a Buffer Descriptor must be a multiple of MaxPacketSize.
            ■ For setup stage of control transfer, BUFSIZ field must be 8 bytes.
            ■ A received zero-length packet still requires a MaxPacketSize buffer. Therefore, if the expected amount
            of data to be received is a multiple of MaxPacketSize, software should add MaxPacketSize bytes to the
            buffer to sink a possible zero-length packet at the end of the transfer.

            For IN endpoints, the following rules apply:
            ■ The number of chained TRBs necessary to construct a single packet must never exceed (DWC_USB3_-
            CACHE_TRBS_PER_TRANSFER – 1). A maximum of one Link TRB can be present in the chain.
            ■ If software wants to indicate a transfer completion to the host by sending a zero-length packet after a
            multiple of MaxPacketSize, it must set up a zero-length TRB following the last TRB in the transfer.
        */
        if (USB_EP_DIR_IS_IN(addr)) {
            trb = config->in_trb[ep_idx].trb;
            if (!udc_ep_buf_has_zlp(buf)) {
                if (buf->len)
                    dwc3_prepare_one_trb(trb, buf->data, DWC3_TRB_SIZE_LENGTH(buf->len), TRB_Normal, false);
                else
                    dwc3_prepare_one_trb(trb, NULL, 0, TRB_Normal_ZLP, false);
            } else {
                __ASSERT(!(buf->len % ep_cfg->mps), "error");
                /* buf->len max is 0xfffff, so use one trb is ok*/
                dwc3_prepare_one_trb(trb, buf->data, DWC3_TRB_SIZE_LENGTH(buf->len), TRB_Normal, true);
                dwc3_prepare_one_trb(++trb, NULL, 0, TRB_Normal_ZLP, false);
            }
            sys_cache_data_flush_range(buf->data, buf->len);
        } else {
            uint16_t remain = buf->size % ep_cfg->mps;
            trb = config->out_trb[ep_idx].trb;
            if (!remain) {
                dwc3_prepare_one_trb(trb, buf->data, DWC3_TRB_SIZE_LENGTH(buf->size), TRB_Normal, false);
            } else {
                /* align mps */
                dwc3_prepare_one_trb(trb, buf->data, DWC3_TRB_SIZE_LENGTH(buf->size), TRB_Normal, true);
                dwc3_prepare_one_trb(++trb, config->bounce_addr, ep_cfg->mps - remain, TRB_Normal, false);
            }
        }
        dwc3_dep_start_transfer(dev, EP_ADDR_2_PHY_EP_IDX(addr), trb, 0);
    } else {
        if (USB_EP_DIR_IS_IN(addr)) {
            trb = config->in_trb[0].trb;
            if (bi->data) {
                udc_ep_set_busy(dev, addr, true);
                dwc3_prepare_one_trb(trb, buf->data, buf->len, TRB_Control_Data, false);
                sys_cache_data_flush_range(buf->data, buf->len);
            } else if (bi->status) {
                k_sem_take(&dwc3_data->ep0_sync, K_FOREVER);
                dwc3_prepare_one_trb(trb, NULL, 0, dwc3_data->three_stage_setup ? TRB_Control_Status_3 : TRB_Control_Status_2, false);
                dwc3_ep0_in_state_update(dev, udc_buf_get(dev, USB_CONTROL_EP_IN));
            }
            dwc3_dep_start_transfer(dev, 1, trb, 0);
        }
    }

    return 0;
}

static int udc_dwc3_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
    LOG_DBG("%s udc ep dequeue, %x", dev->name, cfg->addr);
    struct net_buf *buf;
    buf = udc_buf_get_all(dev, cfg->addr);
    if (buf) {
        udc_submit_ep_event(dev, buf, -ECONNABORTED);
    }
    udc_ep_set_busy(dev, cfg->addr, false);

    return 0;
}

static const struct udc_api udc_dwc3_api = {
    .lock = udc_dwc3_lock,
    .unlock = udc_dwc3_unlock,
    .device_speed = udc_dwc3_device_speed,
    .init = udc_dwc3_init,
    .enable = udc_dwc3_enable,
    .disable = udc_dwc3_disable,
    .shutdown = udc_dwc3_shutdown,
    .set_address = udc_dwc3_set_address,
    .test_mode = udc_dwc3_test_mode,
    .host_wakeup = udc_dwc3_host_wakeup,
    .ep_enable = udc_dwc3_ep_activate,
    .ep_disable = udc_dwc3_ep_deactivate,
    .ep_set_halt = udc_dwc3_ep_set_halt,
    .ep_clear_halt = udc_dwc3_ep_clear_halt,
    .ep_try_config = NULL,
    .ep_enqueue = udc_dwc3_ep_enqueue,
    .ep_dequeue = udc_dwc3_ep_dequeue,
};

static void udc_dwc3_thread_handler(void *dev)
{
    struct udc_dwc3_data *usb_data = (struct udc_dwc3_data *)udc_get_private(dev);
    struct dwc3_msgq_type evt;
    while (1) {
        k_msgq_get(&usb_data->msgq, &evt, K_FOREVER);
        evt.handler(dev, evt.param);
    }
}

#define UDC_DWC3_DEVICE_DEFINE(n)                                                                                                           \
    K_THREAD_STACK_DEFINE(udc_dwc3_stack_##n, CONFIG_UDC_DWC3_STACK_SIZE);                                                                  \
    static void udc_dwc3_irq_enable_func_##n(const struct device *dev)                                                                      \
    {                                                                                                                                       \
        IRQ_CONNECT(DT_INST_IRQN(n),                                                                                                        \
                    DT_INST_IRQ(n, priority),                                                                                               \
                    udc_dwc3_isr_handler,                                                                                                   \
                    DEVICE_DT_INST_GET(n), 0);                                                                                              \
                                                                                                                                            \
        irq_enable(DT_INST_IRQN(n));                                                                                                        \
    }                                                                                                                                       \
                                                                                                                                            \
    static void udc_dwc3_irq_disable_func_##n(const struct device *dev)                                                                     \
    {                                                                                                                                       \
        irq_disable(DT_INST_IRQN(n));                                                                                                       \
    }                                                                                                                                       \
    static void udc_dwc3_thread_##n(void *dev, void *arg1, void *arg2)                                                                      \
    {                                                                                                                                       \
        while (true) {                                                                                                                      \
            udc_dwc3_thread_handler(dev);                                                                                                   \
        }                                                                                                                                   \
    }                                                                                                                                       \
    static void udc_dwc3_make_thread_##n(const struct device *dev)                                                                          \
    {                                                                                                                                       \
        struct udc_dwc3_data *priv = udc_get_private(dev);                                                                                  \
        k_thread_create(&priv->thread,                                                                                                      \
                        udc_dwc3_stack_##n,                                                                                                 \
                        K_THREAD_STACK_SIZEOF(udc_dwc3_stack_##n),                                                                          \
                        udc_dwc3_thread_##n,                                                                                                \
                        (void *)dev, NULL, NULL,                                                                                            \
                        K_PRIO_COOP(CONFIG_UDC_DWC3_THREAD_PRIORITY),                                                                       \
                        K_ESSENTIAL,                                                                                                        \
                        K_NO_WAIT);                                                                                                         \
        k_thread_name_set(&priv->thread, dev->name);                                                                                        \
    }                                                                                                                                       \
    static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_out_eps)];                                                               \
    static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_in_eps)];                                                                 \
    __nocache static struct dwc3_ep_trb ep_in_trb_##n[DT_INST_PROP(n, num_in_eps)];                                                         \
    __nocache static struct dwc3_ep_trb ep_out_trb_##n[DT_INST_PROP(n, num_out_eps)];                                                       \
    static __attribute__((aligned(EVT_BUF_LENGTH_WORDS * sizeof(union evt_buf_u)))) union evt_buf_u dwc3_evt_buf_##n[EVT_BUF_LENGTH_WORDS]; \
    static uint8_t bounce_addr[BOUNCE_ADDR_SIZE];                                                                                           \
    static const struct udc_dwc3_config udc_dwc3_config_##n = {                                                                             \
        .num_out_eps = DT_INST_PROP(n, num_out_eps),                                                                                        \
        .num_in_eps = DT_INST_PROP(n, num_in_eps),                                                                                          \
        .ep_cfg_in = ep_cfg_in_##n,                                                                                                         \
        .ep_cfg_out = ep_cfg_out_##n,                                                                                                       \
        .in_trb = ep_in_trb_##n,                                                                                                            \
        .out_trb = ep_out_trb_##n,                                                                                                          \
        .base = DT_INST_REG_ADDR(n),                                                                                                        \
        .make_thread = udc_dwc3_make_thread_##n,                                                                                            \
        .irq_enable_func = udc_dwc3_irq_enable_func_##n,                                                                                    \
        .irq_disable_func = udc_dwc3_irq_disable_func_##n,                                                                                  \
        .evt_buf = dwc3_evt_buf_##n,                                                                                                        \
        .bounce_addr = bounce_addr,                                                                                                         \
        IF_ENABLED(DT_HAS_CLOCKS(n), (.ccfg = LS_DT_CLK_CFG_ITEM(n), ))                                                                     \
            IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n), ))};                                          \
    static struct udc_dwc3_data udc_priv_##n;                                                                                               \
    static struct udc_data udc_data_##n = {                                                                                                 \
        .mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                                                                   \
        .priv = &udc_priv_##n,                                                                                                              \
    };                                                                                                                                      \
    DEVICE_DT_INST_DEFINE(n, dwc3_driver_preinit, NULL,                                                                                     \
                          &udc_data_##n, &udc_dwc3_config_##n,                                                                              \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                                                  \
                          &udc_dwc3_api);
DT_INST_FOREACH_STATUS_OKAY(UDC_DWC3_DEVICE_DEFINE)