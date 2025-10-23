/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include "xhci.h"

#define DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX 16
#define DWC_USB3_HOST_NUM_U2_ROOT_PORTS 1
#define DWC_USB3_HOST_NUM_U3_ROOT_PORTS_MAX 16
#define DWC_USB3_HOST_NUM_U3_ROOT_PORTS 1
#define DWC_USB3_DEVICE_NUM_INT_MAX 32
#define DWC_USB3_DEVICE_NUM_INT 1

struct dwc3_gevnt_reg{
	volatile uint32_t ADRLO;		//0x300
	volatile uint32_t ADRHI;		//0x304
	volatile uint32_t SIZ;			//0x308
	volatile uint32_t COUNT;		//0x30c
};

#define DWC_USB3_NUM_EPS_MAX 32
#define DWC_USB3_NUM_EPS 8
struct dwc3_dev_ep_reg{
	volatile uint32_t CMDPAR2;
	volatile uint32_t CMDPAR1;
	volatile uint32_t CMDPAR0;
	volatile uint32_t CMD;
};

struct dwc3_global_reg {
	volatile uint32_t GSBUSCFG0;		//0x00
	volatile uint32_t GSBUSCFG1;		//0x04
	volatile uint32_t GTXTHRCFG;		//0x08
	volatile uint32_t GRXTHRCFG;		//0x0c
	volatile uint32_t GCTL;				//0x10
	volatile uint32_t GPMSTS;			//0x14
	volatile uint32_t GSTS;				//0x18
	volatile uint32_t GUCTL1;			//0x1c
	volatile uint32_t GSNPSID;			//0x20
	volatile uint32_t GGPIO;			//0x24
	volatile uint32_t GUID;				//0x28
	volatile uint32_t GUCTL;			//0x2c
	volatile uint32_t GBUSERRADDRLO;	//0x30
	volatile uint32_t GBUSERRADDRHI;	//0x34
	volatile uint32_t GPRTBIMAPLO;		//0x38
	volatile uint32_t GPRTBIMAPHI;		//0x3c
	volatile uint32_t GHWPARAMS0;		//0x40
	volatile uint32_t GHWPARAMS1;		//0x44
	volatile uint32_t GHWPARAMS2;		//0x48
	volatile uint32_t GHWPARAMS3;		//0x4c
	volatile uint32_t GHWPARAMS4;		//0x50
	volatile uint32_t GHWPARAMS5;		//0x54
	volatile uint32_t GHWPARAMS6;		//0x58
	volatile uint32_t GHWPARAMS7;		//0x5c
	volatile uint32_t GDBGFIFOSPACE;	//0x60
	volatile uint32_t GDBGLTSSM;		//0x64
	volatile uint32_t GDBGLNMCC;		//0x68
	volatile uint32_t GDBGBMU;			//0x6c
	volatile uint32_t GDBGLSPMUX;		//0x70
	volatile uint32_t GDBGLSP;			//0x74
	volatile uint32_t GDBGEPINFO0;		//0x78
	volatile uint32_t GDBGEPINFO1;		//0x7c
	volatile uint32_t GPRTBIMAP_HSLO;	//0x80
	volatile uint32_t GPRTBIMAP_HSHI;	//0x84
	volatile uint32_t GPRTBIMAP_FSLO;	//0x88
	volatile uint32_t GPRTBIMAP_FSHI;	//0x8c
	volatile uint32_t RESERVED1;
	volatile uint32_t GERRINJCTL_1;		//0x94
	volatile uint32_t GERRINJCTL_2;		//0x98
	volatile uint32_t GUCTL2;			//0x9c
	volatile uint32_t RESERVED[24];
	volatile uint32_t GUSB2PHYCFG[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; //0x100
	volatile uint32_t RESERVED2[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX-DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
	volatile uint32_t GUSB2I2CCTL[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; //0x140
	volatile uint32_t RESERVED3[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX-DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
	volatile uint32_t GUSB2PHYACC[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; //0x180
	volatile uint32_t RESERVED4[DWC_USB3_HOST_NUM_U2_ROOT_PORTS_MAX-DWC_USB3_HOST_NUM_U2_ROOT_PORTS];
	volatile uint32_t GUSB3PIPECTL[DWC_USB3_HOST_NUM_U3_ROOT_PORTS]; //0x1c0
	volatile uint32_t RESERVED5[DWC_USB3_HOST_NUM_U3_ROOT_PORTS_MAX-DWC_USB3_HOST_NUM_U3_ROOT_PORTS];
	volatile uint32_t GTXFIFOSIZ[32];	//0x200
	volatile uint32_t GRXFIFOSIZ[32];	//0x280
	struct dwc3_gevnt_reg GEVNT[DWC_USB3_DEVICE_NUM_INT]; //0x300
	volatile uint32_t RESERVED6[(DWC_USB3_DEVICE_NUM_INT_MAX-DWC_USB3_DEVICE_NUM_INT)*sizeof(struct dwc3_gevnt_reg)/sizeof(uint32_t)];
	volatile uint32_t GHWPARAMS8;		//0x500
	volatile uint32_t RESERVED7[2];
	volatile uint32_t GUCTL3;			//0x50c
	volatile uint32_t GTXFIFOPRIDEV;	//0x510
	volatile uint32_t RESERVED8;
	volatile uint32_t GTXFIFOPRIHST;	//0x518
	volatile uint32_t GRXFIFOPRIHST;	//0x51c
	volatile uint32_t GFIFOPRIDBC;		//0x520
	volatile uint32_t GDMAHLRATIO;		//0x524
	volatile uint32_t RESERVED9[2];
	volatile uint32_t GFLADJ;			//0x530
	volatile uint32_t RESERVED10[3];
	volatile uint32_t GUSB2RHBCTL[DWC_USB3_HOST_NUM_U2_ROOT_PORTS]; //0x540
};

struct dwc3_dev_reg {
	volatile uint32_t DCFG;				//0x00
	volatile uint32_t DCTL;				//0x04
	volatile uint32_t DEVTEN;			//0x08
	volatile uint32_t DSTS;				//0x0c
	volatile uint32_t DGCMDPAR;			//0x10
	volatile uint32_t DGCMD;			//0x14
	volatile uint32_t RESERVED1[2];
	volatile uint32_t DALEPENA;			//0x20
	volatile uint32_t RESERVED2[55];
	struct dwc3_dev_ep_reg DEP[DWC_USB3_NUM_EPS]; //0x100
	volatile uint32_t RESERVED3[(DWC_USB3_NUM_EPS_MAX-DWC_USB3_NUM_EPS)*sizeof(struct dwc3_dev_ep_reg)/sizeof(uint32_t)];
	volatile uint32_t DEV_IMOD[DWC_USB3_DEVICE_NUM_INT]; //0x300
};

#define DWC3_XHCI_REGS_START		0x0
#define DWC3_XHCI_REGS_END		0x7fff
#define DWC3_GLOBALS_REGS_START		0xc100
#define DWC3_GLOBALS_REGS_END		0xc6ff
#define DWC3_DEVICE_REGS_START		0xc700
#define DWC3_DEVICE_REGS_END		0xcbff
#define DWC3_OTG_REGS_START		0xcc00
#define DWC3_OTG_REGS_END		0xccf

/* Device Control Register */
#define DWC3_DCTL_RUN_STOP	BIT(31)
#define DWC3_DCTL_CSFTRST	BIT(30)
#define DWC3_DCTL_LSFTRST	BIT(29)

#define EVT_BUF_LENGTH_WORDS 16

/* Device Event Enable Register */
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN	BIT(12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN	BIT(11)
#define DWC3_DEVTEN_CMDCMPLTEN		BIT(10)
#define DWC3_DEVTEN_ERRTICERREN		BIT(9)
#define DWC3_DEVTEN_SOFEN		BIT(7)
#define DWC3_DEVTEN_U3L2L1SUSPEN	BIT(6)
#define DWC3_DEVTEN_HIBERNATIONREQEVTEN	BIT(5)
#define DWC3_DEVTEN_WKUPEVTEN		BIT(4)
#define DWC3_DEVTEN_ULSTCNGEN		BIT(3)
#define DWC3_DEVTEN_CONNECTDONEEN	BIT(2)
#define DWC3_DEVTEN_USBRSTEN		BIT(1)
#define DWC3_DEVTEN_DISCONNEVTEN	BIT(0)

union evt_buf_u {
	struct {
		uint32_t :1,
				phy_ep_num:5,
				ep_evt_type:4,
				:2,
				evt_status:4,
				evt_param:16;
	}depevt;
	struct {
		uint32_t :8,
				dev_evt_type:5,
				:3,
				evt_info:9,
				:7;
	}devt;
	uint32_t dev_ep:1;
};

struct dwc3_trb {
	uint32_t BPTRL;
	uint32_t BPTRH;
	uint32_t BUFSIZ:24,
			PCM1:2,
			SPR:1,
			Reserved1:1,
			TRBSTS:4;
	uint32_t HWO:1,
			LST:1,
			CHN:1, // 为 1 的时候，硬件会自动取下一个 trb (当前 trb + sizeof trb)
			CSP:1,
			TRBCTL:6, // 为 link 的时候，BPTRL 指向一个 trb 的地址， bufsize 为 0
			ISP_IMI:1,
			IOC:1,
			Reserved2:2,
			SID_SOFN:16,
			Reserved3:2;
};

enum ep_evt {
	DEPEVT_XferComplete = 1,
	DEPEVT_XferInProgress = 2,
	DEPEVT_XferNotReady = 3,
	DEPEVT_StreamEvt = 6,
	DEPEVT_EPCmdCmplt = 7,
};

#define DWC3_GEVNTSIZ_INTMASK		BIT(31)
#define DWC3_GEVNTSIZ_SIZE(n)		((n) & 0xffff)
#define DWC3_GEVNT_HANDLER_BUSY BIT(31)
#define USB_BASE  (0x40050000)

#define LS_CLK_SET(base, n)   (*(volatile uint32_t *)((base) + (n)))

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

static void udc_dwc3_isr_handler(void *param)
{
	xhci_isr(param);
}

static struct xhci_hcd hcd;

static volatile struct dwc3_global_reg *dwc3_gbl = (struct dwc3_global_reg *)(USB_BASE + DWC3_GLOBALS_REGS_START);

#include <zephyr/shell/shell.h>

static int xhci_noop_command(const struct shell *sh, size_t argc, char **argv)
{
	if(xhci_cmd_noop(&hcd) == 0)
		printk("cmd exec ok!!\n");
	return 0;
}
SHELL_CMD_REGISTER(xhci_noop_command, NULL, "test command ring and event ring", xhci_noop_command);

static int slot_id = 0;

static int xhci_alloc_dev(const struct shell *sh, size_t argc, char **argv)
{
	slot_id = xhci_alloc_device(&hcd);
	xhci_set_dev_speed(&hcd, slot_id, xhci_get_port_speed(&hcd, 0));
	xhci_cmd_address_device(&hcd, slot_id, 0);
	printk("alloc slot_id = %d.\n", slot_id);
	return 0;
}
SHELL_CMD_REGISTER(xhci_alloc_dev, NULL, "alloc device", xhci_alloc_dev);

#include <stdlib.h>
static int xhci_address_dev(const struct shell *sh, size_t argc, char **argv)
{
	int res = xhci_cmd_address_device(&hcd, slot_id, 1);
	printk("xhci_address_dev res = %d.\n", res);
	return 0;
}
SHELL_CMD_REGISTER(xhci_address_dev, NULL, "address device", xhci_address_dev);

static int xhci_disable_slot(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2)
	{
		printk("%s <slot id> \n", argv[0]);
		return 0;
	}
	xhci_cmd_disable_slot(&hcd, atoi(argv[1]));
	slot_id = 0;
	return 0;
}
SHELL_CMD_REGISTER(xhci_disable_slot, NULL, "disable sloat", xhci_disable_slot);

static int xhci_enable_slot(const struct shell *sh, size_t argc, char **argv)
{
	slot_id = xhci_cmd_enable_slot(&hcd);
	printk("slot_id = %d.\n", slot_id);
	return 0;
}
SHELL_CMD_REGISTER(xhci_enable_slot, NULL, "enable sloat", xhci_enable_slot);

volatile uint32_t connect_done = 0;

static int xhci_get_dev_desc(const struct shell *sh, size_t argc, char **argv)
{
	static __attribute__((aligned(32)))  uint8_t dev_desc[64];
	int xfer_len = 0;
	struct usb_setup_packet setup = {
		.bmRequestType = 0x80,
		.bRequest = 0x6,
		.wValue = 0x100,
		.wIndex = 0x0,
		.wLength = 64,
	};
	xfer_len = xhci_xfer_control(&hcd, slot_id, 0, &setup, dev_desc, 64);
	xhci_cache_invalid(dev_desc, 64);

	if (xfer_len >= 0)
	{
		printk("device desc: ");
		for (int i = 0; i < 64 - xfer_len; i++)
		{
			printk("0x%x ", dev_desc[i]);
		}
		printk("\n");
	}
	return 0;
}
SHELL_CMD_REGISTER(xhci_get_dev_desc, NULL, "get device desc", xhci_get_dev_desc);

static int xhci_get_config_desc(const struct shell *sh, size_t argc, char **argv)
{
	static __attribute__((aligned(32)))  uint8_t config_desc[255];
	int xfer_len = 0;

	struct usb_setup_packet setup = {
		.bmRequestType = 0x80,
		.bRequest = 0x6,
		.wValue = 0x200,
		.wIndex = 0x0,
		.wLength = 255,
	};
	xfer_len = xhci_xfer_control(&hcd, slot_id, 0, &setup, config_desc, 255);
	xhci_cache_invalid(config_desc, 255);

	if (xfer_len >= 0)
	{
		printk("config desc: ");
		for (int i = 0; i < 255 - xfer_len; i++)
		{
			printk("0x%x ", config_desc[i]);
		}
		printk("\n");
	}
	return 0;
}
SHELL_CMD_REGISTER(xhci_get_config_desc, NULL, "xhci_get_config_desc", xhci_get_config_desc);

static size_t usb_string_desc_inplace(uint8_t *buf, size_t buf_len)
{
    if (!buf || buf_len < 2 || buf[1] != 0x03)
		return 0;

    size_t available_bytes = buf_len - 2;
    size_t str_len = available_bytes / 2; // 每个字符占 2 字节 (UTF-16LE)

    for (size_t i = 0; i < str_len; i++)
	{
        buf[i] = buf[2 + i*2];
    }
    buf[str_len] = '\0';

    return str_len;
}

static int xhci_get_string_desc(const struct shell *sh, size_t argc, char **argv)
{
	static __attribute__((aligned(32))) uint8_t string_desc[255];
	int xfer_len = 0;

	if (argc < 2)
	{
		printk("%s <string index> \n", argv[0]);
		return 0;
	}

	struct usb_setup_packet setup = {
		.bmRequestType = 0x80,
		.bRequest = 0x6,
		.wValue = 0x300 | atoi(argv[1]),
		.wIndex = 0x409,
		.wLength = 255,
	};
	xfer_len = xhci_xfer_control(&hcd, slot_id, 0, &setup, string_desc, 255);
	xhci_cache_invalid(string_desc, 255);

	if (xfer_len >= 0)
	{
		usb_string_desc_inplace(string_desc, 255 - xfer_len);
		printk("String Descriptor %d: %s\n", atoi(argv[1]), string_desc);
	}
	return 0;
}
SHELL_CMD_REGISTER(xhci_get_string_desc, NULL, "xhci_get_string_desc", xhci_get_string_desc);

static int xhci_control_transfer(const struct shell *sh, size_t argc, char **argv)
{
	static __attribute__((aligned(32)))  uint8_t dev_desc[64];
	int xfer_len = 0;
	struct usb_setup_packet setup = {
		.bmRequestType = 0x81,
		.bRequest = 0x6,
		.wValue = 0x100,
		.wIndex = 0x0,
		.wLength = 64,
	};

	struct xhci_ep_config ep_config = {
		.ep_addr = 0x82,
		.ep_interval = 3,
		.ep_mps = 64,
		.ep_type = EP_CONTROL_BIDIR
	};
	xhci_cmd_add_endpoint(&hcd, slot_id, &ep_config);

	xfer_len = xhci_xfer_control(&hcd, slot_id, 0x82, &setup, dev_desc, 64);
	xhci_cache_invalid(dev_desc, 64);

	if (xfer_len >= 0)
	{
		printk("device desc: ");
		for (int i = 0; i < 64 - xfer_len; i++)
		{
			printk("0x%x ", dev_desc[i]);
		}
		printk("\n");
	}
	return 0;
}
SHELL_CMD_REGISTER(xhci_control_transfer, NULL, "xhci_control_transfer", xhci_control_transfer);

static int xhci_bulk_in_transfer(const struct shell *sh, size_t argc, char **argv)
{
	/* Allowed to begin on a byte address boundary, However, user may find other alignments, such as 64-byte or 128-byte
	alignments, to be more efficient and provide better performance */

	static __attribute__((aligned(64))) uint8_t dev_desc[128];
	int xfer_len = 0;

	struct xhci_ep_config ep_config = {
		.ep_addr = 0x83,
		.ep_interval = 3,
		.ep_mps = 512,
		.ep_type = EP_BULK_IN
	};
	xhci_cmd_add_endpoint(&hcd, slot_id, &ep_config);
	xfer_len = xhci_xfer_bulk(&hcd, slot_id, 0x83, dev_desc, 128);
	xhci_cache_invalid(dev_desc, 128);

	printk("xfer_len = %d.\n", xfer_len);
	if (xfer_len >= 0)
	{
		printk("IN Data: ");
		for (int i = 0; i < 128 - xfer_len; i++)
		{
			printk("0x%x ", dev_desc[i]);
		}
		printk("\n");
	}

	return 0;
}
SHELL_CMD_REGISTER(xhci_bulk_in_transfer, NULL, "xhci_bulk_in_transfer", xhci_bulk_in_transfer);

static int xhci_bulk_out_transfer(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t dev_desc[64];

	for (int i = 0; i < 64; i++)
	{
		dev_desc[i] = i;
	}

	struct xhci_ep_config ep_config = {
		.ep_addr = 0x4,
		.ep_interval = 3,
		.ep_mps = 512,
		.ep_type = EP_BULK_OUT
	};
	xhci_cmd_add_endpoint(&hcd, slot_id, &ep_config);
	xhci_xfer_bulk(&hcd, slot_id, 0x4, dev_desc, 64);

	return 0;
}
SHELL_CMD_REGISTER(xhci_bulk_out_transfer, NULL, "xhci_bulk_out_transfer", xhci_bulk_out_transfer);

static int xhci_show_dev_ctx(const struct shell *sh, size_t argc, char **argv)
{
	xhci_device_ctx_show(&hcd, slot_id);
	return 0;
}
SHELL_CMD_REGISTER(xhci_show_dev_ctx, NULL, "xhci_show_dev_ctx", xhci_show_dev_ctx);

static int xhci_noop_transfer(const struct shell *sh, size_t argc, char **argv)
{	
	/* test ep0 transfer ring */
	if (xhci_xfer_noop(&hcd, slot_id, 0x80, EP_CONTROL_BIDIR) == 0)
		printk("transfer ring ok!!\n");
	else
		printk("transfer ring error!!\n");
	return 0;
}
SHELL_CMD_REGISTER(xhci_noop_transfer, NULL, "send noop for ep transfer ring", xhci_noop_transfer);

static int xhci_show_portsc(const struct shell *sh, size_t argc, char **argv)
{	
	printk("portsc = 0x%x\n", xhci_get_portsc(&hcd, 0));
	return 0;
}
SHELL_CMD_REGISTER(xhci_show_portsc, NULL, "xhci show portsc", xhci_show_portsc);

static int xhci_reset_port(const struct shell *sh, size_t argc, char **argv)
{	
	xhci_port_reset(&hcd, 0);
	return 0;
}
SHELL_CMD_REGISTER(xhci_reset_port, NULL, "xhci reset port", xhci_reset_port);

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

int main(void)
{
	/* clock gate */
	LS_CLK_SET(0x40062000, 0x10) = BIT(0xd);
	/* reset */
	LS_CLK_SET(0x40062000, 0x18) = BIT(0xd);
	LS_CLK_SET(0x40062000, 0x18) = BIT(0xc);
	LS_CLK_SET(0x40062000, 0x10) = BIT(0xc);

	IRQ_CONNECT(29, 3, udc_dwc3_isr_handler, &hcd, 0);
	irq_enable(29);

	/* enable phy pll */
	*(uint32_t *)0x40058014 = 0x131; // pll_en, refclk div, refclk mode (5M or 12M align)
	*(uint32_t *)0x40058004 |= BIT(8); // xhc_bme enable
	*(uint32_t *)0x40058000 |= BIT(5); // Number of USB 2.0 Ports

	dwc3_gbl->GUSB2PHYCFG[0] = 0x40002407;

	uint32_t gctl = dwc3_gbl->GCTL;
	gctl &= ~(0x3 << 12);
	gctl |= 0x1 << 12; // device mode
	dwc3_gbl->GCTL = gctl;

	/* The disconnect detection voltage threshold is set to 490mv */
	uint8_t value = dwc3_phy_cfg_read(0xb);
	value &= ~BIT(6);
	dwc3_phy_cfg_write(0xb, value);

	xhci_init(&hcd, USB_BASE);

	while (1) {
		k_msleep(2000);
	}

	return 0;
}
