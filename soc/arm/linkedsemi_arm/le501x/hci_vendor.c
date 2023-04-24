#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/bluetooth/hci_driver.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <version.h>
#include <ls_hal_flash.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ble_hci_vs);

#define BT_HCI_VS_HW_PLAT 0 
#define BT_HCI_VS_HW_VAR  0

#define BLE_DEVICE_ADDR_OFFSET     0x30
#define BLE_ADDR_LEN    6

static uint16_t _opcode;

static struct net_buf *cmd_status(uint8_t status)
{
	return bt_hci_cmd_status_create(_opcode, status);
}

static void *hci_cmd_complete(struct net_buf **buf, uint8_t plen)
{
	*buf = bt_hci_cmd_complete_create(_opcode, plen);

	return net_buf_add(*buf, plen);
}

static void vs_read_version_info(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_vs_read_version_info *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	rp->hw_platform = sys_cpu_to_le16(BT_HCI_VS_HW_PLAT);
	rp->hw_variant = sys_cpu_to_le16(BT_HCI_VS_HW_VAR);

	rp->fw_variant = 0U;
	rp->fw_version = (KERNEL_VERSION_MAJOR & 0xff);
	rp->fw_revision = sys_cpu_to_le16(KERNEL_VERSION_MINOR);
	rp->fw_build = sys_cpu_to_le32(KERNEL_PATCHLEVEL & 0xffff);
}

static void vs_read_supported_commands(struct net_buf *buf,
				       struct net_buf **evt)
{
	struct bt_hci_rp_vs_read_supported_commands *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	(void)memset(&rp->commands[0], 0, sizeof(rp->commands));

	/* Set Version Information, Supported Commands, Supported Features. */
	rp->commands[0] |= BIT(0) | BIT(1) | BIT(2);
#if defined(CONFIG_BT_HCI_VS_EXT)
	/* Write BD_ADDR, Read Build Info */
	rp->commands[0] |= BIT(5) | BIT(7);
	/* Read Static Addresses */
	rp->commands[1] |= BIT(0);
#endif /* CONFIG_BT_HCI_VS_EXT */
}

static void vs_read_supported_features(struct net_buf *buf,
				       struct net_buf **evt)
{
	struct bt_hci_rp_vs_read_supported_features *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp));

	rp->status = 0x00;
	(void)memset(&rp->features[0], 0x00, sizeof(rp->features));
}

static uint8_t hci_vendor_read_static_addr(struct bt_hci_vs_static_addr addrs[],
					uint8_t size)
{
	ARG_UNUSED(size);

	BT_ADDR_SET_STATIC(&addrs[0].bdaddr);
    hal_flash_fast_read(BLE_DEVICE_ADDR_OFFSET, addrs[0].bdaddr.val, BLE_ADDR_LEN);

	return 1;
}

/* If Zephyr VS HCI commands are not enabled provide this functionality directly
 */
#if !defined(CONFIG_BT_HCI_VS_EXT)
static uint8_t bt_read_static_addr(struct bt_hci_vs_static_addr addrs[], uint8_t size)
{
	return hci_vendor_read_static_addr(addrs, size);
}
#endif /* !defined(CONFIG_BT_HCI_VS_EXT) */

static void vs_read_static_addrs(struct net_buf *buf, struct net_buf **evt)
{
	struct bt_hci_rp_vs_read_static_addrs *rp;

	rp = hci_cmd_complete(evt, sizeof(*rp) +
				   sizeof(struct bt_hci_vs_static_addr));
	rp->status = 0x00;
	rp->num_addrs = hci_vendor_read_static_addr(rp->a, 1);
}

static int hci_vendor_cmd_handle_common(uint16_t ocf, struct net_buf *cmd,
				 struct net_buf **evt)
{
	switch (ocf) {
	case BT_OCF(BT_HCI_OP_VS_READ_VERSION_INFO):
		vs_read_version_info(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_VS_READ_SUPPORTED_COMMANDS):
		vs_read_supported_commands(cmd, evt);
		break;

	case BT_OCF(BT_HCI_OP_VS_READ_SUPPORTED_FEATURES):
		vs_read_supported_features(cmd, evt);
		break;

#if defined(CONFIG_BT_HCI_VS_EXT)
	case BT_OCF(BT_HCI_OP_VS_READ_STATIC_ADDRS):
		vs_read_static_addrs(cmd, evt);
		break;
#endif /* CONFIG_BT_HCI_VS_EXT */

	default:
		return -EINVAL;
	}

	return 0;
}

static struct net_buf *hci_cmd_handle(struct net_buf *cmd)
{
	struct bt_hci_cmd_hdr *chdr;
	struct net_buf *evt = NULL;
	uint16_t ocf;
	int err;

	if (cmd->len < sizeof(*chdr)) {
		LOG_ERR("No HCI Command header");
		return NULL;
	}

	chdr = net_buf_pull_mem(cmd, sizeof(*chdr));
	if (cmd->len < chdr->param_len) {
		LOG_ERR("Invalid HCI CMD packet length");
		return NULL;
	}

	/* store in a global for later CC/CS event creation */
	_opcode = sys_le16_to_cpu(chdr->opcode);

	ocf = BT_OCF(_opcode);

	switch (BT_OGF(_opcode)) {
	case BT_OGF_VS:
		err = hci_vendor_cmd_handle_common(ocf, cmd, &evt);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err == -EINVAL) {
		evt = cmd_status(BT_HCI_ERR_UNKNOWN_CMD);
	}

	return evt;
}

int cmd_handle(struct net_buf *buf)
{
	struct net_buf *evt;
	evt = hci_cmd_handle(buf);
	if (evt) {
		LOG_DBG("Replying with event of %u bytes", evt->len);
		bt_recv_prio(evt);
	}

	return 0;
}