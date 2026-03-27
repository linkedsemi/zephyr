#define DT_DRV_COMPAT linkedsemi_ls_espi_bmc
#include <zephyr/drivers/espi.h>
#include "espi_utils.h"
#include "espi_lpc_common.h"

struct espi_bmc_ls_config {
    struct host_bmc_msg_exch hb_exch;
    struct espi_sysevent_base *sysevent_base;
};

struct espi_bmc_ls_data {
	sys_slist_t callbacks;
};

void bmc_espi_rx_callback(const struct device *dev,void *msg)
{
	struct espi_vwire_msg *vw_msg = msg;
    struct espi_bmc_ls_data *data = dev->data;
    struct espi_event evt = {
        .evt_type = ESPI_BUS_EVENT_VWIRE_RECEIVED,
        .evt_details = vw_msg->vw_idx,
        .evt_data = 0,
    };
    espi_send_callbacks(&data->callbacks,dev,evt);
}

static int espi_ls_configure(const struct device *dev, struct espi_cfg *cfg)
{
    return 0;
}

static bool espi_ls_get_channel_status(const struct device *dev,enum espi_channel ch)
{
    return true;
}

static int espi_ls_read_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    switch(op)
    {
    case E8042_OBF_HAS_CHAR:

    break;
    case E8042_IBF_HAS_CHAR:

    break;
    case E8042_READ_KB_STS:

    break;
    case EACPI_OBF_HAS_CHAR:

    break;
    case EACPI_IBF_HAS_CHAR:

    break;
    case EACPI_READ_STS:

    break;
    default:
        __ASSERT(0,"illegal espi read lpc request op\n");
    break;
    }
    return 0;
}

static int espi_ls_write_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{

    switch(op)
    {
    case E8042_WRITE_KB_CHAR:

    break;
    case E8042_WRITE_MB_CHAR:

    break;
    case E8042_RESUME_IRQ:

    break;
    case E8042_PAUSE_IRQ:

    break;
    case E8042_CLEAR_OBF:

    break;
    case E8042_SET_FLAG:

    break;
    case E8042_CLEAR_FLAG:

    break;
    default:
        __ASSERT(0,"illegal espi write lpc request op\n");
    break;
    }
    return 0;
}

static int espi_ls_send_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t level)
{
    switch(vw)
    {
    case ESPI_VWIRE_SIGNAL_PME:

    break;
    case ESPI_VWIRE_SIGNAL_WAKE:

    break;
    case ESPI_VWIRE_SIGNAL_OOB_RST_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS:

    break;
    case ESPI_VWIRE_SIGNAL_ERR_NON_FATAL:

    break;
    case ESPI_VWIRE_SIGNAL_ERR_FATAL:

    break;
    case ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE:

    break;
    case ESPI_VWIRE_SIGNAL_HOST_RST_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_RST_CPU_INIT:

    break;
    case ESPI_VWIRE_SIGNAL_SMI:

    break;
    case ESPI_VWIRE_SIGNAL_SCI:

    break;
    case ESPI_VWIRE_SIGNAL_DNX_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_ACK:

    break;
    default:
        __ASSERT(0,"illegal espi send vw\n");
    break;
    }
    return 0;
}

static int espi_ls_receive_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t *level)
{
    struct espi_bmc_ls_config *cfg = dev->config;
    switch(vw)
    {
    case ESPI_VWIRE_SIGNAL_SLP_S3:
        *level = (cfg->sysevent_base->s02_ms & ESPI_SYS_EVT_2_SLP_S3_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_SLP_S4:
        *level = (cfg->sysevent_base->s02_ms & ESPI_SYS_EVT_2_SLP_S4_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_SLP_S5:
        *level = (cfg->sysevent_base->s02_ms & ESPI_SYS_EVT_2_SLP_S5_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_OOB_RST_WARN:
        *level = (cfg->sysevent_base->s03_ms & ESPI_SYS_EVT_3_OOB_RST_WARN) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_PLTRST:
        *level = (cfg->sysevent_base->s03_ms & ESPI_SYS_EVT_3_PLTRST_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_SUS_STAT:
        *level = (cfg->sysevent_base->s03_ms & ESPI_SYS_EVT_3_SUS_STAT_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_NMIOUT:
        *level = (cfg->sysevent_base->s07_ms & ESPI_SYS_EVT_7_NMIOUT_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_SMIOUT:
        *level = (cfg->sysevent_base->s07_ms & ESPI_SYS_EVT_7_SMIOUT_N) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:
        *level = (cfg->sysevent_base->s07_ms & ESPI_SYS_EVT_7_HOST_RST_WARN) ? 1:0;
    break;
    case ESPI_VWIRE_SIGNAL_SLP_A:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_PWRDN_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_WARN:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_WLAN:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_LAN:

    break;
    case ESPI_VWIRE_SIGNAL_HOST_C10:

    break;
    case ESPI_VWIRE_SIGNAL_DNX_WARN:

    break;
    default:
        __ASSERT(0,"illegal espi receive vw\n");
    break;
    }
    return 0;
}

static int espi_ls_send_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_receive_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_read(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_write(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_erase(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_manage_callback(const struct device *dev,struct espi_callback *callback,bool set)
{
    struct espi_bmc_ls_data *data = dev->data;
    return espi_manage_callback(&data->callbacks,callback,set);
}

static int espi_bmc_ls_init(const struct device *dev)
{
    const struct espi_bmc_ls_config *cfg = dev->config;
	struct espi_bmc_ls_data *const data = dev->data;
	sys_slist_init(&data->callbacks);
    host_bmc_msg_exch_init(&cfg->hb_exch);
    return 0;
}

static const struct espi_driver_api espi_bmc_ls_driver_api = {
    .config = espi_ls_configure,
    .get_channel_status = espi_ls_get_channel_status,
    .read_lpc_request = espi_ls_read_lpc_request,
    .write_lpc_request = espi_ls_write_lpc_request,
    .send_vwire = espi_ls_send_vwire,
    .receive_vwire = espi_ls_receive_vwire,
    .send_oob = espi_ls_send_oob,
    .receive_oob = espi_ls_receive_oob,
    .flash_read = espi_ls_flash_read,
    .flash_write = espi_ls_flash_write,
    .flash_erase = espi_ls_flash_erase,
    .manage_callback = espi_ls_manage_callback,
};

#define LS_ESPI_BMC_INIT(idx)\
    static struct espi_bmc_ls_data espi_bmc_ls_data_##idx;\
    static const struct espi_bmc_ls_config espi_bmc_ls_cfg_##idx = {\
        .hb_exch = HOST_BMC_MSG_EXCH_INIT(idx,bmc_espi_rx_callback,host_espi_rx_callback),\
        .sysevent_base = (struct espi_sysevent_base *)DT_INST_PROP(idx,sysevent_base),\
    };\
    DEVICE_DT_INST_DEFINE(idx,espi_bmc_ls_init,NULL,&espi_bmc_ls_data_##idx,\
        &espi_bmc_ls_cfg_##idx,PRE_KERNEL_2,CONFIG_ESPI_INIT_PRIORITY,\
        &espi_bmc_ls_driver_api);


DT_INST_FOREACH_STATUS_OKAY(LS_ESPI_BMC_INIT)