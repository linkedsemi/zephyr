#define DT_DRV_COMPAT linkedsemi_ls_ipmi_kcs

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/kcs.h>
#include <zephyr/drivers/ipmi.h>
#include <string.h>
/* misc. constant */
#define KCS_DUMMY_ZERO  0x0
#define KCS_BUF_SIZE    0x100

/* IPMI 2.0 - Table 9-1, KCS Interface Status Register Bits */
#define KCS_STR_STATE_MASK      GENMASK(7, 6)
#define KCS_STR_STATE_SHIFT     6
#define KCS_STR_CMD_DAT         BIT(3)
#define KCS_STR_SMS_ATN         BIT(2)
#define KCS_STR_IBF             BIT(1)
#define KCS_STR_OBF             BIT(0)

/* IPMI 2.0 - Table 9-2, KCS Interface State Bits */
enum kcs_state {
	KCS_STATE_IDLE,
	KCS_STATE_READ,
	KCS_STATE_WRITE,
	KCS_STATE_ERROR,
	KCS_STATE_NUM
};

/* IPMI 2.0 - Table 9-3, KCS Interface Control Codes */
enum kcs_cmd_code {
	KCS_CMD_GET_STATUS_ABORT        = 0x60,
	KCS_CMD_WRITE_START             = 0x61,
	KCS_CMD_WRITE_END               = 0x62,
	KCS_CMD_READ_BYTE               = 0x68,
	KCS_CMD_NUM
};

/* IPMI 2.0 - Table 9-4, KCS Interface Status Codes */
enum kcs_error_code {
	KCS_NO_ERROR                    = 0x00,
	KCS_ABORTED_BY_COMMAND          = 0x01,
	KCS_ILLEGAL_CONTROL_CODE        = 0x02,
	KCS_LENGTH_ERROR                = 0x06,
	KCS_UNSPECIFIED_ERROR           = 0xff
};

/* IPMI 2.0 - Figure 9. KCS Phase in Transfer Flow Chart */
enum kcs_phase {
	KCS_PHASE_IDLE,

	KCS_PHASE_WRITE_START,
	KCS_PHASE_WRITE_DATA,
	KCS_PHASE_WRITE_END_CMD,
	KCS_PHASE_WRITE_DONE,

	KCS_PHASE_WAIT_READ,
	KCS_PHASE_READ,

	KCS_PHASE_ABORT_ERROR1,
	KCS_PHASE_ABORT_ERROR2,
	KCS_PHASE_ERROR,

	KCS_PHASE_NUM
};

struct ipmi_kcs_ls_config{
    const struct device *kcs_dev;
};

struct ipmi_kcs_ls_data{
	uint8_t ibuf[KCS_BUF_SIZE];
	uint32_t ibuf_idx;
	uint32_t ibuf_avail;

	uint8_t obuf[KCS_BUF_SIZE];
	uint32_t obuf_idx;
	uint32_t obuf_data_sz;

	uint32_t phase;
	uint32_t error;
};

static void kcs_set_state(const struct device *dev,enum kcs_state stat)
{
    kcs_update_status(dev,KCS_STR_STATE_MASK,stat<<KCS_STR_STATE_SHIFT);
}

static void kcs_force_abort(const struct device *dev)
{
    const struct ipmi_kcs_ls_config *cfg = (const struct ipmi_kcs_ls_config *)dev->config;
    struct ipmi_kcs_ls_data *ipmi_data = (struct ipmi_kcs_ls_data *)dev->data;
    uint8_t dummy;
	kcs_set_state(cfg->kcs_dev, KCS_STATE_ERROR);
	kcs_read_data(cfg->kcs_dev,&dummy);
	kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);
	ipmi_data->ibuf_avail = 0;
	ipmi_data->ibuf_idx = 0;
	ipmi_data->phase = KCS_PHASE_ERROR;
}

static void ipmi_kcs_handle_cmd(const struct device *dev)
{
    const struct ipmi_kcs_ls_config *cfg = (const struct ipmi_kcs_ls_config *)dev->config;
    struct ipmi_kcs_ls_data *ipmi_data = (struct ipmi_kcs_ls_data *)dev->data;
	uint8_t cmd;

	kcs_set_state(cfg->kcs_dev, KCS_STATE_WRITE);
	kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);

	kcs_read_data(cfg->kcs_dev,&cmd);
	switch (cmd) {
	case KCS_CMD_WRITE_START:
		ipmi_data->phase = KCS_PHASE_WRITE_START;
		ipmi_data->error = KCS_NO_ERROR;
		break;

	case KCS_CMD_WRITE_END:
		if (ipmi_data->phase != KCS_PHASE_WRITE_DATA) {
			kcs_force_abort(dev);
			break;
		}
		ipmi_data->phase = KCS_PHASE_WRITE_END_CMD;
		break;

	case KCS_CMD_GET_STATUS_ABORT:
		if (ipmi_data->error == KCS_NO_ERROR) {
			ipmi_data->error = KCS_ABORTED_BY_COMMAND;
		}

		ipmi_data->phase = KCS_PHASE_ABORT_ERROR1;
		break;

	default:
		kcs_force_abort(dev);
		ipmi_data->error = KCS_ILLEGAL_CONTROL_CODE;
		break;
	}
}

static void ipmi_kcs_handle_data(const struct device *dev)
{
    const struct ipmi_kcs_ls_config *cfg = (const struct ipmi_kcs_ls_config *)dev->config;
    struct ipmi_kcs_ls_data *ipmi_data = (struct ipmi_kcs_ls_data *)dev->data;
	uint8_t data;

	switch (ipmi_data->phase) {
	case KCS_PHASE_WRITE_START:
		ipmi_data->phase = KCS_PHASE_WRITE_DATA;
	/* fall through */
	case KCS_PHASE_WRITE_DATA:
		if (ipmi_data->ibuf_idx < KCS_BUF_SIZE) {
			kcs_set_state(cfg->kcs_dev, KCS_STATE_WRITE);
			kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);
			kcs_read_data(cfg->kcs_dev,&ipmi_data->ibuf[ipmi_data->ibuf_idx]);
			ipmi_data->ibuf_idx++;
		} else {
			kcs_force_abort(dev);
			ipmi_data->error = KCS_LENGTH_ERROR;
		}
		break;

	case KCS_PHASE_WRITE_END_CMD:
		if (ipmi_data->ibuf_idx < KCS_BUF_SIZE) {
			kcs_set_state(cfg->kcs_dev, KCS_STATE_READ);
			kcs_read_data(cfg->kcs_dev,&ipmi_data->ibuf[ipmi_data->ibuf_idx]);
			ipmi_data->ibuf_idx++;
			ipmi_data->ibuf_avail = 1;
			ipmi_data->phase = KCS_PHASE_WRITE_DONE;
		} else {
			kcs_force_abort(cfg->kcs_dev);
			ipmi_data->error = KCS_LENGTH_ERROR;
		}
		break;

	case KCS_PHASE_READ:
		if (ipmi_data->obuf_idx == ipmi_data->obuf_data_sz) {
			kcs_set_state(cfg->kcs_dev, KCS_STATE_IDLE);
		}

		kcs_read_data(cfg->kcs_dev,&data);
		if (data != KCS_CMD_READ_BYTE) {
			kcs_set_state(cfg->kcs_dev, KCS_STATE_ERROR);
			kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);
			break;
		}

		if (ipmi_data->obuf_idx == ipmi_data->obuf_data_sz) {
			kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);
			ipmi_data->phase = KCS_PHASE_IDLE;
			break;
		}

		kcs_write_data(cfg->kcs_dev, ipmi_data->obuf[ipmi_data->obuf_idx]);
		ipmi_data->obuf_idx++;
		break;

	case KCS_PHASE_ABORT_ERROR1:
		kcs_set_state(cfg->kcs_dev, KCS_STATE_READ);
		kcs_read_data(cfg->kcs_dev,&data);
		kcs_write_data(cfg->kcs_dev, ipmi_data->error);
		ipmi_data->phase = KCS_PHASE_ABORT_ERROR2;
		break;

	case KCS_PHASE_ABORT_ERROR2:
		kcs_set_state(cfg->kcs_dev, KCS_STATE_IDLE);
		kcs_read_data(cfg->kcs_dev,&data);
		kcs_write_data(cfg->kcs_dev, KCS_DUMMY_ZERO);
		ipmi_data->phase = KCS_PHASE_IDLE;
		break;

	default:
		kcs_force_abort(dev);
		break;
	}

}

static void kcs_ibf_cb(const struct device *dev,void *param)
{
    const struct device *ipmi_dev = param;
    uint8_t status;
    kcs_read_status(dev,&status);
    if(status & KCS_CMD_DAT)
    {
        ipmi_kcs_handle_cmd(ipmi_dev);
    }else
    {
        ipmi_kcs_handle_data(ipmi_dev);
    }
}

static int ipmi_kcs_ls_init(const struct device *dev)
{
    const struct ipmi_kcs_ls_config *cfg = (const struct ipmi_kcs_ls_config *)dev->config;
    struct ipmi_kcs_ls_data *data = (struct ipmi_kcs_ls_data *)dev->data;
	data->ibuf_idx = 0;
	data->ibuf_avail = 0;

	data->obuf_idx = 0;
	data->obuf_data_sz = 0;

	data->phase = KCS_PHASE_IDLE;
    kcs_set_ibf_callback(cfg->kcs_dev,kcs_ibf_cb,(void *)dev);
    return 0;
}

static int ipmi_kcs_ls_read(const struct device *dev,uint8_t *data,uint32_t size)
{
	int ret;
	struct ipmi_kcs_ls_data *ipmi_data = (struct ipmi_kcs_ls_data *)dev->data;

	if (!ipmi_data || !data) {
		return -EINVAL;
	}

	if (size < ipmi_data->ibuf_idx) {
		return -ENOSPC;
	}

	if (!ipmi_data->ibuf_avail) {
		return -ENODATA;
	}

	if (ipmi_data->phase != KCS_PHASE_WRITE_DONE) {
		// kcs_force_abort(dev);
		return -EPERM;
	}

	memcpy(data, ipmi_data->ibuf, ipmi_data->ibuf_idx);
	ret = ipmi_data->ibuf_idx;

	ipmi_data->phase = KCS_PHASE_WAIT_READ;
	ipmi_data->ibuf_avail = 0;
	ipmi_data->ibuf_idx = 0;

	return ret;

}

static int ipmi_kcs_ls_write(const struct device *dev,uint8_t *data,uint32_t size)
{
    const struct ipmi_kcs_ls_config *cfg = (struct ipmi_kcs_ls_config *)dev->config;
    struct ipmi_kcs_ls_data *ipmi_data = (struct ipmi_kcs_ls_data *)dev->data;

	/* a minimum response size is 3: netfn + cmd + cmplt_code */
	if (size < 3 || size > KCS_BUF_SIZE) {
		return -EINVAL;
	}

	if (ipmi_data->phase != KCS_PHASE_WAIT_READ) {
		return -EPERM;
	}

	ipmi_data->phase = KCS_PHASE_READ;
	ipmi_data->obuf_idx = 1;
	ipmi_data->obuf_data_sz = size;
	memcpy(ipmi_data->obuf, data, size);
	kcs_write_data(cfg->kcs_dev, ipmi_data->obuf[0]);
	return size;
}

static const struct ipmi_driver_api ipmi_kcs_ls_api = {
    .read = ipmi_kcs_ls_read,
    .write = ipmi_kcs_ls_write,
};

#define IPMI_KCS_LS_INIT(n)						     \
	static struct ipmi_kcs_ls_data ipmi_kcs_ls_data_##n;		     \
	static const struct ipmi_kcs_ls_config ipmi_kcs_ls_config_##n = {	     \
        .kcs_dev = DEVICE_DT_GET(DT_INST_PROP(n,kcs_chan)),\
	};								     \
									     \
	DEVICE_DT_INST_DEFINE(n,					     \
			      ipmi_kcs_ls_init,				     \
			      NULL,					     \
			      &ipmi_kcs_ls_data_##n,			     \
			      &ipmi_kcs_ls_config_##n,			     \
			      POST_KERNEL,				     \
			      CONFIG_IPMI_INIT_PRIORITY,	     \
			      &ipmi_kcs_ls_api);

DT_INST_FOREACH_STATUS_OKAY(IPMI_KCS_LS_INIT)