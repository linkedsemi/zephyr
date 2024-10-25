/*
 * Copyright (c) 2024 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_jtag

#include <stdlib.h>
#include <errno.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util_macro.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(jtag_ls, LOG_LEVEL_DBG);
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <reg_mjtag_type.h>
#include "jtag_ls.h"
#include "ls_soc_gpio.h"
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif

#define JTAG_WRITE_DATA_ENABLE	(1)
#define JTAG_START_SEND_DATA	(0)

struct jtag_info {
	enum jtag_ls_tap_states tap_state;
};

static struct jtag_info gjtag;

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct jtag_ls_config {
    irq_cfg_func_t irq_config_func;
	/* jtag controller base address */
	struct reg_mjtag_t *reg;
    uint8_t irq_num;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
};

struct jtag_ls_data {
    struct k_sem trans_sync_sem;
    struct k_sem lock;
	uint8_t *tdo_value;
	uint8_t pinmux[4];
	uint8_t tck_dev;
	uint8_t tdi_dev;
	uint8_t tdo_dev;
	uint8_t tms_dev;
};

void ls_jtag_isr(void *arg)
{
    struct device *dev = (struct device *) arg;
	const struct jtag_ls_config *config = dev->config;
	struct jtag_ls_data *data = dev->data;
    struct reg_mjtag_t *const reg = config->reg;

	if(reg->INTR_STT & MJTAG_INTR_RX_FIFO_ALMOST_FULL_MASK)
	{
		if (data->tdo_value) {
			*data->tdo_value = reg->TDO;
			data->tdo_value++;
		} else {
			reg->TDO;
		}

		reg->INTR_CLR = MJTAG_INTR_RX_FIFO_ALMOST_FULL_MASK;
		reg->INTR_MSK = 0x0;
		k_sem_give(&data->trans_sync_sem);
	}
}

static int jtag_ls_init(const struct device *dev)
{
    const struct jtag_ls_config *const config = dev->config;
    struct jtag_ls_data *const data = dev->data;
    struct reg_mjtag_t *const reg = config->reg;

#if defined(CONFIG_PINCTRL)
    int ret;
	const struct pinctrl_state *state;
	const pinctrl_soc_pin_t *pins;
    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }

	ret = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_DEFAULT, &state);
	if (ret < 0) {
		return ret;
	}
	pins = state->pins;
	for(int i = 0; i< state->pin_cnt; i++)
	{
		data->pinmux[i] = (*pins++).pinmux_un.field.pin;
	}
	data->tck_dev = data->pinmux[0];
	data->tms_dev = data->pinmux[1];
	data->tdi_dev = data->pinmux[2];
	data->tdo_dev = data->pinmux[3];
#endif

    k_sem_init(&data->trans_sync_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&data->lock, 1, 1);
    config->irq_config_func(dev);
    reg->INTR_MSK = 0x0;
    reg->INTR_CLR = MJTAG_INTR_ALL_MASK;
    reg->CTRL = 0x1010008;
    reg->TDO_FT = 0;
	/* Test-Logic Reset state */
	gjtag.tap_state = LS_TAP_RESET;
    return 0;
}

static enum tap_state jtag_ls_covert_tap_state(enum tap_state state)
{
	enum jtag_ls_tap_states ls_state = LS_TAP_CURRENT_STATE;

	switch(state) {
		case TAP_INVALID:
			ls_state = LS_TAP_CURRENT_STATE;
			break;
		case TAP_DREXIT2:
			ls_state = LS_TAP_EXIT2_DR;
			break;
		case TAP_DREXIT1:
			ls_state = LS_TAP_EXIT1_DR;
			break;
		case TAP_DRSHIFT:
			ls_state = LS_TAP_SHIFT_DR;
			break;
		case TAP_DRPAUSE:
			ls_state = LS_TAP_PAUSE_DR;
			break;
		case TAP_IRSELECT:
			ls_state = LS_TAP_SELECT_IR;
			break;
		case TAP_DRUPDATE:
			ls_state = LS_TAP_UPDATE_DR;
			break;
		case TAP_DRCAPTURE:
			ls_state = LS_TAP_CAPTURE_DR;
			break;
		case TAP_DRSELECT:
			ls_state = LS_TAP_SELECT_DR;
			break;
		case TAP_IREXIT2:
			ls_state = LS_TAP_EXIT2_IR;
			break;
		case TAP_IREXIT1:
			ls_state = LS_TAP_EXIT1_IR;
			break;
		case TAP_IRSHIFT:
			ls_state = LS_TAP_SHIFT_IR;
			break;
		case TAP_IRPAUSE:
			ls_state = LS_TAP_PAUSE_IR;
			break;
		case TAP_IDLE:
			ls_state = LS_TAP_IDLE;
			break;
		case TAP_IRUPDATE:
			ls_state = LS_TAP_UPDATE_IR;
			break;
		case TAP_IRCAPTURE:
			ls_state = LS_TAP_CAPTURE_IR;
			break;
		case TAP_RESET:
			ls_state = LS_TAP_RESET;
			break;
		default:
			ls_state = LS_TAP_CURRENT_STATE;
			break;
	}

	return ls_state;
}

static void jtag_ls_set_tap_state(const struct device *dev, enum jtag_ls_tap_states from, enum jtag_ls_tap_states to)
{
    const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	struct reg_mjtag_t *const reg = config->reg;
	uint8_t tmsbits;
	uint8_t count;

	k_sem_init(&data->trans_sync_sem, 0, K_SEM_MAX_LIMIT);

	reg->INTR_MSK = MJTAG_INTR_RX_FIFO_ALMOST_FULL_MASK;

    if (from == to)
    {
        return;
    }

    if(from == LS_TAP_CURRENT_STATE)
    {
        from = gjtag.tap_state;
    }

    if (from > LS_TAP_CURRENT_STATE || to > LS_TAP_CURRENT_STATE)
    {
	    return;
    }

	/* Reset to Test-Logic Reset state:
	 * Also notice that in whatever state the TAP controller may be at,
	 * it will goes back to this state if TMS is set to 1 for 5 consecutive TCK cycles.
	 */
    if(to == LS_TAP_RESET)
    {
        reg->FIFO_WREN = JTAG_WRITE_DATA_ENABLE;
        reg->TMS = 0x1ff;
        reg->TDI = 0x0;
        reg->DW = 0x9;
        reg->FIFO_PUSH = MJTAG_FIFO_PUSH_ALL_MASK;
        reg->FIFO_WREN = JTAG_START_SEND_DATA;
		k_sem_take(&data->trans_sync_sem, K_FOREVER);
        gjtag.tap_state = LS_TAP_RESET;
        return;
    }

    tmsbits = _tms_cycle_lookup[from][to].tms_bits;
	count = _tms_cycle_lookup[from][to].count;

	if (count == 0) {
		return;
	}

    reg->FIFO_WREN = JTAG_WRITE_DATA_ENABLE;
    reg->TMS = tmsbits;
    reg->TDI = 0x0;
    reg->DW = count;
    reg->FIFO_PUSH = MJTAG_FIFO_PUSH_ALL_MASK;
    reg->FIFO_WREN = JTAG_START_SEND_DATA;
	k_sem_take(&data->trans_sync_sem, K_FOREVER);
    gjtag.tap_state = to;
}

static enum tap_state jtag_ls_covert_ls_tap_state(enum jtag_ls_tap_states ls_tap_state)
{
    enum tap_state state = TAP_INVALID;
    switch(ls_tap_state) {
		case LS_TAP_RESET:
			state = TAP_RESET;
			break;
		case LS_TAP_IDLE:
			state = TAP_IDLE;
			break;
		case LS_TAP_SELECT_DR:
			state = TAP_DRSELECT;
			break;
		case LS_TAP_CAPTURE_DR:
			state = TAP_DRCAPTURE;
			break;
		case LS_TAP_SHIFT_DR:
			state = TAP_DRSHIFT;
			break;
		case LS_TAP_EXIT1_DR:
			state = TAP_DREXIT1;
			break;
		case LS_TAP_PAUSE_DR:
			state = TAP_DRPAUSE;
			break;
		case LS_TAP_EXIT2_DR:
			state = TAP_DREXIT2;
			break;
		case LS_TAP_UPDATE_DR:
			state = TAP_DRUPDATE;
			break;
		case LS_TAP_SELECT_IR:
			state = TAP_IRSELECT;
			break;
		case LS_TAP_CAPTURE_IR:
			state = TAP_IRCAPTURE;
			break;
		case LS_TAP_SHIFT_IR:
			state = TAP_IRSHIFT;
			break;
		case LS_TAP_EXIT1_IR:
			state = TAP_IREXIT1;
			break;
		case LS_TAP_PAUSE_IR:
			state = TAP_IRPAUSE;
			break;
		case LS_TAP_EXIT2_IR:
			state = TAP_IREXIT2;
			break;
		case LS_TAP_UPDATE_IR:
			state = TAP_IRUPDATE;
			break;
		default:
			state = TAP_INVALID;
			break;
	}
	
	return state;
}

static enum jtag_ls_tap_states jtag_ls_get_tap_state(void)
{
    return gjtag.tap_state;
}

int jtag_ls_freq_get(const struct device *dev, uint32_t *freq)
{
	return -ENOTSUP;
}

int jtag_ls_freq_set(const struct device *dev, uint32_t freq)
{
	return -ENOTSUP;
}

int jtag_ls_tap_get(const struct device *dev, enum tap_state *state)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }
	k_sem_take(&data->lock, K_FOREVER);

	*state = jtag_ls_covert_ls_tap_state(jtag_ls_get_tap_state());

	k_sem_give(&data->lock);

	return 0;
}

static int jtag_ls_tap_set(const struct device *dev, enum tap_state state)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	enum jtag_ls_tap_states end_state = LS_TAP_CURRENT_STATE;
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }

	k_sem_take(&data->lock, K_FOREVER);

	end_state = jtag_ls_covert_tap_state(state);

    if(end_state == LS_TAP_CURRENT_STATE)
	{
 		return -EINVAL;
	}

	jtag_ls_set_tap_state(dev, LS_TAP_CURRENT_STATE, end_state);

 	k_sem_give(&data->lock);

	return 0;
}

static int jtag_ls_tck_run(const struct device *dev, uint32_t run_count)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	struct reg_mjtag_t *const reg = config->reg;
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }

	k_sem_init(&data->trans_sync_sem, 0, K_SEM_MAX_LIMIT);

	reg->INTR_MSK = MJTAG_INTR_RX_FIFO_ALMOST_FULL_MASK;
	reg->FIFO_WREN = JTAG_WRITE_DATA_ENABLE;
	reg->TMS = 0x0;
	reg->TDI = 0x0;
	reg->DW = 0x9;
	reg->FIFO_PUSH = MJTAG_FIFO_PUSH_ALL_MASK;
	reg->FIFO_WREN = JTAG_START_SEND_DATA;

	k_sem_take(&data->trans_sync_sem, K_FOREVER);
	return 0;
}

static void jtag_ls_xfer_gpio(const struct device *dev, uint32_t out_bits_len, const uint8_t *out_data,
			    uint32_t in_bits_len, uint8_t *in_data, uint8_t last_data)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	struct reg_mjtag_t *const reg = config->reg;
	volatile uint32_t bits_len, count;
	data->tdo_value = in_data;

	bits_len = (out_bits_len > in_bits_len) ? out_bits_len : in_bits_len;
	count = (bits_len % 8) ? (bits_len / 8 + 1) : (bits_len / 8);
	
	for(uint32_t i = 0; i < count; i++)
	{
		k_sem_init(&data->trans_sync_sem, 0, K_SEM_MAX_LIMIT);

		reg->FIFO_WREN = JTAG_WRITE_DATA_ENABLE;

		if (bits_len % 8) {
			if (i == count - 1) {
				reg->TMS = BIT(bits_len % 8 - 1);
				reg->DW = bits_len % 8;
			} else {
				reg->TMS = 0x0;
				reg->DW = 0x8;
			}
		} else {
			if (i == count - 1) {
				reg->TMS = BIT(7);
			} else {
				reg->TMS = 0x0;
			}
			reg->DW = 0x8;
		}

		reg->TDI = *out_data++;
		reg->FIFO_PUSH = MJTAG_FIFO_PUSH_ALL_MASK;
    	reg->FIFO_WREN = JTAG_START_SEND_DATA;
		reg->INTR_MSK = MJTAG_INTR_RX_FIFO_ALMOST_FULL_MASK;

		k_sem_take(&data->trans_sync_sem, K_FOREVER);
	}
}

static void jtag_ls_readwrite_scan(const struct device *dev, int bits_len, const uint8_t *out_data,
					uint8_t *in_data, enum jtag_ls_tap_states end_state)
{
	uint32_t remain_bits = bits_len;

	if (remain_bits) {
		if (end_state != LS_TAP_SHIFT_DR && end_state != LS_TAP_SHIFT_IR &&
		    end_state != LS_TAP_CURRENT_STATE) {
			jtag_ls_xfer_gpio(dev, remain_bits, out_data,
					       remain_bits, in_data, 1);
			gjtag.tap_state = (gjtag.tap_state == LS_TAP_SHIFT_DR) ?
					  LS_TAP_EXIT1_DR : LS_TAP_EXIT1_IR;
		} else {
			jtag_ls_xfer_gpio(dev, remain_bits, out_data,
					       remain_bits, in_data, 0);
		}
	}

	jtag_ls_set_tap_state(dev, LS_TAP_CURRENT_STATE, end_state);
}

static void jtag_ls_ir_scan(const struct device *dev, int bits_len, const uint8_t *out_data,
				uint8_t *in_data, enum jtag_ls_tap_states end_state)
{
	if (bits_len == 0) {
		return;
	}

	jtag_ls_set_tap_state(dev, LS_TAP_CURRENT_STATE, LS_TAP_SHIFT_IR);

	jtag_ls_readwrite_scan(dev, bits_len, out_data, in_data, end_state);
}

static void jtag_ls_dr_scan(const struct device *dev, int bits_len, const uint8_t *out_data,
		uint8_t *in_data, enum jtag_ls_tap_states end_state)
{
	if (bits_len == 0) {
		return;
	}

	jtag_ls_set_tap_state(dev, LS_TAP_CURRENT_STATE, LS_TAP_SHIFT_DR);

	jtag_ls_readwrite_scan(dev, bits_len, out_data, in_data, end_state);
}

static int jtag_ls_xfer(const struct device *dev, struct scan_command_s *scan)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	struct scan_field_s *fields;
	enum jtag_ls_tap_states ls_state = LS_TAP_CURRENT_STATE;

	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }

	k_sem_take(&data->lock, K_FOREVER);

	fields = &scan->fields;
	ls_state = jtag_ls_covert_tap_state(scan->end_state);

	if (ls_state == LS_TAP_CURRENT_STATE)
		return -1;

	if (scan->ir_scan) {
		jtag_ls_ir_scan(dev, fields->num_bits,
				fields->out_value, fields->in_value, ls_state);
	} else {
		jtag_ls_dr_scan(dev, fields->num_bits,
				fields->out_value, fields->in_value, ls_state);
	}

	k_sem_give(&data->lock);

	return 0;
}

static int jtag_ls_sw_xfer(const struct device *dev, enum jtag_pin pin,
				uint8_t value)
{
	const struct jtag_ls_config *const config = dev->config;
	struct jtag_ls_data *const data = dev->data;
	int ret;

	k_sem_take(&data->lock, K_FOREVER);

	switch (pin) {
		case JTAG_TDI:
			if (value == 0)
				io_clr_pin(data->tdi_dev);
			else
				io_set_pin(data->tdi_dev);
			break;
		case JTAG_TCK:
			if (value == 0)
				io_clr_pin(data->tck_dev);
			else
				io_set_pin(data->tck_dev);
			break;
		case JTAG_TMS:
			if (value == 0)
				io_clr_pin(data->tms_dev);
			else
				io_set_pin(data->tms_dev);
			break;
		default:
			return -EINVAL;
	}
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_PRIV_START);
    if(ret != 0) {
        LOG_ERR("JTAG pinctrl init failed (%d)", ret);
        return ret;
    }

	k_sem_give(&data->lock);

	return 0;
}

static int jtag_ls_tdo_get(const struct device *dev, uint8_t *value)
{
	struct jtag_ls_data *const data = dev->data;
	*value = io_read_pin(data->tdo_dev);
	return 0;
}

static const struct jtag_driver_api jtag_ls_driver_api = {
	.freq_get = jtag_ls_freq_get,
	.freq_set = jtag_ls_freq_set,
	.tap_get = jtag_ls_tap_get,
	.tap_set = jtag_ls_tap_set,
	.tck_run = jtag_ls_tck_run,
	.xfer = jtag_ls_xfer,
	.sw_xfer = jtag_ls_sw_xfer,
	.tdo_get = jtag_ls_tdo_get,
};

#define LS_JTAG_IRQ_HANDLER(index)                          \
static void jtag_ls_irq_config_func_##index(const struct device *dev)   \
{                                                           \
        IRQ_CONNECT(DT_INST_IRQN(index),                    \
            DT_INST_IRQ(index, priority),	                \
            ls_jtag_isr,                                    \
            DEVICE_DT_INST_GET(index), 0);                  \
        irq_enable(DT_INST_IRQN(index));                    \
}

#define LS_JTAG_INIT(index)                                 \
    IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(index)));	\
    LS_JTAG_IRQ_HANDLER(index)                              \
                                                            \
static const struct jtag_ls_config jtag_ls_cfg_##index = {  \
    .reg = (struct reg_mjtag_t *)DT_INST_REG_ADDR(index),   \
    .irq_num = DT_INST_IRQN(index),                         \
    .irq_config_func = jtag_ls_irq_config_func_##index,      \
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),)) \
};                                                          \
                                                            \
static struct jtag_ls_data jtag_ls_dev_data_##index = {     \
                                                            \
};                                                          \
                                                            \
DEVICE_DT_INST_DEFINE(index,                                \
            &jtag_ls_init,                                  \
            NULL,                                           \
            &jtag_ls_dev_data_##index, &jtag_ls_cfg_##index,                     \
            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,         \
            &jtag_ls_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_JTAG_INIT)