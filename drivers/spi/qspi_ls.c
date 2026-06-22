/*
 * Copyright (c) 2026 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_qspi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(qspi_ls);

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi_nor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <field_manipulate.h>
#include <hal_flash_int.h>
#include <ls_hal_qspiv2.h>
#include <qspiv2_config.h>
#include <reg_qspiv2_type.h>
#include <soc.h>
#include <soc_clock.h>
#include "spi_context.h"

#define STG_DAT_PHASE_BYTES_MAX 4096U /* stg_byt_num[15:4]: bytes = byt_num + 1 */
#define CALIB_DATA_LEN      64U
#define REF_CAP_DLY   0U
#define REF_CAP_NEG   0U

/* clk_cyc + 1 = bus_hz / spi_hz */
#define QSPI_LS_CLK_CYC(bus_hz, spi_hz) \
	(((bus_hz) / (spi_hz)) - 1U)
#define CALIB_COMBO_MAX      6U
#define CALIB_COMBO_INVALID  UINT8_MAX
/* calib_combo: 0 = pending (BSS); 1..7 = combo index + 1; 0xFF = invalid */

static const struct {
	uint8_t neg;
	uint8_t dly;
} calib_combos[] = {
	{0, 0}, {0, 1}, {1, 1}, {0, 2}, {1, 2}, {0, 3}, {1, 3},
};
typedef void (*qspi_ls_config_t)(void);

struct qspi_ls_config {
	reg_lsqspiv2_t *reg;
	uint32_t clock_frequency;
	qspi_ls_config_t config_func;
	uint8_t fifo_depth;
	bool timing_calibration_disabled;
	bool timing_calibration_auto_detect_content_disable;
	uint32_t timing_calibration_start_off;
	uint32_t timing_calibration_per_block_len;
	uint32_t timing_calibration_clock_frequency;
	IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
	IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
	IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct qspi_ls_data {
	struct k_mutex lock;
	struct spi_context ctx;
	struct k_sem done_sem;
	struct stg_xfer {
		bool active;
		int16_t tx_remain;
		uint32_t *tx_ptr;
		uint8_t *rx_cur;
		uint8_t *rx_end;
		uint8_t rx_dat_offset;
		bool rx_first_word;
	} xfer;
	uint8_t calib_combo;
};

#define STG_IRQ_ALL \
	(LSQSPIV2_INT_FSM_END_MASK | LSQSPIV2_INT_FIFO_RX_MASK | \
	 LSQSPIV2_INT_FIFO_TX_MASK)

#define STG_XFER_TIMEOUT_MS 30000U

struct stg_wire_cfg {
	uint8_t hz_cyc;
	uint8_t mw_wid;
	uint8_t mw_cyc;
	uint8_t sw_cyc;
	bool mw_en;
	bool sw_en;
};

/* Forward declarations used by calibration helpers */
static int stg_cmd(const struct device *dev, struct spi_nor_op_info *op,
		   const struct stg_wire_cfg *wire);
static int stg_reg_read(const struct device *dev, struct spi_nor_op_info *op,
			const struct stg_wire_cfg *wire);
static int stg_transceive(const struct device *dev, struct spi_nor_op_info *op);

static uint32_t qspi_ls_calib_hz(const struct qspi_ls_config *info);

static inline bool op_is_write(const struct spi_nor_op_info *op)
{
	return op->data_direct == SPI_NOR_DATA_DIRECT_OUT;
}

static void stg_cfg_clear_reserved(struct lsqspiv2_stg_cfg *cfg)
{
	cfg->ctrl.reserved0 = 0;
	cfg->ctrl.reserved1 = 0;
	cfg->ctrl.reserved2 = 0;
	cfg->ctrl.reserved3 = 0;
	cfg->ctrl.reserved4 = 0;
	cfg->dat_ctrl.reserved0 = 0;
	cfg->dat_ctrl.reserved1 = 0;
	cfg->dat_ctrl.reserved2 = 0;
}

static void stg_cfg_set_ctrl(struct lsqspiv2_stg_cfg *cfg,
			     const struct stg_wire_cfg *wire)
{
	cfg->ctrl.sw_en = wire->sw_en ? 1 : 0;
	cfg->ctrl.sw_cyc = wire->sw_cyc;
	cfg->ctrl.hz_cyc = wire->hz_cyc;
	cfg->ctrl.mw_wid = wire->mw_wid;
	cfg->ctrl.mw_cyc = wire->mw_cyc;
	cfg->ctrl.mw_en = wire->mw_en ? 1 : 0;
	stg_cfg_clear_reserved(cfg);
}

static void stg_cfg_set_ca(struct lsqspiv2_stg_cfg *cfg, uint8_t opcode,
		       uint32_t addr, uint8_t addr_len)
{
	if (addr_len == 4) {
		cfg->ca_high = (uint32_t)opcode << 24 | (addr >> 8);
		cfg->ca_low = addr << 24;
	} else if (addr_len == 3) {
		cfg->ca_high = (uint32_t)opcode << 24 | (addr & 0xffffff);
		cfg->ca_low = 0;
	} else {
		cfg->ca_high = (uint32_t)opcode << 24;
		cfg->ca_low = 0;
	}
}

static int stg_cfg_set_dat(const struct device *dev, struct lsqspiv2_stg_cfg *cfg,
			   void *buf, size_t len, bool is_write)
{
	if (len == 0) {
		cfg->dat_ctrl.dat_en = 0;
		cfg->dat_ctrl.dat_bytes = 0;
		cfg->dat_ctrl.dat_dir = 0;
		cfg->dat_ctrl.dat_offset = 0;
		cfg->data = NULL;
		return 0;
	}

	if (len > STG_DAT_PHASE_BYTES_MAX) {
		DEV_ERR(dev, "data len %zu exceeds STG max %u", len,
			STG_DAT_PHASE_BYTES_MAX);
		return -EINVAL;
	}

	cfg->dat_ctrl.dat_en = 1;
	cfg->dat_ctrl.dat_bytes = len - 1;
	cfg->dat_ctrl.dat_dir = is_write ? WRITE_TO_FLASH : READ_FROM_FLASH;
	cfg->dat_ctrl.dat_offset = (uint32_t)buf & 0x3;
	cfg->data = buf;

	return 0;
}

static uint8_t stg_wire_cycles(uint8_t bits, uint8_t lines)
{
	return (bits / lines) - 1U;
}

/*
 * Fill STG wire timing from op->mode and transfer context.
 * Returns 0 on success, -ENOTSUP if mode known but not implemented,
 * -EINVAL for unknown mode.
 */
static int stg_wire_cfg_fill(struct stg_wire_cfg *wire,
			     const struct device *dev,
			     const struct spi_nor_op_info *op)
{
	bool has_addr_ca = (op->addr_len != 0);
	uint8_t addr_bits;

	*wire = (struct stg_wire_cfg){0};

	switch (op->mode) {
	case JESD216_MODE_111:
	case JESD216_MODE_111_FAST:
		wire->sw_en = true;
		wire->mw_wid = SINGLE_WIRE;

		if (!has_addr_ca) {
			wire->sw_cyc = stg_wire_cycles(8, 1);
			return 0;
		}

		addr_bits = op->addr_len * 8U;
		if (op_is_write(op)) {
			wire->sw_cyc = stg_wire_cycles(8U + addr_bits, 1);
		} else {
			wire->sw_cyc = stg_wire_cycles(8U + addr_bits, 1) +
				       op->dummy_cycle;
			wire->hz_cyc = op->dummy_cycle;
		}
		return 0;

	case JESD216_MODE_444: {
		/*
		 * 4-4-4 (QPI): opcode, address, and payload all on quad.
		 * addr_len may be 0 for register / command-only ops.
		 */
		wire->mw_en = true;
		wire->mw_wid = QUAD_WIRE;

		if (!has_addr_ca) {
			wire->mw_cyc = stg_wire_cycles(8U, 4);
			return 0;
		}

		if (op->data_len == 0) {
			wire->mw_cyc = stg_wire_cycles(8U + op->addr_len * 8U, 4);
			return 0;
		}

		if (op_is_write(op)) {
			wire->mw_cyc = stg_wire_cycles(8U + op->addr_len * 8U, 4);
			return 0;
		}

		{
			uint8_t mode_clocks = 2U;
			uint8_t wait_cycles;

			if (op->dummy_cycle < mode_clocks) {
				DEV_ERR(dev, "mode 444 dummy_cycle %u < mode_clocks %u",
					op->dummy_cycle, mode_clocks);
				return -EINVAL;
			}
			wait_cycles = op->dummy_cycle - mode_clocks;

			wire->hz_cyc = wait_cycles;
			wire->mw_cyc = stg_wire_cycles(8U + op->addr_len * 8U + 8U, 4) +
					wait_cycles;
		}
		return 0;
	}

	case JESD216_MODE_112: {
		/* 1-1-2: opcode/address/dummy single, data dual */
		if (op->addr_len == 0 || op->data_len == 0) {
			DEV_ERR(dev, "mode 112 requires addr and data (op %02x)",
				op->opcode);
			return -EINVAL;
		}

		if (op_is_write(op)) {
			wire->sw_en = true;
			wire->sw_cyc = stg_wire_cycles(8U + op->addr_len * 8U, 1);
			wire->mw_wid = DUAL_WIRE;
			return 0;
		}

		addr_bits = op->addr_len * 8U;
		wire->sw_en = true;
		wire->sw_cyc = stg_wire_cycles(8U + addr_bits, 1) + op->dummy_cycle;
		wire->hz_cyc = op->dummy_cycle;
		wire->mw_wid = DUAL_WIRE;
		return 0;
	}

	case JESD216_MODE_122: {
		/*
		 * 1-2-2: opcode single; address + 8 mode bits + wait on dual.
		 * dummy_cycle = mode_clocks + wait_states .
		 */
		uint8_t mode_clocks;
		uint8_t wait_cycles;

		if (op->addr_len == 0 || op->data_len == 0) {
			DEV_ERR(dev, "mode 122 requires addr and data (op %02x)",
				op->opcode);
			return -EINVAL;
		}

		if (op_is_write(op)) {
			/* PP 1-2-2: opcode single; address and data on dual */
			wire->sw_en = true;
			wire->sw_cyc = stg_wire_cycles(8U, 1);
			wire->mw_en = true;
			wire->mw_wid = DUAL_WIRE;
			wire->mw_cyc = stg_wire_cycles(op->addr_len * 8U, 2);
			return 0;
		}

		mode_clocks = 4U;
		if (op->dummy_cycle < mode_clocks) {
			DEV_ERR(dev, "mode 122 dummy_cycle %u < mode_clocks %u",
				op->dummy_cycle, mode_clocks);
			return -EINVAL;
		}
		wait_cycles = op->dummy_cycle - mode_clocks;

		wire->sw_en = true;
		wire->sw_cyc = stg_wire_cycles(8U, 1);
		wire->mw_en = true;
		wire->mw_wid = DUAL_WIRE;
		wire->hz_cyc = wait_cycles;
		wire->mw_cyc = stg_wire_cycles(op->addr_len * 8U + 8U, 2) +
				wait_cycles;
		return 0;
	}

	case JESD216_MODE_114: {
		/* 1-1-4: opcode/address/dummy single, data quad */
		if (op->addr_len == 0 || op->data_len == 0) {
			DEV_ERR(dev, "mode 114 requires addr and data (op %02x)",
				op->opcode);
			return -EINVAL;
		}

		if (op_is_write(op)) {
			wire->sw_en = true;
			wire->sw_cyc = stg_wire_cycles(8U + op->addr_len * 8U, 1);
			wire->mw_wid = QUAD_WIRE;
			return 0;
		}

		addr_bits = op->addr_len * 8U;
		wire->sw_en = true;
		wire->sw_cyc = stg_wire_cycles(8U + addr_bits, 1) + op->dummy_cycle;
		wire->hz_cyc = op->dummy_cycle;
		wire->mw_wid = QUAD_WIRE;
		return 0;
	}

	case JESD216_MODE_144: {
		/*
		 * 1-4-4: upper layer only issues memory read/write with
		 * address and payload.
		 */
		uint8_t mode_clocks;
		uint8_t wait_cycles;

		if (op->addr_len == 0 || op->data_len == 0) {
			DEV_ERR(dev, "mode 144 requires addr and data (op %02x)",
				op->opcode);
			return -EINVAL;
		}

		if (op_is_write(op)) {
			/* PP 1-4-4: opcode single; address and data on quad */
			wire->sw_en = true;
			wire->sw_cyc = stg_wire_cycles(8U, 1);
			wire->mw_en = true;
			wire->mw_wid = QUAD_WIRE;
			wire->mw_cyc = stg_wire_cycles(op->addr_len * 8U, 4);
			return 0;
		}

		/*
		 * Quad I/O read 1-4-4 :
		 * opcode single; address + 8 mode bits + wait on quad.
		 * dummy_cycle = mode_clocks + wait_states.
		 */
		mode_clocks = 2U;
		if (op->dummy_cycle < mode_clocks) {
			DEV_ERR(dev, "mode 144 dummy_cycle %u < mode_clocks %u",
				op->dummy_cycle, mode_clocks);
			return -EINVAL;
		}
		wait_cycles = op->dummy_cycle - mode_clocks;

		wire->sw_en = true;
		wire->sw_cyc = stg_wire_cycles(8U, 1);
		wire->mw_en = true;
		wire->mw_wid = QUAD_WIRE;
		wire->hz_cyc = wait_cycles;
		wire->mw_cyc = stg_wire_cycles(op->addr_len * 8U + mode_clocks * 4U, 4) +
				wait_cycles;
		return 0;
	}

	default:
		DEV_ERR(dev, "unknown mode 0x%08x", op->mode);
		return -EINVAL;
	}
}

static void stg_irq_fifo_thr_set(reg_lsqspiv2_t *reg, uint8_t fifo_depth)
{
	reg->FIFO_THR = FIELD_BUILD(LSQSPIV2_FIFO_TX_THR, fifo_depth) |
			FIELD_BUILD(LSQSPIV2_FIFO_RX_THR, 0);
}

static void stg_irq_unmask(reg_lsqspiv2_t *reg, uint32_t mask)
{
	reg->INTR_MSK = mask;
}

static void stg_irq_mask_all(reg_lsqspiv2_t *reg)
{
	reg->INTR_MSK = 0;
}

static void stg_irq_clear(reg_lsqspiv2_t *reg, uint32_t mask)
{
	reg->INTR_CLR = mask;
}

static inline bool stg_rx_is_done(const struct stg_xfer *xfer)
{
	return xfer->rx_end != NULL && xfer->rx_cur >= xfer->rx_end;
}

/*
 * Incremental (wait=false) or finish (wait=true) RX drain.
 */
static void stg_rx_from_fifo(reg_lsqspiv2_t *reg, struct stg_xfer *xfer, bool wait)
{
	uint32_t rx;
	uint8_t *head;
	uint32_t *rx_ptr;
	ptrdiff_t remain;

	if (stg_rx_is_done(xfer)) {
		return;
	}

	if (xfer->rx_first_word) {
		if (!wait && reg->FIFO_FLVL == 0) {
			return;
		}
		while (reg->FIFO_FLVL == 0) {
			;
		}
		rx = reg->FIFO_RDAT;
		head = xfer->rx_cur;
		switch (xfer->rx_dat_offset) {
		case 0:
			*head++ = rx;
			if (head == xfer->rx_end) {
				xfer->rx_cur = xfer->rx_end;
				xfer->rx_first_word = false;
				return;
			}
			__fallthrough;
		case 1:
			*head++ = rx >> 8;
			if (head == xfer->rx_end) {
				xfer->rx_cur = xfer->rx_end;
				xfer->rx_first_word = false;
				return;
			}
			__fallthrough;
		case 2:
			*head++ = rx >> 16;
			if (head == xfer->rx_end) {
				xfer->rx_cur = xfer->rx_end;
				xfer->rx_first_word = false;
				return;
			}
			__fallthrough;
		case 3:
			*head++ = rx >> 24;
			break;
		default:
			break;
		}
		xfer->rx_cur = head;
		xfer->rx_first_word = false;
		if (stg_rx_is_done(xfer)) {
			return;
		}
	}

	rx_ptr = (uint32_t *)xfer->rx_cur;
	while (xfer->rx_end - (uint8_t *)rx_ptr >= 4) {
		if (reg->FIFO_FLVL) {
			*rx_ptr++ = reg->FIFO_RDAT;
		} else if (!wait) {
			xfer->rx_cur = (uint8_t *)rx_ptr;
			return;
		}
	}
	xfer->rx_cur = (uint8_t *)rx_ptr;

	remain = xfer->rx_end - xfer->rx_cur;
	if (remain > 0) {
		if (!wait && reg->FIFO_FLVL == 0) {
			return;
		}
		while (reg->FIFO_FLVL == 0) {
			;
		}
		rx = reg->FIFO_RDAT;
		head = xfer->rx_cur;
		switch (remain) {
		case 3:
			head[2] = rx >> 16;
			__fallthrough;
		case 2:
			head[1] = rx >> 8;
			__fallthrough;
		case 1:
			head[0] = rx;
			break;
		default:
			break;
		}
		xfer->rx_cur = xfer->rx_end;
	}
}

static void stg_tx_irq_fill(struct qspi_ls_data *data, reg_lsqspiv2_t *reg,
			    uint8_t fifo_depth)
{
	struct stg_xfer *xfer = &data->xfer;

	while (xfer->tx_remain > 0 && reg->FIFO_FLVL != fifo_depth) {
		reg->FIFO_WDAT = *xfer->tx_ptr++;
		xfer->tx_remain -= 4;
	}
}

/* Drop late ISR completions and pending IRQs before a new STIG op. */
static void stg_xfer_quiesce(reg_lsqspiv2_t *reg, struct qspi_ls_data *data)
{
	stg_irq_mask_all(reg);
	stg_irq_clear(reg, STG_IRQ_ALL);
	data->xfer.active = false;
	k_sem_reset(&data->done_sem);
	while (k_sem_take(&data->done_sem, K_NO_WAIT) == 0) {
		;
	}
}

static void stg_xfer_complete(struct qspi_ls_data *data, reg_lsqspiv2_t *reg)
{
	if (!data->xfer.active) {
		return;
	}

	data->xfer.active = false;
	stg_irq_mask_all(reg);
	k_sem_give(&data->done_sem);
}

static void stg_xfer_try_complete_read(struct qspi_ls_data *data,
				       reg_lsqspiv2_t *reg)
{
	if (stg_rx_is_done(&data->xfer)) {
		stg_xfer_complete(data, reg);
	}
}

static void qspi_ls_isr(const struct device *dev)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	reg_lsqspiv2_t *reg = info->reg;
	struct stg_xfer *xfer = &data->xfer;
	uint32_t stt = reg->INTR_STT;

	if (!xfer->active) {
		stg_irq_clear(reg, stt & STG_IRQ_ALL);
		return;
	}

	if (stt & LSQSPIV2_INT_FIFO_TX_MASK) {
		stg_irq_clear(reg, LSQSPIV2_INT_FIFO_TX_MASK);
		stg_tx_irq_fill(data, reg, info->fifo_depth);
	}

	if (stt & LSQSPIV2_INT_FIFO_RX_MASK) {
		stg_irq_clear(reg, LSQSPIV2_INT_FIFO_RX_MASK);
		stg_rx_from_fifo(reg, xfer, false);
		stg_xfer_try_complete_read(data, reg);
	}

	if (stt & LSQSPIV2_INT_FSM_END_MASK) {
		stg_irq_clear(reg, LSQSPIV2_INT_FSM_END_MASK);

		if (!xfer->active) {
			return;
		}

		if (xfer->rx_end != NULL) {
			/*
			 * FSM is done — no more data will enter the FIFO.
			 * Drain what is already there; never spin here (wait=true
			 * would deadlock the ISR when capture is out of window).
			 */
			stg_rx_from_fifo(reg, xfer, false);
			stg_xfer_complete(data, reg);
		} else {
			stg_xfer_complete(data, reg);
		}
	}
}

/*
 * STIG transfer with DAC kept disabled
 */
static int stg_read_write(const struct device *dev, struct lsqspiv2_stg_cfg *cfg)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	reg_lsqspiv2_t *reg = info->reg;
	struct stg_xfer *xfer = &data->xfer;
	uint32_t irq_mask = LSQSPIV2_INT_FSM_END_MASK;
	int ret;

	stg_xfer_quiesce(reg, data);
	memset(xfer, 0, sizeof(*xfer));

	REG_FIELD_WR(reg->QSPI_CTRL1, LSQSPIV2_MODE_DAC, 0);
	reg->QSPI_SRST_N = 0;
	reg->STG_REQ_T = reg->STG_REQ_T;
	reg->QSPI_SRST_N = 1;
	barrier_dmem_fence_full();
	stg_irq_clear(reg, STG_IRQ_ALL);
	stg_irq_mask_all(reg);
	stg_irq_fifo_thr_set(reg, info->fifo_depth);

	reg->STG_CTRL = *(uint32_t *)&cfg->ctrl;
	reg->STG_CA_HIGH = cfg->ca_high;
	reg->STG_CA_LOW = cfg->ca_low;
	reg->STG_DAT_CTRL = *(uint32_t *)&cfg->dat_ctrl;
	barrier_dmem_fence_full();

	xfer->active = true;

	if (cfg->dat_ctrl.dat_en != 0) {
		if (cfg->dat_ctrl.dat_dir == READ_FROM_FLASH) {
			xfer->rx_cur = cfg->data;
			xfer->rx_end = xfer->rx_cur + cfg->dat_ctrl.dat_bytes + 1;
			xfer->rx_dat_offset = cfg->dat_ctrl.dat_offset;
			xfer->rx_first_word = true;
			irq_mask |= LSQSPIV2_INT_FIFO_RX_MASK;
		} else {
			xfer->tx_remain = cfg->dat_ctrl.dat_bytes + 1 +
					  cfg->dat_ctrl.dat_offset;
			xfer->tx_ptr = (uint32_t *)(cfg->data -
						    cfg->dat_ctrl.dat_offset);
			irq_mask |= LSQSPIV2_INT_FIFO_TX_MASK;
			stg_tx_irq_fill(data, reg, info->fifo_depth);
		}
	}

	stg_irq_unmask(reg, irq_mask);
	reg->STG_REQ_T = 1;
	barrier_dmem_fence_full();

	ret = k_sem_take(&data->done_sem, K_MSEC(STG_XFER_TIMEOUT_MS));

	/*
	 * Catch FSM_END that completed between quiesce and k_sem_take (lost
	 * wakeup) or fired before the waiter was registered.
	 */
	if (ret != 0 && (reg->INTR_RAW & LSQSPIV2_INT_FSM_END_MASK) != 0) {
		stg_irq_clear(reg, LSQSPIV2_INT_FSM_END_MASK);
		if (cfg->dat_ctrl.dat_en != 0 &&
		    cfg->dat_ctrl.dat_dir == READ_FROM_FLASH) {
			/*
			 * Mirror FSM_END ISR: drain without spinning; wait=true
			 * deadlocks here when FIFO is already empty.
			 */
			stg_rx_from_fifo(reg, xfer, false);
		}
		if (xfer->active) {
			stg_xfer_complete(data, reg);
		}
		ret = 0;
	}

	stg_irq_mask_all(reg);
	xfer->active = false;

	if (ret != 0) {
		DEV_ERR(dev, "STIG transfer timeout");
		return -ETIMEDOUT;
	}

	if (cfg->dat_ctrl.dat_en != 0 &&
	    cfg->dat_ctrl.dat_dir == READ_FROM_FLASH &&
	    !stg_rx_is_done(xfer)) {
		DEV_DBG(dev, "STIG read incomplete (%td bytes left)",
			xfer->rx_end - xfer->rx_cur);
		return -EIO;
	}

	return 0;
}
static void hw_init_default(const struct device *dev)
{
	const struct qspi_ls_config *info = dev->config;
	reg_lsqspiv2_t *reg = info->reg;
	const uint32_t timing_calib_clk = qspi_ls_calib_hz(info);

	reg->QSPI_CTRL0 = FIELD_BUILD(LSQSPIV2_CLK_CYC,
			  (info->clock_frequency / timing_calib_clk - 1)) |
			  FIELD_BUILD(LSQSPIV2_CS_INTV, QSPI_CS_INTERVAL) |
			  FIELD_BUILD(LSQSPIV2_CS_HOLD, QSPI_CS_HOLD) |
			  FIELD_BUILD(LSQSPIV2_CS_SETUP, QSPI_CS_SETUP);
	MODIFY_REG(reg->QSPI_CTRL1,LSQSPIV2_MODE_DAC_MASK | LSQSPIV2_CAP_DLY_MASK |
			   LSQSPIV2_CAP_NEG_MASK,LSQSPIV2_MODE_DAC_MASK |
			   (REF_CAP_DLY << LSQSPIV2_CAP_DLY_POS) |(REF_CAP_NEG << LSQSPIV2_CAP_NEG_POS));
}


static int apply_clk(const struct device *dev, const struct spi_config *config)
{
	const struct qspi_ls_config *info = dev->config;
	reg_lsqspiv2_t *reg = info->reg;
	uint32_t clk_cyc;

	if (config == NULL || config->frequency == 0U) {
		return 0;
	}

	if (config->frequency > info->clock_frequency) {
		DEV_ERR(dev, "SPI freq %u > bus %u", config->frequency,
			info->clock_frequency);
		return -EINVAL;
	}

	clk_cyc = QSPI_LS_CLK_CYC(info->clock_frequency, config->frequency);
	if (clk_cyc > 31U) {
		DEV_ERR(dev, "clk_cyc %u out of range", clk_cyc);
		return -EINVAL;
	}

	MODIFY_REG(reg->QSPI_CTRL0, LSQSPIV2_CLK_CYC_MASK,
		   clk_cyc << LSQSPIV2_CLK_CYC_POS);

	DEV_DBG(dev, "clk_cyc=%u bus=%u spi=%u", clk_cyc, info->clock_frequency,
		config->frequency);

	return 0;
}

static void stg_capture_set(reg_lsqspiv2_t *reg, uint8_t cap_neg, uint8_t cap_dly)
{
	MODIFY_REG(reg->QSPI_CTRL1, LSQSPIV2_CAP_DLY_MASK | LSQSPIV2_CAP_NEG_MASK,
		   (cap_dly << LSQSPIV2_CAP_DLY_POS) |
			   (cap_neg << LSQSPIV2_CAP_NEG_POS));
}

static uint32_t qspi_ls_calib_hz(const struct qspi_ls_config *info)
{
	uint32_t hz = info->timing_calibration_clock_frequency;

	return (hz != 0U) ? hz : 10000000U;
}

static void stg_capture_apply_calibrated(const struct device *dev)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	uint8_t stored = data->calib_combo;

	if (stored == 0U || stored == CALIB_COMBO_INVALID ||
	    stored > (CALIB_COMBO_MAX + 1U)) {
		return;
	}

	stg_capture_set(info->reg, calib_combos[stored - 1U].neg,
			calib_combos[stored - 1U].dly);
}


static bool calib_buf_valid(const uint8_t *buf, size_t len)
{
	const uint32_t *buf_32 = (const uint32_t *)buf;
	uint32_t valid_count = 0;

	for (size_t i = 0; i < (len >> 2); i++) {
		if (buf_32[i] != 0U && buf_32[i] != 0xffffffffU) {
			valid_count++;
		}
		if (valid_count > (len >> 3)) {
			return true;
		}
	}

	return false;
}

static bool stg_capture_hw_applied(reg_lsqspiv2_t *reg)
{
	uint32_t ctrl = reg->QSPI_CTRL1;
	uint8_t cap_dly = (uint8_t)((ctrl & LSQSPIV2_CAP_DLY_MASK) >>
				    LSQSPIV2_CAP_DLY_POS);
	uint8_t cap_neg = (uint8_t)((ctrl & LSQSPIV2_CAP_NEG_MASK) >>
				    LSQSPIV2_CAP_NEG_POS);

	return cap_dly != 0U || cap_neg != 0U;
}

static int calib_read_flash(const struct device *dev,
			    const struct spi_nor_op_info *op_info,
			    uint32_t addr, uint8_t *buf, size_t len)
{
	struct spi_nor_op_info calib_op = *op_info;

	calib_op.addr = addr;
	calib_op.buf = buf;
	calib_op.data_len = len;
	calib_op.data_direct = SPI_NOR_DATA_DIRECT_IN;

	return stg_transceive(dev, &calib_op);
}

static int qspi_ls_timing_calibration(const struct device *dev,
					 const struct spi_config *config,
					 struct spi_nor_op_info *op_info)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	reg_lsqspiv2_t *reg = info->reg;

	uint8_t check_buf[CALIB_DATA_LEN] __aligned(4);
	uint8_t valid_bitmap = 0;
	uint32_t addr = 0;
	const uint32_t flash_size = op_info->data_len;
	const uint32_t calib_hz = qspi_ls_calib_hz(info);
	uint32_t ref_crc;
	struct spi_config low_cfg;
	int ret = 0;

	if (info->timing_calibration_disabled) {
		goto no_calibration;
	}

	if (calib_hz > config->frequency) {
		data->calib_combo = 1U;
		goto no_calibration;
	}

	if (data->calib_combo != 0U || stg_capture_hw_applied(reg)) {
		DEV_INF(dev, "Already executed calibration.");
		goto no_calibration;
	}

	low_cfg = *config;
	low_cfg.frequency = calib_hz;
	ret = apply_clk(dev, &low_cfg);
	if (ret != 0) {
		goto no_calibration;
	}
	stg_capture_set(reg, REF_CAP_NEG, REF_CAP_DLY);

	if (info->timing_calibration_auto_detect_content_disable) {
		addr = info->timing_calibration_start_off;
		ret = calib_read_flash(dev, op_info, addr, check_buf, CALIB_DATA_LEN);
		if (ret != 0) {
			goto no_calibration;
		}
		if (!calib_buf_valid(check_buf, CALIB_DATA_LEN)) {
			DEV_ERR(dev, "Flash data is monotonous, skip calibration.");
			ret = -EINVAL;
			goto no_calibration;
		}
	} else {
		uint32_t stride = info->timing_calibration_per_block_len;
		bool found = false;

		if (stride == 0U) {
			stride = CALIB_DATA_LEN;
		}

		for (uint32_t off = 0; off < flash_size; off += stride) {
			if ((off + CALIB_DATA_LEN) > flash_size) {break;}
			ret = calib_read_flash(dev, op_info, off, check_buf, CALIB_DATA_LEN);
			if (ret != 0) {
				goto no_calibration;
			}
			if (calib_buf_valid(check_buf, CALIB_DATA_LEN)) {
				DEV_DBG(dev, "calibration data at 0x%x", off);
				addr = off;
				found = true;
				break;
			}
		}
		if (!found) {
			DEV_ERR(dev, "All flash data is monotonous, skip calibration.");
			ret = -EINVAL;
			goto no_calibration;
		}
	}

	stg_capture_set(reg, REF_CAP_NEG, REF_CAP_DLY);
	ref_crc = crc32_ieee(check_buf, CALIB_DATA_LEN);

	ret = apply_clk(dev, config);
	if (ret != 0) {
		goto no_calibration;
	}

	for (size_t i = 0; i < ARRAY_SIZE(calib_combos); i++) {
		uint32_t crc;

		stg_capture_set(reg, calib_combos[i].neg, calib_combos[i].dly);
		ret = calib_read_flash(dev, op_info, addr, check_buf, CALIB_DATA_LEN);
		if (ret != 0) {
			/* Capture combo out of window at target Hz — skip it. */
			continue;
		}
		crc = crc32_ieee(check_buf, CALIB_DATA_LEN);
		if (crc == ref_crc) {
			valid_bitmap |= BIT(i);
		}
	}

	{
		size_t best_start = 0;
		size_t best_len = 0;

		for (size_t i = 0; i < ARRAY_SIZE(calib_combos); i++) {
			if ((valid_bitmap & BIT(i)) == 0) {
				continue;
			}
			size_t j = i;

			while (j < ARRAY_SIZE(calib_combos) && (valid_bitmap & BIT(j))) {
				j++;
			}
			size_t len = j - i;

			if (len > best_len) {
				best_len = len;
				best_start = i;
			}
			i = j;
		}

		if (best_len == ARRAY_SIZE(calib_combos)) {
			DEV_INF(dev, "useless calibration, all capture delay ok");
			data->calib_combo = 1U;
			stg_capture_apply_calibrated(dev);
		} else if (best_len > 0) {
			size_t pick = best_start + (best_len >> 1);

			data->calib_combo = (uint8_t)pick + 1U;
			stg_capture_apply_calibrated(dev);
			DEV_INF(dev, "timing calibration success: combo %zu "
				"(cap_neg=%u cap_dly=%u) @ %u Hz",
				pick, calib_combos[pick].neg, calib_combos[pick].dly,
				config->frequency);
		} else {
			DEV_ERR(dev, "timing calibration failed (valid_bitmap=0x%02x)",valid_bitmap);
			ret = -EIO;
			goto no_calibration;
		}
	}

	return 0;

no_calibration:
	if (ret != 0) {
		data->calib_combo = CALIB_COMBO_INVALID;
		low_cfg = *config;
		low_cfg.frequency = calib_hz;
		(void)apply_clk(dev, &low_cfg);
		stg_capture_set(reg, REF_CAP_NEG, REF_CAP_DLY);
		DEV_WRN(dev, "timing calibration skipped (ret %d), stay at %u Hz",
			ret, calib_hz);
	}

	return ret;
}

static int stg_cmd(const struct device *dev, struct spi_nor_op_info *op,
		   const struct stg_wire_cfg *wire)
{
	struct lsqspiv2_stg_cfg cfg = {0};
	int ret;

	stg_cfg_set_ctrl(&cfg, wire);
	cfg.ca_high = (uint32_t)op->opcode << 24;
	ret = stg_cfg_set_dat(dev, &cfg, NULL, 0, false);
	if (ret != 0) {
		return ret;
	}

	return stg_read_write(dev, &cfg);
}

static int stg_reg_read(const struct device *dev, struct spi_nor_op_info *op,
			const struct stg_wire_cfg *wire)
{
	struct lsqspiv2_stg_cfg cfg = {0};
	int ret;

	stg_cfg_set_ctrl(&cfg, wire);
	cfg.ca_high = (uint32_t)op->opcode << 24;
	ret = stg_cfg_set_dat(dev, &cfg, op->buf, op->data_len, false);
	if (ret != 0) {
		return ret;
	}

	return stg_read_write(dev, &cfg);
}

static int stg_reg_write(const struct device *dev, struct spi_nor_op_info *op,
			 const struct stg_wire_cfg *wire)
{
	struct lsqspiv2_stg_cfg cfg = {0};
	int ret;

	stg_cfg_set_ctrl(&cfg, wire);
	cfg.ca_high = (uint32_t)op->opcode << 24;
	ret = stg_cfg_set_dat(dev, &cfg, op->buf, op->data_len, true);
	if (ret != 0) {
		return ret;
	}

	return stg_read_write(dev, &cfg);
}

static int stg_mem_read(const struct device *dev, struct spi_nor_op_info *op,
			const struct stg_wire_cfg *wire)
{
	struct lsqspiv2_stg_cfg cfg = {0};
	size_t remain = op->data_len;
	uint32_t offset = (uint32_t)op->addr;
	uint8_t *data = op->buf;
	int ret;

	stg_cfg_set_ctrl(&cfg, wire);
	cfg.dat_ctrl.dat_en = 1;
	cfg.dat_ctrl.dat_dir = READ_FROM_FLASH;

	while (remain > 0) {
		size_t chunk = MIN(remain, STG_DAT_PHASE_BYTES_MAX);

		stg_cfg_set_ca(&cfg, op->opcode, offset, op->addr_len);
		ret = stg_cfg_set_dat(dev, &cfg, data, chunk, false);
		if (ret != 0) {
			return ret;
		}
		ret = stg_read_write(dev, &cfg);
		if (ret != 0) {
			return ret;
		}

		data += chunk;
		offset += chunk;
		remain -= chunk;
	}

	return 0;
}

static int stg_mem_write(const struct device *dev, struct spi_nor_op_info *op,
			 const struct stg_wire_cfg *wire)
{
	struct lsqspiv2_stg_cfg cfg = {0};
	int ret;

	stg_cfg_set_ctrl(&cfg, wire);
	stg_cfg_set_ca(&cfg, op->opcode, (uint32_t)op->addr, op->addr_len);
	ret = stg_cfg_set_dat(dev, &cfg, op->buf, op->data_len, true);
	if (ret != 0) {
		return ret;
	}

	return stg_read_write(dev, &cfg);
}

static int stg_transceive(const struct device *dev,struct spi_nor_op_info *op)
{
	struct stg_wire_cfg wire;
	bool is_write = op_is_write(op);
	int ret;

	ret = stg_wire_cfg_fill(&wire, dev, op);
	if (ret != 0) {
		return ret;
	}

	if (op->addr_len == 0) {
		if (op->data_len == 0) {
			return stg_cmd(dev, op, &wire);
		}
		return is_write ? stg_reg_write(dev, op, &wire) :
				stg_reg_read(dev, op, &wire);
	}

	if (is_write) {
		return stg_mem_write(dev, op, &wire);
	}

	return stg_mem_read(dev, op, &wire);
}

static int apply_hw_config(const struct device *dev,
			   const struct spi_config *config)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	struct spi_config effective = *config;
	int ret;

	if (!info->timing_calibration_disabled &&
	    data->calib_combo == CALIB_COMBO_INVALID) {
		effective.frequency = qspi_ls_calib_hz(info);
	}

	ret = apply_clk(dev, &effective);
	if (ret != 0) {
		return ret;
	}

	data->ctx.config = config;

	if (!info->timing_calibration_disabled &&
	    data->calib_combo == CALIB_COMBO_INVALID) {
		stg_capture_set(info->reg, REF_CAP_NEG, REF_CAP_DLY);
		} else {
		stg_capture_apply_calibrated(dev);
	}

	{
		uint32_t ctrl = info->reg->QSPI_CTRL1;
		uint8_t cap_dly = (uint8_t)((ctrl & LSQSPIV2_CAP_DLY_MASK) >>
					    LSQSPIV2_CAP_DLY_POS);
		uint8_t cap_neg = (uint8_t)((ctrl & LSQSPIV2_CAP_NEG_MASK) >>
					    LSQSPIV2_CAP_NEG_POS);

		DEV_INF(dev, "hw config: spi=%u Hz cap_neg=%u cap_dly=%u "
			"(calib_combo=0x%02x)",
			effective.frequency, cap_neg, cap_dly, data->calib_combo);
	}

	return 0;
}

static int qspi_ls_nor_transceive(const struct device *dev,
				     const struct spi_config *config,
				     struct spi_nor_op_info *op_info)
{
	struct qspi_ls_data *data = dev->data;
	int ret;

	ARG_UNUSED(config);

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = stg_transceive(dev, op_info);
	k_mutex_unlock(&data->lock);

	return ret;
}

static int qspi_ls_transceive(const struct device *dev,
				 const struct spi_config *config,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);
	ARG_UNUSED(tx_bufs);
	ARG_UNUSED(rx_bufs);

	return -ENOTSUP;
}

static int qspi_ls_release(const struct device *dev,
			      const struct spi_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(config);

	return 0;
}

static int qspi_ls_read_init(const struct device *dev,
						const struct spi_config *config,
						struct spi_nor_op_info *op_info)
{
	struct qspi_ls_data *data = dev->data;

	DEV_DBG(dev, "read init mode %08x cmd %02x dummy %u freq %u",
		op_info->mode, op_info->opcode, op_info->dummy_cycle,
		config->frequency);

	k_mutex_lock(&data->lock, K_FOREVER);
	(void)qspi_ls_timing_calibration(dev, config, op_info);
	k_mutex_unlock(&data->lock);

	return apply_hw_config(dev, config);
}

static int qspi_ls_write_init(const struct device *dev,
						const struct spi_config *config,
						struct spi_nor_op_info *op_info)
{
	DEV_DBG(dev, "write init mode %08x cmd %02x dummy %u freq %u",
		op_info->mode, op_info->opcode, op_info->dummy_cycle,
		config->frequency);

	/* HW clock/capture already applied by read_init() during configure. */
	return 0;
}

static const struct spi_nor_ops qspi_ls_nor_ops = {
	.transceive = qspi_ls_nor_transceive,
	.read_init = qspi_ls_read_init,
	.write_init = qspi_ls_write_init,
};

static const struct spi_driver_api qspi_ls_api = {
	.transceive = qspi_ls_transceive,
	.release = qspi_ls_release,
	.spi_nor_op = &qspi_ls_nor_ops,
};

static int qspi_ls_init(const struct device *dev)
{
	const struct qspi_ls_config *info = dev->config;
	struct qspi_ls_data *data = dev->data;
	__maybe_unused int ret;

#if defined(CONFIG_CLOCK_CONTROL)
	if (info->ccfg.cctl_dev) {
		const struct device *clk_dev = info->ccfg.cctl_dev;

		if (!device_is_ready(clk_dev)) {
			return -ENODEV;
		}
		clock_control_off(clk_dev,
				  (clock_control_subsys_t)&info->ccfg);
	}
#endif

#if defined(CONFIG_RESET)
	if (info->reset.dev != NULL) {
		if (!device_is_ready(info->reset.dev)) {
			return -ENODEV;
		}

		ret = reset_line_toggle(info->reset.dev, info->reset.id);
		if (ret != 0) {
			return ret;
		}
	}
#endif

#if defined(CONFIG_CLOCK_CONTROL)
	if (info->ccfg.cctl_dev) {
		const struct device *clk_dev = info->ccfg.cctl_dev;

		clock_control_on(clk_dev,
				 (clock_control_subsys_t)&info->ccfg);
	}
#endif

#if defined(CONFIG_PINCTRL)
	ret = pinctrl_apply_state(info->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}
#endif

	k_mutex_init(&data->lock);
	k_sem_init(&data->done_sem, 0, 1);
	hw_init_default(dev);

	info->config_func();

	return 0;
}

#define QSPI_LS_IRQ_HANDLER(inst)						\
	void qspi_ls_irq_config_##inst(void)					\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority),	\
			    qspi_ls_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_INST_IRQN(inst));					\
}

#define QSPI_LS_INIT(inst)							\
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))		\
	QSPI_LS_IRQ_HANDLER(inst)						\
	static const struct qspi_ls_config qspi_ls_config_##inst = {	\
		.reg = (reg_lsqspiv2_t *)DT_INST_REG_ADDR(inst),		\
		.clock_frequency = COND_CODE_1(					\
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks),		\
					 clock_frequency),			\
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),\
			(DT_INST_PROP(inst, clock_frequency))),			\
		.timing_calibration_disabled =					\
			DT_INST_PROP_OR(inst, timing_calibration_disabled, false),\
		.timing_calibration_clock_frequency =				\
			DT_INST_PROP_OR(inst, timing_calibration_clock_frequency, 10000000),\
		.config_func = qspi_ls_irq_config_##inst,			\
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),			\
		IF_ENABLED(CONFIG_PINCTRL,					\
			(.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))	\
		IF_ENABLED(DT_HAS_CLOCKS(inst),					\
			(.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))			\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets),			\
			(.reset = RESET_DT_SPEC_INST_GET(inst), ))		\
	};									\
	static struct qspi_ls_data qspi_ls_data_##inst = {		\
		SPI_CONTEXT_INIT_LOCK(qspi_ls_data_##inst, ctx),		\
		SPI_CONTEXT_INIT_SYNC(qspi_ls_data_##inst, ctx),		\
	};									\
	DEVICE_DT_INST_DEFINE(inst,						\
			      qspi_ls_init,					\
			      NULL,						\
			      &qspi_ls_data_##inst,				\
			      &qspi_ls_config_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_SPI_INIT_PRIORITY,				\
			      &qspi_ls_api);

DT_INST_FOREACH_STATUS_OKAY(QSPI_LS_INIT)
