/*
 * Copyright (c) 2015 Intel Corporation.
 * Copyright (c) 2023 Synopsys, Inc. All rights reserved.
 * Copyright (c) 2023 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_spi

/* spi_dw.c - Designware SPI driver implementation */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_dw);

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/pm/device.h>

#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CONFIG_IOAPIC
#include <zephyr/drivers/interrupt_controller/ioapic.h>
#endif

#include <zephyr/drivers/spi_nor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_dw.h>
#include <zephyr/cache.h>
#include <soc_dma.h>

#include "spi_dw.h"
#include "spi_context.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#define SPI_DW_DMA_WAIT_TIMEOUT_MS 100000
#define SCKDV_BIT 0xfe
static inline bool spi_dw_is_slave(struct spi_dw_data *spi)
{
	return (IS_ENABLED(CONFIG_SPI_SLAVE) &&
		spi_context_is_slave(&spi->ctx));
}

static void completed(const struct device *dev, int error)
{
	struct spi_dw_data *spi = dev->data;
	struct spi_context *ctx = &spi->ctx;

	if (error) {
		goto out;
	}

	if (spi_context_tx_on(&spi->ctx) ||
	    spi_context_rx_on(&spi->ctx)) {
		return;
	}

out:
	/* need to give time for FIFOs to drain before issuing more commands */
	while (test_bit_sr_busy(dev) | (!test_bit_sr_tfe(dev))) {
	}

	/* Disabling interrupts */
	write_imr(dev, DW_SPI_IMR_MASK);
	/* Disabling the controller */
	clear_bit_ssienr(dev);

	if (!spi_dw_is_slave(spi)) {
		if (spi_cs_is_gpio(ctx->config)) {
			spi_context_cs_control(ctx, false);
		} else {
			write_ser(dev, 0);
		}
	}

	LOG_DBG("SPI transaction completed %s error",
		    error ? "with" : "without");

	spi_context_complete(&spi->ctx, dev, error);
}

static void push_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t data = 0U;
	uint32_t f_tx;

	if (spi_context_rx_on(&spi->ctx)) {
		f_tx = info->fifo_depth - read_txflr(dev) -
			read_rxflr(dev);
		if ((int)f_tx < 0) {
			f_tx = 0U; /* if rx-fifo is full, hold off tx */
		}
	} else {
		f_tx = info->fifo_depth - read_txflr(dev);
	}

	while (f_tx) {
		if (spi_context_tx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				data = UNALIGNED_GET((uint8_t *)
						     (spi->ctx.tx_buf));
				break;
			case 2:
				data = UNALIGNED_GET((uint16_t *)
						     (spi->ctx.tx_buf));
				break;
			case 4:
				data = UNALIGNED_GET((uint32_t *)
						     (spi->ctx.tx_buf));
				break;
			}
		} else if (spi_context_rx_on(&spi->ctx)) {
			/* No need to push more than necessary */
			if ((int)(spi->ctx.rx_len - spi->fifo_diff) <= 0) {
				break;
			}

			data = 0U;
		} else if (spi_context_tx_on(&spi->ctx)) {
			data = 0U;
		} else {
			/* Nothing to push anymore */
			break;
		}

		write_dr(dev, data);

		spi_context_update_tx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff++;

		f_tx--;
	}

	if (!spi_context_tx_on(&spi->ctx)) {
		/* prevents any further interrupts demanding TX fifo fill */
		write_txftlr(dev, 0);
	}
}

static void pull_data(const struct device *dev)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;

	while (read_rxflr(dev)) {
		uint32_t data = read_dr(dev);

		if (spi_context_rx_buf_on(&spi->ctx)) {
			switch (spi->dfs) {
			case 1:
				UNALIGNED_PUT(data, (uint8_t *)spi->ctx.rx_buf);
				break;
			case 2:
				UNALIGNED_PUT(data, (uint16_t *)spi->ctx.rx_buf);
				break;
			case 4:
				UNALIGNED_PUT(data, (uint32_t *)spi->ctx.rx_buf);
				break;
			}
		}

		spi_context_update_rx(&spi->ctx, spi->dfs, 1);
		spi->fifo_diff--;
	}

	if (!spi->ctx.rx_len && spi->ctx.tx_len < info->fifo_depth) {
		write_rxftlr(dev, spi->ctx.tx_len - 1);
	} else if (read_rxftlr(dev) >= spi->ctx.rx_len) {
		write_rxftlr(dev, spi->ctx.rx_len - 1);
	}
}


static void spi_dw_dma_rx_callback(const struct device *dev_dma,
						void *user, uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)user;
	struct spi_dw_data *spi = dev->data;

	if (status == DMA_STATUS_COMPLETE ||
		status < 0) {
		k_sem_give(&spi->dma_rx_sem);
	}
}

static int spi_dw_configure(const struct device *dev,
			    struct spi_dw_data *spi,
			    const struct spi_config *config)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t ctrlr0 = 0U;

	LOG_DBG("%p (prev %p)", config, spi->ctx.config);

	if (spi_context_configured(&spi->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERROR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Verify if requested op mode is relevant to this controller */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		if (!(info->serial_target)) {
			LOG_ERROR("Slave mode not supported");
			return -ENOTSUP;
		}
	} else {
		if (info->serial_target) {
			LOG_ERROR("Master mode not supported");
			return -ENOTSUP;
		}
	}

	if ((config->operation & SPI_TRANSFER_LSB) ||
	    (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	     (config->operation & (SPI_LINES_DUAL |
				   SPI_LINES_QUAD | SPI_LINES_OCTAL)))) {
		LOG_ERROR("Unsupported configuration");
		return -EINVAL;
	}

	if (info->max_xfer_size < SPI_WORD_SIZE_GET(config->operation)) {
		LOG_ERROR("Max xfer size is %u, word size of %u not allowed",
			info->max_xfer_size, SPI_WORD_SIZE_GET(config->operation));
		return -ENOTSUP;
	}

	/* Word size */
	if (info->max_xfer_size == 32) {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_32(SPI_WORD_SIZE_GET(config->operation));
	} else {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_16(SPI_WORD_SIZE_GET(config->operation));
	}

	/* Determine how many bytes are required per-frame */
	spi->dfs = SPI_WS_TO_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPOL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPH;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		ctrlr0 |= DW_SPI_CTRLR0_SRL;
	}

	/* Installing the configuration */
	write_ctrlr0(dev, ctrlr0);

	/* At this point, it's mandatory to set this on the context! */
	spi->ctx.config = config;

	if (!spi_dw_is_slave(spi)) {
		/* Baud rate and Slave select, for master only */
		write_baudr(dev, SPI_DW_CLK_DIVIDER(info->clock_frequency,
						    config->frequency));
		if ((info->clock_frequency % config->frequency)!=0) {
			LOG_INF("The hardware design results in the actual frequency not being equal to the set frequency,actual freq %u",
				   (info->clock_frequency)/(((SPI_DW_CLK_DIVIDER(info->clock_frequency,config->frequency)))&SCKDV_BIT));
		}
		write_ser(dev, 1 << config->slave);
	}

	if (spi_dw_is_slave(spi)) {
		LOG_DBG("Installed slave config %p:"
			    " ws/dfs %u/%u, mode %u/%u/%u",
			    config,
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0);
	} else {
		LOG_DBG("Installed master config %p: freq %uHz (div = %u),"
			    " ws/dfs %u/%u, mode %u/%u/%u, slave %u",
			    config, config->frequency,
			    SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency),
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0,
			    config->slave);
	}

	return 0;
}

static int spi_dw_configure_support_all(const struct device *dev,
			    struct spi_dw_data *spi,
			    const struct spi_config *config)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t ctrlr0 = 0U;

	LOG_DBG("%p (prev %p)", config, spi->ctx.config);

	if (spi_context_configured(&spi->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	/* Verify if requested op mode is relevant to this controller */
	if (config->operation & SPI_OP_MODE_SLAVE) {
		if (!(info->serial_target)) {
			LOG_ERR("Slave mode not supported");
			return -ENOTSUP;
		}
	} else {
		if (info->serial_target) {
			LOG_ERR("Master mode not supported");
			return -ENOTSUP;
		}
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB-first not supported");
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES)) {
		if ((config->operation & SPI_LINES_MASK)==SPI_LINES_QUAD) {
			ctrlr0 |= DW_SPI_CTRLR0_FRF_QUAD;
			LOG_DBG("Quad Line");
		}else if ((config->operation & SPI_LINES_MASK)==SPI_LINES_SINGLE) {
			ctrlr0 |= DW_SPI_CTRLR0_FRF_STD;
			LOG_DBG("Single Line");
		}else if((config->operation & SPI_LINES_MASK)==SPI_LINES_DUAL) {
			ctrlr0 |= DW_SPI_CTRLR0_FRF_DUAL;
			LOG_DBG("Dual Line");
		}else {
			LOG_DBG("  Unsupported configuration");
			return -EINVAL;
		}
	}

	if (info->max_xfer_size < SPI_WORD_SIZE_GET(config->operation)) {
		LOG_ERR("Max xfer size is %u, word size of %u not allowed",
			info->max_xfer_size, SPI_WORD_SIZE_GET(config->operation));
		return -ENOTSUP;
	}

	/* Word size */
	if (info->max_xfer_size == 32) {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_32(SPI_WORD_SIZE_GET(config->operation));
	} else {
		ctrlr0 |= DW_SPI_CTRLR0_DFS_16(SPI_WORD_SIZE_GET(config->operation));
	}

	/* Determine how many bytes are required per-frame */
	spi->dfs = SPI_WS_TO_DFS(SPI_WORD_SIZE_GET(config->operation));

	/* SPI mode */
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPOL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		ctrlr0 |= DW_SPI_CTRLR0_SCPH;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		ctrlr0 |= DW_SPI_CTRLR0_SRL;
	}

	/* Installing the configuration */
	write_ctrlr0(dev, ctrlr0);

	/* At this point, it's mandatory to set this on the context! */
	spi->ctx.config = config;

	if (!spi_dw_is_slave(spi)) {
		/* Baud rate and Slave select, for master only */
		write_baudr(dev, SPI_DW_CLK_DIVIDER(info->clock_frequency,
						    config->frequency));
		if ((info->clock_frequency % config->frequency)!=0) {
			LOG_INF("The hardware design results in the actual frequency not being equal to the set frequency,actual freq %u",
				   (info->clock_frequency)/(((SPI_DW_CLK_DIVIDER(info->clock_frequency,config->frequency)))&SCKDV_BIT));
		}
		write_ser(dev, 0);
	}

	if (spi_dw_is_slave(spi)) {
		LOG_DBG("Installed slave config %p:"
			    " ws/dfs %u/%u, mode %u/%u/%u",
			    config,
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0);
	} else {
		LOG_DBG("Installed master config %p: freq %uHz (div = %u),"
			    " ws/dfs %u/%u, mode %u/%u/%u, slave %u",
			    config, config->frequency,
			    SPI_DW_CLK_DIVIDER(info->clock_frequency,
					       config->frequency),
			    SPI_WORD_SIZE_GET(config->operation), spi->dfs,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPOL) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_CPHA) ? 1 : 0,
			    (SPI_MODE_GET(config->operation) &
			     SPI_MODE_LOOP) ? 1 : 0,
			    config->slave);
	}

	return 0;
}

static uint32_t spi_dw_compute_ndf(const struct spi_buf *rx_bufs,
				   size_t rx_count, uint8_t dfs)
{
	uint32_t len = 0U;

	for (; rx_count; rx_bufs++, rx_count--) {
		if (len > (UINT16_MAX - rx_bufs->len)) {
			goto error;
		}

		len += rx_bufs->len;
	}

	if (len) {
		return (len / dfs) - 1;
	}
error:
	return UINT32_MAX;
}

static void spi_dw_update_txftlr(const struct device *dev,
				 struct spi_dw_data *spi)
{
	const struct spi_dw_config *info = dev->config;
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 1) / 2;
	uint32_t reg_data = dw_spi_txftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (!spi->ctx.tx_len) {
			reg_data = 0U;
		} else if (spi->ctx.tx_len < dw_spi_txftlr_dflt) {
			reg_data = spi->ctx.tx_len - 1;
		}
	}

	LOG_DBG("TxFTLR: %u", reg_data);

	write_txftlr(dev, reg_data);
}

static int transceive(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t dw_spi_rxftlr_dflt = (info->fifo_depth * 5) / 8;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure(dev, spi, config);
	if (ret) {
		goto out;
	}

	if (!rx_bufs || !rx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_TX;
	} else if (!tx_bufs || !tx_bufs->buffers) {
		tmod = DW_SPI_CTRLR0_TMOD_RX;
	}

	/* ToDo: add a way to determine EEPROM mode */

	if (tmod >= DW_SPI_CTRLR0_TMOD_RX &&
	    !spi_dw_is_slave(spi)) {
		reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
		if (reg_data == UINT32_MAX) {
			ret = -EINVAL;
			goto out;
		}

		write_ctrlr1(dev, reg_data);
	} else {
		write_ctrlr1(dev, 0);
	}

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
		if (tmod == DW_SPI_CTRLR0_TMOD_RX) {
			tmod |= DW_SPI_CTRLR0_SLV_OE;
		} else {
			tmod &= ~DW_SPI_CTRLR0_SLV_OE;
		}
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);

	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);

	spi->fifo_diff = 0U;

	/* Tx Threshold */
	spi_dw_update_txftlr(dev, spi);

	/* Does Rx thresholds needs to be lower? */
	reg_data = dw_spi_rxftlr_dflt;

	if (spi_dw_is_slave(spi)) {
		if (spi->ctx.rx_len &&
		    spi->ctx.rx_len < dw_spi_rxftlr_dflt) {
			reg_data = spi->ctx.rx_len - 1;
		}
	} else {
		if (spi->ctx.rx_len && spi->ctx.rx_len < info->fifo_depth) {
			reg_data = spi->ctx.rx_len - 1;
		}
	}

	/* Rx Threshold */
	write_rxftlr(dev, reg_data);

	/* Enable interrupts */
	reg_data = !rx_bufs ?
		DW_SPI_IMR_UNMASK & DW_SPI_IMR_MASK_RX :
		DW_SPI_IMR_UNMASK;
	write_imr(dev, reg_data);

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}

	LOG_DBG("Enabling controller");
	set_bit_ssienr(dev);

	ret = spi_context_wait_for_completion(&spi->ctx);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&spi->ctx) && !ret) {
		ret = spi->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

out:
	spi_context_release(&spi->ctx, ret);

	pm_device_busy_clear(dev);

	return ret;
}

static int transceive_read(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata,uint32_t opcode,uint32_t addr)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure_support_all(dev, spi, config);
	if (ret) {
		goto out;
	}

	tmod = DW_SPI_CTRLR0_TMOD_RX;

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */

		tmod |= DW_SPI_CTRLR0_SLV_OE;
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);

	if (tmod >= DW_SPI_CTRLR0_TMOD_RX &&
			!spi_dw_is_slave(spi)) {
			reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
			if (reg_data == UINT32_MAX) {
				ret = -EINVAL;
				goto out;
			}
			write_ctrlr1(dev,reg_data);
	} else {
		write_ctrlr1(dev, 0);
	}

	write_dmacr(dev, 0);

	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);


	/* Tx Threshold */
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 3) / 4;
	write_txftlr(dev, dw_spi_txftlr_dflt);

	/* Rx Threshold */
	write_rxftlr(dev, 0);

	/* Enable interrupts */

	reg_data=DW_SPI_IMR_RX_MODE;

	write_imr(dev, reg_data);

	set_bit_ssienr(dev);

	write_dr(dev, opcode);
	write_dr(dev, addr);

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
			write_ser(dev, BIT(config->slave));
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}


	ret = spi_context_wait_for_completion(&spi->ctx);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&spi->ctx) && !ret) {
		ret = spi->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

out:
	spi_context_release(&spi->ctx, ret);

	pm_device_busy_clear(dev);

	return ret;
}

#define SPI_DMA_LLI_BLOCK_WORDS 2047
static int build_rx_lli_chain(struct spi_dw_data *spi,
		      struct dma_block_config *blk,
		      uint32_t *blk_cnt,
		      uintptr_t dr_addr,
		      uintptr_t dst_addr,
		      size_t len_bytes)
{
	if (len_bytes == 0) { *blk_cnt = 0; return 0; }
	if (len_bytes % spi->dfs) { return -EINVAL; }

	uint32_t words = len_bytes / spi->dfs;
	uint32_t wpb   = SPI_DMA_LLI_BLOCK_WORDS;

	uint32_t need  = (words + wpb - 1) / wpb;
	if (need > CONFIG_DMA_DW_LLI_POOL_SIZE) return -E2BIG;

	size_t step_bytes = (size_t)wpb * spi->dfs;
	uint32_t remain = words;

	for (uint32_t i = 0; i < need; i++) {
		uint32_t w = (remain > wpb) ? wpb : remain;
		memset(&blk[i], 0, sizeof(struct dma_block_config));
		blk[i].block_size      = w;
		blk[i].source_address  = (uint32_t)dr_addr;
		blk[i].dest_address    = (uint32_t)(dst_addr + (size_t)i * step_bytes);
		blk[i].source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		blk[i].dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;
		blk[i].next_block      = (i + 1 < need) ? &blk[i + 1] : NULL;
		remain -= w;
	}
	*blk_cnt = need;
	return 0;
}

static int transceive_read_144(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata,uint32_t opcode,uint32_t addr)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;

	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure_support_all(dev, spi, config);
	if (ret) {
		goto out;
	}
	tmod = DW_SPI_CTRLR0_TMOD_RX;

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */

		tmod |= DW_SPI_CTRLR0_SLV_OE;
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);

	if (tmod >= DW_SPI_CTRLR0_TMOD_RX &&
			!spi_dw_is_slave(spi)) {
			reg_data = spi_dw_compute_ndf(rx_bufs->buffers,
					      rx_bufs->count,
					      spi->dfs);
			if (reg_data == UINT32_MAX) {
				ret = -EINVAL;
				goto out;
			}
			write_ctrlr1(dev,reg_data);
	} else {
		write_ctrlr1(dev, 0);
	}
	write_imr(dev, DW_SPI_ISR_ERRORS_MASK);
	clear_interrupts(dev);

	set_bit_ssienr(dev);

	write_dr(dev, opcode);
	write_dr(dev, addr);

	if (info->dev_dma_rx == NULL || !device_is_ready(info->dev_dma_rx)) {
		LOG_ERR("RX DMA device not ready");
		ret = -ENODEV;
		goto end_xfer;
	}
	struct dma_block_config blk[CONFIG_DMA_DW_LLI_POOL_SIZE];
	uint32_t blk_cnt = 0;
	uintptr_t dr_addr  = (uintptr_t)(DEVICE_MMIO_GET(dev) + ((spi->dfs==4)?DW_SPI_DR_REVERSED:DW_SPI_REG_DR));
	uintptr_t dst_addr = (uintptr_t)rx_bufs->buffers[0].buf;
	size_t len_bytes   = rx_bufs->buffers[0].len;

	ret = build_rx_lli_chain(spi, blk, &blk_cnt, dr_addr, dst_addr, len_bytes);
	if (ret) {
		LOG_ERR("build_rx_lli_chain failed: %d (len=%u, dfs=%u)", ret, (unsigned)len_bytes, spi->dfs);
		goto end_xfer;
	}

	/* Config DMA Config */
	memset(&spi->dma_cfg_rx, 0, sizeof(spi->dma_cfg_rx));
	spi->dma_cfg_rx.dma_slot          = info->dma_handshake_rx;
	spi->dma_cfg_rx.channel_direction = PERIPHERAL_TO_MEMORY;
	spi->dma_cfg_rx.complete_callback_en=0;
	spi->dma_cfg_rx.channel_priority = 0;
	spi->dma_cfg_rx.source_data_size  = spi->dfs;
	spi->dma_cfg_rx.dest_data_size    = spi->dfs;
	spi->dma_cfg_rx.source_burst_length = 0;
	spi->dma_cfg_rx.dest_burst_length  = 0;
	spi->dma_cfg_rx.block_count       = blk_cnt;
	spi->dma_cfg_rx.head_block        = &blk[0];
	spi->dma_cfg_rx.user_data         = (void *)dev;
	spi->dma_cfg_rx.dma_callback      = spi_dw_dma_rx_callback;

	k_sem_reset(&spi->dma_rx_sem);

	/* Config DMA controller */
	ret = dma_config(info->dev_dma_rx, info->dma_channel_rx, &spi->dma_cfg_rx);
	if (ret < 0) {
		LOG_ERR("dma_config rx failed %d", ret);
		return ret;
	}

	/* Start DMA */
	ret = dma_start(info->dev_dma_rx, info->dma_channel_rx);
	if (ret < 0) {
		LOG_ERR("dma_start rx failed %d", ret);
		return ret;
	}

	/* Open SPI DMA */
	write_dmardlr(dev, 0);
	write_dmacr(dev, DW_SPI_RDMAE_BIT);  // Enable RX DMA

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
			write_ser(dev, BIT(config->slave));
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}

	/* Wait DMA finish */
	ret = k_sem_take(&spi->dma_rx_sem, K_MSEC(SPI_DW_DMA_WAIT_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("DMA RX timeout");
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	sys_cache_data_invd_range((void *)(rx_bufs->buffers[0].buf), rx_bufs->buffers[0].len);

	write_dmacr(dev, 0);
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_interrupts(dev);
	clear_bit_ssienr(dev);
end_xfer:
	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, false);
			write_ser(dev, 0);
		} else {
			write_ser(dev, 0);
		}
	}

	spi_context_complete(&spi->ctx, dev, 0);
out:
	spi_context_release(&spi->ctx, ret);

#ifdef CONFIG_PM_DEVICE
    pm_device_busy_clear(dev);
#endif

	return ret;
}

static int transceive_write(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata,uint32_t opcode,uint32_t addr)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure_support_all(dev, spi, config);
	if (ret) {
		goto out;
	}

	tmod = DW_SPI_CTRLR0_TMOD_TX;

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
			tmod &= ~DW_SPI_CTRLR0_SLV_OE;
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);


	/* Set buffers info */
	spi_context_buffers_setup(&spi->ctx, tx_bufs, rx_bufs, spi->dfs);

	set_bit_ssienr(dev);
	write_dr(dev, opcode);
	write_dr(dev, addr);

	write_dmacr(dev, 0);
	/* Tx Threshold */
	uint32_t dw_spi_txftlr_dflt = (info->fifo_depth * 3) / 4;
	write_txftlr(dev, dw_spi_txftlr_dflt);


	/* Rx Threshold */
	write_rxftlr(dev, 0);


	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
			write_ser(dev, BIT(config->slave));
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}

	/* Enable interrupts */

	reg_data=DW_SPI_IMR_TX_MODE;

	write_imr(dev, reg_data);


	ret = spi_context_wait_for_completion(&spi->ctx);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&spi->ctx) && !ret) {
		ret = spi->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

out:
	spi_context_release(&spi->ctx, ret);

	pm_device_busy_clear(dev);

	return ret;
}
static void spi_dw_dma_tx_callback(const struct device *dev_dma,
				   void *user, uint32_t channel, int status)
{
	const struct device *dev = (const struct device *)user;
	struct spi_dw_data *spi = dev->data;

	if (status == DMA_STATUS_COMPLETE || status < 0) {
	k_sem_give(&spi->dma_tx_sem);
	}
}
static int build_tx_lli_chain(struct spi_dw_data *spi,
		      struct dma_block_config *blk,
		      uint32_t *blk_cnt,
		      uintptr_t src_addr,
		      uintptr_t dr_addr,
		      size_t len_bytes)
{
	if (len_bytes == 0) { *blk_cnt = 0; return 0; }
	if (len_bytes % spi->dfs) { return -EINVAL; }

	uint32_t words = len_bytes / spi->dfs;
	uint32_t wpb   = SPI_DMA_LLI_BLOCK_WORDS;
	uint32_t need  = (words + wpb - 1) / wpb;
	if (need > CONFIG_DMA_DW_LLI_POOL_SIZE) return -E2BIG;

	size_t step_bytes = (size_t)wpb * spi->dfs;
	uint32_t remain = words;

	for (uint32_t i = 0; i < need; i++) {
	uint32_t w = (remain > wpb) ? wpb : remain;
	memset(&blk[i], 0, sizeof(struct dma_block_config));
	blk[i].block_size      = w;
	blk[i].source_address  = (uint32_t)(src_addr + (size_t)i * step_bytes);
	blk[i].dest_address    = (uint32_t)dr_addr;
	blk[i].source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk[i].dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE;
	blk[i].next_block      = (i + 1 < need) ? &blk[i + 1] : NULL;
	remain -= w;
	}

	*blk_cnt = need;
	return 0;
}
static int transceive_write_144(const struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous,
		      spi_callback_t cb,
		      void *userdata,uint32_t opcode,uint32_t addr)
{
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
	uint32_t tmod = DW_SPI_CTRLR0_TMOD_TX_RX;
	uint32_t reg_data;
	int ret;

	spi_context_lock(&spi->ctx, asynchronous, cb, userdata, config);

#ifdef CONFIG_PM_DEVICE
	if (!pm_device_is_busy(dev)) {
		pm_device_busy_set(dev);
	}
#endif /* CONFIG_PM_DEVICE */

	/* Configure */
	ret = spi_dw_configure_support_all(dev, spi, config);
	if (ret) {
		goto out;
	}

	tmod = DW_SPI_CTRLR0_TMOD_TX;

	if (spi_dw_is_slave(spi)) {
		/* Enabling MISO line relevantly */
			tmod &= ~DW_SPI_CTRLR0_SLV_OE;
	}

	/* Updating TMOD in CTRLR0 register */
	reg_data = read_ctrlr0(dev);
	reg_data &= ~DW_SPI_CTRLR0_TMOD_RESET;
	reg_data |= tmod;

	write_ctrlr0(dev, reg_data);

	write_imr(dev, DW_SPI_ISR_ERRORS_MASK);
	clear_interrupts(dev);

	set_bit_ssienr(dev);
	write_dr(dev, opcode);
	write_dr(dev, addr);

	if (info->dev_dma_tx == NULL || !device_is_ready(info->dev_dma_tx)) {
		LOG_ERR("TX DMA device not ready");
		ret = -ENODEV;
		goto end_xfer;
	}
	struct dma_block_config blk[CONFIG_DMA_DW_LLI_POOL_SIZE];
	uint32_t blk_cnt = 0;
	uintptr_t dr_addr  = (uintptr_t)(DEVICE_MMIO_GET(dev) +
			     ((spi->dfs==4) ? DW_SPI_DR_REVERSED : DW_SPI_REG_DR));
	uintptr_t src_addr = (uintptr_t)tx_bufs->buffers[0].buf;
	size_t len_bytes= tx_bufs->buffers[0].len;

	ret = build_tx_lli_chain(spi, blk, &blk_cnt, src_addr, dr_addr, len_bytes);
	if (ret) {
		LOG_ERR("build_tx_lli_chain failed: %d (len=%u, dfs=%u)",
			      ret, (unsigned)len_bytes, spi->dfs);
	goto end_xfer;
	}

	/* clear cache，Prevent DMA from not reading the latest data */
	sys_cache_data_flush_range((void *)(uintptr_t)src_addr, len_bytes);

	/* Config DMA Config */
	memset(&spi->dma_cfg_tx, 0, sizeof(spi->dma_cfg_tx));
	spi->dma_cfg_tx.dma_slot            = info->dma_handshake_tx;
	spi->dma_cfg_tx.channel_direction   = MEMORY_TO_PERIPHERAL;
	spi->dma_cfg_tx.complete_callback_en= 0;
	spi->dma_cfg_tx.channel_priority    = 0;
	spi->dma_cfg_tx.source_data_size    = spi->dfs;
	spi->dma_cfg_tx.dest_data_size      = spi->dfs;
	spi->dma_cfg_tx.source_burst_length = 0;
	spi->dma_cfg_tx.dest_burst_length   = 0;
	spi->dma_cfg_tx.block_count         = blk_cnt;
	spi->dma_cfg_tx.head_block          = &blk[0];
	spi->dma_cfg_tx.user_data           = (void *)dev;
	spi->dma_cfg_tx.dma_callback        = spi_dw_dma_tx_callback;

	k_sem_reset(&spi->dma_tx_sem);

	/* Config DMA controller */
	ret = dma_config(info->dev_dma_tx, info->dma_channel_tx, &spi->dma_cfg_tx);
	if (ret < 0) {
		LOG_ERR("dma_config tx failed %d", ret);
		goto end_xfer;
	}

	/* Start DMA */
	ret = dma_start(info->dev_dma_tx, info->dma_channel_tx);
	if (ret < 0) {
		LOG_ERR("dma_start tx failed %d", ret);
		goto end_xfer;
	}

	/* Open SPI DMA */
	write_dmatdlr(dev, 0);
	write_dmacr(dev, DW_SPI_TDMAE_BIT);    /* Enable TX DMA */

	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, true);
			write_ser(dev, BIT(config->slave));
		} else {
			write_ser(dev, BIT(config->slave));
		}
	}
	/* Wait DMA finish */
	ret = k_sem_take(&spi->dma_tx_sem, K_MSEC(SPI_DW_DMA_WAIT_TIMEOUT_MS));
	if (ret < 0) {
		LOG_ERR("DMA TX timeout");
		ret = -ETIMEDOUT;
	} else {
		ret = 0;
	}

	LOG_ERR("daolema");

	write_dmacr(dev, 0);
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_interrupts(dev);
	clear_bit_ssienr(dev);
end_xfer:
	if (!spi_dw_is_slave(spi)) {
		/* if cs is not defined as gpio, use hw cs */
		if (spi_cs_is_gpio(config)) {
			spi_context_cs_control(&spi->ctx, false);
			write_ser(dev, 0);
		} else {
			write_ser(dev, 0);
		}
	}
	spi_context_complete(&spi->ctx, dev, 0);

out:
	spi_context_release(&spi->ctx, ret);

#ifdef CONFIG_PM_DEVICE
	pm_device_busy_clear(dev);
#endif

	return ret;
}


static int spi_dw_transceive(const struct device *dev,
			     const struct spi_config *config,
			     const struct spi_buf_set *tx_bufs,
			     const struct spi_buf_set *rx_bufs)
{
	LOG_DBG("%p, %p, %p", dev, tx_bufs, rx_bufs);

	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_dw_transceive_async(const struct device *dev,
				   const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs,
				   spi_callback_t cb,
				   void *userdata)
{
	LOG_DBG("%p, %p, %p, %p, %p", dev, tx_bufs, rx_bufs, cb, userdata);

	return transceive(dev, config, tx_bufs, rx_bufs, true, cb, userdata);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_dw_release(const struct device *dev,
			  const struct spi_config *config)
{
	struct spi_dw_data *spi = dev->data;

	if (!spi_context_configured(&spi->ctx, config)) {
		return -EINVAL;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

void spi_dw_isr(const struct device *dev)
{
	uint32_t int_status;
	int error;

	int_status = read_isr(dev);

	LOG_DBG("SPI %p int_status 0x%x - (tx: %d, rx: %d)", dev, int_status,
		read_txflr(dev), read_rxflr(dev));

	if (int_status & DW_SPI_ISR_ERRORS_MASK) {
		error = -EIO;
		goto out;
	}

	error = 0;

	if (int_status & DW_SPI_ISR_RXFIS) {
		pull_data(dev);
	}

	if (int_status & DW_SPI_ISR_TXEIS) {
		push_data(dev);
	}

out:
	clear_interrupts(dev);
	completed(dev, error);
}

#define SPI_NOR_DUMMY_CYCLE_MAX 5
static int spi_dw_nor_transceive(const struct device *dev,
						const struct spi_config *config,
						struct spi_nor_op_info *op_info)
{
	const struct spi_dw_config *info = dev->config;
	bool is_addressed = (op_info->addr_len > 0);
	bool is_write = (SPI_NOR_DATA_DIRECT_OUT == op_info->data_direct);
	uint8_t buf[5 + SPI_NOR_DUMMY_CYCLE_MAX] = { 0 };
	uint32_t spi_ctrlr0=0;
	uint32_t dr_addr_dual_quad=0;

	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = 1,
		},
		{
			.buf = op_info->buf,
			.len = op_info->data_len,
		}
	};

	struct spi_buf spi_buf_dual_quad[1] = {

		{
			.buf = op_info->buf,
			.len = op_info->data_len,
		},

	};


	buf[0] = op_info->opcode;
	if (is_addressed) {
		bool access_24bit = (3 == op_info->addr_len);
		bool access_32bit = (4 == op_info->addr_len);
		bool use_32bit = (access_32bit || (!access_24bit));
		union {
			uint32_t u32;
			uint8_t u8[4];
		} addr32 = {
			.u32 = sys_cpu_to_be32(op_info->addr),
		};

		if (use_32bit) {
			memcpy(&buf[1], &addr32.u8[0], 4);
			dr_addr_dual_quad =buf[1]<<24|buf[2]<<16|buf[3]<<8|buf[4];
			spi_buf[0].len += 4;
		} else {
			memcpy(&buf[1], &addr32.u8[1], 3);
			dr_addr_dual_quad =buf[1]<<16|buf[2]<<8|buf[3];
			spi_buf[0].len += 3;
		}

		spi_buf[0].len +=
			(op_info->dummy_cycle / (8 / JESD216_GET_ADDR_BUSWIDTH(op_info->mode)));
	};

	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (op_info->data_len != 0) ? 2 : 1,
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2,
	};

	const struct spi_buf_set tx_set_dual_quad = {
		.buffers = spi_buf_dual_quad,
		.count = 1,
	};

	const struct spi_buf_set rx_set_dual_quad = {
		.buffers = spi_buf_dual_quad,
		.count = 1,
	};

	if (!is_write && op_info->mode == JESD216_MODE_144) {

		struct spi_config config_copy = *config;
		/* Set QUAD lines (preserve other flags) */
		config_copy.operation &= ~SPI_LINES_MASK;
		config_copy.operation |= SPI_LINES_QUAD;
		config_copy.operation &= ~SPI_WORD_SIZE_MASK;
		uint32_t dfs_bit = (op_info->data_len < info->fifo_depth) ? 8U : 32U;
		config_copy.operation |= (dfs_bit<<SPI_WORD_SIZE_SHIFT);
		const struct spi_config *config_copy_const = &config_copy;

		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1); // Instruction in Standard，Address in QUAD
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_WAIT_CYCLES(6);

		write_spi_ctrlr0(dev,spi_ctrlr0);

		return transceive_read_144(dev, config_copy_const,NULL, &rx_set_dual_quad, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);
	}else if (!is_write && op_info->mode == JESD216_MODE_114) {
		struct spi_config config_copy = *config;
		/* Set QUAD lines (preserve other flags) */
		config_copy.operation &= ~SPI_LINES_MASK;
		config_copy.operation |= SPI_LINES_QUAD;
		const struct spi_config *config_copy_const = &config_copy;

		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0); // Instruction in Standard，Address in Standard
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_WAIT_CYCLES(op_info->dummy_cycle);

		write_spi_ctrlr0(dev,spi_ctrlr0);

		return transceive_read(dev, config_copy_const,NULL, &rx_set_dual_quad, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);

	}else if (!is_write && op_info->mode == JESD216_MODE_122) {
		struct spi_config config_copy = *config;
		/* Set DUAL lines (preserve other flags) */
		config_copy.operation &= ~SPI_LINES_MASK;
		config_copy.operation |= SPI_LINES_DUAL;
		const struct spi_config *config_copy_const = &config_copy;

		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1); // Instruction in Standard，Address in DUAL
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_WAIT_CYCLES(4);

		write_spi_ctrlr0(dev,spi_ctrlr0);

		return transceive_read(dev, config_copy_const,NULL, &rx_set_dual_quad, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);

	}else if (!is_write && op_info->mode == JESD216_MODE_112) {
		struct spi_config config_copy = *config;
		/* Set DUAL lines (preserve other flags) */
		config_copy.operation &= ~SPI_LINES_MASK;
		config_copy.operation |= SPI_LINES_DUAL;
		const struct spi_config *config_copy_const = &config_copy;

		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0); // Instruction in Standard，Address in Standard
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));
		spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_WAIT_CYCLES(op_info->dummy_cycle);

		write_spi_ctrlr0(dev,spi_ctrlr0);

		return transceive_read(dev, config_copy_const,NULL, &rx_set_dual_quad, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);
	}

	if (is_write) {
		if(op_info->mode == JESD216_MODE_144){
			struct spi_config config_copy = *config;
			/* Set QUAD lines (preserve other flags) */
			config_copy.operation &= ~SPI_LINES_MASK;
			config_copy.operation |= SPI_LINES_QUAD;
			config_copy.operation &= ~SPI_WORD_SIZE_MASK;
			config_copy.operation |= (32U<<SPI_WORD_SIZE_SHIFT);
			const struct spi_config *config_copy_const = &config_copy;

			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(1);  // Instruction in Standard，Address in QUAD
			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));

			write_spi_ctrlr0(dev,spi_ctrlr0);

			return transceive_write_144(dev, config_copy_const,&tx_set_dual_quad, NULL, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);

		}else if (op_info->mode == JESD216_MODE_114) {
			struct spi_config config_copy = *config;
			/* Set QUAD lines (preserve other flags) */
			config_copy.operation &= ~SPI_LINES_MASK;
			config_copy.operation |= SPI_LINES_QUAD;
			const struct spi_config *config_copy_const = &config_copy;

			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_TRANS_TYPE(0);  //Instruction in Standard，Address in Standard
			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_INST_L_8BIT;
			spi_ctrlr0 |= DW_SPI_SPI_CTRLR0_ADDR_L((op_info->addr_len));

			write_spi_ctrlr0(dev,spi_ctrlr0);

			return transceive_write(dev, config_copy_const,&tx_set_dual_quad, NULL, false, NULL, NULL,op_info->opcode,dr_addr_dual_quad);
		}else{
			return transceive(dev, config, &tx_set, NULL, false, NULL, NULL);
		}
	}

	return transceive(dev, config, &tx_set, &rx_set, false, NULL, NULL);;
}

static int spi_dw_nor_read_init(const struct device *dev,
						const struct spi_config *config,
						struct spi_nor_op_info *op_info)
{
	int ret = 0;

	LOG_DBG("mode %08x, cmd: %x, dummy: %d, frequency: %d",
		op_info->mode, op_info->opcode, op_info->dummy_cycle,
		config->frequency);

	return ret;
}

static int spi_dw_nor_write_init(const struct device *dev,
						const struct spi_config *config,
						struct spi_nor_op_info *op_info)
{
	int ret = 0;

	LOG_DBG("mode %08x, cmd: %x, dummy: %d, frequency: %d",
		op_info->mode, op_info->opcode, op_info->dummy_cycle,
		config->frequency);

	return ret;
}

static const struct spi_nor_ops spi_dw_nor_ops = {
	.transceive = spi_dw_nor_transceive,
	.read_init = spi_dw_nor_read_init,
	.write_init = spi_dw_nor_write_init,
};

static const struct spi_driver_api dw_spi_api = {
	.transceive = spi_dw_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_dw_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_dw_release,
	.spi_nor_op = &spi_dw_nor_ops,
};

int spi_dw_init(const struct device *dev)
{
	int err;
	const struct spi_dw_config *info = dev->config;
	struct spi_dw_data *spi = dev->data;
    __maybe_unused int ret;

#if defined(CONFIG_CLOCK_CONTROL)
    if (info->ccfg.cctl_dev) {
        const struct device *clk_dev = info->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&info->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (info->reset.dev != NULL) {
        if (!device_is_ready(info->reset.dev)) {
            LOG_ERROR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(info->reset.dev, info->reset.id);
        if (ret != 0) {
            LOG_ERROR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (info->ccfg.cctl_dev) {
        const struct device *clk_dev = info->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&info->ccfg);
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(info->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_DBG("%s: Could not configure pins", dev->name);
    }
#endif

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	info->config_func();

	/* Masking interrupt and making sure controller is disabled */
	write_imr(dev, DW_SPI_IMR_MASK);
	clear_bit_ssienr(dev);

	k_sem_init(&spi->dma_rx_sem, 0, 1);
	k_sem_init(&spi->dma_tx_sem, 0, 1);


	LOG_DBG("Designware SPI driver initialized on device: %p", dev);

	err = spi_context_cs_configure_all(&spi->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&spi->ctx);

	return 0;
}

#define SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, err_int, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, err_int, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, err_int, irq));

#define SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)					\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rx_avail, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx_req, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, tx_req, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, txo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, txo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxo_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, rxu_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, mst_err, irq),		\
			    DT_INST_IRQ_BY_NAME(inst, mst_err, priority),	\
			    spi_dw_isr, DEVICE_DT_INST_GET(inst),		\
			    0);							\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx_avail, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx_req, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, txo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxo_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rxu_err, irq));		\
		irq_enable(DT_INST_IRQ_BY_NAME(inst, mst_err, irq));

#define SPI_DW_IRQ_HANDLER(inst)                                   \
void spi_dw_irq_config_##inst(void)                                \
{                                                                  \
COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 1),              \
	(IRQ_CONNECT(DT_INST_IRQN(inst),                           \
		DT_INST_IRQ(inst, priority),                       \
		spi_dw_isr, DEVICE_DT_INST_GET(inst),              \
		0);                                                \
	irq_enable(DT_INST_IRQN(inst));),                          \
	(COND_CODE_1(IS_EQ(DT_NUM_IRQS(DT_DRV_INST(inst)), 3),     \
		(SPI_CFG_IRQS_SINGLE_ERR_LINE(inst)),		   \
		(SPI_CFG_IRQS_MULTIPLE_ERR_LINES(inst)))))	   \
}

#define SPI_DW_INIT(inst)                                                                   \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(inst);))                         \
	SPI_DW_IRQ_HANDLER(inst);                                                           \
	static struct spi_dw_data spi_dw_data_##inst = {                                    \
		SPI_CONTEXT_INIT_LOCK(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_INIT_SYNC(spi_dw_data_##inst, ctx),                             \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)                     \
	};                                                                                  \
	static const struct spi_dw_config spi_dw_config_##inst = {                          \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                    \
		.clock_frequency = COND_CODE_1(                                             \
			DT_NODE_HAS_PROP(DT_INST_PHANDLE(inst, clocks), clock_frequency),   \
			(DT_INST_PROP_BY_PHANDLE(inst, clocks, clock_frequency)),           \
			(DT_INST_PROP(inst, clock_frequency))),                             \
		.config_func = spi_dw_irq_config_##inst,                                    \
		.serial_target = DT_INST_PROP(inst, serial_target),                         \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dev_dma_rx = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, rx)),))    \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dma_channel_rx = DT_INST_DMAS_CELL_BY_NAME(inst, rx, channel),))           \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dma_handshake_rx = DT_INST_DMAS_CELL_BY_NAME(inst, rx, handshake),))       \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dev_dma_tx = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(inst, tx)),))       \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dma_channel_tx = DT_INST_DMAS_CELL_BY_NAME(inst, tx, channel),))       \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, dmas),                            \
			(.dma_handshake_tx = DT_INST_DMAS_CELL_BY_NAME(inst, tx, handshake),))       \
		.fifo_depth = DT_INST_PROP(inst, fifo_depth),                               \
		.max_xfer_size = DT_INST_PROP(inst, max_xfer_size),                         \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),)) \
		COND_CODE_1(DT_INST_PROP(inst, aux_reg),                                    \
			(.read_func = aux_reg_read,                                         \
			.write_func = aux_reg_write,                                        \
			.set_bit_func = aux_reg_set_bit,                                    \
			.clear_bit_func = aux_reg_clear_bit,                                \
			.test_bit_func = aux_reg_test_bit,),                                \
			(.read_func = reg_read,                                             \
			.write_func = reg_write,                                            \
			.set_bit_func = reg_set_bit,                                        \
			.clear_bit_func = reg_clear_bit,                                    \
			.test_bit_func = reg_test_bit,))                                    \
		IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), ))                 \
		IF_ENABLED(DT_HAS_CLOCKS(inst), (.ccfg = LS_DT_CLK_CFG_ITEM(inst), ))                       \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets), (.reset = RESET_DT_SPEC_INST_GET(inst), ))  \
	};                                                                                  \
	DEVICE_DT_INST_DEFINE(inst,                                                         \
		spi_dw_init,                                                                \
		NULL,                                                                       \
		&spi_dw_data_##inst,                                                        \
		&spi_dw_config_##inst,                                                      \
		POST_KERNEL,                                                                \
		CONFIG_SPI_INIT_PRIORITY,                                                   \
		&dw_spi_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_DW_INIT)
