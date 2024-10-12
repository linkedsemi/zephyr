/*
 * Copyright (c) 2023 LinkedSemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_ls);

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <ls_hal_spi_i2s.h>
#include <reg_rcc.h>

#include "spi_context.h"

#define CPU_FREQ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)

typedef void (*irq_config_func_t)(const struct device *port);

struct spi_ls_config {
	reg_spi_t *instance;
    const struct pinctrl_dev_config *pcfg; 
#ifdef CONFIG_SPI_LS_INTERRUPT
	irq_config_func_t irq_config;
#endif
};

struct spi_ls_data {
	struct spi_context ctx;
};

static int spi_ls_configure(const struct device *dev,
				       const struct spi_config *config)
{
	const struct spi_ls_config *cfg = dev->config;
	struct spi_ls_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
    reg_spi_t *spi = cfg->instance;
    int err;
    uint32_t clock = CPU_FREQ;
    const uint32_t scaler[] = {
		SPI_BAUDRATEPRESCALER_8,
		SPI_BAUDRATEPRESCALER_16,
		SPI_BAUDRATEPRESCALER_32,
		SPI_BAUDRATEPRESCALER_64,
		SPI_BAUDRATEPRESCALER_128,
		SPI_BAUDRATEPRESCALER_256
	};

    /* Disable the selected SPI peripheral */
    REG_FIELD_WR(spi->CR1, SPI_CR1_SPE, 0);
    
	if (spi_context_configured(ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

    if (config->operation & SPI_FRAME_FORMAT_TI) {
        LOG_ERR("TI mode is not supported");
        return -ENOTSUP;
    }

    /* Word sizes other than 8 and 16 bits has not been implemented */
    if ((SPI_WORD_SIZE_GET(config->operation) != 8)
	    && (SPI_WORD_SIZE_GET(config->operation) != 16)) {
		LOG_ERR("Word sizes other than 8 and 16 bits are not supported");
		return -ENOTSUP;
	}

    if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_MASTER) {
        MODIFY_REG(spi->CR1, SPI_CR1_MSTR_MASK, SPI_MODE_MASTER);
	} else {
        MODIFY_REG(spi->CR1, SPI_CR1_MSTR_MASK, SPI_MODE_SLAVE);
    }

	if (config->operation & SPI_TRANSFER_LSB) {
        MODIFY_REG(spi->CR1, SPI_CR1_LSBFIRST_MASK, SPI_FIRSTBIT_LSB);
	} else {
        MODIFY_REG(spi->CR1, SPI_CR1_LSBFIRST_MASK, SPI_FIRSTBIT_MSB);
    }

    /* Word sizes other than 8 bits and 16 bits has not been implemented */
    if (SPI_WORD_SIZE_GET(config->operation) == 8) {
        MODIFY_REG(spi->CR2, SPI_CR2_DS_MASK, SPI_DATASIZE_8BIT);
	} else { 
        MODIFY_REG(spi->CR2, SPI_CR2_DS_MASK, SPI_DATASIZE_16BIT);
	} 

    if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
        MODIFY_REG(spi->CR1, SPI_CR1_CPOL_MASK, SPI_POLARITY_HIGH);
	} else {
        MODIFY_REG(spi->CR1, SPI_CR1_CPOL_MASK, SPI_POLARITY_LOW);
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
        MODIFY_REG(spi->CR1, SPI_CR1_CPHA_MASK, SPI_PHASE_2EDGE);
	} else {
        MODIFY_REG(spi->CR1, SPI_CR1_CPHA_MASK, SPI_PHASE_1EDGE);
	}

	if (8 * config->frequency > CPU_FREQ) {
		LOG_ERR("Frequency greater than supported in master mode");
		return -EINVAL;
	}

    for (uint8_t i = 0U; i <= ARRAY_SIZE(scaler); i++) {
		uint32_t clk = clock >> (i + 3);
		if (clk <= config->frequency) {
			MODIFY_REG(spi->CR1, SPI_CR1_BR_MASK, scaler[i]);
			break;
		}
	}

    err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("applying SPI pinctrl state failed");
		return err;
	}

    REG_FIELD_WR(spi->CR2, SPI_CR2_SSOE, 1);
    REG_FIELD_WR(spi->CR2, SPI_CR2_TXFTH, 4);

    ctx->config = config;
    spi->IER = SPI_IT_ERR;

	return 0;
}

static int spi_ls_get_err(reg_spi_t *spi)
{
	uint32_t sr = READ_REG(spi->IFM);

	if (sr & SPI_IFM_MODFFM_MASK) {
        LOG_ERR("master mode fault");
		return -EIO;
	}

    if (sr & SPI_IFM_OVRFM_MASK) {
        LOG_ERR("fifo overrun error");
		return -EIO;
	}

    if (sr & SPI_IFM_FREFM_MASK) {
        LOG_ERR("frame format error");
		return -EIO;
	}

	return 0;
}

static void spi_ls_data_exchange_master(reg_spi_t *spi, struct spi_ls_data *data)
{
    uint16_t tx_frame = 0, rx_frame;

    while (!REG_FIELD_RD(spi->SR, SPI_SR_TXE));
    
    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		if (spi_context_tx_buf_on(&data->ctx)) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
		}
        *((uint8_t *)&spi->DR) = tx_frame;
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 1, 1);
	} else {
		if (spi_context_tx_buf_on(&data->ctx)) {
			tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
		}
        spi->DR = tx_frame;
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 2, 1);
	}

    while (!REG_FIELD_RD(spi->SR, SPI_SR_RXNE));

    if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
        rx_frame = spi->DR;
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 1, 1);
	} else {
		rx_frame = spi->DR;
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 2, 1);
	}
}

static void spi_ls_data_exchange_slave(reg_spi_t *spi, struct spi_ls_data *data)
{
    if (REG_FIELD_RD(spi->SR, SPI_SR_TXE) && spi_context_tx_on(&data->ctx)) {
		uint16_t tx_frame;

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
			*((uint8_t *)&spi->DR) = tx_frame;
			spi_context_update_tx(&data->ctx, 1, 1);
		} else {
			tx_frame = UNALIGNED_GET((uint16_t *)(data->ctx.tx_buf));
			spi->DR = tx_frame;
			spi_context_update_tx(&data->ctx, 2, 1);
		}
	}

	if (REG_FIELD_RD(spi->SR, SPI_SR_RXNE) && spi_context_rx_buf_on(&data->ctx)) {
		uint16_t rx_frame;

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			rx_frame = spi->DR;
			UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 1, 1);
		} else {
			rx_frame = spi->DR;
			UNALIGNED_PUT(rx_frame, (uint16_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 2, 1);
		}
	}
}

static int spi_data_exchange(reg_spi_t *spi, struct spi_ls_data *data)
{
    uint16_t operation = data->ctx.config->operation;

	if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER) {
		spi_ls_data_exchange_master(spi, data);
	} else {
		spi_ls_data_exchange_slave(spi, data);
	}

	return spi_ls_get_err(spi);
}

static void spi_ls_complete(const struct device *dev, int status)
{
    const struct spi_ls_config *cfg = dev->config;
	struct spi_ls_data *data = dev->data;
	reg_spi_t *spi = cfg->instance;

#ifdef CONFIG_SPI_LS_INTERRUPT
	spi->IDR = SPI_IT_TXE | SPI_IT_RXNE;
#endif /* CONFIG_SPI_LS_INTERRUPT */

    if (SPI_OP_MODE_GET(data->ctx.config->operation) == SPI_OP_MODE_MASTER) {
        /* Check SR busy status */
        while (REG_FIELD_RD(spi->SR,SPI_SR_BSY) == 1U);

        spi_context_cs_control(&data->ctx, false);
    }

	if (!(data->ctx.config->operation & SPI_HOLD_ON_CS)) {
        /* disable the selected SPI peripheral */
		REG_FIELD_WR(spi->CR1, SPI_CR1_SPE, 0);
	}

#ifdef CONFIG_SPI_LS_INTERRUPT
	spi_context_complete(&data->ctx, dev, status);
#endif
}

#ifdef CONFIG_SPI_LS_INTERRUPT
static void spi_ls_isr(const struct device *dev)
{
	const struct spi_ls_config *cfg = dev->config;
	struct spi_ls_data *data = dev->data;
	reg_spi_t *spi = cfg->instance;
	int err;

	err = spi_ls_get_err(spi);
	if (err) {
		spi_ls_complete(dev, err);
		return;
	}

	if (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
		err = spi_data_exchange(spi, data);
	}

	if (err || !(spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx))) {
        spi_ls_complete(dev, err);
	}

    spi->ICR = SPI_ICR_TXEIC_MASK;
}
#endif

static int spi_ls_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	const struct spi_ls_config *cfg = dev->config;
	struct spi_ls_data *data = dev->data;
    struct spi_context *ctx = &data->ctx;
	reg_spi_t *spi = cfg->instance;
	int err;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

	spi_context_lock(ctx, false, NULL, NULL, config);

	err = spi_ls_configure(dev, config);
	if (err) {
		goto done;
	}

	/* Set buffers info */
	if (SPI_WORD_SIZE_GET(config->operation) == 8) {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
	} else {
		spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 2);
	}

	spi_context_cs_control(ctx, true);

    /* Enable the selected SPI peripheral */
    REG_FIELD_WR(spi->CR1, SPI_CR1_SPE, 1);
    
#ifdef CONFIG_SPI_LS_INTERRUPT
	if (rx_bufs) {
		spi->ICR = SPI_IT_RXNE;
        spi->IER = SPI_IT_RXNE;
	}

    spi->ICR = SPI_IT_TXE;
    spi->IER = SPI_IT_TXE;
	
	err = spi_context_wait_for_completion(&data->ctx);
#else
	do {
		err = spi_data_exchange(spi, data);
	} while (spi_context_tx_on(ctx) || spi_context_rx_on(ctx));
#endif /* CONFIG_SPI_LS_INTERRUPT */

   spi_ls_complete(dev, err);

    #ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&data->ctx) && !err) {
		err = data->ctx.recv_frames;
	}
    #endif /* CONFIG_SPI_SLAVE */

done:
	spi_context_release(ctx, err);
	return err;
}

static int spi_ls_release(const struct device *dev,
				     const struct spi_config *config)
{
	const struct spi_ls_config *cfg = dev->config;
	struct spi_ls_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	if (!spi_context_configured(ctx, config)) {
		return -EINVAL;
	}

	if (REG_FIELD_RD(cfg->instance->SR, SPI_SR_BSY) == 1U) {
		return -EBUSY;
	}

	spi_context_unlock_unconditionally(ctx);
    /* disable the selected SPI peripheral */
	REG_FIELD_WR(cfg->instance->CR1, SPI_CR1_SPE, 0);

	return 0;
}

static const struct spi_driver_api spi_ls_driver_api = {
	.transceive = spi_ls_transceive,
	.release = spi_ls_release,
};

#ifdef CONFIG_SPI_LS_INTERRUPT
#define LS_SPI_IRQ_HANDLER_DECL(id)					\
	static void spi_ls_irq_config_func_##id(const struct device *dev)
#define LS_SPI_IRQ_HANDLER_FUNC(id)					\
	.irq_config = spi_ls_irq_config_func_##id,
#define LS_SPI_IRQ_HANDLER(id)					\
static void spi_ls_irq_config_func_##id(const struct device *dev)		\
{									\
	IRQ_CONNECT(DT_INST_IRQN(id),					\
		    DT_INST_IRQ(id, priority),				\
		    spi_ls_isr, DEVICE_DT_INST_GET(id), 0);		\
	irq_enable(DT_INST_IRQN(id));					\
}
#else
#define LS_SPI_IRQ_HANDLER_DECL(id)
#define LS_SPI_IRQ_HANDLER_FUNC(id)
#define LS_SPI_IRQ_HANDLER(id)
#endif /* CONFIG_SPI_LS_INTERRUPT */

static void spi_clock_init(void)
{
    REG_FIELD_WR(RCC->APB1RST, RCC_SPI2, 1);
    REG_FIELD_WR(RCC->APB1RST, RCC_SPI2, 0);
    REG_FIELD_WR(RCC->APB1EN, RCC_SPI2, 1);
}

static int spi_ls_init(const struct device *dev)
{
	struct spi_ls_data *data __attribute__((unused)) = dev->data;
	int err;

#ifdef CONFIG_SPI_LS_INTERRUPT
    const struct spi_ls_config *cfg = dev->config;
	cfg->irq_config(dev);
#endif

    spi_clock_init();

	err = spi_context_cs_configure_all(&data->ctx);
	if (err < 0) {
		return err;
	}

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define LS_SPI_INIT(id)				\
PINCTRL_DT_INST_DEFINE(id);	    	\
LS_SPI_IRQ_HANDLER_DECL(id);					\
									\
static const struct spi_ls_config spi_ls_cfg_##id = {		\
	.instance = (reg_spi_t *) DT_INST_REG_ADDR(id),			\
    .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),               \
    LS_SPI_IRQ_HANDLER_FUNC(id)					\
};									\
									\
static struct spi_ls_data spi_ls_dev_data_##id = {		    \
	SPI_CONTEXT_INIT_LOCK(spi_ls_dev_data_##id, ctx),		\
	SPI_CONTEXT_INIT_SYNC(spi_ls_dev_data_##id, ctx),		\
    SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(id), ctx)	\
};									\
									\
DEVICE_DT_INST_DEFINE(id, &spi_ls_init, NULL,			\
		    &spi_ls_dev_data_##id, &spi_ls_cfg_##id,	\
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
		    &spi_ls_driver_api);                \
                                    \
LS_SPI_IRQ_HANDLER(id)

DT_INST_FOREACH_STATUS_OKAY(LS_SPI_INIT)

