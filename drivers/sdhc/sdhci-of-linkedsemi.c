/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_sdhci

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#include "sdhci.h"

LOG_MODULE_REGISTER(linkedsemi_sdhci, CONFIG_SDHC_LOG_LEVEL);

#define LINKEDSEMI_SDHCI_RESET_TIMEOUT_VALUE (1000000U)

#define LINKEDSEMI_SDHCI_DEFAULT_TIMEOUT (5000U)

#define LINKEDSEMI_SDHCI_PERI_BUS_FREQ MHZ(50)

struct linkedsemi_sdhci_config {
    uint32_t response_timeout;
    uint32_t cd_debounce_clocks;
    uint32_t data_timeout;
    uint32_t min_bus_freq;
    uint32_t max_bus_freq;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    void (*irq_config_func)(const struct device *dev);
};

struct linkedsemi_sdhci_data {
    struct sdhci_host host;
    struct k_mutex access_mutex;
};

/*
 * SDHCI interrupt service routine
 */
static int linkedsemi_sdhci_isr(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    uint32_t status = sdhci_get_int_status_flag(host);

    if (status & (SDHCI_INT_ERROR | SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | SDHCI_INT_RESPONSE | SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL)) {
        host->error_code = (status >> 16) & 0xffff;
        if (host->error_code) {
            LOG_ERR("error: %#4.4x\n", host->error_code);
        }
        host->irq_status |= status;
        k_sem_give(&host->transfer_sem);
    }
    // if (status & SDHCI_INT_CARD_INT)
    //     sdio_irq_wakeup(host->host);
    sdhci_clear_int_status_flag(host, status);

    return 0;
}

static int linkedsemi_sdhci_reset(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

    k_mutex_lock(&dev_data->access_mutex, K_FOREVER);

    sdhci_reset(host, SDHCI_RESET_ALL);

    k_mutex_unlock(&dev_data->access_mutex);

    return 0;
}

static int linkedsemi_sdhci_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
    const struct linkedsemi_sdhci_config *dev_config = dev->config;

    memset(props, 0, sizeof(*props));
    props->f_max = dev_config->max_bus_freq;
    props->f_min = dev_config->min_bus_freq;
    props->power_delay = 500;
    props->host_caps.high_spd_support = false;
    props->host_caps.suspend_res_support = true;
    props->host_caps.vol_330_support = true;
    props->host_caps.bus_4_bit_support = true;
    props->host_caps.bus_8_bit_support = true;
    props->max_current_330 = 1024;
    return 0;
}

static int linkedsemi_sdhci_set_io(const struct device *dev, struct sdhc_io *ios)
{
    const struct linkedsemi_sdhci_config *dev_config = dev->config;
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    uint8_t ctrl;

    LOG_DBG("%s: sdhci_clk=%d, bus_width:%d\n", __func__, ios->clock, ios->bus_width);

    if (ios->clock != 0
        && (ios->clock <= dev_config->max_bus_freq)
        && (ios->clock >= dev_config->min_bus_freq)) {
        mmc_clock_freq_change(host, ios->clock);
    }

    /* Toggle card power supply */
    if (host->power_mode != ios->power_mode) {
        if (ios->power_mode == SDHC_POWER_OFF) {
            uint8_t val = sdhci_readb(host, SDHCI_POWER_CONTROL);
            val &= ~(SDHCI_POWER_ON);
            sdhci_writeb(host, val, SDHCI_POWER_CONTROL);
        } else if (ios->power_mode == SDHC_POWER_ON) {
            uint8_t val = sdhci_readb(host, SDHCI_POWER_CONTROL);
            val |= SDHCI_POWER_ON;
            sdhci_writeb(host, val, SDHCI_POWER_CONTROL);
        }
        host->power_mode = ios->power_mode;
    }

    ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
    ctrl &= ~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);
    switch (ios->bus_width) {
    case SDHC_BUS_WIDTH1BIT:
        break;
    case SDHC_BUS_WIDTH4BIT:
        ctrl |= SDHCI_CTRL_4BITBUS;
        break;
    case SDHC_BUS_WIDTH8BIT:
        ctrl |= SDHCI_CTRL_8BITBUS;
        break;
    default:
        return -ENOTSUP;
    }
    sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

    return 0;
}

static int linkedsemi_sdhci_get_card_present(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

    return sdhci_get_present_status_flag(host);
}

static int linkedsemi_sdhci_card_busy(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

    return sdhci_card_busy(host);
}

static int32_t linkedsemi_sdhci_wait_command_done(struct sdhci_host *host, struct sdhci_command *command, bool executeTuning)
{
    __ASSERT_NO_MSG(NULL != command);

    /* tuning cmd do not need to wait command done */
    if (executeTuning)
        return 0;
    /* Wait command complete or SDHC encounters error. */
    k_sem_take(&host->transfer_sem, K_FOREVER);
    if (host->error_code & SDHCI_INT_ERROR) {
        LOG_ERR("%s: Error detected in status(0x%X)!\n", __func__, host->error_code);
        host->error_code = 0;
        return -1;
    }

    return sdhci_receive_command_response(host, command);
}

static int32_t linkedsemi_sdhci_transfer_data_blocking(struct sdhci_host *host, struct sdhci_data *data, bool use_dma)
{
#if defined(CONFIG_SDHCI_SDMA_ENABLE)
    uint32_t stat;

    while (1) {
        k_sem_take(&host->transfer_sem, K_FOREVER);
        stat = host->irq_status;
        if (stat & SDHCI_INT_ERROR) {
            LOG_DBG("%s: Error detected in status(0x%x)!\n", __func__, host->error_code);
            sdhci_reg_display(host);
            return -1;
        }
        if (stat & SDHCI_INT_DMA_END) {
            sdhci_writel(host, SDHCI_INT_DMA_END, SDHCI_INT_STATUS);
            sdhci_writel(host, sdhci_readl(host, SDHCI_DMA_ADDRESS), SDHCI_DMA_ADDRESS);
        }
        if (stat & SDHCI_INT_DATA_END) {
            if (data) {
                if (data->rx_data) {
                    sys_cache_data_invd_all();
                }
            }
            return 0;
        }
    }
#else
    uint32_t stat, rdy, mask, timeout, block;

    block = 0;
    timeout = 1000000;
    rdy = SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL;
    mask = SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE;

    while (1) {
        k_sem_take(&host->transfer_sem, K_FOREVER);
        stat = host->irq_status;
        if (stat & SDHCI_INT_ERROR) {
            LOG_ERR("%s: Error detected in status(0x%X)!\n", __func__, stat);
            sdhci_reg_display(host);
            return -1;
        }
        if (stat & rdy) {
            if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & mask)) {
                continue;
            }
            if (data->rx_data) {
                uint16_t block_size = data->block_size / 4;
                for (int i = 0; i < block_size; i++) {
                    data->rx_data[i + block * block_size] = sdhci_readl(host, SDHCI_BUFFER);
                }
            } else {
                uint16_t block_size = data->block_size / 4;
                for (int i = 0; i < block_size; i++) {
                    sdhci_writel(host, data->tx_data[i + block * block_size], SDHCI_BUFFER);
                }
            }
            block++;
            if (block >= data->block_count) {
                return 0;
            }
        }
        if (timeout == 0) {
            LOG_INF("%s: Transfer data timeout\n", __func__);
            return -1;
        }
        timeout--;
        k_msleep(1);
    }
#endif
}

static int32_t linkedsemi_sdhci_transfer_blocking(struct sdhci_host *host)
{
    __ASSERT_NO_MSG(host);
    struct sdhci_command *sdhci_command = host->sdhci_command;
    struct sdhci_data *sdhci_data = host->sdhci_data;
    bool use_dma = false;
    int ret = 0;

    /* Wait until command/data bus out of busy status. */
    while (sdhci_get_present_status_flag(host) & sdhci_command_inhibit_flag) {
    }
    while (sdhci_data && (sdhci_get_present_status_flag(host) & sdhci_data_inhibit_flag)) {
    }
    sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);

    ret = sdhci_set_transfer_config(host, sdhci_command, sdhci_data);
    if (ret != 0) {
        return ret;
    }
    sdhci_writel(host, sdhci_readl(host, SDHCI_SIGNAL_ENABLE) | SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK, SDHCI_SIGNAL_ENABLE);

    host->transfer_status = 0U;
    k_sem_reset(&host->transfer_sem);
    sdhci_send_command(host, sdhci_command, use_dma);
    /* wait command done */
    ret = linkedsemi_sdhci_wait_command_done(host, sdhci_command, ((sdhci_data == NULL) ? false : sdhci_data->executeTuning));
    /* transfer data */
    if ((sdhci_data != NULL) && (ret == 0)) {
        ret = linkedsemi_sdhci_transfer_data_blocking(host, sdhci_data, use_dma);
    }
    while (sdhci_get_present_status_flag(host) & sdhci_command_inhibit_flag) {
    }
    while (sdhci_data && (sdhci_get_present_status_flag(host) & sdhci_data_inhibit_flag)) {
    }
    sdhci_writel(host, sdhci_readl(host, SDHCI_SIGNAL_ENABLE) & ~(SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK), SDHCI_SIGNAL_ENABLE);
    sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
    sdhci_reset(host, SDHCI_RESET_CMD);
    sdhci_reset(host, SDHCI_RESET_DATA);
    return ret;
}

static int linkedsemi_sdhci_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
    int ret;
    int busy_timeout = LINKEDSEMI_SDHCI_DEFAULT_TIMEOUT;
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    struct sdhci_data sdhci_data = { 0 };
    struct sdhci_command sdhci_command = { 0 };

    ret = k_mutex_lock(&dev_data->access_mutex, K_FOREVER);
    if (ret) {
        LOG_ERR("Could not access card");
        return -EBUSY;
    }

    host->irq_status = 0;
    sdhci_command.index = cmd->opcode;
    sdhci_command.argument = cmd->arg;
    /* Mask out part of response type field used for SPI commands */
    sdhci_command.responseType = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
    if (cmd->opcode == SD_STOP_TRANSMISSION) {
        sdhci_command.type = card_command_type_abort;
    } else {
        sdhci_command.type = card_command_type_normal;
    }

    host->sdhci_command = &sdhci_command;

    if (data) {
        sdhci_command.flags |= sdhci_enable_cmd_data_present_flag;
        sdhci_command.flags2 |= sdhci_enable_block_count_flag;

        if (sdhci_data.rx_data) {
            sdhci_command.flags2 |= sdhci_data_read_flag;
        }

        if (sdhci_data.block_count > 1U) {
            sdhci_command.flags2 |= (sdhci_multiple_block_flag);
            /* auto command 12 */
            if (sdhci_data.enableAutoCommand12) {
                /* Enable Auto command 12. */
                sdhci_command.flags2 |= sdhci_enable_auto_command12_flag;
            }
            /* auto command 23 */
            if (sdhci_data.enableAutoCommand23) {
                sdhci_command.flags2 |= sdhci_enable_auto_command23_flag;
            }
        }

        sdhci_data.block_size = data->block_size;
        sdhci_data.block_count = data->blocks;

        switch (cmd->opcode) {
        case SD_WRITE_SINGLE_BLOCK:
        case SD_WRITE_MULTIPLE_BLOCK:
            sdhci_data.enableAutoCommand12 = true;
            sdhci_data.tx_data = data->data;
            break;
        case SD_READ_SINGLE_BLOCK:
        case SD_READ_MULTIPLE_BLOCK:
            sdhci_data.enableAutoCommand12 = true;
            sdhci_data.rx_data = data->data;
            break;
        case SD_APP_SEND_SCR:
        case SD_SWITCH:
        case SD_APP_SEND_NUM_WRITTEN_BLK:
        case MMC_CHECK_BUS_TEST:
        case MMC_SEND_EXT_CSD:
            sdhci_data.rx_data = data->data;
            break;
        default:
            return -ENOTSUP;
        }

        host->sdhci_data = &sdhci_data;
        host->sdhci_data->timeout_ms = data->timeout_ms;
    } else {
        host->sdhci_data = NULL;
    }
    host->sdhci_command->timeout_ms = cmd->timeout_ms;

    do {
        // ret = linkedsemi_sdhci_transfer(dev, cmd, data);
        ret = linkedsemi_sdhci_transfer_blocking(host);
        if (data && ret) {
            /* Send CMD12 to stop transmission after error */
            while (busy_timeout > 0) {
                if (!sdhci_card_busy(host)) {
                    break;
                }
                /* Wait 125us before polling again */
                k_busy_wait(125);
                busy_timeout -= 125;
            }
            if (busy_timeout <= 0) {
                LOG_DBG("Card did not idle after CMD12");
                k_mutex_unlock(&dev_data->access_mutex);
                return -ETIMEDOUT;
            }
        } else {
            cmd->response[0] = host->sdhci_command->response[0];
            cmd->response[1] = host->sdhci_command->response[1];
            cmd->response[2] = host->sdhci_command->response[2];
            cmd->response[3] = host->sdhci_command->response[3];
        }
    } while (ret != 0 && (cmd->retries-- > 0));
    k_mutex_unlock(&dev_data->access_mutex);

    return 0;
}

/*
 * Early system init for SDHC
 */
static int linkedsemi_sdhci_init(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    const struct linkedsemi_sdhci_config *dev_config = dev->config;
    struct sdhci_host *host = &dev_data->host;
#if defined(CONFIG_PINCTRL)
    int ret;
    ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("SDHC pinctrl setup failed (%d)", ret);
        return ret;
    }
#endif

    host->index = 0;
    host->have_phy = false;
    host->mshc_ctrl_r = 0;
    host->rx_delay_line = 0;
    host->tx_delay_line = 0;
    host->io_fixed_1v8 = 0;
    host->max_clk = LINKEDSEMI_SDHCI_PERI_BUS_FREQ;
    sdhci_init(host);

    k_mutex_init(&dev_data->access_mutex);
    k_sem_init(&host->transfer_sem, 0, 1);

    dev_config->irq_config_func(dev);

    return 0;
}

static const struct sdhc_driver_api linkedsemi_sdhci_api = {
    .reset = linkedsemi_sdhci_reset,
    .get_host_props = linkedsemi_sdhci_get_host_props,
    .set_io = linkedsemi_sdhci_set_io,
    .get_card_present = linkedsemi_sdhci_get_card_present,
    .request = linkedsemi_sdhci_request,
    .card_busy = linkedsemi_sdhci_card_busy,
};

#define LINKEDSEMI_SDHCI_INIT(n)                                                                                \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(n)));                                                    \
    static void sdhci_##n##_irq_config_func(const struct device *dev)                                           \
    {                                                                                                           \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), linkedsemi_sdhci_isr, DEVICE_DT_INST_GET(n), 0); \
        irq_enable(DT_INST_IRQN(n));                                                                            \
    }                                                                                                           \
                                                                                                                \
    static struct linkedsemi_sdhci_config sdhci_##n##_config = {                                                \
        .max_bus_freq = DT_INST_PROP(n, max_bus_freq),                                                          \
        .min_bus_freq = DT_INST_PROP(n, min_bus_freq),                                                          \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),))                                \
        .irq_config_func = sdhci_##n##_irq_config_func,                                                         \
    };                                                                                                          \
                                                                                                                \
    static struct linkedsemi_sdhci_data sdhci_##n##_data = {                                                    \
        .host = {                                                                                               \
            .mapbase = DT_INST_REG_ADDR(n),                                                                     \
        },                                                                                                      \
    };                                                                                                          \
                                                                                                                \
    DEVICE_DT_INST_DEFINE(n,                                                                                    \
                          &linkedsemi_sdhci_init,                                                               \
                          NULL,                                                                                 \
                          &sdhci_##n##_data,                                                                    \
                          &sdhci_##n##_config,                                                                  \
                          POST_KERNEL,                                                                          \
                          CONFIG_SDHC_INIT_PRIORITY,                                                            \
                          &linkedsemi_sdhci_api);

DT_INST_FOREACH_STATUS_OKAY(LINKEDSEMI_SDHCI_INIT)
