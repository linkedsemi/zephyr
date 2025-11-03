/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_sdhci

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/cache.h>
#include <zephyr/sys/bitarray.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif
#include "platform.h"
#include "sdhci.h"
#include "reg_sysc_app_awo.h"
#include "reg_sysc_app_cpu.h"

LOG_MODULE_REGISTER(linkedsemi_sdhci, CONFIG_SDHC_LOG_LEVEL);

#define LINKEDSEMI_SDHCI_RESET_TIMEOUT_VALUE (1000000U)

#define LINKEDSEMI_SDHCI_DEFAULT_TIMEOUT (5000U)

#define LINKEDSEMI_SDHCI_TUNING_DELAY_MAX (SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP_MASK >> SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP_POS)

struct linkedsemi_sdhci_config {
    uint32_t response_timeout;
    uint32_t cd_debounce_clocks;
    uint32_t data_timeout;
    uint32_t min_bus_freq;
    uint32_t max_bus_freq;
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
    void (*irq_config_func)(const struct device *dev);
    void (*irq_deconfig_func)(const struct device *dev);
};

struct linkedsemi_sdhci_data {
    uint32_t clock_frequency;
    struct sdhci_host host;
    struct k_mutex access_mutex;
    uint32_t *align_buf;
};

/*
 * SDHCI interrupt service routine
 */
static void linkedsemi_sdhci_isr(const void *arg)
{
    const struct device *dev = (const struct device *)arg;
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    uint32_t status = sdhci_get_int_status_flag(host);
    sdhci_clear_int_status_flag(host, status);

    if (status & (SDHCI_INT_RESPONSE
                | SDHCI_INT_DATA_END
                | SDHCI_INT_DMA_END
                | SDHCI_INT_SPACE_AVAIL
                | SDHCI_INT_DATA_AVAIL
                | SDHCI_INT_ERROR)) {
        host->error_code = (status >> 16) & 0xffff;
        if (host->error_code) {
            if (!host->execute_tuning) {
                uint32_t cmd_r = sdhci_readw(host, SDHCI_COMMAND);
                LOG_ERROR("error: %#4.4x  CMD_R: %#x\n", host->error_code, cmd_r);
            }
        }
        host->irq_status |= status;
        if (status & SDHCI_INT_ERROR) {
            k_sem_give(&host->transfer_sem);
        }
        if (status & SDHCI_INT_RESPONSE) {
            k_sem_give(&host->transfer_sem);
        }
        if (status & (SDHCI_INT_DATA_END
                    | SDHCI_INT_DMA_END
                    | SDHCI_INT_SPACE_AVAIL
                    | SDHCI_INT_DATA_AVAIL)) {
            k_sem_give(&host->transfer_sem);
        }
    }
    // if (status & SDHCI_INT_CARD_INT)
    //     sdio_irq_wakeup(host->host);
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

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

    memset(props, 0, sizeof(*props));
    props->f_max = dev_config->max_bus_freq;
    props->f_min = dev_config->min_bus_freq;
    props->power_delay = 500;
    props->host_caps.high_spd_support = false;
    props->host_caps.suspend_res_support = true;
    props->host_caps.vol_180_support = true;
    props->host_caps.bus_4_bit_support = true;
    props->host_caps.bus_8_bit_support = true;
    props->host_caps.hs200_support = true;
    props->host_caps.hs400_support = false;
    props->max_current_330 = 1024;

    return 0;
}

static int linkedsemi_sdhci_set_io(const struct device *dev, struct sdhc_io *ios)
{
    const struct linkedsemi_sdhci_config *dev_config = dev->config;
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    uint8_t ctrl;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

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
            // tx inv
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_TX_CLK_DELAY_MASK);
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_SD_CLK_OUT_MASK);
            // rx delay
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S0_CCLK_RX_MASK);
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S00_CCLK_RX_MASK);
            // clear delay
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
            REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, 0);
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
    host->bus_width = ios->bus_width;

    sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
    sdhci_writew(host, sdhci_readw(host, 0x52c) | 0x1, 0x52c);
    uint8_t mshc_ctrl_r = sdhci_readb(host, MSHC_CTRL_R);
    if (ios->clock > MHZ(100)) {
        mshc_ctrl_r &= ~CMD_CONFLICT_CHECK_MASK;
        sdhci_writeb(host, 0, MSHC_CTRL_R);
    } else {
        sdhci_writeb(host, 1, MSHC_CTRL_R);
    }

    host->timing = ios->timing;

    uint8_t host_control2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
    if (SDHC_TIMING_HS400 == ios->timing) {
        host_control2 &= ~(SDHCI_CTRL_UHS_MASK);
        host_control2 |= SDHCI_CTRL_EMMC_HS400;
    } else {
        host_control2 &= ~(SDHCI_CTRL_UHS_MASK);

        if ((ios->clock > MMC_CLOCK_26MHZ) && (ios->clock < MMC_CLOCK_HS200)) {
            // tx inv
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_TX_CLK_DELAY_MASK);
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_SD_CLK_OUT_MASK);
            // rx delay
            CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S0_CCLK_RX_MASK);
            SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S00_CCLK_RX_MASK);
        }
    }
    sdhci_writew(host, host_control2, SDHCI_HOST_CONTROL2);

    return 0;
}

static int linkedsemi_sdhci_get_card_present(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

    return sdhci_get_present_status_flag(host);
}

static int linkedsemi_sdhci_card_busy(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

    return sdhci_card_busy(host);
}

static int32_t linkedsemi_sdhci_wait_command_done(struct sdhci_host *host, struct sdhci_command *command, bool execute_tuning)
{
    __ASSERT_NO_MSG(NULL != command);

    /* tuning cmd do not need to wait command done */
    if (execute_tuning)
        return 0;
    /* Wait command complete or SDHC encounters error. */
    k_sem_take(&host->transfer_sem, K_FOREVER);
    if (host->error_code & SDHCI_INT_ERROR) {
        if (!host->execute_tuning) {
            LOG_ERROR("%s: Error detected in status(0x%X)!\n", __func__, host->error_code);
        }
        host->error_code = 0;
        return -1;
    }

    return sdhci_receive_command_response(host, command);
}

static int32_t linkedsemi_sdhci_transfer_data_blocking(struct sdhci_host *host, struct sdhci_data *data, bool use_dma)
{
    if (IS_ENABLED(CONFIG_SDHCI_SDMA_ENABLE) && use_dma) {
        uint32_t stat;

        while (1) {
            k_sem_take(&host->transfer_sem, K_FOREVER);
            stat = host->irq_status;
            if (stat & SDHCI_INT_ERROR) {
                if (!host->execute_tuning) {
                    LOG_ERROR("%s: Error detected in status(0x%x)!\n", __func__, host->error_code);
                }
                sdhci_reg_display(host);
                return -1;
            }
            if (stat & SDHCI_INT_DMA_END) {
                sdhci_writel(host, SDHCI_INT_DMA_END, SDHCI_INT_STATUS);
                sdhci_writel(host, sdhci_readl(host, SDHCI_DMA_ADDRESS), SDHCI_DMA_ADDRESS);
            }
            if (stat & SDHCI_INT_DATA_END) {
                sys_cache_data_invd_range((void *)data->rx_data, data->block_size * data->block_count);
                return 0;
            }
        }
    } else {
        uint32_t stat, rdy, mask, block;

        block = 0;
        rdy = SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL;
        mask = SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE;

        while (1) {
            k_sem_take(&host->transfer_sem, K_FOREVER);
            stat = host->irq_status;
            if (stat & SDHCI_INT_ERROR) {
                if (!host->execute_tuning) {
                    LOG_ERROR("%s: Error detected in status(0x%X)!\n", __func__, stat);
                }
                sdhci_reg_display(host);
                return -1;
            }
            if (stat & rdy) {
                if (!(sdhci_readl(host, SDHCI_PRESENT_STATE) & mask)) {
                    continue;
                }
                if (data->rx_data) {
                    uint16_t block_size = data->block_size >> 2;
                    for (int i = 0; i < block_size; i++) {
                        data->rx_data[i + block * block_size] = sdhci_readl(host, SDHCI_BUFFER);
                    }
                } else {
                    uint16_t block_size = data->block_size >> 2;
                    for (int i = 0; i < block_size; i++) {
                        sdhci_writel(host, data->tx_data[i + block * block_size], SDHCI_BUFFER);
                    }
                }
                block++;
                if (block >= data->block_count) {
                    return 0;
                }
            }
        }
    }
}

#if defined(CONFIG_SOC_LSQSH)
static bool is_psram(uint32_t addr)
{
    return ((addr >= DT_REG_ADDR(DT_NODELABEL(psram)))
            && (addr < (DT_REG_ADDR(DT_NODELABEL(psram)) + DT_REG_SIZE(DT_NODELABEL(psram)))));
}

static bool lsqsh_workaround_psram_use_dma(struct sdhci_host *host)
{
    struct sdhci_data *sdhci_data = host->sdhci_data;
    if (sdhci_data) {
        if (sdhci_data->rx_data) {
            return !is_psram((uint32_t)sdhci_data->rx_data);
        } else if (sdhci_data->tx_data) {
            return !is_psram((uint32_t)sdhci_data->tx_data);
        }
    }

    return false;
}
#endif

static int32_t linkedsemi_sdhci_transfer_blocking(struct sdhci_host *host)
{
    __ASSERT_NO_MSG(host);
    struct sdhci_command *sdhci_command = host->sdhci_command;
    struct sdhci_data *sdhci_data = host->sdhci_data;
    bool use_dma = IS_ENABLED(CONFIG_SDHCI_SDMA_ENABLE);
    int ret = 0;
#if defined(CONFIG_SOC_LSQSH)
    use_dma = lsqsh_workaround_psram_use_dma(host);
#endif
    /* Wait until command/data bus out of busy status. */
    while (sdhci_get_present_status_flag(host) & SDHCI_COMMAND_INHIBIT_FLAG) {
    }
    while (sdhci_data && (sdhci_get_present_status_flag(host) & SDHCI_DATA_INHIBIT_FLAG)) {
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
    ret = linkedsemi_sdhci_wait_command_done(host, sdhci_command, ((sdhci_data == NULL) ? false : sdhci_data->execute_tuning));
    /* transfer data */
    if ((sdhci_data != NULL) && (ret == 0) && (!(host->irq_status & SDHCI_INT_ERROR))) {
        ret = linkedsemi_sdhci_transfer_data_blocking(host, sdhci_data, use_dma);
    }
    while ((sdhci_get_present_status_flag(host) & SDHCI_COMMAND_INHIBIT_FLAG) && (!(host->irq_status & SDHCI_INT_ERROR)));
    while ((sdhci_data && (sdhci_get_present_status_flag(host) & SDHCI_DATA_INHIBIT_FLAG) && (!(host->irq_status & SDHCI_INT_ERROR))));
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

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

    ret = k_mutex_lock(&dev_data->access_mutex, K_FOREVER);
    if (ret) {
        if (!host->execute_tuning) {
            LOG_ERROR("Could not access card");
        }
        return -EBUSY;
    }

    host->irq_status = 0;
    sdhci_command.index = cmd->opcode;
    sdhci_command.argument = cmd->arg;

    /* Mask out part of response type field used for SPI commands */
    sdhci_command.response_type = (cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
    if (cmd->opcode == SD_STOP_TRANSMISSION) {
        sdhci_command.type = CARD_COMMAND_TYPE_ABORT;
    } else {
        sdhci_command.type = CARD_COMMAND_TYPE_NORMAL;
    }

    host->sdhci_command = &sdhci_command;

    if (data) {
        sdhci_data.block_size = data->block_size;
        sdhci_data.block_count = data->blocks;

        switch (cmd->opcode) {
        case SD_WRITE_SINGLE_BLOCK:
        case SD_WRITE_MULTIPLE_BLOCK:
            sdhci_data.enable_auto_command12 = true;
            sdhci_data.tx_data = data->data;
            break;
        case MMC_SEND_BUS_TEST:
            sdhci_data.tx_data = data->data;
            break;
        case SD_READ_SINGLE_BLOCK:
        case SD_READ_MULTIPLE_BLOCK:
            sdhci_data.enable_auto_command12 = true;
            sdhci_data.rx_data = data->data;
            break;
        case SD_APP_SEND_SCR:
        case SD_SWITCH:
        case SD_APP_SEND_NUM_WRITTEN_BLK:
        case MMC_CHECK_BUS_TEST:
        case MMC_SEND_EXT_CSD:
        case MMC_SEND_TUNING_BLOCK:
            sdhci_data.rx_data = data->data;
            break;
        default:
            LOG_ERROR("invalid opcode: %#x", cmd->opcode);
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

static const uint8_t tuning_blk_pattern_4bit[] = {
    0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
    0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
    0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
    0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
    0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
    0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
    0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
    0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

static const uint8_t tuning_blk_pattern_8bit[] = {
    0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
    0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
    0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
    0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
    0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
    0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
    0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
    0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
    0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
    0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
    0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
    0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
    0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
    0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
    0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
    0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};

static int dll_lock()
{
    REG_FIELD_WR(SYSC_APP_CPU->EMMC1_DLL_CTRL, SYSC_APP_CPU_EMMC1_DLL_CTRL_DECODER_R, 0);
    for (uint8_t phase_detect_sel = 1; phase_detect_sel <= 0x7; phase_detect_sel++) {
        LOG_DBG("phase_detect_sel: %x", phase_detect_sel);
        REG_FIELD_WR(SYSC_APP_CPU->EMMC1_DLL_CTRL, SYSC_APP_CPU_EMMC1_DLL_CTRL_PHASE_DETECT_SEL_N, phase_detect_sel);
        for (uint16_t num = 0; num <= 0x7f; num++) {
            k_msleep(10);
            bool lock = READ_BIT(SYSC_APP_CPU->EMMC1_DLL_CTRL, SYSC_APP_CPU_EMMC1_DLL_CTRL_RD_LOCK_MASK) > 0;
            if (lock) {
                LOG_DBG("lock: EMMC1_DLL_CTRL: %x", SYSC_APP_CPU->EMMC1_DLL_CTRL);
                return 0;
            } else {
                LOG_DBG("EMMC1_DLL_CTRL: %x", SYSC_APP_CPU->EMMC1_DLL_CTRL);
                REG_FIELD_WR(SYSC_APP_CPU->EMMC1_DLL_CTRL, SYSC_APP_CPU_EMMC1_DLL_CTRL_DECODER_R, num);
            }
        }
    }

    return -EIO;
}

static int find_max_consecutive_ones_region(sys_bitarray_t *bitarray, size_t *start_pos) {
    int ret_val;
    int current_val;
    size_t current_length = 0;
    size_t max_length = 0;
    size_t current_start = 0;
    size_t best_start = 0;

    if (bitarray == NULL) {
        return -1;
    }

    size_t num_bits = bitarray->num_bits;

    for (size_t i = 0; i < num_bits; i++) {
        ret_val = sys_bitarray_test_bit(bitarray, i, &current_val);
        if (ret_val != 0) {
            return ret_val;
        }

        if (current_val == 1) {
            if (current_length == 0) {
                current_start = i;
            }
            current_length++;
            if (current_length > max_length) {
                max_length = current_length;
                best_start = current_start;
            }
        } else {
            current_length = 0;
        }
    }

    if (start_pos != NULL) {
        *start_pos = best_start;
    }

    return max_length;
}

static int linkedsemi_sdhci_execute_mmc_bus_test_tuning(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    struct sdhc_command cmd = { 0 };
    struct sdhc_data data = {0};
    uint8_t block_size = host->bus_width;
    const uint8_t *tuning_data_wr;
    bool need_tuning;
    __aligned(MAX(4, CONFIG_SDHC_BUFFER_ALIGNMENT)) const uint8_t bus_test_pattern_8bit[CONFIG_SDHC_BUFFER_ALIGNMENT] = { 0x55, 0xaa, 0, 0, 0, 0, 0, 0 };
    __aligned(MAX(4, CONFIG_SDHC_BUFFER_ALIGNMENT)) const uint8_t bus_test_pattern_4bit[CONFIG_SDHC_BUFFER_ALIGNMENT] = { 0x5a, 0, 0, 0 };
    __aligned(MAX(4, CONFIG_SDHC_BUFFER_ALIGNMENT)) uint8_t tuning_data_rd[CONFIG_SDHC_BUFFER_ALIGNMENT];

    SYS_BITARRAY_DEFINE(delay_bitmap, LINKEDSEMI_SDHCI_TUNING_DELAY_MAX + 1);
    int ret = 0;

    if (SDHC_BUS_WIDTH8BIT == host->bus_width) {
        tuning_data_wr = bus_test_pattern_8bit;
    } else if (SDHC_BUS_WIDTH4BIT == host->bus_width) {
        tuning_data_wr = bus_test_pattern_4bit;
    } else {
        return -ENOTSUP;
    }

    LOG_DBG("%s: tuning %d(HZ) begin", __func__, host->current_speed);
    host->execute_tuning = true;

    // tx inv
    CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_TX_CLK_DELAY_MASK);
    CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_SD_CLK_OUT_MASK);
    // rx delay
    CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S0_CCLK_RX_MASK);
    SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S00_CCLK_RX_MASK);

    need_tuning = true;
    uint8_t emmc1_dll_ctrl_decoder_r = LINKEDSEMI_SDHCI_TUNING_DELAY_MAX;
    for (uint16_t delay = 0; delay <= emmc1_dll_ctrl_decoder_r; delay++) {
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
        REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, delay);
        SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);

        sys_cache_data_flush_range((void *)tuning_data_wr, CONFIG_SDHC_BUFFER_ALIGNMENT);

        cmd.opcode = MMC_SEND_BUS_TEST;
        cmd.arg = 0;
        cmd.response_type = SD_RSP_TYPE_R1;
        cmd.timeout_ms = CONFIG_SD_CMD_TIMEOUT;

        data.block_size = block_size;
        data.blocks = 1;
        data.data = (void *)tuning_data_wr;
        data.timeout_ms = CONFIG_SD_DATA_TIMEOUT;

        ret = sdhc_request(dev, &cmd, &data);
        if (ret) {
            LOG_ERROR("CMD19 (MMC_SEND_BUS_TEST) failed: %d", ret);
            host->execute_tuning = false;
            return ret;
        }

        memset(tuning_data_rd, 0, sizeof(tuning_data_rd));
        sys_cache_data_invd_range(tuning_data_rd, sizeof(tuning_data_rd));

        cmd.opcode = MMC_CHECK_BUS_TEST;
        data.data = tuning_data_rd;

        ret = sdhc_request(dev, &cmd, &data);
        if (ret) {
            LOG_ERROR("CMD14 (MMC_CHECK_BUS_TEST) failed: %d", ret);
            host->execute_tuning = false;
            return ret;
        }
        sys_cache_data_invd_range(tuning_data_rd, sizeof(tuning_data_rd));

        bool delay_good = true;
        for (int i = 0; i < block_size >> 2; i++) {
            if ((tuning_data_rd[i] ^ tuning_data_wr[i]) != 0xff) {
                LOG_DBG("bad delay: %x", delay);
                delay_good = false;
                break;
            }
        }
        if (delay_good) {
            if (0 == delay) {
                need_tuning = false;
                break;
            }
            sys_bitarray_set_bit(&delay_bitmap, delay);
            LOG_DBG("ok delay: %x", delay);
        }
    }

    if (need_tuning) {
        size_t max_consecutive_start = 0;
        int max_length = find_max_consecutive_ones_region(&delay_bitmap, &max_consecutive_start);
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
        if (max_length > 0) {
            LOG_INF("%s: tuning len: %d start bit: %zu", __func__, max_length, max_consecutive_start);
            LOG_INF("%s: tuning delay: %#x", __func__, max_consecutive_start + (max_length >> 1));
            LOG_INF("%s: tuning %d(HZ) success", __func__, host->current_speed);
            REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, max_consecutive_start + (max_length >> 1));
            ret = 0;
        } else {
            LOG_ERROR("%s: tuning err: %d", __func__, max_length);
            LOG_ERROR("%s: tuning %d(HZ) fail", __func__, host->current_speed);
            REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, 0);
            ret = -EIO;
        }
        SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
    } else {
        LOG_INF("%s: no need for tuning", __func__);
    }

    host->execute_tuning = false;

    return ret;
}

static int linkedsemi_sdhci_execute_hs200_tuning(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;
    struct sdhc_command cmd = { 0 };
    struct sdhc_data data = {0};
    uint8_t block_size;
    const uint8_t *tuning_data_cmp;
    uint8_t tuning_data[sizeof(tuning_blk_pattern_8bit)] __aligned(MAX(4, CONFIG_SDHC_BUFFER_ALIGNMENT));
    SYS_BITARRAY_DEFINE(delay_bitmap, LINKEDSEMI_SDHCI_TUNING_DELAY_MAX + 1);
    int ret = 0;

    if (SDHC_BUS_WIDTH8BIT == host->bus_width) {
        block_size = sizeof(tuning_blk_pattern_8bit);
        tuning_data_cmp = tuning_blk_pattern_8bit;
    } else if (SDHC_BUS_WIDTH4BIT == host->bus_width) {
        block_size = sizeof(tuning_blk_pattern_4bit);
        tuning_data_cmp = tuning_blk_pattern_4bit;
    } else {
        return -ENOTSUP;
    }

    cmd.opcode = MMC_SEND_TUNING_BLOCK;
    cmd.arg = 0;
    cmd.response_type = SD_RSP_TYPE_R1;
    cmd.timeout_ms = CONFIG_SD_CMD_TIMEOUT;

    data.block_size = block_size;
    data.blocks = 1;
    data.data = tuning_data;
    data.timeout_ms = CONFIG_SD_DATA_TIMEOUT;

    if(dll_lock()) {
        return -EIO;
    }
    LOG_DBG("%s: tuning EMMC1_DLL_CTRL: %x", __func__, SYSC_APP_CPU->EMMC1_DLL_CTRL);
    LOG_DBG("%s: tuning %d(HZ) begin", __func__, host->current_speed);
    host->execute_tuning = true;

    if (SDHC_TIMING_HS200 == host->timing) {
        // tx inv
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_TX_CLK_DELAY_MASK);
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_TX_CLK_SEL_SD_CLK_OUT_MASK);
        // rx delay
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S0_CCLK_RX_MASK);
        SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_SEL_S00_CCLK_RX_MASK);
    } else if (SDHC_TIMING_HS400 == host->timing) {
        // tx no inv
        // tx dll delay
        // rx dll delay
        while(1);
    } else {
        // error
        while(1);
    }

    uint8_t emmc1_dll_ctrl_decoder_r = REG_FIELD_RD(SYSC_APP_CPU->EMMC1_DLL_CTRL, SYSC_APP_CPU_EMMC1_DLL_CTRL_DECODER_R);
    /* __ASSERT_NO_MSG((SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP_MASK >> SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP_POS) >= emmc1_dll_ctrl_decoder_r); */
    for (uint16_t delay = 0; delay <= emmc1_dll_ctrl_decoder_r; delay++) {
        CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
        REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, delay);
        SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);

        memset(tuning_data, 0, sizeof(tuning_data));
        sys_cache_data_flush_range(tuning_data, sizeof(tuning_data));
        ret = sdhc_request(dev, &cmd, &data);
        if (ret) {
            LOG_ERROR("CMD21 (MMC_SEND_TUNING_BLOCK) failed: %d", ret);
            host->execute_tuning = false;
            return ret;
        }
        if (memcmp(tuning_data, tuning_data_cmp, block_size)) {
            LOG_DBG("bad delay: %x", delay);
        } else {
            sys_bitarray_set_bit(&delay_bitmap, delay);
            LOG_DBG("ok delay: %x", delay);
        }
    }

    size_t max_consecutive_start = 0;
    int max_length = find_max_consecutive_ones_region(&delay_bitmap, &max_consecutive_start);
    CLEAR_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);
    if (max_length > 0) {
        LOG_INF("%s: tuning len: %d start bit: %zu", __func__, max_length, max_consecutive_start);
        LOG_INF("%s: tuning delay: %#x", __func__, max_consecutive_start + (max_length >> 1));
        LOG_INF("%s: tuning %d(HZ) success", __func__, host->current_speed);
        REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, max_consecutive_start + (max_length >> 1));
        ret = 0;
    } else {
        LOG_ERROR("%s: tuning err: %d", __func__, max_length);
        LOG_ERROR("%s: tuning %d(HZ) fail", __func__, host->current_speed);
        REG_FIELD_WR(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_STP, 0);
        ret = -EIO;
    }
    SET_BIT(SYSC_APP_CPU->EMMC1_CTRL, SYSC_APP_CPU_EMMC1_RX_CLK_DLY_CTL_DLY_VAL_ACTIVE_MASK);

    host->execute_tuning = false;

    return ret;
}

static int linkedsemi_sdhci_execute_tuning(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    struct sdhci_host *host = &dev_data->host;

    if (host->current_speed < CONFIG_SDHCI_LINKEDSEMI_TUNING_LOWEST_FREQUENCY) {
        return linkedsemi_sdhci_execute_mmc_bus_test_tuning(dev);
    } else if (host->current_speed <= MMC_CLOCK_HS200) {
        return linkedsemi_sdhci_execute_hs200_tuning(dev);
    } else {
        __ASSERT_NO_MSG(0);
    }
}

/*
 * Early system init for SDHC
 */
static int linkedsemi_sdhci_init(const struct device *dev)
{
    struct linkedsemi_sdhci_data *dev_data = dev->data;
    const struct linkedsemi_sdhci_config *dev_config = dev->config;
    struct sdhci_host *host = &dev_data->host;
    __maybe_unused int ret;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    if (is_app_cpu_running()) {
        LOG_INF("app_cpu_running");
        return -EBUSY;
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERROR("SDHC pinctrl setup failed (%d)", ret);
        return ret;
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            LOG_ERROR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERROR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

    host->index = 0;
    host->have_phy = false;
    host->mshc_ctrl_r = 0;
    host->rx_delay_line = 0;
    host->tx_delay_line = 0;
    host->io_fixed_1v8 = 0;
    sdhci_init(host);

    k_mutex_init(&dev_data->access_mutex);
    k_sem_init(&host->transfer_sem, 0, 2);

    dev_config->irq_config_func(dev);

    return 0;
}

int linkedsemi_sdhci_reinit(const struct device *dev)
{
    return linkedsemi_sdhci_init(dev);
}

int linkedsemi_sdhci_deinit(const struct device *dev)
{
    const struct linkedsemi_sdhci_config *dev_config = dev->config;

    dev_config->irq_deconfig_func(dev);

    return 0;
}

static const struct sdhc_driver_api linkedsemi_sdhci_api = {
    .reset = linkedsemi_sdhci_reset,
    .get_host_props = linkedsemi_sdhci_get_host_props,
    .set_io = linkedsemi_sdhci_set_io,
    .get_card_present = linkedsemi_sdhci_get_card_present,
    .request = linkedsemi_sdhci_request,
    .card_busy = linkedsemi_sdhci_card_busy,
    .execute_tuning = linkedsemi_sdhci_execute_tuning,
};

#define LINKEDSEMI_SDHCI_INIT(n)                                                                                \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(n)));                                                    \
    static void sdhci_##n##_irq_config_func(const struct device *dev)                                           \
    {                                                                                                           \
        ARG_UNUSED(dev);                                                                                        \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), linkedsemi_sdhci_isr, DEVICE_DT_INST_GET(n), 0); \
        irq_enable(DT_INST_IRQN(n));                                                                            \
    }                                                                                                           \
    static void sdhci_##n##_irq_deconfig_func(const struct device *dev)                                         \
    {                                                                                                           \
        ARG_UNUSED(dev);                                                                                        \
        irq_disable(DT_INST_IRQN(n));                                                                           \
    }                                                                                                           \
                                                                                                                \
    static struct linkedsemi_sdhci_config sdhci_##n##_config = {                                                \
        .max_bus_freq = DT_INST_PROP(n, max_bus_freq),                                                          \
        .min_bus_freq = DT_INST_PROP(n, min_bus_freq),                                                          \
        .irq_config_func = sdhci_##n##_irq_config_func,                                                         \
        .irq_deconfig_func = sdhci_##n##_irq_deconfig_func,                                                     \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n), ))                               \
        IF_ENABLED(DT_HAS_CLOCKS(n), (.ccfg = LS_DT_CLK_CFG_ITEM(n), ))                                         \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets), (.reset = RESET_DT_SPEC_INST_GET(n), ))                    \
    };                                                                                                          \
                                                                                                                \
    static struct linkedsemi_sdhci_data sdhci_##n##_data = {                                                    \
        .host = {                                                                                               \
            .mapbase = DT_INST_REG_ADDR(n),                                                                     \
            .max_clk = DT_INST_PROP(n, clock_frequency),                                                        \
            .bus_width = 1,                                                                                     \
            .execute_tuning = false,                                                                            \
        },                                                                                                      \
        .clock_frequency = DT_INST_PROP(n, clock_frequency),                                                    \
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
