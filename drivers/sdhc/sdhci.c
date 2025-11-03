#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/sys/barrier.h>
#include <errno.h>
#include <zephyr/logging/log.h>

#include "sdhci.h"

LOG_MODULE_REGISTER(sdhci, CONFIG_SDHC_LOG_LEVEL);

void sdhci_reg_display(struct sdhci_host *host)
{
    if (host->execute_tuning) {
        return;
    }
    LOG_INF("SD_MASA_R:%x", sdhci_readl(host, SDHCI_DMA_ADDRESS));
    LOG_INF("BLCOKSIZE_R:%x", sdhci_readw(host, SDHCI_BLOCK_SIZE));
    LOG_INF("BLOCKCOUNT_R:%x", sdhci_readw(host, SDHCI_BLOCK_COUNT));
    LOG_INF("ARGUMENT_R:%x", sdhci_readl(host, SDHCI_ARGUMENT));
    LOG_INF("XFER_MODE_R:%x", sdhci_readw(host, SDHCI_TRANSFER_MODE));
    LOG_INF("CMD_R:%x", sdhci_readw(host, SDHCI_COMMAND));
    LOG_INF("RESP0_R:%x", sdhci_readl(host, SDHCI_RESPONSE));
    LOG_INF("RESP1_R:%x", sdhci_readl(host, SDHCI_RESPONSE + 4));
    LOG_INF("RESP2_R:%x", sdhci_readl(host, SDHCI_RESPONSE + 8));
    LOG_INF("RESP3_R:%x", sdhci_readl(host, SDHCI_RESPONSE + 12));
    LOG_INF("BUF_DATA_R:%x", sdhci_readl(host, SDHCI_BUFFER));
    LOG_INF("PSTATE_REG_R:%x", sdhci_readl(host, SDHCI_PRESENT_STATE));
    LOG_INF("HOST_CTL_R:%x", sdhci_readb(host, SDHCI_HOST_CONTROL));
    LOG_INF("PWR_CTRL_R:%x", sdhci_readb(host, SDHCI_POWER_CONTROL));
    LOG_INF("BGAP_CTRL_R:%x", sdhci_readb(host, SDHCI_BLOCK_GAP_CONTROL));
    LOG_INF("WUP_CTRL_R:%x", sdhci_readb(host, SDHCI_WAKE_UP_CONTROL));
    LOG_INF("CLK_CTRL_R:%x", sdhci_readw(host, SDHCI_CLOCK_CONTROL));
    LOG_INF("TOUT_CTRL_R:%x", sdhci_readb(host, SDHCI_TIMEOUT_CONTROL));
    LOG_INF("SW_RSR_R:%x", sdhci_readb(host, SDHCI_SOFTWARE_RESET));
    LOG_INF("NORMAL_INT_STAT_R:%x", sdhci_readw(host, SDHCI_INT_STATUS));
    LOG_INF("ERROR_INT_STAT_R:%x", sdhci_readw(host, SDHCI_INT_STATUS + 2));
    LOG_INF("NORMAL_INT_STAT_EN_R:%x", sdhci_readw(host, SDHCI_INT_ENABLE));
    LOG_INF("ERROR_INT_STAT_EN_R:%x", sdhci_readw(host, SDHCI_INT_ENABLE + 2));
    LOG_INF("NORNAL_INT_SIGNAL_EN_R:%x", sdhci_readw(host, SDHCI_SIGNAL_ENABLE));
    LOG_INF("ERROR_INT_SIGNAL_EN_R:%x", sdhci_readw(host, SDHCI_SIGNAL_ENABLE + 2));
    LOG_INF("AUTO_CMD_STAT_R:%x", sdhci_readw(host, SDHCI_AUTO_CMD_STATUS));
    LOG_INF("HOST_CTRL2_R:%x", sdhci_readw(host, SDHCI_HOST_CONTROL2));
    LOG_INF("CAPABILITIES1_R:%x", sdhci_readl(host, SDHCI_CAPABILITIES));
    LOG_INF("CAPABILITIES2_R:%x", sdhci_readl(host, SDHCI_CAPABILITIES_1));
    LOG_INF("FORCE_AUTO_CMD_STAT_R:%x", sdhci_readw(host, SDHCI_MAX_CURRENT));
    LOG_INF("FORCE_ERROR_INT_STAT_R:%x", sdhci_readw(host, SDHCI_SET_ACMD12_ERROR));
}

void sdhci_reset(struct sdhci_host *host, uint8_t mask)
{
    unsigned long timeout;

    /* Wait max 100 ms */
    timeout = 100;
    sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
    while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
        if (timeout == 0) {
            LOG_ERROR("%s: Reset 0x%x never completed.", __func__, (int)mask);
            return;
        }
        timeout--;
        k_msleep(1);
    }
}

uint32_t sdhci_get_present_status_flag(struct sdhci_host *sdhci_host)
{
    return sdhci_readl(sdhci_host, SDHCI_PRESENT_STATE);
}

uint32_t sdhci_card_busy(struct sdhci_host *sdhci_host)
{
    return (!(sdhci_get_present_status_flag(sdhci_host) & SDHCI_DATA0_LINE_LEVEL_FLAG));
}

uint32_t sdhci_get_int_status_flag(struct sdhci_host *sdhci_host)
{
    return sdhci_readl(sdhci_host, SDHCI_INT_STATUS);
}

void sdhci_clear_int_status_flag(struct sdhci_host *sdhci_host, uint32_t mask)
{
    sdhci_writel(sdhci_host, mask, SDHCI_INT_STATUS);
}

void sdhic_error_recovery(struct sdhci_host *sdhci_host)
{
    uint32_t status;
    /* get host present status */
    status = sdhci_get_present_status_flag(sdhci_host);
    /* check command inhibit status flag */
    if ((status & SDHCI_CMD_INHIBIT) != 0U) {
        /* reset command line */
        sdhci_reset(sdhci_host, SDHCI_RESET_CMD);
    }
    /* check data inhibit status flag */
    if ((status & SDHCI_DATA_INHIBIT) != 0U) {
        /* reset data line */
        sdhci_reset(sdhci_host, SDHCI_RESET_DATA);
    }
}

int32_t sdhci_receive_command_response(struct sdhci_host *sdhci_host, struct sdhci_command *command)
{
    if (command->response_type == CARD_RESPONSE_TYPE_R2) {
        /* CRC is stripped so we need to do some shifting. */
        for (int i = 0; i < 4; i++) {
            command->response[3 - i] = sdhci_readl(sdhci_host, SDHCI_RESPONSE + (3 - i) * 4) << 8;
            if (i != 3) {
                command->response[3 - i] |= sdhci_readb(sdhci_host, SDHCI_RESPONSE + (3 - i) * 4 - 1);
            }
        }
    } else {
        command->response[0] = sdhci_readl(sdhci_host, SDHCI_RESPONSE);
    }
    /* check response error flag */
    if ((command->response_error_flags != 0U)
        && ((command->response_type == CARD_RESPONSE_TYPE_R1)
        || (command->response_type == CARD_RESPONSE_TYPE_R1B)
        || (command->response_type == CARD_RESPONSE_TYPE_R6)
        || (command->response_type == CARD_RESPONSE_TYPE_R5))) {
        if (((command->response_error_flags) & (command->response[0U])) != 0U) {
            return -1;
        }
    }

    return 0;
}

void sdhci_send_command(struct sdhci_host *sdhci_host, struct sdhci_command *command, bool use_dma)
{
    __ASSERT_NO_MSG(NULL != command);

    uint32_t cmd_r, xfer_mode;
    struct sdhci_data *sdhci_data = sdhci_host->sdhci_data;

    cmd_r = SDHCI_MAKE_CMD(command->index, command->flags);
    if (sdhci_data != NULL) {
        if (IS_ENABLED(CONFIG_SDHCI_SDMA_ENABLE) && use_dma) {
            uint32_t start_addr;
            if (sdhci_data->rx_data) {
                start_addr = (uint32_t)((uint8_t *)sdhci_data->rx_data);
                sys_cache_data_invd_range((void *)start_addr, sdhci_data->block_size * sdhci_data->block_count);
            } else {
                start_addr = (uint32_t)((uint8_t *)sdhci_data->tx_data);
                sys_cache_data_flush_range((void *)start_addr, sdhci_data->block_size * sdhci_data->block_count);
            }
            command->flags2 |= SDHCI_ENABLE_DMA_FLAG;
            sdhci_writel(sdhci_host, start_addr, SDHCI_DMA_ADDRESS);
        }
        sdhci_writew(sdhci_host, SDHCI_MAKE_BLKSZ(SDHCI_DEFAULT_BOUNDARY_ARG, sdhci_data->block_size), SDHCI_BLOCK_SIZE);
        sdhci_writew(sdhci_host, sdhci_data->block_count, SDHCI_BLOCK_COUNT);
    }
    xfer_mode = command->flags2 & 0x1ff;

    sdhci_writew(sdhci_host, xfer_mode, SDHCI_TRANSFER_MODE);
    sdhci_writel(sdhci_host, command->argument, SDHCI_ARGUMENT);
    sdhci_writew(sdhci_host, cmd_r, SDHCI_COMMAND);
}

int32_t sdhci_set_transfer_config(struct sdhci_host *sdhci_host, struct sdhci_command *sdhci_command, struct sdhci_data *sdhci_data)
{
    __ASSERT_NO_MSG(sdhci_command);
    /* Define the flag corresponding to each response type. */
    switch (sdhci_command->response_type) {
    case CARD_RESPONSE_TYPE_NONE:
        break;
    case CARD_RESPONSE_TYPE_R1: /* Response 1 */
    case CARD_RESPONSE_TYPE_R5: /* Response 5 */
    case CARD_RESPONSE_TYPE_R6: /* Response 6 */
    case CARD_RESPONSE_TYPE_R7: /* Response 7 */

        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT | SDHCI_ENABLE_CMD_CRC_FLAG | SDHCI_ENABLE_CMD_INDEX_CHK_FLAG);
        break;

    case CARD_RESPONSE_TYPE_R1B: /* Response 1 with busy */
    case CARD_RESPONSE_TYPE_R5B: /* Response 5 with busy */
        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_ENABLE_CMD_CRC_FLAG | SDHCI_ENABLE_CMD_INDEX_CHK_FLAG);
        break;

    case CARD_RESPONSE_TYPE_R2: /* Response 2 */
        sdhci_command->flags |= (SDHCI_CMD_RESP_LONG | SDHCI_ENABLE_CMD_CRC_FLAG);
        break;

    case CARD_RESPONSE_TYPE_R3: /* Response 3 */
    case CARD_RESPONSE_TYPE_R4: /* Response 4 */
        sdhci_command->flags |= (SDHCI_CMD_RESP_SHORT);
        break;

    default:
        break;
    }

    if (sdhci_command->type == CARD_COMMAND_TYPE_ABORT) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_ABORT;
    } else if (sdhci_command->type == CARD_COMMAND_TYPE_RESUME) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_RESUME;
    } else if (sdhci_command->type == CARD_COMMAND_TYPE_SUSPEND) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_SUSPEND;
    } else if (sdhci_command->type == CARD_COMMAND_TYPE_NORMAL) {
        sdhci_command->flags |= SDHCI_ENABLE_COMMAND_TYPE_NORMAL;
    }

    if (sdhci_data) {
        sdhci_command->flags |= SDHCI_ENABLE_CMD_DATA_PRESENT_FLAG;
        sdhci_command->flags2 |= SDHCI_ENABLE_BLOCK_COUNT_FLAG;

        if (sdhci_data->rx_data) {
            sdhci_command->flags2 |= SDHCI_DATA_READ_FLAG;
        }
        if (sdhci_data->block_count > 1U) {
            sdhci_command->flags2 |= (SDHCI_MULTIPLE_BLOCK_FLAG);
            /* auto command 12 */
            if (sdhci_data->enable_auto_command12) {
                /* Enable Auto command 12. */
                sdhci_command->flags2 |= SDHCI_ENABLE_AUTO_COMMAND12_FLAG;
            }
            /* auto command 23 */
            if (sdhci_data->enable_auto_command23) {
                sdhci_command->flags2 |= SDHCI_ENABLE_AUTO_COMMAND23_FLAG;
            }
        }
    }
    return 0;
}

void sdhci_init(struct sdhci_host *host)
{
    sdhci_reset(host, SDHCI_RESET_ALL);
    /* high speed support*/
    // sdhci_writeb(host, SDHCI_CTRL_HISPD, SDHCI_HOST_CONTROL);
    sdhci_writeb(host, 0x7, SDHCI_TIMEOUT_CONTROL);
    sdhci_writeb(host, SDHCI_POWER_ON | SDHCI_POWER_330, SDHCI_POWER_CONTROL);
    sdhci_writew(host, SDHCI_CLOCK_INT_EN, SDHCI_CLOCK_CONTROL);
    while ((sdhci_readw(host, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE) == 0);
    sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK, SDHCI_INT_ENABLE);
    sdhci_writel(host, SDHCI_INT_CARD_INT, SDHCI_SIGNAL_ENABLE);

    host->power_mode = SDHC_POWER_ON;
}

#if defined(CONFIG_DIV_REG_VAILD)

void mmc_clock_freq_change(struct sdhci_host *host, uint32_t clock)
{
    uint32_t div, val;

    if (clock == 0)
        return;

    val = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
    val &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_PROG_CLOCK_MODE);
    sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);


    if (host->max_clk <= clock) {
        div = 1;
    } else {
        for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
            if ((host->max_clk / div) <= clock)
                break;
        }
    }
    div >>= 1;
    val &= ~((SDHCI_DIV_MASK << SDHCI_DIVIDER_SHIFT) | SDHCI_DIV_HI_MASK);
    val |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
    val |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
        << SDHCI_DIVIDER_HI_SHIFT;
    val |= SDHCI_CLOCK_CARD_EN | SDHCI_PROG_CLOCK_MODE;
    sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);
    while ((sdhci_readw(host, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE) == 0);
}

#else /* CONFIG_DIV_REG_VAILD */

extern void lsqsh_emmc_txck_rxck_config(uint32_t dev, uint32_t base_clock, uint32_t target_clock);

void mmc_clock_freq_change(struct sdhci_host *host, uint32_t clock)
{
    uint32_t div;
    uint32_t val;

    if (!host->execute_tuning) {
        LOG_INF("%s: %d(HZ)", __func__, clock);
    }
    host->current_speed = clock;

    if (clock == 0)
        return;

    div = 0;
    val = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
    val &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_PROG_CLOCK_MODE);
    sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);

    k_msleep(1);

    val &= ~((SDHCI_DIV_MASK << SDHCI_DIVIDER_SHIFT) | SDHCI_DIV_HI_MASK);
    val |= (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
    val |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
        << SDHCI_DIVIDER_HI_SHIFT;
    val |= SDHCI_CLOCK_CARD_EN | SDHCI_PROG_CLOCK_MODE;

    lsqsh_emmc_txck_rxck_config(host->mapbase, host->max_clk, clock);
    sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);
    while ((sdhci_readw(host, SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE) == 0);
}

#endif /* CONFIG_DIV_REG_VAILD */
