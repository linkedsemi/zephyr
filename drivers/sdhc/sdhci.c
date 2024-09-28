#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <errno.h>
#include <zephyr/logging/log.h>

#include "sdhci.h"

LOG_MODULE_REGISTER(sdhci, CONFIG_SDHC_LOG_LEVEL);

void sdhci_reg_display(struct sdhci_host *host)
{
    LOG_INF("SD_MASA_R:%x\n", sdhci_readl(host, SDHCI_DMA_ADDRESS));
    LOG_INF("BLCOKSIZE_R:%x\n", sdhci_readw(host, SDHCI_BLOCK_SIZE));
    LOG_INF("BLOCKCOUNT_R:%x\n", sdhci_readw(host, SDHCI_BLOCK_COUNT));
    LOG_INF("ARGUMENT_R:%x\n", sdhci_readl(host, SDHCI_ARGUMENT));
    LOG_INF("XFER_MODE_R:%x\n", sdhci_readw(host, SDHCI_TRANSFER_MODE));
    LOG_INF("CMD_R:%x\n", sdhci_readw(host, SDHCI_COMMAND));
    LOG_INF("RESP0_R:%x\n", sdhci_readl(host, SDHCI_RESPONSE));
    LOG_INF("RESP1_R:%x\n", sdhci_readl(host, SDHCI_RESPONSE + 4));
    LOG_INF("RESP2_R:%x\n", sdhci_readl(host, SDHCI_RESPONSE + 8));
    LOG_INF("RESP3_R:%x\n", sdhci_readl(host, SDHCI_RESPONSE + 12));
    LOG_INF("BUF_DATA_R:%x\n", sdhci_readl(host, SDHCI_BUFFER));
    LOG_INF("PSTATE_REG_R:%x\n", sdhci_readl(host, SDHCI_PRESENT_STATE));
    LOG_INF("HOST_CTL_R:%x\n", sdhci_readb(host, SDHCI_HOST_CONTROL));
    LOG_INF("PWR_CTRL_R:%x\n", sdhci_readb(host, SDHCI_POWER_CONTROL));
    LOG_INF("BGAP_CTRL_R:%x\n", sdhci_readb(host, SDHCI_BLOCK_GAP_CONTROL));
    LOG_INF("WUP_CTRL_R:%x\n", sdhci_readb(host, SDHCI_WAKE_UP_CONTROL));
    LOG_INF("CLK_CTRL_R:%x\n", sdhci_readw(host, SDHCI_CLOCK_CONTROL));
    LOG_INF("TOUT_CTRL_R:%x\n", sdhci_readb(host, SDHCI_TIMEOUT_CONTROL));
    LOG_INF("SW_RSR_R:%x\n", sdhci_readb(host, SDHCI_SOFTWARE_RESET));
    LOG_INF("NORMAL_INT_STAT_R:%x\n", sdhci_readw(host, SDHCI_INT_STATUS));
    LOG_INF("ERROR_INT_STAT_R:%x\n", sdhci_readw(host, SDHCI_INT_STATUS + 2));
    LOG_INF("NORMAL_INT_STAT_EN_R:%x\n", sdhci_readw(host, SDHCI_INT_ENABLE));
    LOG_INF("ERROR_INT_STAT_EN_R:%x\n", sdhci_readw(host, SDHCI_INT_ENABLE + 2));
    LOG_INF("NORNAL_INT_SIGNAL_EN_R:%x\n", sdhci_readw(host, SDHCI_SIGNAL_ENABLE));
    LOG_INF("ERROR_INT_SIGNAL_EN_R:%x\n", sdhci_readw(host, SDHCI_SIGNAL_ENABLE + 2));
    LOG_INF("AUTO_CMD_STAT_R:%x\n", sdhci_readw(host, SDHCI_AUTO_CMD_STATUS));
    LOG_INF("HOST_CTRL2_R:%x\n", sdhci_readw(host, SDHCI_HOST_CONTROL2));
    LOG_INF("CAPABILITIES1_R:%x\n", sdhci_readl(host, SDHCI_CAPABILITIES));
    LOG_INF("CAPABILITIES2_R:%x\n", sdhci_readl(host, SDHCI_CAPABILITIES_1));
    LOG_INF("FORCE_AUTO_CMD_STAT_R:%x\n", sdhci_readw(host, SDHCI_MAX_CURRENT));
    LOG_INF("FORCE_ERROR_INT_STAT_R:%x\n", sdhci_readw(host, SDHCI_SET_ACMD12_ERROR));
    LOG_INF("AMDA_ERR_STAT_STAT_R:%x\n", sdhci_readl(host, SDHCI_ADMA_ERROR));
    LOG_INF("AMDA_SA_LOW_STAT_R:%x\n", sdhci_readl(host, SDHCI_ADMA_ADDRESS));
    LOG_INF("AMDA_SA_HIGH_STAT_R:%x\n", sdhci_readl(host, SDHCI_ADMA_ADDRESS_HI));
}

void dwcmshc_phy_1_8v_init(struct sdhci_host *host)
{
    sdhci_writew(host, DWC_MSHC_PHY_PAD_EMMC_DAT, DWC_MSHC_CMDPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_EMMC_DAT, DWC_MSHC_DATPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_EMMC_CLK, DWC_MSHC_CLKPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_EMMC_STB, DWC_MSHC_STBPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_EMMC_DAT, DWC_MSHC_RSTNPAD_CNFG);
}

void dwcmshc_phy_3_3v_init(struct sdhci_host *host)
{
    sdhci_writew(host, DWC_MSHC_PHY_PAD_SD_DAT, DWC_MSHC_CMDPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_SD_DAT, DWC_MSHC_DATPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_SD_CLK, DWC_MSHC_CLKPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_SD_STB, DWC_MSHC_STBPAD_CNFG);
    sdhci_writew(host, DWC_MSHC_PHY_PAD_SD_DAT, DWC_MSHC_RSTNPAD_CNFG);
}

void dwcmshc_phy_delay_config(struct sdhci_host *host)
{
    sdhci_writeb(host, 1, DWC_MSHC_COMMDL_CNFG);
    if (host->tx_delay_line > 256) {
        LOG_ERR("host%d: tx_delay_line err\n", host->index);
    } else if (host->tx_delay_line > 128) {
        sdhci_writeb(host, 0x1, DWC_MSHC_SDCLKDL_CNFG);
        sdhci_writeb(host, host->tx_delay_line - 128, DWC_MSHC_SDCLKDL_DC);
    } else {
        sdhci_writeb(host, 0x0, DWC_MSHC_SDCLKDL_CNFG);
        sdhci_writeb(host, host->tx_delay_line, DWC_MSHC_SDCLKDL_DC);
    }
    sdhci_writeb(host, host->rx_delay_line, DWC_MSHC_SMPLDL_CNFG);
    sdhci_writeb(host, 0xc, DWC_MSHC_ATDL_CNFG);
    sdhci_writel(host, (sdhci_readl(host, SDHCI_VENDER_AT_CTRL_REG) | BIT(16) | BIT(17) | BIT(19) | BIT(20)), SDHCI_VENDER_AT_CTRL_REG);
    sdhci_writel(host, 0x0, SDHCI_VENDER_AT_STAT_REG);
}

int dwcmshc_phy_init(struct sdhci_host *host)
{
    uint32_t reg;
    uint32_t timeout = 15000;
    /* reset phy */
    sdhci_writew(host, 0, DWC_MSHC_PHY_CNFG);

    /* Disable the clock */
    sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

    if (host->io_fixed_1v8) {
        uint32_t data = sdhci_readw(host, SDHCI_HOST_CONTROL2);
        data |= SDHCI_CTRL_VDD_180;
        sdhci_writew(host, data, SDHCI_HOST_CONTROL2);
        dwcmshc_phy_1_8v_init(host);
    } else {
        dwcmshc_phy_3_3v_init(host);
    }

    dwcmshc_phy_delay_config(host);

    /* Wait max 150 ms */
    while (1) {
        reg = sdhci_readl(host, DWC_MSHC_PHY_CNFG);
        if (reg & PHY_PWRGOOD)
            break;
        if (!timeout) {
            return -1;
        }
        timeout--;

        k_msleep(1);
    }

    reg = PAD_SN_DEFAULT | PAD_SP_DEFAULT;
    sdhci_writel(host, reg, DWC_MSHC_PHY_CNFG);

    /* de-assert the phy */
    reg |= PHY_RSTN;
    sdhci_writel(host, reg, DWC_MSHC_PHY_CNFG);

    return 0;
}

void sdhci_reset(struct sdhci_host *host, uint8_t mask)
{
    unsigned long timeout;

    /* Wait max 100 ms */
    timeout = 100;
    sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
    while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
        if (timeout == 0) {
            LOG_ERR("%s: Reset 0x%x never completed.\n",
                    __func__,
                    (int)mask);
            return;
        }
        timeout--;
        k_msleep(1);
    }
    if (mask == SDHCI_RESET_ALL) {
        if (host->index == 0) {
            uint16_t emmc_ctl = sdhci_readw(host, EMMC_CTRL_R);
            if (host->is_emmc_card)
                emmc_ctl |= (1 << CARD_IS_EMMC);
            else
                emmc_ctl &= ~(1 << CARD_IS_EMMC);
            sdhci_writeb(host, emmc_ctl, EMMC_CTRL_R);
        }
        if (host->have_phy)
            dwcmshc_phy_init(host);
        else
            sdhci_writeb(host, host->mshc_ctrl_r, MSHC_CTRL_R);
    }
}

uint32_t sdhci_get_present_status_flag(struct sdhci_host *sdhci_host)
{
    return sdhci_readl(sdhci_host, SDHCI_PRESENT_STATE);
}

uint32_t sdhci_card_busy(struct sdhci_host *sdhci_host)
{
    return sdhci_get_present_status_flag(sdhci_host) & sdhci_command_inhibit_flag;
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
    if (command->responseType == card_response_type_r2) {
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
    if ((command->responseErrorFlags != 0U)
        && ((command->responseType == card_response_type_r1)
        || (command->responseType == card_response_type_r1b)
        || (command->responseType == card_response_type_r6)
        || (command->responseType == card_response_type_r5))) {
        if (((command->responseErrorFlags) & (command->response[0U])) != 0U) {
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
#if defined(CONFIG_SDHCI_SDMA_ENABLE)
        uint32_t start_addr;
        if (sdhci_data->rx_data) {
            start_addr = (uint32_t)((uint8_t *)sdhci_data->rx_data);
        } else {
            start_addr = (uint32_t)((uint8_t *)sdhci_data->tx_data);
            sys_cache_data_flush_all();
        }
        command->flags2 |= sdhci_enable_dma_flag;
        sdhci_writel(sdhci_host, start_addr, SDHCI_DMA_ADDRESS);
#endif
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
    switch (sdhci_command->responseType) {
    case card_response_type_none:
        break;
    case card_response_type_r1: /* Response 1 */
    case card_response_type_r5: /* Response 5 */
    case card_response_type_r6: /* Response 6 */
    case card_response_type_r7: /* Response 7 */

        sdhci_command->flags |= (sdhci_cmd_resp_short | sdhci_enable_cmd_crc_flag | sdhci_enable_cmd_index_chk_flag);
        break;

    case card_response_type_r1b: /* Response 1 with busy */
    case card_response_type_r5b: /* Response 5 with busy */
        sdhci_command->flags |= (sdhci_cmd_resp_short_busy | sdhci_enable_cmd_crc_flag | sdhci_enable_cmd_index_chk_flag);
        break;

    case card_response_type_r2: /* Response 2 */
        sdhci_command->flags |= (sdhci_cmd_resp_long | sdhci_enable_cmd_crc_flag);
        break;

    case card_response_type_r3: /* Response 3 */
    case card_response_type_r4: /* Response 4 */
        sdhci_command->flags |= (sdhci_cmd_resp_short);
        break;

    default:
        break;
    }

    if (sdhci_command->type == card_command_type_abort) {
        sdhci_command->flags |= sdhci_enable_command_type_abort;
    } else if (sdhci_command->type == card_command_type_resume) {
        sdhci_command->flags |= sdhci_enable_command_type_resume;
    } else if (sdhci_command->type == card_command_type_suspend) {
        sdhci_command->flags |= sdhci_enable_command_type_suspend;
    } else if (sdhci_command->type == card_command_type_normal) {
        sdhci_command->flags |= sdhci_enable_command_type_normal;
    }

    if (sdhci_data) {
        sdhci_command->flags |= sdhci_enable_cmd_data_present_flag;
        sdhci_command->flags2 |= sdhci_enable_block_count_flag;

        if (sdhci_data->rx_data) {
            sdhci_command->flags2 |= sdhci_data_read_flag;
        }
        if (sdhci_data->block_count > 1U) {
            sdhci_command->flags2 |= (sdhci_multiple_block_flag);
            /* auto command 12 */
            if (sdhci_data->enableAutoCommand12) {
                /* Enable Auto command 12. */
                sdhci_command->flags2 |= sdhci_enable_auto_command12_flag;
            }
            /* auto command 23 */
            if (sdhci_data->enableAutoCommand23) {
                sdhci_command->flags2 |= sdhci_enable_auto_command23_flag;
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

void mmc_clock_freq_change(struct sdhci_host *host, uint32_t clock)
{
    uint32_t div, val;

    val = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
    val &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_PROG_CLOCK_MODE);
    sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);

    if (clock == 0)
        return;

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
