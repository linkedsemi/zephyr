/*
 * Copyright (c) 2025 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_adc

#include <stdlib.h>
#include <errno.h>
#include <field_manipulate.h>
#include <zephyr/drivers/adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER 1
#include "adc_context.h"
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/devicetree.h>
#include <soc.h>
#include "reg_base_addr.h"
#if(CONFIG_SOC_LS1010)
    #include "reg_v33_rg.h"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_ls, LOG_LEVEL_DBG);
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <reg_adcv2_type.h>
#include <ls_msp_adc.h>
#include "ls_soc_gpio.h"

#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif

#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif

#define regular_mode    0x00000000U
#define inject_mode     0x00000001U
#define loop_mode       0x00000002U

typedef void (*irq_cfg_func_t)(const struct device *dev);
struct adc_ls_config {
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
    irq_cfg_func_t irq_config_func;
    reg_adc_t *reg;
    uint8_t irq_num;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    uint8_t conversion_mode;
    uint32_t data_align;
    uint8_t continuous_conv_mode;
    uint32_t nbr_of_conversion;
    uint8_t discontinuous_conv_mode;
    uint32_t nbr_of_disc_conversion;
    uint32_t trig_type;
    uint32_t adc_drive_type;
    uint32_t adc_clk_div;
    uint8_t clk_cfg;
    uint16_t injected_offset;
    uint16_t fif_ctrl1;
    uint32_t clock_source;
};
struct adc_ls_data {
    struct adc_context ctx;
    const struct device *dev;
    uint16_t *buffer;
    uint8_t conversion_rank;
};

static void ADC_RegularGetValue(const struct device *dev)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    if(reg->INTR_R & ADC_INTR_REG_ERR_MASK)
    {
        reg->INTR_C |= ADC_INTR_REG_ERR_MASK;
        LOG_ERR("Sampling error\n");
    }

    uint16_t *buffer = (uint16_t *)data->ctx.sequence.buffer;

    for(uint8_t read_count = 1; read_count <= config->nbr_of_conversion; read_count++)
    {
        switch (read_count)
        {
        case ADC_REGULAR_RANK_1:
            *buffer = reg->REG_DAT00 & 0xffff;
            break;
        case ADC_REGULAR_RANK_2:
            *buffer = reg->REG_DAT01 & 0xffff;
            break;
        case ADC_REGULAR_RANK_3:
            *buffer = reg->REG_DAT02 & 0xffff;
            break;
        case ADC_REGULAR_RANK_4:
            *buffer = reg->REG_DAT03 & 0xffff;
            break;
        case ADC_REGULAR_RANK_5:
            *buffer = reg->REG_DAT04 & 0xffff;
            break;
        case ADC_REGULAR_RANK_6:
            *buffer = reg->REG_DAT05 & 0xffff;
            break;    
        case ADC_REGULAR_RANK_7:
            *buffer = reg->REG_DAT06 & 0xffff;
            break;
        case ADC_REGULAR_RANK_8:
            *buffer = reg->REG_DAT07 & 0xffff;
            break;
        case ADC_REGULAR_RANK_9:
            *buffer = reg->REG_DAT08 & 0xffff;
            break;
        case ADC_REGULAR_RANK_10:
            *buffer = reg->REG_DAT09 & 0xffff;
            break;
        case ADC_REGULAR_RANK_11:
            *buffer = reg->REG_DAT10 & 0xffff;
            break;
        case ADC_REGULAR_RANK_12:
            *buffer = reg->REG_DAT11 & 0xffff;
            break;
    #if(CONFIG_SOC_LSQSH)
        case ADC_REGULAR_RANK_13:
            *buffer = reg->REG_DAT12 & 0xffff;
            break;
        case ADC_REGULAR_RANK_14:
            *buffer = reg->REG_DAT13 & 0xffff;
            break;
        case ADC_REGULAR_RANK_15:
            *buffer = reg->REG_DAT14 & 0xffff;
            break;
        case ADC_REGULAR_RANK_16:
            *buffer = reg->REG_DAT15 & 0xffff;
            break;
    #endif
        default:
            __ASSERT(false, "REGULAR_RANK illegal\n");
            break;
        }
        buffer++;
    }
}

static void ADC_InjectGetValue(const struct device *dev)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    if(reg->INTR_R & ADC_INTR_INJ_ERR_MASK)
    {
        reg->INTR_C |= ADC_INTR_INJ_ERR_MASK;
        LOG_ERR("Sampling error\n");
    }

    uint16_t *buffer = (uint16_t *)data->ctx.sequence.buffer;

    for(uint8_t read_count = 1; read_count <= config->nbr_of_conversion; read_count++)
    {
        switch (read_count)
        {
        case ADC_INJECTED_RANK_1:
            *buffer = reg->INJ_DAT00 & 0xffff;
            break;
        case ADC_INJECTED_RANK_2:
            *buffer = reg->INJ_DAT01 & 0xffff;
            break;
        case ADC_INJECTED_RANK_3:
            *buffer = reg->INJ_DAT02 & 0xffff;
            break;
        case ADC_INJECTED_RANK_4:
            *buffer = reg->INJ_DAT03 & 0xffff;
            break;
        default:
            __ASSERT(false, "REGULAR_RANK illegal\n");
            break;
        }
        buffer++;
    }
}

static void ADC_LoopGetValue(const struct device *dev)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    uint16_t *buffer = (uint16_t *)data->ctx.sequence.buffer;

    uint8_t fif_lvl_num = reg->FIFO_FLVL;

    for(uint8_t read_count = 0; read_count <= fif_lvl_num; read_count++)
    {

       if (reg->INTR_R & ADC_INTR_FIF_ERR_MASK) {
            reg->INTR_C |= ADC_INTR_FIF_ERR_MASK;
            LOG_ERR("Sampling error\n");
        }

        if (reg->INTR_R & ADC_INTR_FIF_OVR_MASK) {
            reg->INTR_C |= ADC_INTR_FIF_OVR_MASK;
            LOG_ERR("FIF Overrun\n");
            REG_FIELD_WR(reg->FIF_CTRL0, ADC_FIFO_CLR, 1);
        }

        *buffer = reg->FIF_DAT;
        buffer++;
    }
}

static void ADC_GetValue(const struct device *dev, uint8_t conversion_mode)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    if(conversion_mode == regular_mode) {
        ADC_RegularGetValue(dev);
        REG_FIELD_WR(reg->TRIG, ADC_REG_TRIG, 0);
    }else if(conversion_mode == inject_mode) {
        ADC_InjectGetValue(dev);
        REG_FIELD_WR(reg->TRIG, ADC_INJ_TRIG, 0);
    }else if(conversion_mode == loop_mode) {
        ADC_LoopGetValue(dev);
        REG_FIELD_WR(reg->TRIG, ADC_FIF_TRIG, 0);
        REG_FIELD_WR(reg->FIF_CTRL0, ADC_FIFO_CLR, 1);
    }

    data->conversion_rank = 0;
}

static int ADC_VrefType_SetConfig(const struct adc_ls_config* config, enum adc_reference Vtype)
{
    uint32_t tmp_adr = 0;

    switch (Vtype)
    {
    case ADC_REF_VDD_1:
    tmp_adr = FIELD_BUILD(ADC_ADR_BP, 1)|FIELD_BUILD(ADC_ADR_VREFBUF_EN, 0)|
            FIELD_BUILD(ADC_ADR_VCM_EN, 1)|FIELD_BUILD(ADC_ADR_VREF_EN, 0)|
            FIELD_BUILD(ADC_ADR_VRSEL, 1);
    break;
    case ADC_REF_EXTERNAL0:
    tmp_adr = FIELD_BUILD(ADC_ADR_BP, 0)|FIELD_BUILD(ADC_ADR_VREFBUF_EN, 0)|
            FIELD_BUILD(ADC_ADR_VCM_EN, 1)|FIELD_BUILD(ADC_ADR_VREF_EN, 1)|
            FIELD_BUILD(ADC_ADR_VRSEL, 2);
    break;
    case ADC_REF_INTERNAL:
    default:
    tmp_adr = FIELD_BUILD(ADC_ADR_BP, 1)|FIELD_BUILD(ADC_ADR_VREFBUF_EN, 1)| 
            FIELD_BUILD(ADC_ADR_VCM_EN, 1)|FIELD_BUILD(ADC_ADR_VREF_EN, 1)|
            FIELD_BUILD(ADC_ADR_VRSEL, 4);
    break;
    }

    if(config->adc_drive_type == BINBUF_DIRECT_DRIVE_ADC)
    {
        tmp_adr |= FIELD_BUILD(ADC_ADR_EN_INBUF_A, 0)|FIELD_BUILD(ADC_ADR_EN_INBUF_B, 0);
    }else{
        tmp_adr |= FIELD_BUILD(ADC_ADR_EN_INBUF_A, 1)|FIELD_BUILD(ADC_ADR_EN_INBUF_B, 1);
    }

    MODIFY_REG(config->reg->ADR,
               ADC_ADR_EN_INBUF_A_MASK|ADC_ADR_EN_INBUF_B_MASK|ADC_ADR_VREFBUF_EN_MASK|
               ADC_ADR_BP_MASK|ADC_ADR_VCM_EN_MASK|ADC_ADR_VREF_EN_MASK|ADC_ADR_VRSEL_MASK,
               tmp_adr);

    return 0;
}

static void ADC_Sample_Data_Path_set(const struct adc_ls_config * config, uint8_t channel_id)
{
    uint32_t byp_cfg_tmp = 0;

    byp_cfg_tmp = config->reg->BYP_CFG;

    if(config->adc_drive_type == EINBUF_DRIVE_ADC) {
        byp_cfg_tmp &= ~(1 << channel_id);
        byp_cfg_tmp |= 1<<16<< channel_id;
    }else if(config->adc_drive_type == INRES_ONETHIRD_EINBUF_DRIVE_ADC) {
        byp_cfg_tmp &= ~(1 << channel_id);
        byp_cfg_tmp &= ~(1<<16 <<channel_id);
    }else {
        byp_cfg_tmp |= 1 << channel_id;
        byp_cfg_tmp |= 1<<16<< channel_id;
    }

    config->reg->BYP_CFG |= byp_cfg_tmp;
}

static void ADC_Ch_cfg_Reg_SetConfig(reg_adc_t* reg, uint8_t channel_id)
{
    // ADC Channel Enable
    reg->CH_CFG |= 1<<channel_id;

#if(CONFIG_SOC_LS1010)
    if(channel_id > ADC_CHANNEL_3 && channel_id <= ADC_CHANNEL_7) {
        reg->CH_CFG |= 1<<16<<channel_id;
    }else {
        reg->CH_CFG &= ~(1<<16<<channel_id);
    }  
#endif

#if(CONFIG_SOC_LSQSH)
     reg->CH_CFG &= ~(1<<16<<channel_id);
#endif
}

#if(CONFIG_SOC_LS1010)
static void load_trim_value(reg_adc_t* reg, uint16_t addr)
{
    uint32_t adc_trim_value[6] = {0};

    hal_flash_read_security_area(1, addr, (uint8_t *)adc_trim_value, sizeof(adc_trim_value));

    if(adc_trim_value[0] == ~adc_trim_value[1]) {
        reg->ADR = adc_trim_value[0];
    }

    if(adc_trim_value[4] == ~adc_trim_value[5]) {
        MODIFY_REG(reg->ADCH, ADC_ADCH_OS_CALV_MASK, (adc_trim_value[4] >> 16) << ADC_ADCH_OS_CALV_POS);
    }
}
#endif

static int adc_ls_init(const struct device *dev)
{
    const struct adc_ls_config * config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;
    __maybe_unused int ret;

#if defined(CONFIG_CLOCK_CONTROL)
    if (config->ccfg.cctl_dev) {
        const struct device *clk_dev = config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (config->reset.dev != NULL) {
        if (!device_is_ready(config->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(config->reset.dev, config->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (config->ccfg.cctl_dev) {
        const struct device *clk_dev = config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&config->ccfg);
    #if(CONFIG_SOC_LSQSH)
        uint32_t rate;
        clock_control_get_rate(clk_dev, (clock_control_subsys_t)&config->clock_source, &rate);
    #endif
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Could not configure pins");
    }
#endif

    data->dev = dev;

    adc_context_init(&data->ctx);

    config->irq_config_func(dev);

    __ASSERT(IS_ADC_DATA_ALIGN(config->data_align), "Invalid data alignment");

    __ASSERT(IS_FUNCTIONAL_STATE(config->continuous_conv_mode), "Invalid conversion mode");

#if(CONFIG_SOC_LS1010)
    MODIFY_REG(V33_RG->MISC_CTRL1, V33_RG_PD_ADC12_MASK, 0 << V33_RG_PD_ADC12_POS);

    if(reg == (reg_adc_t *)0x40089000) {
        load_trim_value(reg, 0x30);
    }else {
        load_trim_value(reg, 0x38);
    }
#endif

    REG_FIELD_WR(reg->ADCH, ADC_ADCH_TRIM_EN, 1);

    uint32_t tmp_misc_ctrl = 0;
    tmp_misc_ctrl |= config->data_align;
    tmp_misc_ctrl |= config->adc_clk_div << ADC_ADC_DIV_POS;
    tmp_misc_ctrl |= ADC_ADC_RES_MASK;

    MODIFY_REG(reg->MISC_CTRL,
           ADC_ADC_DIV_MASK|ADC_ADC_EN_MASK|ADC_DATA_ALIGN_MASK|ADC_DMA_EN_MASK,
           tmp_misc_ctrl);

    data->conversion_rank = 0;

    adc_context_unlock_unconditionally(&data->ctx);

    return 0;
}

static void ADC_RegularChannelConfig(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
    const struct adc_ls_config * config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    #if(CONFIG_SOC_LS1010)
        __ASSERT(data->conversion_rank <= ADC_REGULAR_RANK_12, "LS1010 regular mode rank must be less than or equal to 12");
        REG_FIELD_WR(reg->REG_CTRL1, ADC_REG_SEQLEN, config->nbr_of_conversion - 1);
    #endif

    #if(CONFIG_SOC_LSQSH)
        __ASSERT(data->conversion_rank <= ADC_REGULAR_RANK_16, "LSQSH regular mode rank must be less than or equal to 16");
        REG_FIELD_WR(reg->INJ_CTRL, ADC_REG_SEQLEN, config->nbr_of_conversion - 1);
    #endif

    if (data->conversion_rank <= ADC_REGULAR_RANK_8)
    {
        reg->REG_CTRL0 |= ADC_REG_SEQ0_RK(channel_cfg->channel_id, data->conversion_rank);
    }
    /*ls1010: For Rank 9 to 12 ; lsqsh: For Rank 9 to 16*/
    else 
    {
        reg->REG_CTRL1 |= ADC_REG_SEQ1_RK(channel_cfg->channel_id, data->conversion_rank);
    }
}

static void ADC_InjectChannelConfig(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
    const struct adc_ls_config * config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;
    
    __ASSERT(data->conversion_rank <= ADC_INJECTED_RANK_4, "inject mode rank must be less than or equal to 4");

    REG_FIELD_WR(reg->INJ_CTRL, ADC_INJ_SEQLEN, config->nbr_of_conversion - 1);

    reg->INJ_CTRL |= ADC_INJ_SEQ_RK(channel_cfg->channel_id, data->conversion_rank);

    /* Configure the offset: offset enable/disable, InjectedChannel, offset value */
    switch(data->conversion_rank)
    {
        case ADC_INJECTED_RANK_1:
        /* Set injected channel 1 offset */
        MODIFY_REG(reg->INJ_OFF00,
                    ADC_INJ_OFFSET0_MASK,
                    data->conversion_rank);
        break;
        case ADC_INJECTED_RANK_2:
        /* Set injected channel 2 offset */
        MODIFY_REG(reg->INJ_OFF01,
                    ADC_INJ_OFFSET1_MASK,
                    data->conversion_rank);
        break;
        case ADC_INJECTED_RANK_3:
        /* Set injected channel 3 offset */
        MODIFY_REG(reg->INJ_OFF02,
                    ADC_INJ_OFFSET2_MASK,
                    data->conversion_rank);
        break;
        case ADC_INJECTED_RANK_4:
        MODIFY_REG(reg->INJ_OFF03,
                    ADC_INJ_OFFSET3_MASK,
                    data->conversion_rank);
        break;
        default:
        break;
    }
}

static void ADC_LoopChannelConfig(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
    const struct adc_ls_config * config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    __ASSERT(data->conversion_rank <= ADC_LOOP_RANK_4, "loop mode rank must be less than or equal to 4");

    while(REG_FIELD_RD(reg->FIFO_FLVL, ADC_FIFO_FLVL) != 0){
        REG_FIELD_WR(reg->FIF_CTRL0, ADC_FIFO_CLR, 1);
    }

    REG_FIELD_WR(reg->FIF_CTRL0, ADC_FIF_SEQLEN, config->nbr_of_conversion - 1);

    reg->FIF_CTRL1 = config->fif_ctrl1;

    reg->FIF_CTRL0 |= ADC_FIF_SEQ_RK(channel_cfg->channel_id, data->conversion_rank);
}

static int adc_ls_channel_setup(const struct device *dev, const struct adc_channel_cfg *channel_cfg)
{
    const struct adc_ls_config * config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    ADC_VrefType_SetConfig(config, channel_cfg->reference);

    ADC_Sample_Data_Path_set(config, channel_cfg->channel_id);

    // Configure the clock of the channel
    reg->CLK_CFG |= ADC_CH_CLK_CFG(config->clk_cfg, channel_cfg->channel_id);

    // Configure the sampling interval of the channel
    reg->TSMP |= ADC_TSMP(channel_cfg->acquisition_time, channel_cfg->channel_id);

    data->conversion_rank++;

    if(data->conversion_rank <= config->nbr_of_conversion) {
        if ((config->conversion_mode == regular_mode) || (config->conversion_mode == inject_mode)) {
            uint32_t disc_num = 0;
            uint32_t disc_en = 0;

            if (config->discontinuous_conv_mode == ENABLE) {
                disc_num = config->nbr_of_disc_conversion - 1;
                disc_en = 1;
            } if (config->continuous_conv_mode == ENABLE) {
                disc_num = config->nbr_of_conversion - 1;
                disc_en = 0;
            }

            REG_FIELD_WR(reg->DISC_CTRL, ADC_DISC_NUM, disc_num);

            if(config->conversion_mode == regular_mode) {
                REG_FIELD_WR(reg->DISC_CTRL, ADC_REG_DISCEN, disc_en);
                ADC_RegularChannelConfig(dev, channel_cfg);
            }else if(config->conversion_mode == inject_mode) {
                REG_FIELD_WR(reg->DISC_CTRL, ADC_INJ_DISCEN, disc_en);
                ADC_InjectChannelConfig(dev, channel_cfg);
            }
        }else if(config->conversion_mode == loop_mode) {
            ADC_LoopChannelConfig(dev, channel_cfg);
        }
    }else{
        __ASSERT(false, "Regular Channel Mode Illegal rank");
    }

    if(channel_cfg->channel_id == ADC1_CHANNEL_VBAT) {
        #if(CONFIG_SOC_LS1010)
            adc_channel_vbat_enable();
        #endif
    }else if(channel_cfg->channel_id == ADC1_CHANNEL_TEMPSENSOR) {

    }else if(channel_cfg->channel_id == ADC2_CHANNEL_AMIC) {
        #if(CONFIG_SOC_LS1010)
            HAL_AMIC_MSP_Init();
        #endif
    }

    ADC_Ch_cfg_Reg_SetConfig(reg, channel_cfg->channel_id);

    return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
    struct adc_ls_data *data = CONTAINER_OF(ctx, struct adc_ls_data, ctx);
    const struct device *dev = data->dev;
    const struct adc_ls_config *config = dev->config;
    reg_adc_t *const reg = config->reg;

    SET_BIT(reg->MISC_CTRL, ADC_ADC_EN_MASK);

    if(IS_ADC_SOFTWARE_TRIGTYPE(config->trig_type))
    {
        if(config->conversion_mode == regular_mode) {
            REG_FIELD_WR(reg->TRIG, ADC_REG_TRIG, 1);
        }else if(config->conversion_mode == inject_mode) {
            REG_FIELD_WR(reg->TRIG, ADC_INJ_TRIG, 1);
        }else if(config->conversion_mode == loop_mode) {
            REG_FIELD_WR(reg->TRIG, ADC_FIF_TRIG, 1);
        }
    }
}

static int adc_ls_read(const struct device *dev, const struct adc_sequence *sequence)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    adc_context_lock(&data->ctx, false, NULL);

    adc_context_start_read(&data->ctx, sequence);

    while((reg->INTR_R &(ADC_INTR_REG_END_MASK | ADC_INTR_INJ_END_MASK | ADC_INTR_FIF_END_MASK))== 0);

    ADC_GetValue(dev, config->conversion_mode);

    adc_context_release(&data->ctx, 0);

    return 0;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_ls_read_async(const struct device *dev,
                const struct adc_sequence *sequence,
                struct k_poll_signal *async)
{
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    adc_context_lock(&data->ctx, true, async);

    if(config->conversion_mode == regular_mode) {
        reg->INTR_C |= ADC_INTR_REG_END_MASK;
        reg->INTR_M |= ADC_INTR_REG_END_MASK;
    }else if(config->conversion_mode == inject_mode) {
        reg->INTR_C |= ADC_INTR_INJ_END_MASK;
        reg->INTR_M |= ADC_INTR_INJ_END_MASK;
    }else if(config->conversion_mode == loop_mode) {
        reg->INTR_C |= ADC_INTR_FIF_END_MASK;
        reg->INTR_M |= ADC_INTR_FIF_END_MASK;
    }

    adc_context_start_read(&data->ctx, sequence);

    adc_context_release(&data->ctx, 0);

    return 0;
}
#endif /* CONFIG_ADC_ASYNC */


static void adc_context_update_buffer_pointer(struct adc_context *ctx,
                         bool repeat_sampling){}

void ls_adc_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    const struct adc_ls_config *const config = dev->config;
    struct adc_ls_data *const data = dev->data;
    reg_adc_t *const reg = config->reg;

    if(reg->INTR_S & ADC_INTR_REG_END_MASK) {
        reg->INTR_C |= ADC_INTR_REG_END_MASK;
        REG_FIELD_WR(reg->INTR_M, ADC_INTR_REG_END, 0);
    }

    if(reg->INTR_S & ADC_INTR_INJ_END_MASK) {
        reg->INTR_C |= ADC_INTR_INJ_END_MASK;
        REG_FIELD_WR(reg->INTR_M, ADC_INTR_INJ_END, 0);
    }

    if(reg->INTR_S & ADC_INTR_FIF_END_MASK) {
        reg->INTR_C |= ADC_INTR_FIF_END_MASK;
        REG_FIELD_WR(reg->INTR_M, ADC_INTR_FIF_END, 0);
    }

    if(reg->INTR_S & ADC_INTR_AWDH_MASK) {
        reg->INTR_C |= ADC_INTR_AWDH_MASK;
        REG_FIELD_WR(reg->INTR_M, ADC_INTR_AWDH, 0);
    }

    if(reg->INTR_S & ADC_INTR_AWDL_MASK) {
        reg->INTR_C |= ADC_INTR_AWDL_MASK;
        REG_FIELD_WR(reg->INTR_M, ADC_INTR_AWDL, 0);
    }

    ADC_GetValue(dev, config->conversion_mode);

    adc_context_on_sampling_done(&data->ctx, dev);
}


static const struct adc_driver_api adc_ls_driver_api = {
    .channel_setup  = adc_ls_channel_setup,
    .read           = adc_ls_read,
#ifdef CONFIG_ADC_ASYNC
    .read_async     = adc_ls_read_async,
#endif
};

#define LS_ADC_IRQ_HANDLER(index)                           \
static void adc_ls_irq_config_func_##index(const struct device *dev)   \
{                                                           \
        IRQ_CONNECT(DT_INST_IRQN(index),                    \
            DT_INST_IRQ(index, priority),                   \
            ls_adc_isr,                                     \
            DEVICE_DT_INST_GET(index), 0);                  \
        irq_enable(DT_INST_IRQN(index));                    \
}

#define LS_ADC_INIT(index)                                      \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));\
    LS_ADC_IRQ_HANDLER(index)                                   \
                                                                \
static const struct adc_ls_config adc_ls_cfg_##index = {        \
    .reg = (reg_adc_t *)DT_INST_REG_ADDR(index),                \
    .irq_num = DT_INST_IRQN(index),                             \
    .conversion_mode = DT_INST_PROP(index, conversion_mode),    \
    .data_align = DT_INST_PROP(index, data_align),              \
    .continuous_conv_mode = DT_INST_PROP(index, continuous_conv_mode),      \
    .nbr_of_conversion = DT_INST_PROP(index, nbr_of_conversion),            \
    .discontinuous_conv_mode = DT_INST_PROP(index, discontinuous_conv_mode),\
    .nbr_of_disc_conversion = DT_INST_PROP(index, nbr_of_disc_conversion),  \
    .trig_type = DT_INST_PROP(index, trig_type),                            \
    .adc_drive_type = DT_INST_PROP(index, adc_drive_type),                  \
    .adc_clk_div = DT_INST_PROP(index, adc_clk_div),                        \
    .clk_cfg = DT_INST_PROP(index, clk_cfg),                                \
    .injected_offset = DT_INST_PROP(index, injected_offset),                \
    .fif_ctrl1 = DT_INST_PROP(index, fif_ctrl1),                \
    .clock_source = DT_INST_PROP(index, clock_source),                \
    .irq_config_func = adc_ls_irq_config_func_##index,          \
    IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), )) \
    IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), ))       \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), )) \
};                                                          \
                                                            \
static struct adc_ls_data adc_ls_dev_data_##index = {       \
                                                            \
};                                                          \
                                                            \
DEVICE_DT_INST_DEFINE(index,                                \
            &adc_ls_init,                                   \
            NULL,                                           \
            &adc_ls_dev_data_##index, &adc_ls_cfg_##index,  \
            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
            &adc_ls_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_ADC_INIT)