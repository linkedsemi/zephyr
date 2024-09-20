/*
 * Copyright (c) 2024 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_peci

#include <errno.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/peci.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <reg_peci_type.h>
#include <ls_msp_peci.h>
#include <field_manipulate.h>
#include <soc_clock.h>

#if defined (CONFIG_SOC_LS1010)
    #include <ls_msp_peci.h>
#else
    #define PECI_LS_MAX_TX_BUF_LEN 24
    #define PECI_LS_MAX_RX_BUF_LEN 24

    #define PECI_PRE_DIV_VAL      5

    #define PECI_DAT_LEN_VAL      6

    #define PECI_A_BIT_CYC_VAL    11
    #define PECI_A_TGT_IDX0_VAL   3
    #define PECI_M_TGT_IDX0_VAL   3
    #define PECI_A_SMP_IDX_VAL    9
    #define PECI_A_TGT_IDX1_VAL   8
    #define PECI_M_TGT_IDX1_VAL   8
#endif

LOG_MODULE_REGISTER(peci_ls, LOG_LEVEL_DBG);
#define CPU_FREQ (DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)/1000000)

const uint8_t BitReverseTable256[] =
{
    0X00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
    0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
    0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
    0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
    0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
    0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
    0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
    0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
    0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
    0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
    0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
    0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
    0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
    0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
    0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
typedef void (*irq_cfg_func_t)(const struct device *dev);

struct peci_ls_config {
    irq_cfg_func_t irq_config_func;
	/* peci controller base address */
	struct reg_peci_t *reg;
    uint8_t irq_num;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    struct ls_clk_cfg clk_cfg;
};

struct peci_ls_data {
	struct k_sem trans_sync_sem;
	struct k_sem lock;
};

static void peci_core_reg_print(const struct device *dev)
{

    const struct peci_ls_config *const config = dev->config;
	struct reg_peci_t *const reg = config->reg;
    
    LOG_DBG("-------------------------\n");
    LOG_DBG("INTR_MSK = %08x\n", reg->INTR_MSK);
    LOG_DBG("INTR_CLR = %08x\n", reg->INTR_CLR);
    LOG_DBG("INTR_STT = %08x\n", reg->INTR_STT);
    LOG_DBG("INTR_RAW = %08x\n", reg->INTR_RAW);
    LOG_DBG("PECI_CTRL = %08x\n", reg->PECI_CTRL);
    LOG_DBG("TXRX_REQ = %08x\n", reg->TXRX_REQ);

    LOG_DBG("PECI_A_TIM0 = %08x\n", reg->PECI_A_TIM0);
    LOG_DBG("PECI_A_TIM1 = %08x\n", reg->PECI_A_TIM1);
    LOG_DBG("PECI_M_TIM0 = %08x\n", reg->PECI_M_TIM0);
    LOG_DBG("PECI_M_TIM1 = %08x\n", reg->PECI_M_TIM1);
    LOG_DBG("TX_DAT0 = %08x\n", reg->TX_DAT0);
    LOG_DBG("TX_DAT1 = %08x\n", reg->TX_DAT1);
    LOG_DBG("TX_DAT2 = %08x\n", reg->TX_DAT2);
    LOG_DBG("TX_DAT3 = %08x\n", reg->TX_DAT3);
    LOG_DBG("TX_DAT4 = %08x\n", reg->TX_DAT4);
    LOG_DBG("TX_DAT5 = %08x\n", reg->TX_DAT5);

    LOG_DBG("RX_DAT0 = %08x\n", reg->RX_DAT0);
    LOG_DBG("RX_DAT1 = %08x\n", reg->RX_DAT1);
    LOG_DBG("RX_DAT2 = %08x\n", reg->RX_DAT2);
    LOG_DBG("RX_DAT3 = %08x\n", reg->RX_DAT3);
    LOG_DBG("RX_DAT4 = %08x\n", reg->RX_DAT4);
    LOG_DBG("RX_DAT5 = %08x\n", reg->RX_DAT5);
    LOG_DBG("-------------------------\n\n");
}

void ls_peci_isr(void *arg)
{
    struct device *dev = (struct device *) arg;
	const struct peci_ls_config *config = dev->config;
	struct peci_ls_data *data = dev->data;
    struct reg_peci_t *const reg = config->reg;

    WRITE_REG(reg->INTR_CLR, PECI_INTR_CLR_MASK);
    WRITE_REG(reg->INTR_MSK,0);
    k_sem_give(&data->trans_sync_sem);
}

static int peci_ls_init(const struct device *dev)
{
    const struct device *const clk_dev = DEVICE_DT_GET(LS_CLK_CTRL_NODE);
    const struct peci_ls_config *const config = dev->config;
    struct peci_ls_data *const data = dev->data;
    struct reg_peci_t *const reg = config->reg;

#if defined(CONFIG_PINCTRL)
    int ret;

	if (!device_is_ready(clk_dev)) {
        LOG_DBG("%s device not ready", clk_dev->name);
		return -ENODEV;
	}

    ret = clock_control_on(clk_dev, (clock_control_subsys_t)&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on PECI clock fail %d", ret);
		return ret;
	}

    ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret != 0) {
		LOG_ERR("XEC PECI pinctrl init failed (%d)", ret);
		return ret;
	}
#endif

    reg->PECI_CTRL = FIELD_BUILD(PECI_PRE_DIV, PECI_PRE_DIV_VAL) | FIELD_BUILD(PECI_DAT_LEN, PECI_DAT_LEN_VAL);

	k_sem_init(&data->trans_sync_sem, 0, 1);
	k_sem_init(&data->lock, 1, 1);
    config->irq_config_func(dev);
    return 0;
}

static int peci_ls_configure(const struct device *dev, uint32_t bitrate)
{
    const struct peci_ls_config *const config = dev->config;
	struct peci_ls_data *const data = dev->data;
	struct reg_peci_t *const reg = config->reg;

	k_sem_take(&data->lock, K_FOREVER);

    reg->PECI_A_TIM0 = FIELD_BUILD(PECI_A_BIT_CYC, PECI_A_BIT_CYC_VAL) | FIELD_BUILD(PECI_A_SMP_IDX, PECI_A_SMP_IDX_VAL);
    reg->PECI_A_TIM1 = FIELD_BUILD(PECI_A_TGT_IDX0, PECI_A_TGT_IDX0_VAL) | FIELD_BUILD(PECI_A_TGT_IDX1, PECI_A_TGT_IDX1_VAL);
    reg->PECI_M_TIM0 = FIELD_BUILD(PECI_A_BIT_CYC, PECI_A_BIT_CYC_VAL) | FIELD_BUILD(PECI_A_SMP_IDX, PECI_A_SMP_IDX_VAL);
    reg->PECI_M_TIM1 = FIELD_BUILD(PECI_M_TGT_IDX0, PECI_M_TGT_IDX0_VAL) | FIELD_BUILD(PECI_M_TGT_IDX1, PECI_M_TGT_IDX1_VAL);

    k_sem_give(&data->lock);

    return 0;
}

static int peci_ls_enable(const struct device *dev)
{
    const struct peci_ls_config *const config = dev->config;
    struct peci_ls_data *const data = dev->data;
    struct reg_peci_t *const reg = config->reg;

    k_sem_take(&data->lock, K_FOREVER);

    WRITE_REG(reg->INTR_CLR, PECI_INTR_CLR_MASK);

    k_sem_give(&data->lock);

    return 0;
}

static int peci_ls_disable(const struct device *dev)
{
    struct peci_ls_data *const data = dev->data;
    const struct peci_ls_config *config = dev->config;

    k_sem_take(&data->lock, K_FOREVER);

    irq_disable(config->irq_num);

    k_sem_give(&data->lock);

    return 0;
}

static int peci_ls_transfer(const struct device *dev, struct peci_msg *msg)
{

    const struct peci_ls_config *const config = dev->config;
	struct peci_ls_data *const data = dev->data;
	struct reg_peci_t *const reg = config->reg;
	struct peci_buf *peci_rx_buf = &msg->rx_buffer;
	struct peci_buf *peci_tx_buf = &msg->tx_buffer;
	int ret = 0;
    uint8_t txbuf8[24] = {0};
    uint8_t rxbuf8[24] = {0};
    uint32_t txbuf32[6] = {0};
    uint32_t rxbuf32[6] = {0};
    volatile uint8_t i = 0;

    if(peci_tx_buf->len > PECI_LS_MAX_TX_BUF_LEN || peci_rx_buf->len > PECI_LS_MAX_RX_BUF_LEN)
    {
       ret = -EINVAL;
       goto out;
    }

    k_sem_take(&data->lock, K_FOREVER);

    txbuf8[0] = BitReverseTable256[msg->addr];
    txbuf8[1] = BitReverseTable256[peci_tx_buf->len];
    txbuf8[2] = BitReverseTable256[peci_rx_buf->len];
    txbuf8[3] = BitReverseTable256[msg->cmd_code];

    if(peci_tx_buf->len > 1)
    {
        for(i = 0; i < peci_tx_buf->len-1; i++)
        {
           txbuf8[i+4] = BitReverseTable256[peci_tx_buf->buf[i]];
        }
    }

    memcpy(txbuf32, txbuf8, 24);

    WRITE_REG(reg->INTR_CLR,PECI_INTR_CLR_MASK);
    WRITE_REG(reg->INTR_MSK,PECI_INTR_MSK_MASK);
    reg->TX_DAT0 = txbuf32[0];
    reg->TX_DAT1 = txbuf32[1];
    reg->TX_DAT2 = txbuf32[2];
    reg->TX_DAT3 = txbuf32[3];
    reg->TX_DAT4 = txbuf32[4];
    reg->TX_DAT5 = txbuf32[5];
    WRITE_REG(reg->TXRX_REQ,PECI_TXRX_REQ_MASK);

    k_sem_take(&data->trans_sync_sem, K_FOREVER);

    peci_core_reg_print(dev);

    rxbuf32[0] = reg->RX_DAT0;
    rxbuf32[1] = reg->RX_DAT1;
    rxbuf32[2] = reg->RX_DAT2;
    rxbuf32[3] = reg->RX_DAT3;
    rxbuf32[4] = reg->RX_DAT4;
    rxbuf32[5] = reg->RX_DAT5;
    memcpy(rxbuf8, rxbuf32, 24);

    for(i = 0; i < peci_rx_buf->len; i++)
    {
        peci_rx_buf->buf[i] = BitReverseTable256[rxbuf8[peci_tx_buf->len+3+1+i]];
    }
    k_sem_give(&data->lock);

out:
    return ret;
}

static const struct peci_driver_api peci_ls_driver_api = {
    .config = peci_ls_configure,
    .enable = peci_ls_enable,
    .disable = peci_ls_disable,
    .transfer = peci_ls_transfer,
};

#if defined(CONFIG_PINCTRL)
    #define LS_PECI_IRQ_HANDLER(index)                          \
    static void peci_ls_irq_config_func_##index(const struct device *dev)   \
    {                                                           \
            IRQ_CONNECT(DT_INST_IRQN(index),                    \
                DT_INST_IRQ(index, priority),	                \
                ls_peci_isr,                                    \
                DEVICE_DT_INST_GET(index), 0);                  \
            irq_enable(DT_INST_IRQN(index));                    \
    }                                                           \
                                                                
    #define LS_PECI_INIT(index)                                 \
        PINCTRL_DT_INST_DEFINE(index);                          \
        LS_PECI_IRQ_HANDLER(index)                              \
                                                                \
    static const struct peci_ls_config peci_ls_cfg_##index = {  \
        .reg = (struct reg_peci_t *)DT_INST_REG_ADDR(index),   \
        .irq_num = DT_INST_IRQN(index),                         \
        .irq_config_func = peci_ls_irq_config_func_##index,      \
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),          \
        .clk_cfg = LS_DT_CLK_CFG_ITEM(index),                   \
    };                                                          \
                                                                \
    static struct peci_ls_data peci_ls_dev_data_##index = {     \
                                                                \
    };                                                          \
                                                                \
    DEVICE_DT_INST_DEFINE(index,                                \
                &peci_ls_init,                                  \
                NULL,                                           \
                &peci_ls_dev_data_##index, &peci_ls_cfg_##index, \
                POST_KERNEL, CONFIG_PECI_INIT_PRIORITY,         \
                &peci_ls_driver_api);
#else
    #define LS_PECI_IRQ_HANDLER(index)                          \
    static void peci_ls_irq_config_func_##index(const struct device *dev)   \
    {                                                           \
            IRQ_CONNECT(DT_INST_IRQN(index),                    \
                DT_INST_IRQ(index, priority),	                \
                ls_peci_isr,                                    \
                DEVICE_DT_INST_GET(index), 0);                  \
            irq_enable(DT_INST_IRQN(index));                    \
    }                                                           \
                                                                
    #define LS_PECI_INIT(index)                                 \
        LS_PECI_IRQ_HANDLER(index)                              \
                                                                \
    static const struct peci_ls_config peci_ls_cfg_##index = {  \
        .reg = (struct reg_peci_t *)DT_INST_REG_ADDR(index),   \
        .irq_num = DT_INST_IRQN(index),                         \
        .irq_config_func = peci_ls_irq_config_func_##index,      \
        .clk_cfg = LS_DT_CLK_CFG_ITEM(index),                   \
    };                                                          \
                                                                \
    static struct peci_ls_data peci_ls_dev_data_##index = {     \
                                                                \
    };                                                          \
                                                                \
    DEVICE_DT_INST_DEFINE(index,                                \
                &peci_ls_init,                                  \
                NULL,                                           \
                &peci_ls_dev_data_##index, &peci_ls_cfg_##index, \
                POST_KERNEL, CONFIG_PECI_INIT_PRIORITY,         \
                &peci_ls_driver_api);
#endif
DT_INST_FOREACH_STATUS_OKAY(LS_PECI_INIT)