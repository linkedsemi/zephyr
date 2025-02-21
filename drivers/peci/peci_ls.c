/*
 * Copyright (c) 2024 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_ls_peci

#include <errno.h>
#include <soc.h>
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/peci.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/peci-legacy.h>
#include <reg_peci_type.h>
#include <field_manipulate.h>
#include <zephyr/drivers/clock_control.h>
#include <soc_clock.h>

/* PECI protocol */
#define PECI_ADDR_LEN  1
#define PECI_FCS_LEN   1
#define PECI_WRLEN_LEN 1
#define PECI_RDLEN_LEN 1

/* feature */
#define PECI_LS_MAX_TX_BUF_LEN 24
#define PECI_LS_MAX_RX_BUF_LEN 24

/* reg val */
#define PECI_PRE_DIV_VAL    0x20
#define PECI_A_BIT_CYC_VAL  11
#define PECI_A_TGT_IDX0_VAL 3
#define PECI_M_TGT_IDX0_VAL 3
#define PECI_A_SMP_IDX_VAL  6
#define PECI_A_TGT_IDX1_VAL 8
#define PECI_M_TGT_IDX1_VAL 8

LOG_MODULE_REGISTER(peci_ls, LOG_LEVEL_DBG);

static int peci_lib_xfer_base_ls(struct peci_adapter *adapter, struct peci_xfer_msg *msg);

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct peci_ls_config {
    irq_cfg_func_t irq_config_func;
    /* peci controller base address */
    reg_peci_t *reg;
    uint8_t irq_num;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    struct ls_clk_cfg cctl_cfg;
};

struct peci_ls_data {
	struct k_sem trans_sync_sem;
	struct k_sem lock;
    
    struct device *dev;
    struct peci_adapter *adapter;
};

uint8_t crc8_ls(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void peci_core_reg_print(const struct device *dev)
{
    const struct peci_ls_config *const dev_config = dev->config;
    reg_peci_t *const reg = dev_config->reg;

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

void peci_ls_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    const struct peci_ls_config *dev_config = dev->config;
    struct peci_ls_data *dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;

    WRITE_REG(reg->INTR_CLR, PECI_INTR_CLR_MASK);
    k_sem_give(&dev_data->trans_sync_sem);
}

static int peci_ls_init(struct device *dev)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *dev_data;
    reg_peci_t *const reg = dev_config->reg;
    struct peci_adapter *adapter;

    if (dev_config->cctl_cfg.cctl_dev) {
        const struct device *clk_dev = dev_config->cctl_cfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_DBG("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->cctl_cfg);
    }

    adapter = peci_alloc_adapter(dev, sizeof(*dev_data));
    if(!adapter)
        return -ENOMEM;
    
    dev_data = adapter->dev.data;
    LOG_WRN("Debug: peci dev = %p\n", (void *)dev);
    LOG_WRN("Debug in %s: data = %p to %p\n", __func__, (void *)dev_data, (void *)dev_data+sizeof(*dev_data));
    LOG_DBG("Debug in %s: dev->data = %p\n", __func__, (void *)dev->data);

    dev_data->adapter = adapter;
    LOG_DBG("Debug: data->adapter = %p\n", (void *)dev_data->adapter);
    LOG_DBG("Debug: adapter = %p\n", (void *)adapter);

    dev_data->dev = dev;
    dev->data = dev_data;
    LOG_DBG("Debug in %s: adapter->dev = %p\n", __func__, (void *)&adapter->dev);
    LOG_DBG("Debug in %s: data->adapter->dev = %p\n", __func__, (void *)&dev_data->adapter->dev);
    LOG_DBG("Debug in %s: adapter->dev->data = %p, dev->data = %p\n", __func__, (void *)adapter->dev.data, (void *)dev->data);

    strncpy(dev_data->adapter->name, dev->name, sizeof(dev_data->adapter->name));
    LOG_DBG("PECI adapter %s initialized\n", dev->name);
    dev_data->adapter->xfer = peci_lib_xfer_base_ls;
    dev_data->adapter->use_dma = false;

    peci_core_init();
    peci_add_adapter(dev_data->adapter);

#if defined(CONFIG_PINCTRL)
    if (dev_config->pcfg) {
        int ret;
        ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
        if (ret != 0) {
            LOG_DBG("maybe no PECI pinctrl node (%d)", ret);
        }
    }
#endif

    reg->PECI_CTRL = FIELD_BUILD(PECI_PRE_DIV, PECI_PRE_DIV_VAL);

    k_sem_init(&dev_data->trans_sync_sem, 0, K_SEM_MAX_LIMIT);
    k_sem_init(&dev_data->lock, 1, 1);
    dev_config->irq_config_func(dev);

    return 0;
}

static int peci_ls_configure(const struct device *dev, uint32_t bitrate)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *const dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;

    k_sem_take(&dev_data->lock, K_FOREVER);

    reg->PECI_A_TIM0 = FIELD_BUILD(PECI_A_BIT_CYC, PECI_A_BIT_CYC_VAL) | FIELD_BUILD(PECI_A_SMP_IDX, PECI_A_SMP_IDX_VAL);
    reg->PECI_A_TIM1 = FIELD_BUILD(PECI_A_TGT_IDX0, PECI_A_TGT_IDX0_VAL) | FIELD_BUILD(PECI_A_TGT_IDX1, PECI_A_TGT_IDX1_VAL);
    reg->PECI_M_TIM0 = FIELD_BUILD(PECI_A_BIT_CYC, PECI_A_BIT_CYC_VAL) | FIELD_BUILD(PECI_A_SMP_IDX, PECI_A_SMP_IDX_VAL);
    reg->PECI_M_TIM1 = FIELD_BUILD(PECI_M_TGT_IDX0, PECI_M_TGT_IDX0_VAL) | FIELD_BUILD(PECI_M_TGT_IDX1, PECI_M_TGT_IDX1_VAL);

    k_sem_give(&dev_data->lock);

    return 0;
}

static int peci_ls_enable(const struct device *dev)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *const dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;

    k_sem_take(&dev_data->lock, K_FOREVER);

    WRITE_REG(reg->INTR_CLR, PECI_INTR_CLR_MASK);
    SET_BIT(reg->PECI_CTRL2, PECI_BIT_REVERSE_MASK);

    k_sem_give(&dev_data->lock);

    return 0;
}

static int peci_ls_disable(const struct device *dev)
{
    struct peci_ls_data *const dev_data = dev->data;
    const struct peci_ls_config *dev_config = dev->config;

    k_sem_take(&dev_data->lock, K_FOREVER);

    irq_disable(dev_config->irq_num);

    k_sem_give(&dev_data->lock);

    return 0;
}

static int peci_ls_transfer(const struct device *dev, struct peci_msg *msg)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *const dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;
    struct peci_buf *peci_rx_buf = &msg->rx_buffer;
    struct peci_buf *peci_tx_buf = &msg->tx_buffer;
    int ret = 0;
    uint32_t idx = 0;
    uint32_t txbuf32[8] = {};
    uint32_t rxbuf32[8] = {};
    uint8_t *txbuf8 = (uint8_t *)txbuf32;
    uint8_t *rxbuf8 = (uint8_t *)rxbuf32;
    uint8_t crc_result = 0;
    uint32_t reg_len = 0;

    if (peci_tx_buf->len > PECI_LS_MAX_TX_BUF_LEN || peci_rx_buf->len > PECI_LS_MAX_RX_BUF_LEN) {
        ret = -EINVAL;
        goto out;
    }

    k_sem_take(&dev_data->lock, K_FOREVER);

    reg_len = PECI_RDLEN_LEN + peci_tx_buf->len + PECI_FCS_LEN;
    if (peci_rx_buf) {
        reg_len += peci_rx_buf->len + PECI_FCS_LEN;
    }
    MODIFY_REG(reg->PECI_CTRL, PECI_DAT_LEN_MASK, reg_len << PECI_DAT_LEN_POS);

    idx = 0;
    txbuf8[idx++] = msg->addr;
    txbuf8[idx++] = peci_tx_buf->len;
    txbuf8[idx++] = peci_rx_buf->len;
    txbuf8[idx++] = msg->cmd_code;
    for (uint8_t i = 0; i < peci_tx_buf->len - 1; i++) {
        txbuf8[idx++] = peci_tx_buf->buf[i];
    }
    crc_result = crc8_ls(txbuf8, idx);
    txbuf8[idx] = crc_result;
    WRITE_REG(reg->INTR_CLR, PECI_INTR_CLR_MASK);
    WRITE_REG(reg->INTR_MSK, PECI_INTR_MSK_MASK);
    reg->TX_DAT0 = txbuf32[0];
    reg->TX_DAT1 = txbuf32[1];
    reg->TX_DAT2 = txbuf32[2];
    reg->TX_DAT3 = txbuf32[3];
    reg->TX_DAT4 = txbuf32[4];
    reg->TX_DAT5 = txbuf32[5];
    reg->TX_DAT6 = txbuf32[6];
    reg->TX_DAT7 = txbuf32[7];
    WRITE_REG(reg->TXRX_REQ, PECI_TXRX_REQ_MASK);

    k_sem_take(&dev_data->trans_sync_sem, K_FOREVER);

    WRITE_REG(reg->INTR_MSK, 0);

    __ASSERT(reg->TX_DAT0 == reg->RX_DAT0, "check waveform sample fail");

    peci_core_reg_print(dev);

    rxbuf32[0] = reg->RX_DAT0;
    rxbuf32[1] = reg->RX_DAT1;
    rxbuf32[2] = reg->RX_DAT2;
    rxbuf32[3] = reg->RX_DAT3;
    rxbuf32[4] = reg->RX_DAT4;
    rxbuf32[5] = reg->RX_DAT5;
    rxbuf32[6] = reg->RX_DAT6;
    rxbuf32[7] = reg->RX_DAT7;

    for (uint8_t i = 0; i < peci_rx_buf->len; i++) {
        peci_rx_buf->buf[i] = rxbuf8[PECI_ADDR_LEN + PECI_WRLEN_LEN + PECI_RDLEN_LEN + peci_tx_buf->len + PECI_FCS_LEN + i];
    }
    k_sem_give(&dev_data->lock);

out:
    return ret;
}

static int peci_lib_xfer_base_ls(struct peci_adapter *adapter, struct peci_xfer_msg *msg)
{
    struct peci_msg *msg_ls;
    struct peci_ls_data *ls_data = adapter->dev.data;
    int ret;

    LOG_DBG("Debug in %s: \n", __func__);
    
    LOG_DBG("Debug in %s: msg->rx_buf = %p\n", __func__, msg->rx_buf);
    LOG_DBG("Debug in %s: msg->tx_buf = %p\n", __func__, msg->tx_buf);
    LOG_DBG("Debug in %s: msg = %p\n", __func__, msg);

    msg_ls = malloc(sizeof(struct peci_msg));
    if (msg_ls == NULL)
        return -ENOMEM;
        
    LOG_DBG("Debug in %s: finish malloc msg_ls\n", __func__);
    msg_ls->addr = msg->addr;
    msg_ls->cmd_code = 0;
    msg_ls->tx_buffer.buf = NULL;
    msg_ls->rx_buffer.buf = NULL;
    msg_ls->tx_buffer.len = 0;
    msg_ls->rx_buffer.len = 0;
    LOG_DBG("Debug in %s: msg_ls->addr = %x\n", __func__, msg_ls->addr);

    if(msg->tx_buf != NULL){
        LOG_DBG("Debug in %s: msg->tx_buf != NULL\n", __func__);
        msg_ls->cmd_code = msg->tx_buf[0];
        msg_ls->tx_buffer.buf = msg->tx_buf + 1; 
    }

    if(msg->rx_buf != NULL){
        LOG_DBG("Debug in %s: msg->rx_buf != NULL\n", __func__);
        msg_ls->rx_buffer.buf = msg->rx_buf;
    }

    msg_ls->tx_buffer.len = msg->tx_len;
    msg_ls->rx_buffer.len = msg->rx_len;

    LOG_DBG("Debug in %s: msg_ls->cmd_code = %x\n", __func__, msg_ls->cmd_code);
    LOG_DBG("Debug in %s: adapter = %p, adapter->dev = %p\n", __func__, adapter, ls_data->dev);
    ret = peci_ls_transfer(ls_data->dev, msg_ls);

    LOG_DBG("Debug in %s: msg->rx_buf = %p\n", __func__, msg->rx_buf);
    LOG_DBG("Debug in %s: msg->tx_buf = %p\n", __func__, msg->tx_buf);
    LOG_DBG("Debug in %s: msg = %p\n", __func__, msg);

    free(msg_ls);
    return ret;
}

static const struct peci_driver_api peci_ls_driver_api = {
    .config = peci_ls_configure,
    .enable = peci_ls_enable,
    .disable = peci_ls_disable,
    .transfer = peci_ls_transfer,
};

#define LS_PECI_IRQ_HANDLER(index)                                        \
    static void peci_ls_irq_config_func_##index(const struct device *dev) \
    {                                                                     \
        IRQ_CONNECT(DT_INST_IRQN(index),                                  \
                    DT_INST_IRQ(index, priority),                         \
                    peci_ls_isr,                                          \
                    DEVICE_DT_INST_GET(index),                            \
                    0);                                                   \
        irq_enable(DT_INST_IRQN(index));                                  \
    }

#define LS_PECI_INIT(index)                                                             \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));                        \
    LS_PECI_IRQ_HANDLER(index)                                                          \
                                                                                        \
    static const struct peci_ls_config peci_ls_cfg_##index = {                          \
        .reg = (reg_peci_t *)DT_INST_REG_ADDR(index),                                   \
        .irq_num = DT_INST_IRQN(index),                                                 \
        .irq_config_func = peci_ls_irq_config_func_##index,                             \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), ))   \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(index), ))     \
    };                                                                                  \
                                                                                        \
    static struct peci_ls_data peci_ls_dev_data_##index = {};                           \
                                                                                        \
    DEVICE_DT_INST_DEFINE(index,                                                        \
                          &peci_ls_init,                                                \
                          NULL,                                                         \
                          &peci_ls_dev_data_##index,                                    \
                          &peci_ls_cfg_##index,                                         \
                          POST_KERNEL,                                                  \
                          CONFIG_PECI_INIT_PRIORITY,                                    \
                          &peci_ls_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_PECI_INIT)
