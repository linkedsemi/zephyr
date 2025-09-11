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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/peci-legacy.h>
#include <reg_peci_type.h>
#include <field_manipulate.h>
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

/* PECI protocol */
#define PECI_ADDR_LEN  1
#define PECI_FCS_LEN   1
#define PECI_WRLEN_LEN 1
#define PECI_RDLEN_LEN 1

/* feature */
#define PECI_LS_MAX_XFER_LEN 1024

/* reg val */

#define PECI_A_BIT_CYC_VAL  11
#define PECI_A_TGT_IDX0_VAL 3
#define PECI_M_TGT_IDX0_VAL 3
#define PECI_A_SMP_IDX_VAL  6
#define PECI_A_TGT_IDX1_VAL 8
#define PECI_M_TGT_IDX1_VAL 8

LOG_MODULE_REGISTER(peci_ls, LOG_LEVEL_DBG);

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct peci_ls_config {
    irq_cfg_func_t irq_config_func;
    /* peci controller base address */
    reg_peci_t *reg;
    uint8_t irq_num;
    uint32_t clock_source;
    uint32_t peci_frequence;
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct peci_ls_data {
    struct k_sem xfer_sync_sem;
    struct k_sem lock;
    uint8_t buf_idx;
    
    struct device *dev;
    struct peci_adapter *adapter;
};

static int peci_init_adapter(struct device *dev, struct peci_ls_data *dev_data, struct peci_adapter *adapter);
static int peci_lib_xfer_base_ls(struct peci_adapter *adapter, struct peci_xfer_msg *msg);

__unused static void peci_core_reg_print(const struct device *dev)
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

    WRITE_REG(reg->INTR_CLR, reg->INTR_STT);
    k_sem_give(&dev_data->xfer_sync_sem);
}

static int peci_ls_init(struct device *dev)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;
    struct peci_adapter *adapter;
    __maybe_unused int ret;

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

    ret = peci_init_adapter(dev, dev_data, adapter);

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

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERROR("Could not configure pins");
    }
#endif

    const uint32_t max_div = PECI_PRE_DIV_MASK >> PECI_PRE_DIV_POS;
#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        uint32_t rate;
        clock_control_get_rate(clk_dev, (clock_control_subsys_t)&dev_config->clock_source, &rate);

        uint32_t div = dev_config->peci_frequence ?
                        ((rate / dev_config->peci_frequence) / (PECI_A_BIT_CYC_VAL + 1)) : max_div;

        if (div > max_div) {
            LOG_ERR("peci_frequence value: %d error", dev_config->peci_frequence);
            return -EINVAL;
        }
        while((((rate / (div + 1)) / (PECI_A_BIT_CYC_VAL + 1)) > dev_config->peci_frequence) && (div > 0)) {
            div++;
        }
        reg->PECI_CTRL = FIELD_BUILD(PECI_PRE_DIV, div);
    }
#else
    reg->PECI_CTRL = FIELD_BUILD(PECI_PRE_DIV, max_div);
#endif

    k_sem_init(&dev_data->xfer_sync_sem, 0, K_SEM_MAX_LIMIT);
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

static void peci_wr_pingpong_buf(const struct device *dev, uint32_t *txbuf32, bool pingpong)
{
    const struct peci_ls_config *const dev_config = dev->config;
    reg_peci_t *const reg = dev_config->reg;

    if (pingpong) {
        reg->TX_H_DAT0 = txbuf32[0];
        reg->TX_H_DAT1 = txbuf32[1];
        reg->TX_H_DAT2 = txbuf32[2];
        reg->TX_H_DAT3 = txbuf32[3];
        reg->TX_H_DAT4 = txbuf32[4];
        reg->TX_H_DAT5 = txbuf32[5];
        reg->TX_H_DAT6 = txbuf32[6];
        reg->TX_H_DAT7 = txbuf32[7];
    } else {
        reg->TX_DAT0 = txbuf32[0];
        reg->TX_DAT1 = txbuf32[1];
        reg->TX_DAT2 = txbuf32[2];
        reg->TX_DAT3 = txbuf32[3];
        reg->TX_DAT4 = txbuf32[4];
        reg->TX_DAT5 = txbuf32[5];
        reg->TX_DAT6 = txbuf32[6];
        reg->TX_DAT7 = txbuf32[7];
    }
}

static void peci_rd_pingpong_buf(const struct device *dev, uint32_t *rxbuf32, bool pingpong)
{
    const struct peci_ls_config *const dev_config = dev->config;
    reg_peci_t *const reg = dev_config->reg;

    if (pingpong) {
        rxbuf32[0] = reg->RX_H_DAT0;
        rxbuf32[1] = reg->RX_H_DAT1;
        rxbuf32[2] = reg->RX_H_DAT2;
        rxbuf32[3] = reg->RX_H_DAT3;
        rxbuf32[4] = reg->RX_H_DAT4;
        rxbuf32[5] = reg->RX_H_DAT5;
        rxbuf32[6] = reg->RX_H_DAT6;
        rxbuf32[7] = reg->RX_H_DAT7;
    } else {
        rxbuf32[0] = reg->RX_DAT0;
        rxbuf32[1] = reg->RX_DAT1;
        rxbuf32[2] = reg->RX_DAT2;
        rxbuf32[3] = reg->RX_DAT3;
        rxbuf32[4] = reg->RX_DAT4;
        rxbuf32[5] = reg->RX_DAT5;
        rxbuf32[6] = reg->RX_DAT6;
        rxbuf32[7] = reg->RX_DAT7;
    }
}

static bool peci_tx_byte(const struct device *dev, uint8_t *buf_u8, uint8_t byte)
{
    struct peci_ls_data *dev_data = dev->data;
    bool full = false;

    buf_u8[dev_data->buf_idx++] = byte;

    if (dev_data->buf_idx % 32 == 0) {
        dev_data->buf_idx = 0;
        full = true;
    }

    return full;
}

static int peci_ls_transfer(const struct device *dev, struct peci_msg *msg)
{
    const struct peci_ls_config *const dev_config = dev->config;
    struct peci_ls_data *const dev_data = dev->data;
    reg_peci_t *const reg = dev_config->reg;
    uint8_t crc_result = 0;
    bool pingpong = false;
    bool is_to_req = true;
    int ret = 0;

    union {
        uint32_t u32[8];
        uint8_t u8[32];
    } buf = {};

    /* pre-calculate lengths */
    const uint16_t xfer_tx_len = PECI_ADDR_LEN + PECI_WRLEN_LEN + PECI_RDLEN_LEN + msg->tx_buffer.len + PECI_FCS_LEN;
    const uint16_t xfer_rx_len = msg->rx_buffer.len ? (msg->rx_buffer.len + PECI_FCS_LEN) : 0;
    const uint16_t xfer_len = xfer_tx_len + xfer_rx_len;
    const uint8_t reg_tx_tail_len = xfer_tx_len % 32;
    const uint16_t reg_rx_len = reg_tx_tail_len + xfer_rx_len;

    if (xfer_len > PECI_LS_MAX_XFER_LEN) {
        ret = -EINVAL;
    }

    /* initialize device state */
    dev_data->buf_idx = 0;

    k_sem_take(&dev_data->lock, K_FOREVER);

    /* configure registers */
    SET_BIT(reg->INTR_CLR, PECI_INTR_MSK_MASK | PECI_INTR_32BYTES_END_MSK_MASK);
    if (xfer_len > 32) {
        SET_BIT(reg->INTR_MSK, PECI_INTR_32BYTES_END_MSK_MASK);
        SET_BIT(reg->PECI_CTRL, PECI_WT_MODE_MASK);
        MODIFY_REG(reg->PECI_CTRL, PECI_DAT_WLEN_MASK, xfer_tx_len << PECI_DAT_WLEN_POS);
    }
    MODIFY_REG(reg->PECI_CTRL, PECI_DAT_LEN_MASK, (xfer_len - PECI_ADDR_LEN - PECI_WRLEN_LEN) << PECI_DAT_LEN_POS);

    /* send header */
    peci_tx_byte(dev, buf.u8, msg->addr);
    peci_tx_byte(dev, buf.u8, msg->tx_buffer.len);
    peci_tx_byte(dev, buf.u8, msg->rx_buffer.len);

    const int16_t tx_len = msg->tx_buffer.len - 1;
    if (msg->cmd_code == PECI_CMD_PING) {
        crc_result = crc8(buf.u8, 3, 0x7, crc_result, false);
    } else {
        peci_tx_byte(dev, buf.u8, msg->cmd_code);

        /* calculate crc */
        crc_result = crc8(buf.u8, 4, 0x7, crc_result, false);
    }

    if (tx_len > 0) {
         /* msg->tx_buffer.len - 1: because msg->cmd_code is the first byte */
        const uint8_t *tx_buf = msg->tx_buffer.buf;

        crc_result = crc8(tx_buf, tx_len, 0x7, crc_result, false);

        pingpong = false;
        /* send payload */
        for (uint16_t i = 0; i < tx_len; i++) {
            bool full = peci_tx_byte(dev, buf.u8, tx_buf[i]);
            if (full) {
                peci_wr_pingpong_buf(dev, buf.u32, pingpong);
                if (is_to_req && pingpong) {
                    WRITE_REG(reg->TXRX_REQ, PECI_TXRX_REQ_MASK);
                    is_to_req = false;
                }
                pingpong ^= true;
                if (!is_to_req) {
                    k_sem_take(&dev_data->xfer_sync_sem, K_FOREVER);
                }
            }
        }
    }

    /* send crc */
    peci_tx_byte(dev, buf.u8, crc_result);
    peci_wr_pingpong_buf(dev, buf.u32, pingpong);
    if (is_to_req) {
        WRITE_REG(reg->TXRX_REQ, PECI_TXRX_REQ_MASK);
        /* is_to_req = false; */
    }

    if (xfer_rx_len > 0) {
        /* receive handling */
        pingpong = false;
        uint8_t *rx_dest = msg->rx_buffer.buf;
        bool is_first_rx = true;
        const uint16_t rx_loop_count = reg_rx_len / 32;
        uint8_t rx_valid;

        for (uint16_t i = 0; i < rx_loop_count; i++) {
            k_sem_take(&dev_data->xfer_sync_sem, K_FOREVER);
            peci_rd_pingpong_buf(dev, buf.u32, pingpong);
            rx_valid = is_first_rx ? (32 - reg_tx_tail_len) : 32;
            memcpy(rx_dest, buf.u8 + (is_first_rx ? reg_tx_tail_len : 0), rx_valid);
            rx_dest += rx_valid;
            is_first_rx = false;
            pingpong ^= true;
        }

        MODIFY_REG(reg->INTR_MSK, PECI_INTR_32BYTES_END_MSK_MASK | PECI_INTR_MSK_MASK, PECI_INTR_MSK_MASK);

        /* handle remaining bytes */
        const uint8_t rx_remain = reg_rx_len % 32;
        if (rx_remain) {
            k_sem_take(&dev_data->xfer_sync_sem, K_FOREVER);
            peci_rd_pingpong_buf(dev, buf.u32, pingpong);
            memcpy(rx_dest, buf.u8 + (is_first_rx ? reg_tx_tail_len : 0), rx_remain - (is_first_rx ? reg_tx_tail_len : 0));
            /* is_first_rx = false; */
        }
    } else {
        MODIFY_REG(reg->INTR_MSK, PECI_INTR_32BYTES_END_MSK_MASK | PECI_INTR_MSK_MASK, PECI_INTR_MSK_MASK);
        k_sem_take(&dev_data->xfer_sync_sem, K_FOREVER);
    }

    CLEAR_BIT(reg->INTR_MSK, PECI_INTR_MSK_MASK | PECI_INTR_32BYTES_END_MSK_MASK);

    k_sem_give(&dev_data->lock);

out:
    return ret;
}

static int peci_lib_xfer_base_ls(struct peci_adapter *adapter, struct peci_xfer_msg *msg)
{
    struct peci_msg *msg_ls;
    struct peci_ls_data *ls_data = adapter->dev.data;
    int ret;

    if (!adapter || !adapter->dev.data || !msg) {
        return -EINVAL;
    }

    LOG_DBG("Debug in %s: \n", __func__);
    
    LOG_DBG("Debug in %s: msg->rx_buf = %p\n", __func__, msg->rx_buf);
    LOG_DBG("Debug in %s: msg->tx_buf = %p\n", __func__, msg->tx_buf);
    LOG_DBG("Debug in %s: msg = %p\n", __func__, msg);

    msg_ls = calloc(1, sizeof(struct peci_msg));
    if (msg_ls == NULL)
        return -ENOMEM;

    LOG_DBG("Debug in %s: finish malloc msg_ls\n", __func__);
    msg_ls->addr = msg->addr;
    msg_ls->tx_buffer.buf = NULL;
    msg_ls->rx_buffer.buf = NULL;
    msg_ls->tx_buffer.len = msg->tx_len;
    msg_ls->rx_buffer.len = msg->rx_len;

    if (msg->tx_buf && msg->tx_len > 0) {
        msg_ls->cmd_code = msg->tx_buf[0];
        msg_ls->tx_buffer.buf = malloc(msg->tx_len);
        if (msg_ls->tx_buffer.buf == NULL) {
            free(msg_ls);
            return -ENOMEM;
        }
        memcpy(msg_ls->tx_buffer.buf, msg->tx_buf, msg->tx_len);
    }

    if (msg->rx_buf && msg->rx_len > 0) {
        msg_ls->rx_buffer.buf = malloc(msg->rx_len + PECI_FCS_LEN);
        if (msg_ls->rx_buffer.buf == NULL) {
            free(msg_ls->tx_buffer.buf);
            free(msg_ls);
            return -ENOMEM;
        }
    }

    LOG_DBG("Debug in %s: msg_ls->cmd_code = %x\n", __func__, msg_ls->cmd_code);
    LOG_DBG("Debug in %s: adapter = %p, adapter->dev = %p\n", __func__, adapter, ls_data->dev);
    ret = peci_ls_transfer(ls_data->dev, msg_ls);

    if(msg->rx_buf && msg->rx_len > 0 && msg_ls->rx_buffer.buf) {
        memcpy(msg->rx_buf, msg_ls->rx_buffer.buf, msg->rx_len);
    }

    LOG_DBG("Debug in %s: msg->rx_buf = %p\n", __func__, msg->rx_buf);
    LOG_DBG("Debug in %s: msg->rx_len = %d\n", __func__, msg->rx_len);
    LOG_DBG("Debug in %s: msg->tx_buf = %p\n", __func__, msg->tx_buf);
    LOG_DBG("Debug in %s: msg->tx_len = %d\n", __func__, msg->tx_len);
    LOG_DBG("Debug in %s: msg = %p\n", __func__, msg);
    LOG_DBG("Debug in %s: msg_ls->rx_buffer.buf = %p\n", __func__, msg_ls->rx_buffer.buf);
    LOG_DBG("Debug in %s: msg_ls->rx_buffer.len = %d\n", __func__, msg_ls->rx_buffer.len);
    LOG_DBG("Debug in %s: msg_ls->tx_buffer.buf = %p\n", __func__, msg_ls->tx_buffer.buf);
    LOG_DBG("Debug in %s: msg_ls->tx_buffer.len = %d\n", __func__, msg_ls->tx_buffer.len);

    free(msg_ls->tx_buffer.buf);
    LOG_DBG("Debug in %s: free(msg_ls->tx_buffer.buf)\n", __func__);
    free(msg_ls->rx_buffer.buf);
    LOG_DBG("Debug in %s: free(msg_ls->rx_buffer.buf)\n", __func__);
    free(msg_ls);
    LOG_DBG("Debug in %s: ret = %d\n", __func__, ret);
    return ret;
}

static int peci_init_adapter(struct device *dev, struct peci_ls_data *dev_data, struct peci_adapter *adapter)
{
    int ret;

    adapter = peci_alloc_adapter(dev, sizeof(*dev_data));
    if(!adapter)
        return -ENOMEM;
    LOG_DBG("Debug in %s: adapter->cmd_mask = %d\n", __func__, adapter->cmd_mask);
    LOG_DBG("Debug: dev = %p\n", (void *)dev);
    LOG_DBG("Debug in %s: adapter->dev = %p\n", __func__, (void *)&adapter->dev);
    
    dev_data = adapter->dev.data;
    LOG_DBG("Debug: peci dev = %p\n", (void *)dev);
    LOG_DBG("Debug in %s: data = %p to %p\n", __func__, (void *)dev_data, (void *)dev_data+sizeof(*dev_data));
    LOG_DBG("Debug in %s: adapter->dev.data = %p\n", __func__, (void *)adapter->dev.data);
    LOG_DBG("Debug in %s: dev->data = %p\n", __func__, (void *)dev->data);

    dev_data->adapter = adapter;
    LOG_DBG("Debug: data->adapter = %p\n", (void *)dev_data->adapter);
    LOG_DBG("Debug: adapter = %p\n", (void *)adapter);

    dev_data->dev = dev;
    LOG_DBG("Debug in %s: adapter->dev = %p\n", __func__, (void *)&adapter->dev);
    LOG_DBG("Debug in %s: data->adapter->dev = %p\n", __func__, (void *)&dev_data->adapter->dev);
    LOG_DBG("Debug in %s: adapter->dev->data = %p, dev->data = %p\n", __func__, (void *)adapter->dev.data, (void *)dev->data);

    strncpy(dev_data->adapter->name, dev->name, sizeof(dev_data->adapter->name));
    LOG_DBG("PECI adapter %s initialized\n", dev->name);
    dev_data->adapter->xfer = peci_lib_xfer_base_ls;
    dev_data->adapter->use_dma = false;

    peci_core_init();
    ret = peci_add_adapter(dev_data->adapter);

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

#define LS_PECI_INIT(index)                                                                           \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));                                      \
    LS_PECI_IRQ_HANDLER(index)                                                                        \
                                                                                                      \
    static const struct peci_ls_config peci_ls_cfg_##index = {                                        \
        .reg = (reg_peci_t *)DT_INST_REG_ADDR(index),                                                 \
        .irq_num = DT_INST_IRQN(index),                                                               \
        .irq_config_func = peci_ls_irq_config_func_##index,                                           \
        .clock_source = DT_INST_PROP_OR(index, clock_source, 0),                                      \
        .peci_frequence = DT_INST_PROP_OR(index, peci_frequence, 0),                          \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), ))                 \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), ))                       \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), ))  \
    };                                                                                                \
                                                                                                      \
    static struct peci_ls_data peci_ls_dev_data_##index = {};                                         \
                                                                                                      \
    DEVICE_DT_INST_DEFINE(index,                                                                      \
                          &peci_ls_init,                                                              \
                          NULL,                                                                       \
                          &peci_ls_dev_data_##index,                                                  \
                          &peci_ls_cfg_##index,                                                       \
                          POST_KERNEL,                                                                \
                          CONFIG_PECI_INIT_PRIORITY,                                                  \
                          &peci_ls_driver_api);
DT_INST_FOREACH_STATUS_OKAY(LS_PECI_INIT)
