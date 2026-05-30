#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_ls, CONFIG_I2C_LOG_LEVEL);

#include "platform.h"
#include "field_manipulate.h"
#include "reg_i2c_type.h"
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
#include "i2c_bitbang.h"
#include "i2c-priv.h"
#include <soc.h>

#define DT_DRV_COMPAT linkedsemi_ls_i2c

#define MASTER_NACK_RECEIVED      BIT(0)
#define I2C_BUS_TIMOUT            BIT(1)
#define BUS_ERROR_DETECTED        BIT(2)
#define ARBITRATION_LOSS_DETECTED BIT(3)
#define OVERRUN_DETECTED          BIT(4)
#define PEC_ERROR_DETECTED        BIT(5)
#define TIMEOUT_DETECTED          BIT(6)
#define I2C_LS_CNT_MAX            255
#define I2C_LS_FIFO_DEEPTH        8
#define I2C_LS_FILTER_NS          50
#define I2C_LS_FILTER_FACTOR_MAX  15
#define I2C_LS_FILTER_FACTOR_MIN  1

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct i2c_ls_config {
    irq_cfg_func_t irq_config_func;
    reg_i2c_t *reg;
    uint32_t clock_frequency;
    uint32_t init_bus_frequency;
    struct gpio_dt_spec scl;
    struct gpio_dt_spec sda;
    bool pinctrl_noinit;
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct i2c_ls_data {
    struct k_sem master_complete_sem;
    struct k_sem bus_mutex;
    uint32_t config;
    struct i2c_target_config *slave_cfg;

    struct i2c_msg *msg;
    struct i2c_msg *msg_curr;
    uint32_t msg_num;
    uint8_t slave_addr;
    uint8_t xfer_len;
    uint8_t xfer_remain;
    uint8_t errs;
    bool stop_pending;
    bool quick_command;

    uint8_t pin[2];
};

struct i2c_speed_config_t {
    uint32_t scll : 8;
    uint32_t sclh : 8;
    uint32_t sdadel : 4;
    uint32_t scldel : 4;
    uint32_t role : 4;
    uint32_t presc : 4;
};

#if defined(CONFIG_I2C_SHOW_STATE)
const uint8_t *i2c_reg_stat_fsm_str[] = {
    [I2C_STAT_FSM_ST_IDLE] = "ST_IDLE",
    [I2C_STAT_FSM_M_WAIT] = "M_WAIT",
    [I2C_STAT_FSM_M_START] = "M_START",
    [I2C_STAT_FSM_M_ADDR1] = "M_ADDR1",
    [I2C_STAT_FSM_M_ADDR2] = "M_ADDR2",
    [I2C_STAT_FSM_M_DATA] = "M_DATA",
    [I2C_STAT_FSM_M_RESTART] = "M_RESTART",
    [I2C_STAT_FSM_M_HOLD] = "M_HOLD",
    [I2C_STAT_FSM_M_PEC] = "M_PEC",
    [I2C_STAT_FSM_M_STOP] = "M_STOP",
    [I2C_STAT_FSM_S_START] = "S_START",
    [I2C_STAT_FSM_S_ADDR1] = "S_ADDR1",
    [I2C_STAT_FSM_S_ADDR2] = "S_ADDR2",
    [I2C_STAT_FSM_S_DATA] = "S_DATA",
    [I2C_STAT_FSM_S_HOLD] = "S_HOLD",
    [I2C_STAT_FSM_S_PEC] = "S_PEC",
};

static void i2c_ls_show_state(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    DEV_INF(dev, "FSM: %s", i2c_reg_stat_fsm_str[REG_FIELD_RD(dev_config->reg->STAT, I2C_STAT_FSM_STAT)]);
    DEV_INF(dev, "SMBA_OE: %x", REG_FIELD_RD(dev_config->reg->STAT, I2C_STAT_SMBA_OE));
    DEV_INF(dev, "SDA_OE: %x", REG_FIELD_RD(dev_config->reg->STAT, I2C_STAT_SDA_OE));
    DEV_INF(dev, "SCL_OE: %x", REG_FIELD_RD(dev_config->reg->STAT, I2C_STAT_SCL_OE));
}
#endif

#if defined(CONFIG_PINCTRL)
static void i2c_ls_bitbang_set_scl(void *io_context, int state)
{
    const struct i2c_ls_config *dev_config = io_context;

    gpio_pin_set_dt(&dev_config->scl, state);
}

static void i2c_ls_bitbang_set_sda(void *io_context, int state)
{
    const struct i2c_ls_config *dev_config = io_context;

    gpio_pin_set_dt(&dev_config->sda, state);
}

static int i2c_ls_bitbang_get_sda(void *io_context)
{
    const struct i2c_ls_config *dev_config = io_context;

    return gpio_pin_get_dt(&dev_config->sda) == 0 ? 0 : 1;
}

enum i2c_bus_status {
    I2C_BUS_IDLE,
    I2C_BUS_BUSY,
    I2C_BUS_NEED_RECOVERY,
};

static enum i2c_bus_status i2c_ls_bus_status(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    uint8_t scl_val;
    uint8_t sda_val;
    enum i2c_bus_status ret = I2C_BUS_IDLE;

    if ((NULL != dev_config->scl.port) && (NULL != dev_config->sda.port)) {
        scl_val = gpio_pin_get_dt(&dev_config->scl);
        sda_val = gpio_pin_get_dt(&dev_config->sda);
        if (!((1 == scl_val) && (1 == sda_val))) {
            DEV_DBG(dev, "bus busy");
            IF_ENABLED(CONFIG_I2C_SHOW_STATE, (i2c_ls_show_state(dev)));
            if ((1 == scl_val) && (0 == sda_val)) {
                ret = I2C_BUS_NEED_RECOVERY;
                goto end;
            } else {
                DEV_DBG(dev, "scl: %d.  sda: %d.", scl_val, sda_val);
                ret = I2C_BUS_BUSY;
                goto end;
            }
        }
    }
end:
    return ret;
}

static int i2c_ls_recover_bus_handle(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_bitbang bitbang_ctx;
    struct i2c_bitbang_io bitbang_io = {
        .set_scl = i2c_ls_bitbang_set_scl,
        .set_sda = i2c_ls_bitbang_set_sda,
        .get_sda = i2c_ls_bitbang_get_sda,
    };
    uint32_t bitrate_cfg;
    int error = 0;

    DEV_ERR(dev, "attempting to recover bus");

    if (!gpio_is_ready_dt(&dev_config->scl)) {
        DEV_ERR(dev, "SCL GPIO device not ready");
        return -EIO;
    }

    if (!gpio_is_ready_dt(&dev_config->sda)) {
        DEV_ERR(dev, "SDA GPIO device not ready");
        return -EIO;
    }

    pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_PRIV_START);

    error = gpio_pin_configure_dt(&dev_config->scl, GPIO_OUTPUT_HIGH | GPIO_PULL_UP | GPIO_LINE_OPEN_DRAIN);
    if (error != 0) {
        DEV_ERR(dev, "failed to configure SCL GPIO (err %d)", error);
        goto restore;
    }

    error = gpio_pin_configure_dt(&dev_config->sda, GPIO_OUTPUT_HIGH | GPIO_PULL_UP | GPIO_LINE_OPEN_DRAIN);
    if (error != 0) {
        DEV_ERR(dev, "failed to configure SDA GPIO (err %d)", error);
        goto restore;
    }

    i2c_bitbang_init(&bitbang_ctx, &bitbang_io, (void *)dev_config);

    bitrate_cfg = i2c_map_dt_bitrate(I2C_BITRATE_STANDARD) | I2C_MODE_CONTROLLER;
    error = i2c_bitbang_configure(&bitbang_ctx, bitrate_cfg);
    if (error != 0) {
        DEV_ERR(dev, "failed to configure I2C bitbang (err %d)", error);
        goto restore;
    }

    error = i2c_bitbang_recover_bus(&bitbang_ctx);
    if (error != 0) {
        DEV_ERR(dev, "failed to recover bus (err %d)", error);
    }

restore:
    (void)pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);

    return error;
}

static int i2c_ls_recover_bus(const struct device *dev)
{
    int ret = 0;

    enum i2c_bus_status i2c_bus_status = i2c_ls_bus_status(dev);
    if (i2c_bus_status != I2C_BUS_IDLE) {
        if (i2c_bus_status == I2C_BUS_NEED_RECOVERY) {
            DEV_WRN(dev, "try recovery");
            ret = i2c_ls_recover_bus_handle(dev);
            if (ret) {
                goto err;
            }
        } else {
            ret = -EIO;
            goto err;
        }
    } else {
        DEV_INF(dev, "bus idle");
    }

err:
    return ret;
}

int i2c_idle_check_prepare(const struct device *dev, const struct pinctrl_dev_config *pcfg, uint8_t pinctrl_state)
{
    struct i2c_ls_data *dev_data = dev->data;
    const struct pinctrl_state *state;
    int ret = -EINVAL;

    ret = pinctrl_lookup_state(pcfg, pinctrl_state, &state);
    if (!ret) {
        dev_data->pin[0] = pinctrl_pin2code(&state->pins[0]);
        dev_data->pin[1] = pinctrl_pin2code(&state->pins[1]);
        if (dev_data->pin[0] == dev_data->pin[1]) {
            DEV_ERR(dev, "scl pin and sda pin can not be duplicated: %#x", dev_data->pin[0]);
            ret = -EINVAL;
        } else {
            ret = 0;
        }
    } else {
        dev_data->pin[0] = 0;
        dev_data->pin[1] = 0;
    }

    return ret;
}
#endif

static void i2c_noise_filter_set(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    uint8_t factor = DIV_ROUND_UP(I2C_LS_FILTER_NS, 1000000000 / dev_config->clock_frequency);
    if ((factor <= I2C_LS_FILTER_FACTOR_MAX) && (factor >= I2C_LS_FILTER_FACTOR_MIN)) {
        REG_FIELD_WR(dev_config->reg->CR1, I2C_CR1_DNF, factor);
        DEV_DBG(dev, "filter factor: %d", factor);
    } else {
        DEV_WRN(dev, "not support noise filter factor: %d", factor);
    }
}

static void i2c_timing_param_set(const struct device *dev, uint32_t i2c_clk)
{
    const struct i2c_ls_config *dev_config = dev->config;
    uint16_t cycle_count = 0;
    uint8_t prescalar = 0;
    int16_t scll = 0;
    int16_t sclh = 0;
    int16_t scldel = 0;
    int16_t sdadel = 0;

    uint8_t __prescalar = 1;
    uint16_t __cycle_count;
    int16_t __scll;

    while (1) {
        __prescalar++;
        __cycle_count = dev_config->clock_frequency / i2c_clk / __prescalar;
        __scll = __cycle_count >> 1;

        if (((__cycle_count > 256) || (__scll > 16)) && (__cycle_count >= 16) && (__prescalar <= 16)) {
            prescalar = __prescalar;
            cycle_count = __cycle_count;
            scll = __scll;
        } else {
            break;
        }
    }

    if (!((cycle_count >= 16) && (prescalar <= 16) && (scll < 48))) {
        DEV_ERR(dev, "Invalid i2c timing");
        return;
    }

    scldel = (scll >> 1) > 16 ? 15 : (scll >> 1);
    sclh = scll;
    sdadel = 2;
    MODIFY_REG(dev_config->reg->TIMINGR,
               (I2C_TIMINGR_PRESC_MASK | I2C_TIMINGR_SCLH_MASK | I2C_TIMINGR_SCLL_MASK | I2C_TIMINGR_SDADEL_MASK | I2C_TIMINGR_SCLDEL_MASK),
               (prescalar - 1) << I2C_TIMINGR_PRESC_POS | sclh << I2C_TIMINGR_SCLH_POS | scll << I2C_TIMINGR_SCLL_POS | sdadel << I2C_TIMINGR_SDADEL_POS | scldel << I2C_TIMINGR_SCLDEL_POS);
    i2c_noise_filter_set(dev);
}

static void i2c_slave_timing_param_set(const struct i2c_ls_config *dev_config)
{
    MODIFY_REG(dev_config->reg->TIMINGR,
               (I2C_TIMINGR_PRESC_MASK | I2C_TIMINGR_SCLH_MASK | I2C_TIMINGR_SCLL_MASK | I2C_TIMINGR_SDADEL_MASK | I2C_TIMINGR_SCLDEL_MASK),
               0 << I2C_TIMINGR_PRESC_POS | 0 << I2C_TIMINGR_SCLH_POS | 3 << I2C_TIMINGR_SCLL_POS | 0 << I2C_TIMINGR_SDADEL_POS | 0 << I2C_TIMINGR_SCLDEL_POS);
}

static inline void slv_single_byte(const struct i2c_ls_config *dev_config)
{
    dev_config->reg->CR2_2 = 1;
    dev_config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
}

static void i2c_reenable(const struct device *dev, bool master)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    dev_config->reg->CR1 &= ~I2C_CR1_PE_MASK;
    if (master) {
        uint32_t i2c_clk = 0;
        switch (I2C_SPEED_GET(dev_data->config)) {
        case I2C_SPEED_STANDARD:
            i2c_clk = 100000;
            break;
        case I2C_SPEED_FAST:
            i2c_clk = 400000;
            break;
        case I2C_SPEED_FAST_PLUS:
            i2c_clk = 1000000;
            break;
        default:
            DEV_ERR(dev, "i2c speed not supported");
            break;
        }
        i2c_timing_param_set(dev, i2c_clk);
    } else {
        i2c_slave_timing_param_set(dev_config);
    }
    dev_config->reg->CFR = 0xffff;
    dev_config->reg->ICR = 0xffff;         // clear pending irq
    dev_config->reg->SR = I2C_SR_TXE_MASK; //clear tx fifo
    dev_config->reg->CR1 |= I2C_CR1_SBC_MASK | I2C_CR1_PE_MASK;
    slv_single_byte(dev_config);
}

static void i2c_slave_addr_reenable(reg_i2c_t *reg)
{
    uint32_t oar1 = reg->OAR1;
    uint32_t oar2 = reg->OAR2;
    reg->OAR1 = oar1 & ~I2C_OAR1_OA1EN_MASK;
    reg->OAR2 = oar2 & ~I2C_OAR2_OA2EN_MASK;
    reg->OAR1 = oar1;
    reg->OAR2 = oar2;
}

static void i2c_ls_write_fifo(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;

    uint8_t txflv = REG_FIELD_RD(dev_config->reg->SR, I2C_SR_TXFLV);
    while (dev_data->xfer_remain && (txflv < I2C_LS_FIFO_DEEPTH)) {
        dev_config->reg->TXDR = *dev_data->msg_curr->buf++;
        dev_data->xfer_remain--;
        txflv++;
    }
}

static int i2c_ls_start_or_reload(const struct device *dev, struct i2c_msg *msg, uint16_t slave_addr)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    uint32_t cr2_0_1 = msg->flags & I2C_MSG_ADDR_10_BITS ? I2C_CR2_SADD10_MASK | slave_addr << I2C_CR2_SADD0_POS : slave_addr << I2C_CR2_SADD1_7_POS;
    bool read = (dev_data->msg_curr->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
    bool pre_send_time = (dev_data->xfer_len == 0);
    bool start = ((dev_data->msg_curr == msg) || (dev_data->msg_curr->flags & I2C_MSG_RESTART)) && pre_send_time;

    /* set CR2_3 reload bit before set CR2_2 is essential */
    if (start) {
        dev_config->reg->CR2_3 &= ~I2C_CR2_RELOAD_MASK;
    } else {
        dev_config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
    }
    dev_data->xfer_len = dev_data->msg_curr->len > I2C_LS_CNT_MAX ? I2C_LS_CNT_MAX : dev_data->msg_curr->len;
    dev_data->xfer_remain = dev_data->xfer_len;
    dev_config->reg->CR2_2 = dev_data->xfer_len;
    if (dev_data->msg_curr->len <= I2C_LS_CNT_MAX) {
        dev_config->reg->CR2_3 &= ~I2C_CR2_RELOAD_MASK;
    } else {
        dev_config->reg->CR2_3 |= I2C_CR2_RELOAD_MASK;
    }
    if (start) {
        if (read) {
            cr2_0_1 |= I2C_CR2_RD_WEN_MASK;
        } else {
            cr2_0_1 &= ~I2C_CR2_RD_WEN_MASK;
        }
        dev_config->reg->CR2_0_1 = cr2_0_1 | I2C_CR2_START_MASK;
    }
    dev_data->msg_curr->len -= dev_data->xfer_len;

    if (read) {
        dev_config->reg->IER = I2C_INT_RXNE_MASK;
    } else {
        i2c_ls_write_fifo(dev);
        if (dev_data->xfer_remain) {
            dev_config->reg->IER = I2C_INT_TXE_MASK;
        } else {
            dev_config->reg->IER = I2C_INT_TCR_MASK | I2C_INT_TC_MASK;
        }
    }

    return 0;
}

static void i2c_ls_isr_error_handle(const struct device *dev, uint32_t irq)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    bool master_mode = (dev_data->msg_curr != NULL);

    if (irq & I2C_INT_BERR_MASK) {
        dev_config->reg->ICR = I2C_INT_BERR_MASK;
        dev_data->errs |= BUS_ERROR_DETECTED;
        DEV_ERR(dev, "i2c@%08x bus err", (uint32_t)dev_config->reg);
    }
    if (irq & I2C_INT_ARLO_MASK) {
        dev_config->reg->ICR = I2C_INT_ARLO_MASK;
        dev_data->errs |= ARBITRATION_LOSS_DETECTED;
        DEV_ERR(dev, "i2c@%08x arb loss", (uint32_t)dev_config->reg);
    }
    if (irq & I2C_INT_OVR_MASK) {
        dev_config->reg->ICR = I2C_INT_OVR_MASK;
        dev_data->errs |= OVERRUN_DETECTED;
        DEV_ERR(dev, "i2c@%08x overrun err", (uint32_t)dev_config->reg);
    }
    if (irq & I2C_INT_PECE_MASK) {
        dev_config->reg->ICR = I2C_INT_PECE_MASK;
        dev_data->errs |= PEC_ERROR_DETECTED;
        DEV_ERR(dev, "i2c@%08x pec err", (uint32_t)dev_config->reg);
    }
    if (irq & I2C_INT_TOUT_MASK) {
        dev_config->reg->ICR = I2C_INT_TOUT_MASK;
        dev_data->errs |= TIMEOUT_DETECTED;
        DEV_ERR(dev, "i2c@%08x timeout err", (uint32_t)dev_config->reg);
    }

    if (dev_data->errs && ((dev_data->errs & MASTER_NACK_RECEIVED) != MASTER_NACK_RECEIVED) && master_mode) {
        k_sem_give(&dev_data->master_complete_sem);
    }
}

static void i2c_ls_isr_normal_handle(const struct device *dev, uint32_t irq)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    bool master_mode = (dev_data->msg_curr != NULL);

    /* * * * * * master/slave state begin * * * * * */
    if (irq & I2C_INT_TXE_MASK) {
        dev_config->reg->ICR = I2C_INT_TXE_MASK;
        if (master_mode) { /* --> <self>/<tc>/<tcr> */
            i2c_ls_write_fifo(dev);
            if (!dev_data->xfer_remain) {
                dev_config->reg->IDR = I2C_INT_TXE_MASK;
                dev_config->reg->IER = I2C_INT_TCR_MASK | I2C_INT_TC_MASK;
            }
        } else { /* --> <self> */
            uint8_t val;
            int ret = dev_data->slave_cfg->callbacks->read_processed(dev_data->slave_cfg, &val);
            if (!ret) {
                k_busy_wait(10);
                dev_config->reg->TXDR = val;
            }
        }
    } else if (irq & I2C_INT_RXNE_MASK) {
        if (master_mode) { /* --> <self>/<tc>/<tcr> */
            do {
                *dev_data->msg_curr->buf++ = dev_config->reg->RXDR;
                if (--dev_data->xfer_remain == 0) {
                    dev_config->reg->IDR = I2C_INT_RXNE_MASK;
                    dev_config->reg->IER = I2C_INT_TCR_MASK | I2C_INT_TC_MASK;
                    break;
                }
            } while (dev_config->reg->SR & I2C_SR_RXNE_MASK);
        } else { /* --> <self> */
            /* workaround begin: slave mode cannot trigger address match interrupt under restart condtition */
            i2c_slave_addr_reenable(dev_config->reg);
            /* workaround end */
            do {
                int ret = dev_data->slave_cfg->callbacks->write_received(dev_data->slave_cfg, dev_config->reg->RXDR);
                if (ret) {
                    k_busy_wait(10);
                    dev_config->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
                }
                dev_config->reg->CR2_2 = 1;
            } while (dev_config->reg->SR & I2C_SR_RXNE_MASK);
        }
        dev_config->reg->ICR = I2C_INT_RXNE_MASK;
    }
    if (irq & I2C_INT_NACK_MASK) { /* --> <stop> */
        dev_config->reg->ICR = I2C_INT_NACK_MASK;
        if (master_mode) {
            dev_data->errs |= MASTER_NACK_RECEIVED;
#if 0
            if(dev_data->xfer_remain) {
                k_sem_give(&dev_data->device_sync_sem);
            }
#endif
            dev_data->stop_pending = true;
            dev_config->reg->CR2_0_1 |= I2C_CR2_STOP_MASK;
        }
    }
    if (irq & I2C_INT_STOP_MASK) { /* --> |end| */
        dev_config->reg->ICR = I2C_INT_STOP_MASK;
        dev_config->reg->IDR = I2C_INT_TXE_MASK | I2C_INT_RXNE_MASK | I2C_INT_STOP_MASK;
        if (master_mode) {
            dev_data->stop_pending = false;
            k_sem_give(&dev_data->master_complete_sem);
        } else if (dev_data->slave_cfg) {
            /* workaound: clear i2c slave internal counter after stop */
        //     dev_config->reg->CR1 &= ~I2C_CR1_PE_MASK;
        //     dev_config->reg->CR1 |= I2C_CR1_PE_MASK;
            /* --------------------------------------------- */

            dev_config->reg->SR = 1;
            while (dev_config->reg->SR & I2C_SR_RXNE_MASK) {
                dev_config->reg->RXDR;
            }
            dev_data->slave_cfg->callbacks->stop(dev_data->slave_cfg);
            k_sem_give(&dev_data->bus_mutex);
        }
    }
    /* * * * * * master/slave state end * * * * * */

    /* * * * * * master state begin * * * * * */
    if (irq & I2C_INT_TC_MASK) { /* --> <restart> or <stop> */
        dev_config->reg->ICR = I2C_INT_TC_MASK;
        dev_config->reg->IDR = I2C_INT_TCR_MASK | I2C_INT_TC_MASK;

        /*  dev_data->msg_curr->len == 0 */
        if (((dev_data->msg_curr + 1) == &dev_data->msg[dev_data->msg_num])) { /* --> <stop> */
            dev_data->stop_pending = true;
            dev_config->reg->CR2_0_1 |= I2C_CR2_STOP_MASK;

        } else { /* --> <restart> */
            dev_data->msg_curr++;
            dev_data->xfer_len = 0;
            i2c_ls_start_or_reload(dev, dev_data->msg, dev_data->slave_addr);
        }
    }
    if (irq & I2C_INT_TCR_MASK) { /* --> <tx>/<rx> */
        dev_config->reg->ICR = I2C_INT_TCR_MASK;
        dev_config->reg->IDR = I2C_INT_TCR_MASK | I2C_INT_TC_MASK;

        if (dev_data->msg_curr->len == 0) {
            dev_data->msg_curr++;
            dev_data->xfer_len = 0;
        }
        // restart
        i2c_ls_start_or_reload(dev, dev_data->msg, dev_data->slave_addr);
    }
    /* * * * * * master state end * * * * * */

    /* * * * * * slave state begin * * * * * */
    if (irq & I2C_INT_ADDR_MASK) { /* --> <tx>/<rx> */
        /* Since the hardware IP triggers a stop interrupt during address match interrupt,
            the I2C_INT_STOP_MASK is cleared here. */
        dev_config->reg->ICR = I2C_INT_ADDR_MASK | I2C_INT_STOP_MASK;
        if (!master_mode) {
            k_sem_take(&dev_data->bus_mutex, K_NO_WAIT);
            uint32_t status = dev_config->reg->SR;
            if (status & I2C_SR_DIR_MASK) { /* read */
                uint8_t val;
                int ret = dev_data->slave_cfg->callbacks->read_requested(dev_data->slave_cfg, &val);
                if (!ret) {
                    dev_config->reg->TXDR = val;
                    dev_config->reg->IER = I2C_INT_TXE_MASK | I2C_INT_STOP_MASK;
                }
            } else { /* write */
                int ret = dev_data->slave_cfg->callbacks->write_requested(dev_data->slave_cfg);
                if (!ret) {
                    dev_config->reg->IER = I2C_INT_RXNE_MASK | I2C_INT_STOP_MASK;
                } else {
                    dev_config->reg->CR2_0_1 |= I2C_CR2_NACK_MASK;
                }
            }
        }
    }
    /* * * * * * slave_addr state end * * * * * */

    if (irq & I2C_INT_ALERT_MASK) {
        DEV_INF(dev, "i2c@%08x smbus alert", (uint32_t)dev_config->reg);
    }
}

static void i2c_ls_isr(void *arg)
{
    struct device *dev = (struct device *)arg;
    const struct i2c_ls_config *dev_config = dev->config;

    while (1) {
        uint32_t irq = dev_config->reg->IFM;
        if (0 == irq) {
            break;
        }

        i2c_ls_isr_error_handle(dev, irq);
        i2c_ls_isr_normal_handle(dev, irq);
    }
}

static int i2c_ls_transfer(const struct device *dev, struct i2c_msg *msg, uint8_t num_msgs, uint16_t slave_addr)
{
    struct i2c_ls_data *dev_data = dev->data;
    const struct i2c_ls_config *dev_config = dev->config;
    int ret = 0;

    k_sem_take(&dev_data->bus_mutex, K_FOREVER);
#if defined(CONFIG_PINCTRL)
    enum i2c_bus_status i2c_bus_status = i2c_ls_bus_status(dev);
    if (i2c_bus_status != I2C_BUS_IDLE) {
        if (i2c_bus_status == I2C_BUS_NEED_RECOVERY) {
            DEV_WRN(dev, "try recovery");
            ret = i2c_ls_recover_bus_handle(dev);
            if (ret) {
                goto err;
            }
        } else {
            ret = -EIO;
            goto err;
        }
    }
#endif
    dev_data->errs = 0;
    if (k_sem_count_get(&dev_data->master_complete_sem) != 0) {
        DEV_WRN(dev, "master_complete_sem count: %d", k_sem_count_get(&dev_data->master_complete_sem));
        k_sem_reset(&dev_data->master_complete_sem);
    }
    i2c_reenable(dev, true);
    dev_config->reg->SR = I2C_SR_TXE_MASK; //clear tx fifo
    dev_config->reg->ICR = I2C_INT_STOP_MASK;
    dev_config->reg->IER = I2C_INT_STOP_MASK;
    uint32_t cr2_0_1 = msg->flags & I2C_MSG_ADDR_10_BITS ? I2C_CR2_SADD10_MASK | slave_addr << I2C_CR2_SADD0_POS : slave_addr << I2C_CR2_SADD1_7_POS;

    dev_data->msg_curr = msg;
    dev_data->msg = msg;
    dev_data->msg_num = num_msgs;
    dev_data->slave_addr = slave_addr;
    dev_data->xfer_len = 0;
    bool read = (dev_data->msg_curr->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;

    dev_data->quick_command = (dev_data->msg_curr->len == 0);
    if (dev_data->quick_command) {
        if (read) {
            dev_config->reg->CR2_3 |= 0x30;
        } else {
            dev_config->reg->CR2_3 &= ~0x30;
        }
        dev_config->reg->CR2_0_1 = cr2_0_1 | I2C_CR2_START_MASK | I2C_CR2_STOP_MASK;
    } else {
        i2c_ls_start_or_reload(dev, dev_data->msg, dev_data->slave_addr);
    }
    if (k_sem_take(&dev_data->master_complete_sem, K_MSEC(CONFIG_LS_I2C_BUS_TIMEOUT_MS)) == (-EAGAIN)) {
        dev_data->errs |= I2C_BUS_TIMOUT;
        DEV_ERR(dev, "i2c bus timeout");
        goto err;
    }
err:
    if (dev_data->errs) {
        ret = -EIO;
        if ((dev_data->errs & MASTER_NACK_RECEIVED) == MASTER_NACK_RECEIVED) {
            DEV_DBG(dev, "err: %#x", dev_data->errs);
        } else {
            DEV_ERR(dev, "err: %#x", dev_data->errs);
        }
    }
    if (dev_data->quick_command) {
        dev_config->reg->CR2_3 &= ~0x30;
    }
    dev_config->reg->IDR = I2C_INT_STOP_MASK | I2C_INT_TCR_MASK | I2C_INT_TC_MASK;
    i2c_reenable(dev, false);
    dev_data->msg_curr = NULL;
    k_sem_give(&dev_data->bus_mutex);

    return ret;
}

static int i2c_runtime_configure(const struct device *dev, uint32_t config)
{
    struct i2c_ls_data *dev_data = dev->data;
    dev_data->config = config;
    return 0;
}

int i2c_ls_pinctrl(const struct device *dev, uint32_t pinctrl_state)
{
    const struct i2c_ls_config *dev_config = dev->config;
    int ret = 0;

    if (NULL == dev) {
        return -EINVAL;
    }

    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(dev_config->pcfg, pinctrl_state);
    if (ret < 0) {
        DEV_DBG(dev, "Could not configure pins");
    }
    i2c_idle_check_prepare(dev, dev_config->pcfg, pinctrl_state);

    return ret;
}

static int i2c_ls_init(const struct device *dev)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    __maybe_unused int ret;

    k_sem_init(&dev_data->master_complete_sem, 0, K_SEM_MAX_LIMIT);
    k_sem_init(&dev_data->bus_mutex, 1, 1);

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            DEV_DBG(dev, "%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            DEV_ERR(dev, "Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            DEV_ERR(dev, "toggle reset line failed");
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
    if (!dev_config->pinctrl_noinit) {
        i2c_ls_pinctrl(dev, PINCTRL_STATE_DEFAULT);
    }
#endif

    i2c_reenable(dev, false);
    dev_config->reg->CR2_3 |= 1 << 3; // slv nbytes upd hw workaround
    dev_config->reg->ICR = 0xffff;
    dev_config->reg->IER = I2C_INT_NACK_MASK | I2C_INT_BERR_MASK
        | I2C_INT_ARLO_MASK | I2C_INT_OVR_MASK | I2C_INT_PECE_MASK
        | I2C_INT_TOUT_MASK | I2C_INT_ALERT_MASK;

    if (i2c_configure(dev, i2c_map_dt_bitrate(dev_config->init_bus_frequency) | I2C_MODE_CONTROLLER)) {
        DEV_ERR(dev, "%s: config failed", dev->name);
    }

    dev_config->irq_config_func(dev);

    return 0;
}

static int i2c_ls_get_config(const struct device *dev, uint32_t *dev_config)
{
    struct i2c_ls_data *dev_data = dev->data;
    *dev_config = dev_data->config;
    return 0;
}

static int i2c_ls_target_register(const struct device *dev, struct i2c_target_config *target_cfg)
{
    const struct i2c_ls_config *dev_config = dev->config;
    struct i2c_ls_data *dev_data = dev->data;
    dev_data->slave_cfg = target_cfg;
    if (target_cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
        dev_config->reg->OAR1 = I2C_OAR1_OA1EN_MASK | I2C_OAR1_OA1MODE_MASK | target_cfg->address << I2C_OAR1_OA10_POS;
    } else {
        dev_config->reg->OAR1 = I2C_OAR1_OA1EN_MASK | target_cfg->address << I2C_OAR1_OA11_7_POS;
    }
    dev_config->reg->IER = I2C_INT_ADDR_MASK;
    return 0;
}

static int i2c_ls_target_unregister(const struct device *dev, struct i2c_target_config *target_cfg)
{
    struct i2c_ls_data *dev_data = dev->data;
    const struct i2c_ls_config *dev_config = dev->config;
    dev_config->reg->IDR = I2C_INT_ADDR_MASK;
    dev_config->reg->OAR1 = 0;
    dev_data->slave_cfg = NULL;
    return 0;
}

static const struct i2c_driver_api api_funcs = {
    .configure = i2c_runtime_configure,
    .get_config = i2c_ls_get_config,
    .transfer = i2c_ls_transfer,
    .target_register = i2c_ls_target_register,
    .target_unregister = i2c_ls_target_unregister,
#if defined(CONFIG_PINCTRL)
    .recover_bus = i2c_ls_recover_bus,
#endif
};

#define LS_I2C_IRQ_HANDLER(index)                                        \
    static void i2c_ls_irq_config_func_##index(const struct device *dev) \
    {                                                                    \
        IRQ_CONNECT(DT_INST_IRQN(index),                                 \
                    DT_INST_IRQ(index, priority),                        \
                    i2c_ls_isr,                                          \
                    DEVICE_DT_INST_GET(index),                           \
                    0);                                                  \
        irq_enable(DT_INST_IRQN(index));                                 \
    }

#define LS_I2C_INIT(index)                                                                                   \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));                                             \
    LS_I2C_IRQ_HANDLER(index)                                                                                \
    static const struct i2c_ls_config i2c_ls_cfg_##index = {                                                 \
        .reg = (reg_i2c_t *)DT_INST_REG_ADDR(index),                                                         \
        .irq_config_func = i2c_ls_irq_config_func_##index,                                                   \
        .clock_frequency = COND_CODE_1(                                                                      \
            DT_NODE_HAS_PROP(DT_INST_PHANDLE(index, clocks), clock_frequency),                               \
            (DT_INST_PROP_BY_PHANDLE(index, clocks, clock_frequency)),                                       \
            (DT_INST_PROP(index, clock_frequency))),                                                         \
        .init_bus_frequency = DT_INST_PROP_OR(index, bus_frequency, I2C_BITRATE_STANDARD),                   \
        .scl = GPIO_DT_SPEC_INST_GET_OR(index, scl_gpios, { 0 }),                                            \
        .sda = GPIO_DT_SPEC_INST_GET_OR(index, sda_gpios, { 0 }),                                            \
        .pinctrl_noinit = DT_INST_PROP_OR(index, pinctrl_noinit, 0),                                         \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), ))                        \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.ccfg = LS_DT_CLK_CFG_ITEM(index), ))                              \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(index, resets), (.reset = RESET_DT_SPEC_INST_GET(index), ))         \
    };                                                                                                       \
    static struct i2c_ls_data i2c_ls_dev_data_##index;                                                       \
    I2C_DEVICE_DT_INST_DEFINE(index,                                                                         \
                              i2c_ls_init,                                                                   \
                              NULL,                                                                          \
                              &i2c_ls_dev_data_##index,                                                      \
                              &i2c_ls_cfg_##index,                                                           \
                              POST_KERNEL,                                                                   \
                              CONFIG_I2C_INIT_PRIORITY,                                                      \
                              &api_funcs);

DT_INST_FOREACH_STATUS_OKAY(LS_I2C_INIT)
