/*
 * Copyright (c) 2021 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/misc/linkedsemi/spid_linkedsemi.h>
#include <zephyr/drivers/misc/linkedsemi/reg_spid.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_SPID_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif

LOG_MODULE_REGISTER(spid_linkedsem);

#define DT_DRV_COMPAT linkedsemi_spid

struct spid_linkedsemi_data {
    spid_callback_t cb;
    void *user_data;
    const struct device *dev;
    uint32_t data;
};

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct spid_linkedsemi_config {
    mem_addr_t reg;
    uint32_t data;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    irq_cfg_func_t irq_config_func;
};

static void linkedsemi_spid_isr(const struct device *dev)
{
    struct spid_linkedsemi_data *data = dev->data;
    struct spid_linkedsemi_config *cfg = (struct spid_linkedsemi_config *)dev->config;

    uint32_t stat = sys_read32(cfg->reg + INTR_STATE);
    printk("%s stat: 0x%x\n", __func__, stat);
    if (stat) {
        data->cb(dev, 0, data->user_data, NULL);
    }

    sys_write32(0xff, cfg->reg + INTR_STATE); 
}

int spid_linkedsemi_register_callback(const struct device *dev,
                                      uint32_t callback_idx,
                                      spid_callback_t cb,
                                      void *user_data)
{
    struct spid_linkedsemi_data *data = dev->data;

    data->cb = cb;
    data->user_data = user_data;

    return 0;
}

#define SPID_MODE_REG   0x400120fc
#define ENABLE_SPIS_REG 0x400130a4

enum {
    INTF_FIFO_MODE,
    INTF_CRB_MODE,
};

void init_spid_registers(const struct device *dev, int mode)
{
    struct spid_linkedsemi_config *cfg = (struct spid_linkedsemi_config *)dev->config;

    // invalid_locality=1
    if(mode == INTF_FIFO_MODE){
        sys_write32(0x11, cfg->reg + TPM_CFG); 
    }else if(mode == INTF_CRB_MODE){
        sys_write32(0x13, cfg->reg + TPM_CFG); 
    }else{
        printk("***Fatal error, invalid interface mode %d\n", mode);
    }
    sys_write32(0x22aaaa, SPID_MODE_REG);  // both spid1 & spid2 enable TPM spis
    sys_write32(0x0000f000, ENABLE_SPIS_REG); // enable spis2 io(MISO)
}

static int spid_linkedsemi_init(const struct device *dev)
{
    const struct spid_linkedsemi_config *cfg = dev->config;
#if defined(CONFIG_PINCTRL)
    int ret;
    /* Configure dt provided device signals when available */
    ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        return ret;
    }
#endif

    cfg->irq_config_func(dev);

    init_spid_registers(dev, INTF_CRB_MODE);
    sys_write32(0xff, cfg->reg + INTR_ENABLE);
#if 0
    k_msleep(1);
    sys_write32(0x1, cfg->reg + INTR_TEST);
#endif

    return 0;
}

#define SPID_LINKEDSEMI_IRQ_HANDLER(index)                                        \
    static void spid_linkedsemi_irq_config_func_##index(const struct device *dev) \
    {                                                                             \
        IRQ_CONNECT(DT_INST_IRQN(index),                                          \
                    DT_INST_IRQ(index, priority),                                 \
                    linkedsemi_spid_isr,                                          \
                    DEVICE_DT_INST_GET(index),                                    \
                    0);                                                           \
        irq_enable(DT_INST_IRQN(index));                                          \
    }

#define SPID_LINKEDSEMI_INIT(index)                                                     \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(index)));                        \
    SPID_LINKEDSEMI_IRQ_HANDLER(index)                                                  \
    static const struct spid_linkedsemi_config spid_linkedsemi_cfg_##index = {          \
        .reg = (mem_addr_t)DT_INST_REG_ADDR(index),                                     \
        .irq_config_func = spid_linkedsemi_irq_config_func_##index,                     \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index), ))   \
            IF_ENABLED(DT_HAS_CLOCKS(index), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(index), )) \
    };                                                                                  \
    static struct spid_linkedsemi_data spid_linkedsemi_dev_data_##index;                \
    DEVICE_DT_INST_DEFINE(index,                                                        \
                          spid_linkedsemi_init,                                         \
                          NULL,                                                         \
                          &spid_linkedsemi_dev_data_##index,                            \
                          &spid_linkedsemi_cfg_##index,                                 \
                          POST_KERNEL,                                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                           \
                          NULL);
DT_INST_FOREACH_STATUS_OKAY(SPID_LINKEDSEMI_INIT)
