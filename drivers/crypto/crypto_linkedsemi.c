/*
 * Copyright (c) 2021 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#define LOG_LEVEL CONFIG_CRYPTO_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <string.h>
#include <ls_hal_crypt.h>
#include <ls_hal_sha.h>
#include <ls_hal_sm4.h>
#include <ls_hal_otbn.h>
#include <ls_msp_otbn.h>

LOG_MODULE_REGISTER(crypto_linkedsem);

#define DT_DRV_COMPAT linkedsemi_crypto

struct crypto_linkedsemi_data {
    void *user_data;
    const struct device *dev;
    uint32_t data;
};

typedef void (*irq_cfg_func_t)(const struct device *dev);

struct crypto_linkedsemi_config {
    uint32_t data;
#if defined(CONFIG_PINCTRL)
    const struct pinctrl_dev_config *pcfg;
#endif
    irq_cfg_func_t irq_config_func;
};

static void linkedsemi_crypto_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_LSCRYPT_IRQHandler();
}

static void linkedsemi_sha_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    LSSHA_IRQHandler();
}

static void linkedsemi_sm4_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_SM4_IRQHandler();
}

static void linkedsemi_otbn_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_OTBN_IRQHandler();
}

static void linkedsemi_sysc_otbn_isr(const struct device *dev)
{
    ARG_UNUSED(dev);
    HAL_OTBN_SYSC_IRQHandler();
}

static int crypto_linkedsemi_init(const struct device *dev)
{
    const struct crypto_linkedsemi_config *cfg = dev->config;

    cfg->irq_config_func(dev);

    return 0;
}

#define CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, irq_name)              \
    do {                                                            \
        IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, irq_name, irq),      \
                    DT_INST_IRQ_BY_NAME(index, irq_name, priority), \
                    linkedsemi_##irq_name##_isr,                    \
                    DEVICE_DT_INST_GET(index),                      \
                    0);                                             \
        irq_enable(DT_INST_IRQ_BY_NAME(index, irq_name, irq));      \
    } while (false)

#define CRYPTO_LINKEDSEMI_IRQ_HANDLER(index)                                        \
    static void crypto_linkedsemi_irq_config_func_##index(const struct device *dev) \
    {                                                                               \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, crypto);                               \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sha);                                  \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sm4);                                  \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, otbn);                                 \
        CRYPTO_LINKEDSEMI_IRQ_CONNECT(index, sysc_otbn);                            \
    }

#define CRYPTO_LINKEDSEMI_INIT(index)                                               \
    CRYPTO_LINKEDSEMI_IRQ_HANDLER(index)                                            \
    static const struct crypto_linkedsemi_config crypto_linkedsemi_cfg_##index = {  \
        .irq_config_func = crypto_linkedsemi_irq_config_func_##index,               \
        IF_ENABLED(DT_HAS_CLOCKS(index), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(index), )) \
    };                                                                              \
    static struct crypto_linkedsemi_data crypto_linkedsemi_dev_data_##index;        \
    DEVICE_DT_INST_DEFINE(index,                                                    \
                          crypto_linkedsemi_init,                                   \
                          NULL,                                                     \
                          &crypto_linkedsemi_dev_data_##index,                      \
                          &crypto_linkedsemi_cfg_##index,                           \
                          POST_KERNEL,                                              \
                          CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                       \
                          NULL);
DT_INST_FOREACH_STATUS_OKAY(CRYPTO_LINKEDSEMI_INIT)
