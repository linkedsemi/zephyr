/*
 * Copyright (c) 2024 Linkedsemi.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_


#include "soc_common.h"
#define IRQ_NESTED_MAX 10

#define IRQ_TYPE_NONE         0
#define IRQ_TYPE_EDGE_RISING  1
#define IRQ_TYPE_EDGE_FALLING 2
#define IRQ_TYPE_EDGE_BOTH    (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH   4
#define IRQ_TYPE_LEVEL_LOW    8

#define DEV_ERR(dev, fmt, ...) LOG_ERR("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_WRN(dev, fmt, ...) LOG_WRN("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_INF(dev, fmt, ...) LOG_INF("%s: " fmt, (dev)->name, ##__VA_ARGS__)
#define DEV_DBG(dev, fmt, ...) LOG_DBG("%s: " fmt, (dev)->name, ##__VA_ARGS__)

#if defined(CONFIG_LINKEDSEMI_TPM_WWDT)
int wwdt1_tpm_init(const struct device *tpm_spis_dev, uint32_t timeout_ms);
#endif

#if defined(CONFIG_SOC_LSQSH)
struct adc_otp_trim {
	uint32_t : 20, adc12b_os_cal_adc0 : 8, : 4;
	uint32_t : 24, adc12b_os_cal_adc1 : 8;
	uint32_t adc12b_vref_trim_adc0 : 5, : 11,
		 adc12b_vref_trim_adc1 : 5, : 11;
};

#if DT_NODE_EXISTS(DT_NODELABEL(otp_config_adc_trim_memory))
#define LS_SHARED_ADC_TRIM_ADDR \
	((uintptr_t)DT_REG_ADDR(DT_NODELABEL(otp_config_adc_trim_memory)))
#endif
#endif /* CONFIG_SOC_LSQSH */

#endif /* _SOC_H_ */
