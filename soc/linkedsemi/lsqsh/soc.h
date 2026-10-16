/*
 * Copyright (c) 2024 Linkedsemi.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_H_
#define _SOC_H_


#include "soc_common.h"

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

BUILD_ASSERT(sizeof(struct adc_otp_trim) == 12);


#define SHMEM_BASE_ADDR ((uintptr_t)DT_REG_ADDR(DT_NODELABEL(share_memory)))
#define SHMEM_SIZE (DT_REG_SIZE(DT_NODELABEL(share_memory)))
#define FLASH_LS_SHARED_DATA_SIZE (sizeof(struct flash_ls_shared_data))
#define LS_SHARED_ADC_TRIM_SIZE (sizeof(struct adc_otp_trim))
#define FLASH_LS_SHARED_DATA_0 (SHMEM_BASE_ADDR)
#define FLASH_LS_SHARED_DATA_1 (FLASH_LS_SHARED_DATA_0 + FLASH_LS_SHARED_DATA_SIZE)
#define LS_SHARED_ADC_TRIM_ADDR (FLASH_LS_SHARED_DATA_1 + FLASH_LS_SHARED_DATA_SIZE)

#define RESET_REASON_APP_ADDR (SHMEM_BASE_ADDR + SHMEM_SIZE - 4)
#endif /* CONFIG_SOC_LSQSH */

#endif /* _SOC_H_ */
