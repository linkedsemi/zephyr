/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <ls_hal_iwdgv2.h>

#define BOOT_WDG_VALUE_BASE_S     (1000000 * 10)
#define BOOT_WDG_VALUE_BASE_MS    ((1000000 * 10) / 1000)
#define APP_CPU_RST_IRQN           82
#define APP_CPU_RST_INTR_MSK       0x60
#define APP_CPU_RST_INTR_CLR       0x64
#define APP_CPU_RST_INTR_STT       0x68
#define APP_CPU_RST_INTR_RAW       0x6c

void app_cpu_rst_isr()
{
//     printf("%s\n", __func__);
//     printf("%s: stt: %#lx\n", __func__, (sys_read32(SEC_SYSC_CPU_SEC_ADDR + APP_CPU_RST_INTR_STT) & BIT(24)) >> 24);
//     printf("%s: raw: %#lx\n", __func__, (sys_read32(SEC_SYSC_CPU_SEC_ADDR + APP_CPU_RST_INTR_RAW) & BIT(24)) >> 24);
    HAL_IWDG_Refresh(APP_IWDG);
    sys_write32(BIT(24), SEC_SYSC_CPU_SEC_ADDR + APP_CPU_RST_INTR_CLR);
}

int main(void)
{
    // io_cfg_output(PA00);
    // printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    sys_write32(BIT(24), SEC_SYSC_CPU_SEC_ADDR + APP_CPU_RST_INTR_CLR);
    sys_write32(BIT(24), SEC_SYSC_CPU_SEC_ADDR + APP_CPU_RST_INTR_MSK);
    IRQ_CONNECT(APP_CPU_RST_IRQN, 0, app_cpu_rst_isr, NULL, 0);
    irq_enable(APP_CPU_RST_IRQN);
    // HAL_IWDG_Init(APP_IWDG, BOOT_WDG_VALUE_BASE_S * 5);
    HAL_IWDG_Init(APP_IWDG, BOOT_WDG_VALUE_BASE_MS * 1);
    // HAL_IWDG_Refresh(APP_IWDG);
    while(1)

    return 0;
}
