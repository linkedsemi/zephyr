/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/misc/linkedsemi/spid_linkedsemi.h>
#include <zephyr/drivers/misc/linkedsemi/reg_spid.h>
#include <stdlib.h>
#include <stdio.h>

#define SPID1_BASE  0x4000c000
#define SPID2_BASE  0x4000e000

static void spid_callback(const struct device *dev,
                uint32_t callback_idx, void *user_data,
                void *drv_data)
{
    uint32_t cmd_addr = 0;
    ARG_UNUSED(dev);
    ARG_UNUSED(callback_idx);
    ARG_UNUSED(user_data);
    ARG_UNUSED(drv_data);

    printk("irq\n");
    cmd_addr = sys_read32(SPID2_BASE + SPID_TPM_CMD_ADDR);
    printk("irq cmd_addr=0x%x\n", cmd_addr);
}

void poll_tpm_command(uint32_t *cmd_addr)
{
    uint32_t tpm_status;
    while(true){
        tpm_status = sys_read32(SPID2_BASE + SPID_TPM_STATUS);
        if(tpm_status & BIT(0)){ // cmdaddr_notempty
            *cmd_addr = sys_read32(SPID2_BASE + SPID_TPM_CMD_ADDR);
            printk("Got tpm_status 0x%x, cmd_addr 0x%x\n", tpm_status, *cmd_addr);
            return;
        }
    }
}

int main(void)
{
    const struct device *const spid = DEVICE_DT_GET(DT_NODELABEL(spid));
    uint32_t cmd_addr;
    int ret = spid_linkedsemi_register_callback(spid, 0, spid_callback, NULL);
    if(ret) {
        return -1;
    }

    printf("Hello World! %s\n", CONFIG_BOARD);
    poll_tpm_command(&cmd_addr);

    return 0;
}
