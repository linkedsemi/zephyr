/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/ipmi.h>
#include <stdio.h>
#include "ipmi.h"
#include "kcs.h"

// #define LOG_LEVEL LOG_LEVEL_DBG
#define LOG_LEVEL LOG_LEVEL_INF
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define KCS_POLL_STACK_SIZE 4096
#define KCS_POLLING_INTERVAL 100
#define KCS_BUFF_SIZE 256
#define KCS_MAX_CHANNEL_NUM 0x0F

uint8_t ibuf[KCS_BUFF_SIZE] = {0};
const uint8_t zero[KCS_BUFF_SIZE] = {0};
uint64_t cnt = 0;

static bool proc_kcs_ok = false;

bool get_kcs_ok()
{
	return proc_kcs_ok;
}

void reset_kcs_ok()
{
	proc_kcs_ok = false;
}

// bool pal_request_msg_to_BIC_from_HOST(uint8_t netfn, uint8_t cmd)
// {
// 	if (netfn == NETFN_APP_REQ) {
// 		if (cmd == CMD_APP_GET_SYSTEM_GUID) {
// 			return true;
// 		}
// 	}

// 	return false;
// }


int main(void)
{

    printf("version: %8.8x\n", *(volatile uint32_t *)0x4001f3fc);
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    printf("ipmi_read...\n");

    ipmi_init();
    do {
        const struct device *const ipmi_dev = DEVICE_DT_GET(DT_NODELABEL(ipmi_kcs));
        printf("ipmi_dev: %8.8x\n", (uint32_t)ipmi_dev);
        int rc = 0;

        ipmi_msg_cfg current_msg;
        while (1) {
            k_msleep(KCS_POLLING_INTERVAL);
            cnt++;
            if (cnt % 100 == 0) {
                cnt = 0;
                printf("ipmi_read...\n");
            }

            rc = ipmi_read(ipmi_dev, ibuf, sizeof(ibuf));
            if (rc < 0) {
                if (rc != -ENODATA)
                    LOG_ERR("Failed to read KCS data, rc = %d", rc);
            }
            if (memcmp(ibuf, zero, KCS_BUFF_SIZE) != 0) {
                uint32_t j = KCS_BUFF_SIZE - 1;
                for(; j >= 0; j--) {
                    if (ibuf[j] != 0) {
                        break;
                    }
                }
                j++;
                for(uint32_t i = 0; i <= j; i++) {
                    printf("%2.2x ", ibuf[i]);
                }
                printf("\n");
            } else {
                continue;
            }

            LOG_HEXDUMP_DBG(&ibuf[0], rc, "host KCS read dump data:");

            proc_kcs_ok = true;
            struct kcs_request *req;
            req = (struct kcs_request *)ibuf;
            req->netfn = req->netfn >> 2;

            if (pal_request_msg_to_BIC_from_HOST(
                    req->netfn, req->cmd)) { // In-band update command, not bridging to bmc
                current_msg.buffer.InF_source = HOST_KCS_1;
                current_msg.buffer.netfn = req->netfn;
                current_msg.buffer.cmd = req->cmd;
                current_msg.buffer.data_len = rc - 2; // exclude netfn, cmd
                if (current_msg.buffer.data_len != 0) {
                    memcpy(current_msg.buffer.data, req->data,
                        current_msg.buffer.data_len);
                }

                LOG_DBG("KCS to ipmi netfn 0x%x, cmd 0x%x, length %d",
                    current_msg.buffer.netfn, current_msg.buffer.cmd,
                    current_msg.buffer.data_len);
                notify_ipmi_client(&current_msg);
            }
        }
    } while (0);

    return 0;
}
