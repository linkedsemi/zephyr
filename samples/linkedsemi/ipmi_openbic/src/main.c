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

BUILD_ASSERT(DT_NODE_EXISTS(DT_NODELABEL(ipmi_kcs)), "check configuration");
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(ipmi_kcs), okay), "check configuration");

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

#if defined(CONFIG_INTEL_AVENUE_CITY_CRB)
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "ls_soc_gpio.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_NODELABEL(bmc_boot_done_btn)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// GPIO19
#define ONCTL PD03
// GPIO69
#define SLPS3 PM06

int bhs_bmc_ready(void)
{
    int ret;

    printf("Hello World! %s\n", CONFIG_BOARD);

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    ret = gpio_pin_set_dt(&led, 1);
    if (ret < 0) {
        return 0;
    }

    io_cfg_input(ONCTL);
    io_cfg_input(SLPS3);

    while(io_read_pin(SLPS3) == 1);
    printf("%s: %d\n", __func__, __LINE__);

    io_cfg_output(ONCTL);
    io_write_pin(ONCTL, 0);

    return 0;
}
#endif

int main(void)
{
    printf("FPGA version: %8.8x\n", *(volatile uint32_t *)0x4001f3fc);
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

#if defined(CONFIG_INTEL_AVENUE_CITY_CRB)
    bhs_bmc_ready();
#endif

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
                if (rc != -ENODATA) {
                    LOG_ERR("Failed to read KCS data, rc = %d", rc);
                }
                continue;
            }
#if 0
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
#endif
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
