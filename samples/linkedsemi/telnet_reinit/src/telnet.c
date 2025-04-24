/*
 * Copyright (c) 2017 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_telnet_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>

#include "net_sample_common.h"
#include <stdlib.h>
#include <zephyr/cache.h>
#include <platform.h>
#include <reg_sysc_app_cpu.h>

#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/shell/shell_telnet.h>
#include <reg_sysc_app_per.h>

#if defined(CONFIG_NET_IPV6)
    #define MCAST_IP6ADDR "ff84::2"

static void setup_ipv6(void)
{
    struct in6_addr addr;
    struct net_if *iface = net_if_get_default();

    if (net_addr_pton(AF_INET6, MCAST_IP6ADDR, &addr)) {
        LOG_ERR("Invalid address: %s", MCAST_IP6ADDR);
        return;
    }

    net_if_ipv6_maddr_add(iface, &addr);
}
#else
    #define setup_ipv6(...)
#endif /* CONFIG_NET_IPV6 */

#include <zephyr/drivers/mdio.h>

int mdio_set_phy(void)
{
    const struct device *const mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio1));

    uint16_t reg = 0x0;
    uint16_t val;
    int rc = 0;

    for (uint16_t addr = 0x0; addr < 0x2; addr++) {
        printk("phy addr: %d  id: ", addr);
        rc = mdio_read(mdio_dev, addr, 0x2, &val);
        printk("%4.4x", val);
        rc = mdio_read(mdio_dev, addr, 0x3, &val);
        printk("%4.4x\n", val);

        printk("phy addr: %d  reg: %d :", addr, reg);
        rc = mdio_read(mdio_dev, addr, reg, &val);
        printk("%4.4x\n", val);

        /* 10Mbps */
        // sys_clear_bits((mem_addr_t)&val, BIT(6) | BIT(12) | BIT(13));
        // rc = mdio_write(mdio_dev, 0x0, reg, val);
        rc = mdio_write(mdio_dev, 0x0, reg, 0);

        printk("phy addr: %d  reg: %d :", addr, reg);
        rc = mdio_read(mdio_dev, addr, reg, &val);
        printk("%4.4x\n", val);
    }

    return rc;
}

volatile uint8_t *g_flag = (volatile uint8_t *)(SRAM1_ADDR + KB(511));
const struct device *const eth_dev = DEVICE_DT_GET(DT_NODELABEL(eth1));
extern int dwmac_reinit(const struct device *const dev);
extern void dwmac_deinit(const struct device *const dev);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)

int main(void)
{
    const struct shell *sh = shell_backend_uart_get_ptr();
    bool is_cpu1_use_eth = true;
    int err;
    uint32_t g_idx = 0;

    LOG_INF("Starting Telnet sample");

    while (1) {
        g_idx++;
        printk("g_idx: %d  is_cpu1_use_eth: %s\n", g_idx, is_cpu1_use_eth ? "Y" : "N");

        if (is_cpu1_use_eth) {
            is_cpu1_use_eth = !is_cpu1_use_eth;
            app_cpu_reset();

            if (g_idx > 0) {
                err = shell_execute_cmd(sh, "net ipv4 del 1   192.168.1.123");
                __ASSERT(err == 0, "failed to execute shell command (err %d)", err);

                dwmac_deinit(eth_dev);
                dwmac_reinit(eth_dev);

                err = shell_execute_cmd(sh, "net ipv4");
                __ASSERT(err == 0, "failed to execute shell command (err %d)", err);
                err = shell_execute_cmd(sh, "net ipv4 add 1 192.168.1.123 255.255.255.0");
                __ASSERT(err == 0, "failed to execute shell command (err %d)", err);
                err = shell_execute_cmd(sh, "net ipv4");
                __ASSERT(err == 0, "failed to execute shell command (err %d)", err);
            }

        } else {
            is_cpu1_use_eth = !is_cpu1_use_eth;

            *g_flag = 0x1;
            sys_cache_instr_flush_all();
            sys_cache_data_flush_and_invd_all();
            dwmac_deinit(eth_dev);

            SYSC_APP_PER->PD_PER_SRST1 = SYSC_APP_PER_SRST_CLR_UART4_N_MASK;
            SYSC_APP_PER->PD_PER_SRST1 = SYSC_APP_PER_SRST_SET_UART4_N_MASK;
            app_cpu_dereset();
            printk("app_cpu_dereset_by_addr\n");

            /* wait for cpu1 access emmc done */
            int cnt = 0;
            while (*g_flag) {
                sys_cache_data_invd_all();
                printk("*g_flag: %#x  cnt: %d\n", *g_flag, cnt++);
                k_msleep(100);
            }
            printk("next\n");
            *g_flag = 0x1;
            continue;
        }

        mdio_set_phy();

        wait_for_network();
        setup_ipv6();

        err = shell_execute_cmd(sh, "net udp send 192.168.1.255 8080 \"hello world, i am cpu1\"");
        __ASSERT(err == 0, "failed to execute shell command (err %d)", err);

        k_msleep(1000);
    }

    return 0;
}
#else

int main(void)
{
    const struct shell *sh = shell_backend_uart_get_ptr();
    int err;

    printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

    sys_cache_data_invd_all();
    *g_flag = 0x1;
    sys_cache_data_flush_all();
    LOG_INF("Starting Telnet sample");

    mdio_set_phy();

    wait_for_network();
    setup_ipv6();

    err = shell_execute_cmd(sh, "net udp send 192.168.1.255 8080 \"hello world, i am cpu2\"");
    __ASSERT(err == 0, "failed to execute shell command (err %d)", err);

    dwmac_deinit(eth_dev);
    printk("byebye World! %s\n", CONFIG_BOARD_TARGET);

    *g_flag = 0x0;
    sys_cache_data_flush_all();

    int cnt = 0;
    while (1) {
        printk("*g_flag: %#x  cnt: %d\n", *g_flag, cnt++);
        k_msleep(10 * 1000);
    }

    return 0;
}
#endif
