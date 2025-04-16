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
		sys_clear_bits((mem_addr_t)&val, BIT(6) | BIT(12) | BIT(13));
		rc = mdio_write(mdio_dev, 0x0, reg, val);

		printk("phy addr: %d  reg: %d :", addr, reg);
		rc = mdio_read(mdio_dev, addr, reg, &val);
		printk("%4.4x\n", val);
	}

    return rc;
}

int main(void)
{
	LOG_INF("Starting Telnet sample");

	mdio_set_phy();

	wait_for_network();
	setup_ipv6();
	return 0;
}
