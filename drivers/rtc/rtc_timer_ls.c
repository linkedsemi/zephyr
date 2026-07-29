/*
 * Copyright (c) 2026 Linkedsemi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr RTC driver for the BSTIM + PIS + GPTIMA software timer chain
 * implemented in platform.c (rtc_timer_init/set_time/get_time).
 */

#define DT_DRV_COMPAT linkedsemi_ls_rtc_timer

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include "rtc_utils.h"
#include "platform.h"

LOG_MODULE_REGISTER(rtc_timer_ls, LOG_LEVEL_DBG);

#define RTC_TIMER_LS_SET_MASK    \
    (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |    \
    RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_YEAR)

struct rtc_timer_ls_data {
	struct k_spinlock lock;
};

static int rtc_timer_ls_set_time(const struct device *dev, const struct rtc_time *tm)
{
	struct rtc_timer_ls_data *data = dev->data;
	struct rtc_time copy = *tm;

	if (!rtc_utils_validate_rtc_time(tm, RTC_TIMER_LS_SET_MASK)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (rtc_timer_set_time(rtc_time_to_tm(&copy)) != 0) {
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int rtc_timer_ls_get_time(const struct device *dev, struct rtc_time *tm)
{
	struct rtc_timer_ls_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (rtc_timer_get_time(rtc_time_to_tm(tm)) != 0) {
		tm->tm_nsec = 0;
		k_spin_unlock(&data->lock, key);
		return -EIO;
	}
	tm->tm_nsec = 0;

	k_spin_unlock(&data->lock, key);

	return 0;
}

static int rtc_timer_ls_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	rtc_timer_init();

	return 0;
}

static const struct rtc_driver_api rtc_timer_ls_api = {
	.set_time = rtc_timer_ls_set_time,
	.get_time = rtc_timer_ls_get_time,
};

#define RTC_TIMER_LS_INIT(inst)                                                                    \
	static struct rtc_timer_ls_data rtc_timer_ls_data_##inst;                                  \
	DEVICE_DT_INST_DEFINE(inst,                                                                \
			      rtc_timer_ls_init,                                                   \
			      NULL,                                                                \
			      &rtc_timer_ls_data_##inst,                                           \
			      NULL,                                                                \
			      POST_KERNEL,                                                         \
			      CONFIG_RTC_INIT_PRIORITY,                                            \
			      &rtc_timer_ls_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_TIMER_LS_INIT)
