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
#include <zephyr/logging/log.h>
#include <soc.h>
#include "platform.h"

LOG_MODULE_REGISTER(rtc_timer_ls, LOG_LEVEL_DBG);

struct rtc_timer_ls_data {
	struct k_mutex lock;
};

static bool rtc_timer_ls_is_leap_year(int year)
{
	return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

static bool rtc_timer_ls_calendar_valid(const struct rtc_time *tm)
{
	int max_day;
	bool leap;

	/* hardware epoch floor 1970 */
	if (tm->tm_year < 70 || tm->tm_year > 199) {return false;}
	if (tm->tm_mon < 0 || tm->tm_mon > 11) {return false;}
	leap = rtc_timer_ls_is_leap_year(tm->tm_year + 1900);
	switch (tm->tm_mon + 1) {
		case 4:case 6:case 9:case 11:
			max_day = 30;
			break;
		case 2:
			max_day = leap ? 29 : 28;
			break;
		default:
			max_day = 31;
			break;
	}
	if (tm->tm_mday < 1 || tm->tm_mday > max_day) {return false;}
	if (tm->tm_hour < 0 || tm->tm_hour > 23) {return false;}
	if (tm->tm_min < 0 || tm->tm_min > 59) {return false;}
	if (tm->tm_sec < 0 || tm->tm_sec > 59) {return false;}
	if (tm->tm_wday < -1 || tm->tm_wday > 6) {return false;}
	if (tm->tm_yday != -1 && (tm->tm_yday < 0 || ( (tm->tm_yday) >= (leap ? 366 : 365)))) {return false;}
	if (tm->tm_isdst != -1 && tm->tm_isdst != 0 && tm->tm_isdst != 1) {return false;}
	if (tm->tm_nsec < 0 || tm->tm_nsec > 999999999) {return false;}

	return true;
}

static int rtc_timer_ls_set_time(const struct device *dev, const struct rtc_time *tm)
{
	struct rtc_timer_ls_data *data = dev->data;
	struct rtc_time copy = *tm;
	int ret;

	if (!rtc_timer_ls_calendar_valid(tm)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = rtc_timer_set_time(rtc_time_to_tm(&copy));
	k_mutex_unlock(&data->lock);

	return ret == 0 ? 0 : -EIO;
}

static int rtc_timer_ls_get_time(const struct device *dev, struct rtc_time *tm)
{
	struct rtc_timer_ls_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = rtc_timer_get_time(rtc_time_to_tm(tm));
	k_mutex_unlock(&data->lock);

	if (ret != 0) {
		return -ENODATA;
	}

	/* DST/ns not supported by HW */
	tm->tm_isdst = -1;
	tm->tm_nsec = 0;
	return 0;
}

static int rtc_timer_ls_init(const struct device *dev)
{
	struct rtc_timer_ls_data *data = dev->data;

	k_mutex_init(&data->lock);
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
