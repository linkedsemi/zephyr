#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdint.h>

#define DT_DRV_COMPAT linkedsemi_ls_rtc
const struct device *rtc = DEVICE_DT_INST_GET(0);
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const char *weekday_str[] = {
    "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday","Sunday"
};

static void alarm_callback(const struct device *dev, uint16_t id, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(id);
    ARG_UNUSED(user_data);
    LOG_INF("================================");
    LOG_INF("Alarm callback triggered!");
    LOG_INF("================================");
}

static void update_alarm_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    LOG_INF("================================");
    LOG_INF("Update alarm callback triggered!");

}

void main(void)
{
    struct rtc_time now;
    const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc0));

    //get rtc_driver_api
    const struct rtc_driver_api *api = (const struct rtc_driver_api *)rtc_dev->api;

    //alarm pending
    bool pending = false;

    if (!device_is_ready(rtc)) {

        LOG_INF("RTC device not ready!\n");
        return;
    }
    else  {

        LOG_INF("RTC device ready is ok!\n");
    }

    LOG_INF("rtc driver ready? %d\n", device_is_ready(DEVICE_DT_INST_GET(0)));

    k_sleep(K_SECONDS(2));

    // ============  set_time  =============
    struct rtc_time set_tm = {

        .tm_year = 123,
        .tm_mon  = 02,
        .tm_mday = 28,
        .tm_wday = 7,
        .tm_hour = 23,
        .tm_min  = 59,
        .tm_sec  = 55
    };

    if (rtc_set_time(rtc, &set_tm) != 0) {

        LOG_INF("Failed to set RTC time!\n");
        return;
    }

    LOG_INF("RTC time set to:  %04d.%02d.%02d %s %02d:%02d:%02d\n",
           set_tm.tm_year + 1900, set_tm.tm_mon, set_tm.tm_mday, weekday_str[set_tm.tm_wday-1],
           set_tm.tm_hour, set_tm.tm_min, set_tm.tm_sec);

           k_sleep(K_SECONDS(1));

    // ============ alarm_set_time =============
    struct rtc_time alarm_tm ={

        .tm_year = 123,
        .tm_mon  = 3,
        .tm_mday = 1,
        .tm_wday = 1,
        .tm_hour = 0,
        .tm_min  = 0,
        .tm_sec  = 0
    };

    uint16_t alarm_mask =   RTC_ALARM_TIME_MASK_SECOND |
                             RTC_ALARM_TIME_MASK_MINUTE |
                             RTC_ALARM_TIME_MASK_HOUR   |
                             RTC_ALARM_TIME_MASK_MONTHDAY |
                             RTC_ALARM_TIME_MASK_MONTH  |
                             RTC_ALARM_TIME_MASK_YEAR;

    rtc_alarm_set_callback(rtc, 0, alarm_callback, NULL);

    rtc_alarm_set_time(rtc, 0, alarm_mask, &alarm_tm);

    LOG_INF("Alarm time set to:  %04d.%02d.%02d %s %02d:%02d:%02d\n",
           alarm_tm.tm_year + 1900, alarm_tm.tm_mon, alarm_tm.tm_mday, weekday_str[alarm_tm.tm_wday-1],
           alarm_tm.tm_hour, alarm_tm.tm_min, alarm_tm.tm_sec);

    struct rtc_time alarm_readback;
    uint16_t alarm_mask_read;

    if (rtc_alarm_get_time(rtc, 0, &alarm_mask_read, &alarm_readback) == 0) {

        LOG_INF(">>> [ReadBack Alarm] %04d.%02d.%02d %s %02d:%02d:%02d\n",
               alarm_readback.tm_year + 1900, alarm_readback.tm_mon, alarm_readback.tm_mday,
               weekday_str[alarm_readback.tm_wday - 1],
               alarm_readback.tm_hour, alarm_readback.tm_min, alarm_readback.tm_sec);
               LOG_INF(">>> Alarm mask: 0x%04x\n", alarm_mask_read);
    } else {

        LOG_INF(">>> Failed to read back alarm time.\n");
    }

    int ret = rtc_update_set_callback(rtc, update_alarm_callback, false);
    if (ret == -ENOTSUP) {

        LOG_INF("The update callback function is not supported");
    } else {

        LOG_ERR("rtc_update_set_callback=0x%04x\n",ret);
    }

    // ============ set_calibration =============
    int ret_setcali = rtc_set_calibration(rtc_dev, -1300000000);
    if (ret_setcali == 0) {

        LOG_INF("RTC calibration set successfully.");
    }
    else if (ret_setcali == -ENOTSUP) {

        LOG_ERR("RTC hardware does not support calibration.");
    } else {

        LOG_ERR("Failed to set RTC calibration, error code: %d", ret_setcali);
    }

    int32_t calibration_value = 0;


    while(1){

        // ============ get_time =============
        rtc_get_time(rtc, &now);

        LOG_INF("[LOOP] Current RTC time: %04d.%02d.%02d %s %02d:%02d:%02d\n",
                now.tm_year + 1900, now.tm_mon, now.tm_mday,  weekday_str[now.tm_wday-1],
                now.tm_hour, now.tm_min, now.tm_sec);

        // ============ alarm_is_pending =============
        if (api->alarm_is_pending) {

        pending = api->alarm_is_pending(rtc_dev, 0);
        LOG_INF("Alarm pending: %s\n", pending ? "YES" : "NO");
        } else {

        LOG_INF("alarm_is_pending() not implemented in driver\n");
        }

        //rtc_ls_get_calibration
        int result = rtc_get_calibration(rtc, &calibration_value);
        k_sleep(K_SECONDS(1));
        if (result == 0) {

        LOG_INF("RTC calibration: %d ppb\n", calibration_value);
        } else {

        LOG_INF("get calibration fault, result: %d\n", result);
        }

        k_sleep(K_SECONDS(1));
    }
}
