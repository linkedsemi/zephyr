/*
 * Copyright (c) 2026 Linkedsemi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rtc_au1901c.c
 * @brief Aurasemi AU1901C I2C RTC driver (secondary I2C interface)
 *
 * Datasheet: AU1901C_Timing_RTC_DS_ZH_V1.0.pdf
 *
 * The AU1901C is a dual-I2C ultra-low-power RTC (RTC slave address 0x6F,
 * SRAM 0x57). On this board the BMC sits on the **secondary** I2C
 * interface. Per datasheet 3.1 and table 7 the secondary interface may
 * write only
 *   - the time/calendar registers, while the TWO (Time-register Write
 *     Ownership) bit is 0,
 *   - SRAM (through MWO) and the CLKOUT control register,
 * and may only read
 *   - CONTROL (0x0A): ST/DM/HF/DSM/AIE/OFIE/CIE/TWO,
 *   - STATUS  (0x0B): AF/OF/RTCF/CIF/BVL,
 *   - the alarm registers (0x01/0x03/0x05).
 *
 * This driver therefore never writes CONTROL or STATUS. It reads CONTROL
 * once at init and then follows whatever the primary side configured:
 *   - DM  selects BCD or binary calendar format,
 *   - HF  selects 12-hour (bit 7 of 0x04 is AM/PM) or 24-hour format,
 *   - TWO says who owns time-register writes (it can change at runtime).
 * STATUS is read for reporting only; its flags are not cleared.
 *
 * Register map (table 7): time registers are interleaved with the alarm
 * registers, so a burst read has to pick the fields by index:
 *   0x00 SECONDS  0x01 SECOND_ALARM  0x02 MINUTES  0x03 MINUTE_ALARM
 *   0x04 HOURS    0x05 HOUR_ALARM    0x06 DAY      0x07 DATE
 *   0x08 MONTH    0x09 YEAR          0x0A CONTROL  0x0B STATUS
 *   0x0C CLKOUT_CTRL  0x0D CONTROL2  0x0E SCRATCH  0x0F VERSION
 *   0x10 VENDOR_ID    0x11 VBAT_DET_CTRL
 *
 * Hardware DAY is 1..7 (1 = Sunday); rtc_time uses 0..6 with Sunday = 0
 * and -1 for "unknown". YEAR holds 0..99, i.e. 2000..2099.
 */

#define DT_DRV_COMPAT aurasemi_au1901c

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
/* DEV_ERR/DEV_WRN/DEV_INF for this SoC family. */
#include <soc.h>

LOG_MODULE_REGISTER(rtc_au1901c, LOG_LEVEL_DBG);

/* 7-bit RTC slave address (datasheet table 6). */
#define AU1901C_I2C_ADDR 0x6F

/* Register addresses (datasheet table 7). */
#define AU1901C_REG_SECONDS       0x00
#define AU1901C_REG_SECONDS_ALARM 0x01
#define AU1901C_REG_MINUTES       0x02
#define AU1901C_REG_MINUTES_ALARM 0x03
#define AU1901C_REG_HOURS         0x04
#define AU1901C_REG_HOURS_ALARM   0x05
#define AU1901C_REG_DAY_OF_WEEK   0x06
#define AU1901C_REG_DATE          0x07
#define AU1901C_REG_MONTH         0x08
#define AU1901C_REG_YEAR          0x09
#define AU1901C_REG_CONTROL       0x0A
#define AU1901C_REG_STATUS        0x0B
#define AU1901C_REG_CLKOUT_CTRL   0x0C
#define AU1901C_REG_CONTROL2      0x0D
#define AU1901C_REG_SCRATCHPAD    0x0E
#define AU1901C_REG_VERSION       0x0F
#define AU1901C_REG_VENDOR_ID     0x10
#define AU1901C_REG_VBAT_DET_CTRL 0x1E

/* Field masks. They must cover both BCD and binary encodings, e.g. the date
 * register is 0x01..0x31 in BCD mode and 0x01..0x1F in binary mode.
 */
#define AU1901C_SECONDS_MASK     GENMASK(6, 0)
#define AU1901C_MINUTES_MASK     GENMASK(6, 0)
#define AU1901C_HOURS_MASK       GENMASK(5, 0)
/*
 * HOUR register (0x04), 12-hour mode (datasheet 4.1.5):
 *   bit 7   PM_AM : 0 = AM, 1 = PM
 *   bit 6   RSV   : reserved
 *   bit 5:0       : 01-12 (BCD) / 0x01-0x0C (binary)
 * So the hour value lives in [5:0] and the AM/PM flag is bit 7; both are
 * part of the 12-hour field, and bit 6 must stay 0.
 */
#define AU1901C_HOURS_12H_MASK   (GENMASK(5, 0) | BIT(7))
#define AU1901C_HOURS_12H_VALUE  GENMASK(5, 0)
#define AU1901C_HOURS_12H_PM     BIT(7)
#define AU1901C_DAY_OF_WEEK_MASK GENMASK(2, 0)
#define AU1901C_DATE_MASK        GENMASK(5, 0)
#define AU1901C_MONTH_MASK       GENMASK(4, 0)
#define AU1901C_YEAR_MASK        GENMASK(7, 0)

/* AU1901C year register counts from 2000. */
#define AU1901C_YEAR_OFFSET 100

/* Control register (0x0A) - read-only for the secondary interface. */
#define AU1901C_CONTROL_ST   BIT(7) /* 1: RTC stopped */
#define AU1901C_CONTROL_DM   BIT(6) /* 1: binary, 0: BCD */
#define AU1901C_CONTROL_HF   BIT(5) /* 1: 24-hour, 0: 12-hour */
#define AU1901C_CONTROL_DSM  BIT(4) /* daylight saving mode */
#define AU1901C_CONTROL_AIE  BIT(3) /* alarm interrupt enable */
#define AU1901C_CONTROL_OFIE BIT(2) /* oscillator fail interrupt enable */
#define AU1901C_CONTROL_CIE  BIT(1) /* RTC clear interrupt enable */
#define AU1901C_CONTROL_TWO  BIT(0) /* 0: secondary owns time writes */

/* Status register (0x0B) - read-only for the secondary interface. */
#define AU1901C_STATUS_AF       BIT(7) /* alarm flag */
#define AU1901C_STATUS_OF       BIT(6) /* oscillator fail */
#define AU1901C_STATUS_RTCF     BIT(5) /* full power fail */
#define AU1901C_STATUS_CIF      BIT(4) /* RTC_CLR# detected */
#define AU1901C_STATUS_BVL_MASK GENMASK(2, 0) /* battery voltage level */

/* Vendor/version registers. */
#define AU1901C_VENDOR_ID           0x03
#define AU1901C_VERSION_MAJOR_MASK  GENMASK(7, 4)
#define AU1901C_VERSION_MAJOR_SHIFT 4
#define AU1901C_VERSION_MINOR_MASK  GENMASK(3, 0)

/*
 * One burst read covers the time registers 0x00..0x09 (the alarm registers
 * in between are simply skipped when picking the fields). The burst starts
 * at SECONDS (0x00), so a buffer index equals the register address and the
 * AU1901C_REG_* values below are used for both.
 */
#define AU1901C_TIME_BLOCK_LEN (AU1901C_REG_YEAR + 1)

struct rtc_au1901c_config {
    struct i2c_dt_spec i2c;
};

struct rtc_au1901c_data {
    struct k_mutex lock;
    /* Format the primary side selected in CONTROL (read-only for us). */
    bool bcd_mode;
    bool hour12_mode;
    /* Time-register write permission: 1 = allowed, 0 = denied (TWO). */
    bool time_write_permission;
};

static inline bool au1901c_is_leap_year(int tm_year)
{
    int year = tm_year + 1900;

    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

/* Register value -> plain integer, for the format the part is using. */
static uint8_t au1901c_decode_field(uint8_t raw, uint8_t mask, bool bcd)
{
    raw &= mask;

    return bcd ? bcd2bin(raw) : raw;
}

/* Plain integer -> register value, for the format the part is using. */
static uint8_t au1901c_encode_field(int value, uint8_t mask, bool bcd)
{
    uint8_t raw = (uint8_t)value & mask;

    return bcd ? bin2bcd(raw) : raw;
}

static int au1901c_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct rtc_au1901c_config *config = dev->config;
    int ret;

    ret = i2c_write_read_dt(&config->i2c, &reg, sizeof(reg), val, 1);
    if (ret < 0) {
        DEV_ERR(dev, "Failed to read reg 0x%02x: %d", reg, ret);
        return ret;
    }

    return 0;
}

static int au1901c_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct rtc_au1901c_config *config = dev->config;
    uint8_t buf[2] = {reg, val};
    int ret;

    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        DEV_ERR(dev, "Failed to write reg 0x%02x: %d", reg, ret);
        return ret;
    }

    return 0;
}

static int au1901c_read_regs(const struct device *dev, uint8_t start_reg,
                 uint8_t *buf, uint8_t len)
{
    const struct rtc_au1901c_config *config = dev->config;
    int ret;

    ret = i2c_write_read_dt(&config->i2c, &start_reg, sizeof(start_reg),
                buf, len);
    if (ret < 0) {
        DEV_ERR(dev, "Failed to read regs from 0x%02x: %d",
            start_reg, ret);
        return ret;
    }

    return 0;
}

static int au1901c_read_device_id(const struct device *dev)
{
    uint8_t vendor_id;
    uint8_t version;
    int ret;

    ret = au1901c_read_reg(dev, AU1901C_REG_VENDOR_ID, &vendor_id);
    if (ret < 0) {
        return ret;
    }
    ret = au1901c_read_reg(dev, AU1901C_REG_VERSION, &version);
    if (ret < 0) {
        return ret;
    }

    if (vendor_id != AU1901C_VENDOR_ID) {
        DEV_ERR(dev, "Unexpected vendor id 0x%02x", vendor_id);
        return -ENODEV;
    }

    DEV_INF(dev, "Device ID: vendor=0x%02x, version=%u.%u", vendor_id,
        (unsigned int)((version & AU1901C_VERSION_MAJOR_MASK) >>
                   AU1901C_VERSION_MAJOR_SHIFT),
        (unsigned int)(version & AU1901C_VERSION_MINOR_MASK));

    return 0;
}

static bool au1901c_is_time_valid(const struct device *dev,
                 const struct rtc_time *timeptr)
{
    int max_day = 31;

    /* AU1901C year register stores only 0-99, interpreted as 2000-2099. */
    if (timeptr->tm_year < AU1901C_YEAR_OFFSET || timeptr->tm_year > 199) {
        DEV_ERR(dev, "Year out of range (2000-2099): %d",
            timeptr->tm_year + 1900);
        return false;
    }

    switch (timeptr->tm_mon + 1) {
    case 4:
    case 6:
    case 9:
    case 11:
        max_day = 30;
        break;
    case 2:
        max_day = au1901c_is_leap_year(timeptr->tm_year) ? 29 : 28;
        break;
    default:
        max_day = 31;
        break;
    }

    if (timeptr->tm_mon < 0 || timeptr->tm_mon > 11 ||
        timeptr->tm_mday < 1 || timeptr->tm_mday > max_day ||
        timeptr->tm_hour < 0 || timeptr->tm_hour > 23 ||
        timeptr->tm_min < 0 || timeptr->tm_min > 59 ||
        timeptr->tm_sec < 0 || timeptr->tm_sec > 59 ||
        /* tm_wday == -1 means "unknown" (hardware DAY not set yet). */
        timeptr->tm_wday < -1 || timeptr->tm_wday > 6) {
        DEV_ERR(dev,
            "Invalid time fields: %04d-%02d-%02d %02d:%02d:%02d wday=%d",
            timeptr->tm_year + 1900, timeptr->tm_mon + 1,
            timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min,
            timeptr->tm_sec, timeptr->tm_wday);
        return false;
    }

    return true;
}

/* Encode tm_hour for the hour format the part is using. */
static uint8_t au1901c_encode_hour(const struct rtc_au1901c_data *data, int hour)
{
    if (!data->hour12_mode) {
        return au1901c_encode_field(hour, AU1901C_HOURS_MASK,
                        data->bcd_mode);
    }

    int h12 = hour % 12;
    uint8_t raw;

    if (h12 == 0) {
        h12 = 12;
    }
    raw = au1901c_encode_field(h12, AU1901C_HOURS_12H_VALUE,
                   data->bcd_mode);
    if (hour >= 12) {
        raw |= AU1901C_HOURS_12H_PM;
    }

    return raw;
}

/*
 * Decode the hour register for the hour format the part is using.
 * Returns the hour as 0-23, or -EINVAL when the register holds a value that
 * is illegal for the selected format (12-hour mode is 01-12, so 0 and >12
 * mean the data cannot be trusted).
 */
static int au1901c_decode_hour(const struct device *dev,
                   const struct rtc_au1901c_data *data, uint8_t raw)
{
    if (!data->hour12_mode) {
        return au1901c_decode_field(raw, AU1901C_HOURS_MASK, data->bcd_mode);
    }

    raw &= AU1901C_HOURS_12H_MASK;

    int hour = au1901c_decode_field(raw, AU1901C_HOURS_12H_VALUE,
                    data->bcd_mode);

    if (hour < 1 || hour > 12) {
        DEV_WRN(dev, "Illegal 12-hour value 0x%02x",
            (unsigned int)(raw & AU1901C_HOURS_12H_VALUE));
        return -EINVAL;
    }

    return (raw & AU1901C_HOURS_12H_PM) ? (hour % 12) + 12 : hour % 12;
}

/*
 * The primary side may change the calendar format (DM), the hour format (HF)
 * or the time-register write ownership (TWO) at any time, so re-read CONTROL
 * before every transfer instead of caching it for the whole runtime.
 */
static int au1901c_refresh_config(const struct device *dev)
{
    struct rtc_au1901c_data *data = dev->data;
    uint8_t ctrl;
    int ret;

    ret = au1901c_read_reg(dev, AU1901C_REG_CONTROL, &ctrl);
    if (ret < 0) {
        return ret;
    }

    bool bcd = (ctrl & AU1901C_CONTROL_DM) == 0U;
    bool hour12 = (ctrl & AU1901C_CONTROL_HF) == 0U;
    bool permission = (ctrl & AU1901C_CONTROL_TWO) == 0U;

    if (bcd != data->bcd_mode || hour12 != data->hour12_mode ||
        permission != data->time_write_permission) {
        DEV_INF(dev, "CONTROL changed to 0x%02x: %s calendar, %s, time-register write permission %s",
            ctrl, bcd ? "BCD" : "binary",
            hour12 ? "12-hour" : "24-hour",
            permission ? "yes" : "no");
    }

    data->bcd_mode = bcd;
    data->hour12_mode = hour12;
    data->time_write_permission = permission;

    return 0;
}

static int rtc_au1901c_set_time(const struct device *dev,
                   const struct rtc_time *timeptr)
{
    struct rtc_au1901c_data *data = dev->data;
    uint8_t expect[AU1901C_TIME_BLOCK_LEN];
    uint8_t buf[AU1901C_TIME_BLOCK_LEN];
    int ret;

    if (!au1901c_is_time_valid(dev, timeptr)) {
        DEV_WRN(dev, "RTC time is invalid, ignoring write");
        return -EINVAL;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    /* The primary side may have changed the format or the write ownership. */
    ret = au1901c_refresh_config(dev);
    if (ret < 0) {
        goto unlock;
    }

    if (!data->time_write_permission) {
        DEV_ERR(dev, "No time-register write permission (TWO=1, owned by the primary I2C interface)");
        ret = -EACCES;
        goto unlock;
    }

    expect[AU1901C_REG_SECONDS] =
        au1901c_encode_field(timeptr->tm_sec, AU1901C_SECONDS_MASK,
                     data->bcd_mode);
    expect[AU1901C_REG_MINUTES] =
        au1901c_encode_field(timeptr->tm_min, AU1901C_MINUTES_MASK,
                     data->bcd_mode);
    expect[AU1901C_REG_HOURS] = au1901c_encode_hour(data, timeptr->tm_hour);
    expect[AU1901C_REG_DAY_OF_WEEK] = (timeptr->tm_wday >= 0) ?
        ((timeptr->tm_wday + 1) & AU1901C_DAY_OF_WEEK_MASK) : 0U;
    expect[AU1901C_REG_DATE] =
        au1901c_encode_field(timeptr->tm_mday, AU1901C_DATE_MASK,
                     data->bcd_mode);
    expect[AU1901C_REG_MONTH] =
        au1901c_encode_field(timeptr->tm_mon + 1, AU1901C_MONTH_MASK,
                     data->bcd_mode);
    expect[AU1901C_REG_YEAR] =
        au1901c_encode_field(timeptr->tm_year - AU1901C_YEAR_OFFSET,
                     AU1901C_YEAR_MASK, data->bcd_mode);

    /*
     * The alarm registers sit between the time registers, so write them one
     * by one instead of a burst that would overwrite the alarm values.
     */
    ret = au1901c_write_reg(dev, AU1901C_REG_SECONDS,
                expect[AU1901C_REG_SECONDS]);
    if (ret == 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_MINUTES,
                    expect[AU1901C_REG_MINUTES]);
    }
    if (ret == 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_HOURS,
                    expect[AU1901C_REG_HOURS]);
    }
    if (ret == 0 && timeptr->tm_wday >= 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_DAY_OF_WEEK,
                    expect[AU1901C_REG_DAY_OF_WEEK]);
    }
    if (ret == 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_DATE,
                    expect[AU1901C_REG_DATE]);
    }
    if (ret == 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_MONTH,
                    expect[AU1901C_REG_MONTH]);
    }
    if (ret == 0) {
        ret = au1901c_write_reg(dev, AU1901C_REG_YEAR,
                    expect[AU1901C_REG_YEAR]);
    }

    /*
     * Verify: this interface may be locked out (TWO=1, or registers that are
     * read-only for the secondary interface). Compare everything but the
     * seconds field, which may advance while we read back.
     */
    if (ret == 0) {
        ret = au1901c_read_regs(dev, AU1901C_REG_SECONDS, buf,
                    ARRAY_SIZE(buf));
    }
    if (ret == 0) {
        static const uint8_t check[] = {
            AU1901C_REG_MINUTES, AU1901C_REG_HOURS, AU1901C_REG_DATE,
            AU1901C_REG_MONTH, AU1901C_REG_YEAR,
        };

        for (uint8_t i = 0; i < ARRAY_SIZE(check); i++) {
            uint8_t idx = check[i];

            if (buf[idx] != expect[idx]) {
                DEV_ERR(dev, "Time write did not stick (reg 0x%02x: 0x%02x != 0x%02x), TWO=1?",
                    idx, buf[idx], expect[idx]);
                ret = -EIO;
                break;
            }
        }
    }

    if (ret == 0 && timeptr->tm_wday >= 0 &&
        (buf[AU1901C_REG_DAY_OF_WEEK] & AU1901C_DAY_OF_WEEK_MASK) !=
            (expect[AU1901C_REG_DAY_OF_WEEK] & AU1901C_DAY_OF_WEEK_MASK)) {
        DEV_WRN(dev, "Day-of-week write did not stick (0x%02x != 0x%02x)",
            buf[AU1901C_REG_DAY_OF_WEEK], expect[AU1901C_REG_DAY_OF_WEEK]);
    }

unlock:
    k_mutex_unlock(&data->lock);

    if (ret < 0) {
        DEV_ERR(dev, "Failed to set time (%d)", ret);
        return ret;
    }

    DEV_INF(dev, "Time set to %04d-%02d-%02d %02d:%02d:%02d",
        timeptr->tm_year + 1900, timeptr->tm_mon + 1,
        timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min,
        timeptr->tm_sec);

    return 0;
}

static int rtc_au1901c_get_time(const struct device *dev,
                   struct rtc_time *timeptr)
{
    struct rtc_au1901c_data *data = dev->data;
    uint8_t buf[AU1901C_TIME_BLOCK_LEN];
    uint8_t day_of_week;
    int ret;

    k_mutex_lock(&data->lock, K_FOREVER);
    /* The primary side may have changed the format in the meantime. */
    ret = au1901c_refresh_config(dev);
    if (ret == 0) {
        /* Burst read starts at 0x00, so index == register address. */
        ret = au1901c_read_regs(dev, AU1901C_REG_SECONDS, buf,
                       ARRAY_SIZE(buf));
    }
    k_mutex_unlock(&data->lock);
    if (ret < 0) {
        DEV_ERR(dev, "Failed to get time");
        return ret;
    }

    timeptr->tm_sec = au1901c_decode_field(buf[AU1901C_REG_SECONDS],
                          AU1901C_SECONDS_MASK, data->bcd_mode);
    timeptr->tm_min = au1901c_decode_field(buf[AU1901C_REG_MINUTES],
                          AU1901C_MINUTES_MASK, data->bcd_mode);

    ret = au1901c_decode_hour(dev, data, buf[AU1901C_REG_HOURS]);
    if (ret < 0) {
        DEV_WRN(dev, "Invalid time read from RTC");
        return ret;
    }
    timeptr->tm_hour = ret;

    day_of_week = buf[AU1901C_REG_DAY_OF_WEEK] & AU1901C_DAY_OF_WEEK_MASK;
    if (day_of_week >= 1U && day_of_week <= 7U) {
        timeptr->tm_wday = (int)day_of_week - 1;
    } else {
        timeptr->tm_wday = -1;
    }

    timeptr->tm_mday = au1901c_decode_field(buf[AU1901C_REG_DATE],
                           AU1901C_DATE_MASK, data->bcd_mode);
    timeptr->tm_mon = (int)au1901c_decode_field(buf[AU1901C_REG_MONTH],
                              AU1901C_MONTH_MASK,
                              data->bcd_mode) - 1;
    timeptr->tm_year = au1901c_decode_field(buf[AU1901C_REG_YEAR],
                          AU1901C_YEAR_MASK, data->bcd_mode) +
               AU1901C_YEAR_OFFSET;
    timeptr->tm_yday = -1;
    timeptr->tm_isdst = -1;
    timeptr->tm_nsec = 0;

    if (!au1901c_is_time_valid(dev, timeptr)) {
        DEV_WRN(dev, "Invalid time read from RTC");
        return -EINVAL;
    }

    return 0;
}

static int rtc_au1901c_init(const struct device *dev)
{
    const struct rtc_au1901c_config *config = dev->config;
    struct rtc_au1901c_data *data = dev->data;
    uint8_t ctrl;
    uint8_t status;
    int ret;

    k_sleep(K_MSEC(50));

    k_mutex_init(&data->lock);
    k_mutex_lock(&data->lock, K_FOREVER);

    if (!i2c_is_ready_dt(&config->i2c)) {
        DEV_ERR(dev, "I2C bus not ready");
        ret = -ENODEV;
        goto unlock;
    }

    for (int attempt = 0; attempt < 5; attempt++) {
        ret = au1901c_read_device_id(dev);
        if (ret == 0) {
            break;
        }
        if (attempt < 4) {
            DEV_WRN(dev, "Device ID read failed (%d), retrying", ret);
            k_sleep(K_MSEC(20));
        }
    }
    if (ret < 0) {
        DEV_ERR(dev, "Device ID check failed");
        goto unlock;
    }

    /*
     * CONTROL and STATUS are read-only for the secondary interface, so take
     * the format and flags the primary side has configured instead of
     * trying to change them.
     */
    ret = au1901c_read_reg(dev, AU1901C_REG_CONTROL, &ctrl);
    if (ret < 0) {
        DEV_ERR(dev, "Failed to read control register");
        goto unlock;
    }

    data->bcd_mode = (ctrl & AU1901C_CONTROL_DM) == 0U;
    data->hour12_mode = (ctrl & AU1901C_CONTROL_HF) == 0U;
    data->time_write_permission = (ctrl & AU1901C_CONTROL_TWO) == 0U;

    DEV_INF(dev, "CONTROL=0x%02x (st=%u dm=%u hf=%u dsm=%u aie=%u ofie=%u cie=%u two=%u)",
        ctrl, (ctrl & AU1901C_CONTROL_ST) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_DM) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_HF) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_DSM) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_AIE) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_OFIE) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_CIE) ? 1U : 0U,
        (ctrl & AU1901C_CONTROL_TWO) ? 1U : 0U);
    DEV_INF(dev, "Using %s calendar, %s format; time-register write permission: %s",
        data->bcd_mode ? "BCD" : "binary",
        data->hour12_mode ? "12-hour" : "24-hour",
        data->time_write_permission ? "yes (TWO=0, this secondary I2C port)"
                        : "no (TWO=1, owned by the primary I2C)");

    if (ctrl & AU1901C_CONTROL_ST) {
        DEV_WRN(dev, "RTC is stopped (ST=1), the time will not advance");
    }

    ret = au1901c_read_reg(dev, AU1901C_REG_STATUS, &status);
    if (ret < 0) {
        DEV_ERR(dev, "Failed to read status register");
        goto unlock;
    }

    DEV_INF(dev, "STATUS=0x%02x (af=%u of=%u rtcf=%u cif=%u bvl=%u)",
        status, (status & AU1901C_STATUS_AF) ? 1U : 0U,
        (status & AU1901C_STATUS_OF) ? 1U : 0U,
        (status & AU1901C_STATUS_RTCF) ? 1U : 0U,
        (status & AU1901C_STATUS_CIF) ? 1U : 0U,
        (unsigned int)(status & AU1901C_STATUS_BVL_MASK));
    if (status & (AU1901C_STATUS_OF | AU1901C_STATUS_RTCF)) {
        DEV_WRN(dev, "Oscillator/power fail flagged, time may be invalid");
    }

    DEV_INF(dev, "AU1901C RTC initialized successfully");

    ret = 0;

unlock:
    k_mutex_unlock(&data->lock);

    return ret;
}

static const struct rtc_driver_api rtc_au1901c_api = {
    .set_time = rtc_au1901c_set_time,
    .get_time = rtc_au1901c_get_time,
};

#define RTC_AU1901C_DEVICE(n)                                             \
    static const struct rtc_au1901c_config rtc_au1901c_config_##n = {     \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                   \
    };                                                                    \
                                                                          \
    static struct rtc_au1901c_data rtc_au1901c_data_##n;                  \
                                                                          \
    DEVICE_DT_INST_DEFINE(n, rtc_au1901c_init, NULL,                      \
                          &rtc_au1901c_data_##n,                          \
                          &rtc_au1901c_config_##n,                        \
                          POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,          \
                          &rtc_au1901c_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_AU1901C_DEVICE)
