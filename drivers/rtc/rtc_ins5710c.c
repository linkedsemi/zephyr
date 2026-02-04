/**
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Linkedsemi Corporation
 * 
 * @file rtc_ins5710c.c
 * @brief DAPU INS5710C I2C RTC driver
 * @author ZouWei <wzou@linkedsemi.com>
 * @date 2026-02-01
 * @note I2C 7-bit address: 0x32 (datasheet [0110010*]) Reserved register bits are not specially handled
 */

#define DT_DRV_COMPAT dapu_ins5710c

#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(rtc_ins5710c, LOG_LEVEL_DBG);

/* I2C 7-bit address (datasheet Table 12: [0110010*]) */
#define INS5710C_I2C_ADDR		0x32

/* Register definitions - Basic Time and Calendar (0x00-0x0F) */
#define INS5710C_REG_RAM		0x07	/* RAM User data */

/* Extended Register Group 1 (0x10-0x1F) - 0x10-0x16 mirror 0x00-0x06 */
#define INS5710C_REG_SEC		0x10	/* Seconds BCD 0-59 */
#define INS5710C_REG_MIN		0x11	/* Minutes BCD 0-59 */
#define INS5710C_REG_HOUR		0x12	/* Hours BCD 0-23 */
#define INS5710C_REG_WEEK		0x13	/* Week bit0-6 */
#define INS5710C_REG_DAY		0x14	/* Day BCD 1-31 */
#define INS5710C_REG_MONTH		0x15	/* Month BCD 1-12 */
#define INS5710C_REG_YEAR		0x16	/* Year BCD 0-99 */
#define INS5710C_REG_TEMP		0x17	/* Temperature value (Table 8) */

/*  BIT MASKS (Per Datasheet Section 6.2.1)  */
#define SEC_BITS    GENMASK(6, 0)  /* bit7 = Reserved (init to 0) */
#define MIN_BITS    GENMASK(6, 0)  /* bit7 = Reserved */
#define HOUR_BITS   GENMASK(5, 0)  /* bit6-7 = Reserved (24h mode) */
#define DATE_BITS   GENMASK(5, 0)  /* bit6-7 = Reserved */
#define MONTH_BITS  GENMASK(4, 0)  /* bit5-7 = Reserved */
#define YEAR_BITS   GENMASK(7, 0)  /* Full byte (00-99) */
#define WDAY_BITS   GENMASK(6, 0)  /* bit0-6: Sun-Sat (single-bit), bit7=Reserved */

/* 0x08-0x0C: Reserved */
#define INS5710C_REG_EXT		0x1D	/* Extension Register (TEST, FSEL) */
#define INS5710C_REG_FLAG		0x1E	/* Flag Register (VLF) */
#define INS5710C_REG_CTRL		0x1F	/* Control Register (RESET) */

/* Extended Register Group 2 (0x20-0x30) */
#define INS5710C_REG_DEVICE_ID	0x20	/* Device ID + Version */
#define INS5710C_REG_CTRL1		0x21	/* Control Register 1, datasheet: must be 0x80 */
#define INS5710C_REG_CTRL1_VAL	0x80	/* Datasheet requires high bit must be 0x80 */
#define INS5710C_REG_SUBSEC		0x27	/* Subsecond time 1/16s */
#define INS5710C_REG_FOUT_CTRL	0x28	/* FOUT Output Control (FOE[1:0]) */

/* Register bit definitions */
#define INS5710C_FLAG_VLF		BIT(1)	/* Voltage Low Flag (Table 8: 0x0E/1E) */
#define INS5710C_CTRL_RESET		BIT(0)	/* RESET bit (Table 8: 0x0F/1F) */

/* Extension Register (0x0D/1D) */
#define INS5710C_EXT_TEST			BIT(7)	/* TEST bit, must be 0 */
#define INS5710C_EXT_FSEL_MASK		(BIT(3) | BIT(2))	/* FSEL[1:0] */
#define INS5710C_EXT_FSEL_SHIFT		2


/* FOUT Control (0x28) */
#define INS5710C_FOUT_FOE_MASK		(BIT(7) | BIT(6))	/* FOE[1:0] */
#define INS5710C_FOUT_FOE_SHIFT		6
#define INS5710C_FOUT_ENABLE		(BIT(7) | BIT(6))	/* 11b = Output enabled */
#define INS5710C_FOUT_DISABLE		0			/* 00b = No output */

/* FOUT Frequency Selection */
#define INS5710C_FSEL_32768HZ		0x00
#define INS5710C_FSEL_1024HZ		0x01
#define INS5710C_FSEL_1HZ			0x02

/* Device ID (Table 9: 0x20) */
#define INS5710C_VENDOR_ID			0x0D	/* VendorID[3:0] = 1101b = Dh */
#define INS5710C_DEVID_VENDER_SHIFT	4
#define INS5710C_DEVID_VER_MASK		0x0F
#define INS5710C_VENDOR_ID_MASK		0x0F

/* Total bytes for continuous write: 0x00 ~ 0x06 total 7 bytes */
#define INS5710C_TIME_REG_BUF_LEN  	7

/* Driver configuration structure */
struct rtc_ins5710c_config {
	struct i2c_dt_spec i2c;
};

/* Driver runtime data structure */
struct rtc_ins5710c_data {
	struct rtc_time current_time;
};

// Determine whether it is a leap year
static inline bool is_leap_year(int year) {
    year += 1900;  // The year of RTC is the year minus 1900
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

/* INPUT VALIDATION (HARDWARE RANGE CHECK) */
static bool ins5710c_is_time_valid(const struct device *dev, const struct rtc_time *tm)
{
	if(tm->tm_year < 100 || tm->tm_year > 199)
	{
		//Validate year: INS5710C supports ONLY 2000-2099 (tm_year 100-199)
		DEV_ERR(dev, "Year out of range (2000-2099): %d", tm->tm_year + 1900);
		return false;
	}

	int max_day = 31;
    switch (tm->tm_mon + 1) {
        case 4: case 6: case 9: case 11: max_day = 30; break;
        case 2: max_day = is_leap_year(tm->tm_year) ? 29 : 28; break;
		default: max_day = 31;
        break;
    }

    /* Validate all time fields (defense-in-depth) */
    if (tm->tm_mon < 0 || tm->tm_mon > 11 ||
        tm->tm_mday < 1 || tm->tm_mday > max_day ||
        tm->tm_hour < 0 || tm->tm_hour > 23 ||
        tm->tm_min < 0 || tm->tm_min > 59 ||
        tm->tm_sec < 0 || tm->tm_sec > 59 ||
        tm->tm_wday < 0 || tm->tm_wday > 6)		
		{
		DEV_ERR(dev, "Invalid time field(s): "
                "Y%04d M%d D%d %02d:%02d:%02d wday=%d",
                tm->tm_year, tm->tm_mon, tm->tm_mday,
                tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_wday);
			return false;
		}
	return true;
}

/**
 * @brief Read register from INS5710C
 */
static int ins5710c_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct rtc_ins5710c_config *config = dev->config;
	int ret;
	
	ret = i2c_write_read_dt(&config->i2c, &reg, 1, val, 1);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to read reg 0x%02x: %d", reg, ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Write register to INS5710C
 */
static int ins5710c_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct rtc_ins5710c_config *config = dev->config;
	uint8_t buf[2] = {reg, val};
	int ret;

	ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
	if (ret < 0) {
		DEV_ERR(dev, "Failed to write reg 0x%02x: %d", reg, ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Read multiple registers (auto-increment)
 */
static int ins5710c_read_regs(const struct device *dev, uint8_t start_reg,
			      uint8_t *buf, uint8_t len)
{
	const struct rtc_ins5710c_config *config = dev->config;
	int ret;

	ret = i2c_write_read_dt(&config->i2c, &start_reg, 1, buf, len);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to read regs from 0x%02x: %d", start_reg, ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Write multiple registers (auto-increment)
 */
static int ins5710c_write_regs(const struct device *dev, uint8_t start_reg,
			       const uint8_t *buf, uint8_t len)
{
	const struct rtc_ins5710c_config *config = dev->config;
	uint8_t tx_buf[INS5710C_TIME_REG_BUF_LEN + 1]; /* Max 7 bytes time + 1 reg addr */
	int ret;

	if (len > INS5710C_TIME_REG_BUF_LEN) {
		return -EINVAL;
	}

	tx_buf[0] = start_reg;
	memcpy(&tx_buf[1], buf, len);

	ret = i2c_write_dt(&config->i2c, tx_buf, len + 1);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to write regs from 0x%02x: %d", start_reg, ret);
		return ret;
	}

	return 0;
}

/**
 * @brief Check and clear VLF flag (datasheet: Set when voltage below 1.6V)
 */
static int ins5710c_check_vlf(const struct device *dev)
{
	uint8_t flag;
	int ret;

	ret = ins5710c_read_reg(dev, INS5710C_REG_FLAG, &flag);
	if (ret < 0) {
		return ret;
	}

	if (flag & INS5710C_FLAG_VLF) {
		DEV_WRN(dev, "VLF detected, RTC may have lost time");
		/* Datasheet: VLF bit can only be cleared by writing 0 */
		ret = ins5710c_write_reg(dev, INS5710C_REG_FLAG, flag & ~INS5710C_FLAG_VLF);
		if (ret < 0) {
			return ret;
		}
		return 1; /* VLF was set */
	}

	return 0; /* VLF not set */
}

/**
 * @brief Reset RTC (datasheet: Control Register bit0)
 */
__maybe_unused
static int ins5710c_reset(const struct device *dev)
{
	int ret;

	/* Set RESET bit */
	ret = ins5710c_write_reg(dev, INS5710C_REG_CTRL, INS5710C_CTRL_RESET);
	if (ret < 0) {
		return ret;
	}

	/* Clear RESET bit */
	ret = ins5710c_write_reg(dev, INS5710C_REG_CTRL, 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Read device ID and verify (datasheet Table 9: 0x20)
 */
static int ins5710c_read_device_id(const struct device *dev)
{
	uint8_t dev_id;
	uint8_t vendor_id;
	uint8_t version;
	int ret;

	ret = ins5710c_read_reg(dev, INS5710C_REG_DEVICE_ID, &dev_id);
	if (ret < 0) {
		return ret;
	}

	vendor_id = (dev_id >> INS5710C_DEVID_VENDER_SHIFT) & INS5710C_VENDOR_ID_MASK;
	version = dev_id & INS5710C_DEVID_VER_MASK;

	DEV_INF(dev, "Device ID: 0x%02x, Vendor: 0x%x, Version: %d",
		dev_id, vendor_id, version);

	/* Datasheet: VendorID[3:0] = 1101b = 0x0D */
	if (vendor_id != INS5710C_VENDOR_ID) {
		DEV_ERR(dev, "Invalid vendor ID: 0x%x, expected 0x%x",
			vendor_id, INS5710C_VENDOR_ID);
		return -ENODEV;
	}

	return 0;
}

/**
 * @brief Configure FOUT frequency (datasheet Table 8: FSEL[1:0])
 */
__maybe_unused
static int ins5710c_set_fout_freq(const struct device *dev, uint8_t freq_sel)
{
	uint8_t ext;
	int ret;

	if (freq_sel > 3) {
		return -EINVAL;
	}
	
	ret = ins5710c_read_reg(dev, INS5710C_REG_EXT, &ext);
	if (ret < 0) {
		return ret;
	}

	/* Clear FSEL bits, set new value */
	ext &= ~INS5710C_EXT_FSEL_MASK;
	ext |= (freq_sel << INS5710C_EXT_FSEL_SHIFT);
	/* Ensure TEST bit7 = 0 (datasheet requirement) */
	ext &= ~INS5710C_EXT_TEST;

	ret = ins5710c_write_reg(dev, INS5710C_REG_EXT, ext);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Enable/disable FOUT output)
 */
__maybe_unused
static int ins5710c_set_fout_enable(const struct device *dev, bool enable)
{
	uint8_t fout_ctrl;
	int ret;

	ret = ins5710c_read_reg(dev, INS5710C_REG_FOUT_CTRL, &fout_ctrl);
	if (ret < 0) {
		return ret;
	}

	fout_ctrl &= ~INS5710C_FOUT_FOE_MASK;
	if (enable) {
		fout_ctrl |= INS5710C_FOUT_ENABLE; /* 11b */
	} else {
		fout_ctrl |= INS5710C_FOUT_DISABLE; /* 00b */
	}

	ret = ins5710c_write_reg(dev, INS5710C_REG_FOUT_CTRL, fout_ctrl);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/**
 * @brief Read temperature (datasheet: Temperature formula)
 */
__maybe_unused
static int ins5710c_read_temperature(const struct device *dev, int32_t *temp_milli)
{
	uint8_t temp_raw;
	int ret;

	ret = ins5710c_read_reg(dev, INS5710C_REG_TEMP, &temp_raw);
	if (ret < 0) {
		return ret;
	}

	/* Datasheet formula: Temperature[°C] = (TEMP[7:0] * 2 - 187.19) / 3.218 */
	/* Convert to integer calculation to avoid floating point */
	*temp_milli = ((temp_raw * 2000) - 187190) / 3218;

	DEV_DBG(dev, "Temperature raw: 0x%02x, calculated: %d.%03d C",
		temp_raw, *temp_milli / 1000, *temp_milli % 1000);

	return 0;
}

/**
 * @brief RTC API: Set time
 */
static int rtc_ins5710c_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	uint8_t buf[INS5710C_TIME_REG_BUF_LEN];
	int ret;
	if (!ins5710c_is_time_valid(dev, timeptr))
	{
		DEV_WRN(dev, "RTC time is invalid, please reconfigure\n");
		return -EINVAL;
	}

	/* Convert to BCD and package (datasheet Table 10: WEEK is one-hot encoding) */
	buf[0] = bin2bcd(timeptr->tm_sec) & SEC_BITS;   /* SEC, bit7=0 */
	buf[1] = bin2bcd(timeptr->tm_min) & MIN_BITS;   /* MIN, bit7=0 */
	buf[2] = bin2bcd(timeptr->tm_hour) & HOUR_BITS;  /* HOUR, bit7-6=0 */
	/* WEEK: one-hot encoding (Table 10) */
	buf[3] = (1 << timeptr->tm_wday) & WDAY_BITS;
	buf[4] = bin2bcd(timeptr->tm_mday) & DATE_BITS;  /* DAY, bit7-6=0 */
	buf[5] = bin2bcd(timeptr->tm_mon + 1) & MONTH_BITS; /* MONTH, bit7-5=0 */
	buf[6] = bin2bcd(timeptr->tm_year - 100) & YEAR_BITS;   /* YEAR */

	/* Write 0x00-0x06 in one operation */
	ret = ins5710c_write_regs(dev, INS5710C_REG_SEC, buf, 7);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to set time");
		return ret;
	}

	DEV_INF(dev, "Time set to %04d-%02d-%02d %02d:%02d:%02d",
		timeptr->tm_year + 1900, timeptr->tm_mon + 1, timeptr->tm_mday,
		timeptr->tm_hour, timeptr->tm_min, timeptr->tm_sec);

	return 0;
}

/**
 * @brief RTC API: Get time
 */
static int rtc_ins5710c_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	uint8_t buf[INS5710C_TIME_REG_BUF_LEN];
	int ret;

	/* Read 0x00-0x06 in one operation */
	ret = ins5710c_read_regs(dev, INS5710C_REG_SEC, buf, INS5710C_TIME_REG_BUF_LEN);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to get time");
		return ret;
	}

	/* BCD to binary conversion */
	timeptr->tm_sec = bcd2bin(buf[0] & SEC_BITS);
	timeptr->tm_min = bcd2bin(buf[1] & MIN_BITS);
	timeptr->tm_hour = bcd2bin(buf[2] & HOUR_BITS);
    uint8_t week_bits = buf[3] & WDAY_BITS;

	timeptr->tm_wday = 0;
	for (uint8_t i = 0; i < 7; i++) {
		if (week_bits & (1 << i)) {
			timeptr->tm_wday = i;
			break;
		}
	}

    // Use __builtin_ctz to find position of set bit (0~6)
	timeptr->tm_wday = __builtin_ctz(week_bits);
	timeptr->tm_mday = bcd2bin(buf[4] & DATE_BITS);
	timeptr->tm_mon = bcd2bin(buf[5] & MONTH_BITS) - 1; /* tm_mon 0-11 */
	timeptr->tm_year = bcd2bin(buf[6] & YEAR_BITS) + 100;    /* 2000-2099 */
	timeptr->tm_isdst = -1;
	
	/* Validate time validity */
	if (!ins5710c_is_time_valid(dev, timeptr))
	{
		DEV_WRN(dev, "Invalid time read from RTC\n");
		return -EINVAL;
	}

	return 0;
}



/**
 * @brief Device initialization
 * Datasheet requirement: All registers must be initialized after power-on initialization
 */
static int rtc_ins5710c_init(const struct device *dev)
{
	const struct rtc_ins5710c_config *config = dev->config;

	int ret;

	/* Check I2C bus ready */
	if (!i2c_is_ready_dt(&config->i2c)) {
		DEV_ERR(dev, "I2C bus not ready");
		return -ENODEV;
	}

	/* Datasheet: Oscillation requires above 2.5V after power-on, wait for stabilization */
	k_msleep(200);

	/* Read and verify Device ID */
	ret = ins5710c_read_device_id(dev);
	if (ret < 0) {
		DEV_ERR(dev, "Device ID check failed");
		return ret;
	}

	/* Check VLF flag (datasheet: Set when voltage below 1.6V) */
	ret = ins5710c_check_vlf(dev);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to check VLF");
		return ret;
	}
	else if (ret > 0) {
		DEV_WRN(dev, "VLF was set, time may be invalid");
		ret = 0;
	}

	/* 
	 * Datasheet Table 9 (0x21 Control Register 1):
	 * "Reserved bits: Must be 0x80"
	 * Default value is 0x80, but datasheet requires writing 0x80
	 */
	ret = ins5710c_write_reg(dev, INS5710C_REG_CTRL1, INS5710C_REG_CTRL1_VAL);
	if (ret < 0) {
		DEV_ERR(dev, "Failed to set Control Register 1");
		return ret;
	}

	/* Disable FOUT output (default) */
	ret = ins5710c_set_fout_enable(dev, false);
	if (ret < 0) {
		DEV_WRN(dev, "Failed to disable FOUT");
	}

	/* Configure FOUT frequency to 32768Hz (default) */
	ret = ins5710c_set_fout_freq(dev, INS5710C_FSEL_32768HZ);
	if (ret < 0) {
		DEV_WRN(dev, "Failed to set FOUT frequency");
	}

	DEV_INF(dev, "INS5710C RTC initialized successfully");

	return 0;
}

/* RTC driver API */
static const struct rtc_driver_api rtc_ins5710c_api = {
	.set_time = rtc_ins5710c_set_time,
	.get_time = rtc_ins5710c_get_time,
#if defined(CONFIG_RTC_ALARM)
	alarm_get_supported_fields = z_impl_sys_not_supported,
	alarm_set_time = z_impl_sys_not_supported,
	alarm_get_time = z_impl_sys_not_supported,
	alarm_is_pending = z_impl_sys_not_supported,
	alarm_set_callback = z_impl_sys_not_supported,
#endif /* CONFIG_RTC_ALARM */
#if defined(CONFIG_RTC_UPDATE)
	update_set_callback = z_impl_sys_not_supported,
#endif /* CONFIG_RTC_UPDATE */
#if defined(CONFIG_RTC_CALIBRATION)
	set_calibration = z_impl_sys_not_supported,
	get_calibration = z_impl_sys_not_supported,
#endif /* CONFIG_RTC_CALIBRATION */
};

#define RTC_INS5710C_DEVICE(n)						\
	static struct rtc_ins5710c_data rtc_ins5710c_data_##n;		\
									\
	static const struct rtc_ins5710c_config rtc_ins5710c_config_##n = { \
		.i2c = I2C_DT_SPEC_INST_GET(n),				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      rtc_ins5710c_init,			\
			      NULL,					\
			      &rtc_ins5710c_data_##n,			\
			      &rtc_ins5710c_config_##n,			\
			      POST_KERNEL,				\
			      CONFIG_RTC_INIT_PRIORITY,			\
			      &rtc_ins5710c_api);

/* Iterate all instances with status="okay" in devicetree */
DT_INST_FOREACH_STATUS_OKAY(RTC_INS5710C_DEVICE)