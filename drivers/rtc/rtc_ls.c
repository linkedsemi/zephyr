#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <reg_rtcv2_type.h>
#include <soc_clock.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

LOG_MODULE_REGISTER(rtc_ls, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT linkedsemi_ls_rtc
#define RTC_CTRL_ENABLE_MASK         BIT(0)  // RTC Enable
#define RTC_CTRL_CALIB_EN_MASK       BIT(1)  // Calibration Enable
#define RTC_CTRL_ALARM_EN_MASK       BIT(2)  // Alarm Enable
#define RTC_CTRL_SET_TGGL_MASK       BIT(3)  // Time setting trigger toggle bit
#define RTC_CTRL_INTR_CLR_MASK       BIT(4)  // Clear interrupt flag
#define RTC_CALIB_MAX_PPB        1000000000  // The maximum calibration value，1 ppb = 10 billionths


struct rtc_ls_config {
    struct ls_clk_cfg clk_cfg;
    uintptr_t base;
    void (*irq_config_func)(const struct device *dev);
    uint32_t cyc_1hz;
    uint32_t calib_cyc;
    bool calib_enable;
};

struct rtc_ls_data {
    rtc_alarm_callback alarm_cb;
    void *alarm_cb_user_data;
    struct k_spinlock lock;

};

#define RTC_REGS(dev) \
	((volatile  reg_rtc_t *)((const struct rtc_ls_config *)dev->config)->base)

static void rtc_cycle_config(const struct device *dev, uint32_t cyc_1hz, uint32_t calib_cyc, bool calib_en){
    RTC_REGS(dev)->CALIB = ((calib_cyc & 0xFF) << 24) | ((cyc_1hz - 1) & 0xFFFFF);

    if (calib_en)
        RTC_REGS(dev)->CTRL |= RTC_CTRL_CALIB_EN_MASK;
    else
        RTC_REGS(dev)->CTRL &= ~RTC_CTRL_CALIB_EN_MASK;
}

static void rtc_irq_handler(const void *arg){
    const struct device *dev = (const struct device *)arg;
    uint32_t intr = RTC_REGS(dev)->INTR;
    struct rtc_ls_data *data = dev->data;

    if (intr) {
        LOG_INF(">>>> Alarm interrupt!\n");
        RTC_REGS(dev)->CTRL |= RTC_CTRL_INTR_CLR_MASK;
        RTC_REGS(dev)->CTRL &= ~RTC_CTRL_INTR_CLR_MASK;

        RTC_REGS(dev)->CTRL &= ~RTC_CTRL_ALARM_EN_MASK;//Alarm Disable

        if (data->alarm_cb) {
            data->alarm_cb(dev, 0, data->alarm_cb_user_data);
        }
        LOG_INF("================================\n");
    }
}

static int rtc_ls_get_time(const struct device *dev, struct rtc_time *tm){
    struct rtc_ls_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    uint32_t cur1_pre1;
    uint32_t cur0_pre1;
    uint32_t cur1;
    uint32_t cur0;

    cur1_pre1 = RTC_REGS(dev)->CURCAL;
    do{
       cur0_pre1 = RTC_REGS(dev)->CURTIME;
       cur1 = RTC_REGS(dev)->CURCAL;
        if(cur1 == cur1_pre1)
        {
            cur0 = cur0_pre1 ;
            break;
        }else
        {
            cur1_pre1 = cur1;
        }
    }while(1);

    k_spin_unlock(&data->lock, key);

    tm->tm_sec  =((cur0 >>  0) & 0x3F) ;
    tm->tm_min  = (cur0 >>  8) & 0x3F;
    tm->tm_hour = (cur0 >> 16) & 0x1F;
    tm->tm_mday = (cur0 >> 24) & 0x1F;
    // The hardware return value is between 1 and 60. It needs to be subtracted by 1.
    if (tm->tm_sec > 0) tm->tm_sec -= 1;
    if (tm->tm_min > 0) tm->tm_min -= 1;
    if (tm->tm_hour> 0) tm->tm_hour-=1;

    tm->tm_year = (cur1 >> 8) & 0xFF;
    tm->tm_mon  = (cur1 >> 4) & 0x0F;
    tm->tm_wday = (cur1 >> 0) & 0x07;

    return 0;
}

// Determine whether it is a leap year
static inline bool is_leap_year(int year) {
    year += 1900;  // The year of RTC is the year minus 1900
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

//  Verify whether the date is valid
static bool is_valid_time(const struct rtc_time *t) {
    if (t->tm_mon < 1 || t->tm_mon > 12) return false;
    if (t->tm_wday < 1 || t->tm_wday > 7) return false;
    if (t->tm_hour > 23 || t->tm_min > 59 || t->tm_sec > 59) return false;

    int max_day = 31;
    switch (t->tm_mon) {
        case 4: case 6: case 9: case 11: max_day = 30; break;
        case 2: max_day = is_leap_year(t->tm_year) ? 29 : 28; break;
    }
    if (t->tm_mday < 1 || t->tm_mday > max_day) return false;

    return true;
}

/*
 * set time，rtc_ctrl(0x000)
 * When setting the year, month, and day related registers, you must first set rtc_en to 0,
 * and then set the corresponding year, month, and day. Setting the year, month, and day is not effective* immediately.
 * You need to toggle the rtc_set_tggl register, and then set rtc_en to 1,
 * "Note: Before setting the time, you must clear CTRL.rtc_en = 0; set SET0/SET1; raise CTRL.rtc_set_tggl; and then raise CTRL.rtc_en = 1
*/
static int rtc_ls_set_time(const struct device *dev, const struct rtc_time *tm) {
    struct rtc_ls_data *data = dev->data;
    if (!is_valid_time(tm)) {
    LOG_INF("The set time is illegal. The operation has been aborted!\n");
    return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    RTC_REGS(dev)->CTRL &= ~RTC_CTRL_ENABLE_MASK;
    RTC_REGS(dev)->CTRL |= RTC_CTRL_INTR_CLR_MASK;
    RTC_REGS(dev)->CTRL &= ~RTC_CTRL_INTR_CLR_MASK;

    RTC_REGS(dev)->TIME  =
        ((tm->tm_mday & 0x1F) << 24) |
        (((tm->tm_hour+1) & 0x1F) << 16) |
        (((tm->tm_min+1)  & 0x3F) << 8)  |
        (((tm->tm_sec+1)  & 0x3F) << 0);

    RTC_REGS(dev)->CAL   =
        ((tm->tm_year & 0xFF) << 8) |
        ((tm->tm_mon  & 0x0F) << 4) |
        ((tm->tm_wday & 0x07) << 0);

    RTC_REGS(dev)->CTRL ^= RTC_CTRL_SET_TGGL_MASK;
    k_busy_wait(10);

    RTC_REGS(dev)->CTRL |= RTC_CTRL_ENABLE_MASK;
    k_spin_unlock(&data->lock, key);

    return 0;
}

#if defined(CONFIG_RTC_ALARM)
static int rtc_ls_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask, const struct rtc_time *tm){
    ARG_UNUSED(id);
    ARG_UNUSED(mask);

    struct rtc_ls_data *data = dev->data;

    if (!is_valid_time(tm)) {
    LOG_INF("The set time is illegal. The operation has been aborted!\n");
    return -EINVAL;
    }

    LOG_INF("alarm_set_time(): tm->hour=%d min=%d sec=%d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    RTC_REGS(dev)->CTRL &= ~RTC_CTRL_ALARM_EN_MASK;
    RTC_REGS(dev)->TIME = ((tm->tm_mday & 0x1F) << 24) |
                (((tm->tm_hour+1) & 0x1F) << 16) |
                (((tm->tm_min+1)  & 0x3F) << 8)  |
                (((tm->tm_sec+1)  & 0x3F) << 0);
    RTC_REGS(dev)->CAL = ((tm->tm_year & 0xFF) << 8) |
                ((tm->tm_mon  & 0x0F) << 4) |
                ((tm->tm_wday & 0x07) << 0);

    RTC_REGS(dev)->CTRL |= RTC_CTRL_ALARM_EN_MASK;
    k_spin_unlock(&data->lock, key);

    LOG_INF("rtc_ctrl     = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->CTRL, RTC_REGS(dev)->CTRL);
    LOG_INF("rtc_calib    = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->CALIB, RTC_REGS(dev)->CALIB);
    LOG_INF("rtc_set_tgt0 = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->TIME, RTC_REGS(dev)->TIME);
    LOG_INF("rtc_set_tgt1 = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->CAL, RTC_REGS(dev)->CAL);
    LOG_INF("rtc_cur0     = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->CURTIME, RTC_REGS(dev)->CURTIME);
    LOG_INF("rtc_cur1     = %08x[%08x]", (uint32_t)&RTC_REGS(dev)->CURCAL, RTC_REGS(dev)->CURCAL);
    LOG_INF("rtc_intr     = %08x[%08x]\n", (uint32_t)&RTC_REGS(dev)->INTR, RTC_REGS(dev)->INTR);

    return 0;
}

static int rtc_ls_alarm_set_callback(const struct device *dev, uint16_t id, rtc_alarm_callback callback, void *user_data){
    ARG_UNUSED(id);
    struct rtc_ls_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->alarm_cb = callback;
    data->alarm_cb_user_data = user_data;
    k_spin_unlock(&data->lock, key);
    return 0;
}

static int rtc_ls_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask, struct rtc_time *tm){
    ARG_UNUSED(id);
    ARG_UNUSED(mask);

    struct rtc_ls_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    uint32_t tgt0 = RTC_REGS(dev)->TIME;
    uint32_t tgt1 = RTC_REGS(dev)->CAL;

    k_spin_unlock(&data->lock, key);

    tm->tm_sec  = ((tgt0 >>  0) & 0x3F);
    tm->tm_min  = ((tgt0 >>  8) & 0x3F);
    tm->tm_hour = ((tgt0 >> 16) & 0x1F);
    tm->tm_mday = ((tgt0 >> 24) & 0x1F);

    if (tm->tm_sec > 0)  tm->tm_sec  -= 1;
    if (tm->tm_min > 0)  tm->tm_min  -= 1;
    if (tm->tm_hour > 0) tm->tm_hour -= 1;

    tm->tm_year = (tgt1 >> 8) & 0xFF;
    tm->tm_mon  = (tgt1 >> 4) & 0x0F;
    tm->tm_wday = (tgt1 >> 0) & 0x07;

    return 0;
}


/*
 *Check if the interruption has been triggered but not yet cleared
 * Query whether there are any suspended (not yet triggered) alarms at present
*/

static int rtc_ls_alarm_is_pending(const struct device *dev, uint16_t id){

    ARG_UNUSED(id); // If the RTC has only one alarm, the ID can be ignored
    if(RTC_REGS(dev)->CTRL & RTC_CTRL_ALARM_EN_MASK){

        return 1; // The alarm is pending, not triggered.
    }
    else{

        return 0; // The alarm is triggered or not pending
    }
}

static int rtc_ls_alarm_get_supported_fields(const struct device *dev, uint16_t id, uint16_t *mask){
    ARG_UNUSED(dev);
    ARG_UNUSED(id); // LS101x only one alarm path

    //  The hardware only supports complete matching of year, month, day, hour, minute and second.
    uint16_t internal_mask = RTC_ALARM_TIME_MASK_SECOND |
                             RTC_ALARM_TIME_MASK_MINUTE |
                             RTC_ALARM_TIME_MASK_HOUR   |
                             RTC_ALARM_TIME_MASK_MONTHDAY |
                             RTC_ALARM_TIME_MASK_MONTH  |
                             RTC_ALARM_TIME_MASK_YEAR;
     *mask = internal_mask;

    return 0;
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int rtc_ls_update_set_callback(const struct device *dev, rtc_update_callback callback, void *user_data){
    ARG_UNUSED(dev);
	ARG_UNUSED(callback);
	ARG_UNUSED(user_data);
	return -ENOTSUP;
}
#endif /* CONFIG_RTC_UPDATE */



/*
 *  Function description: Set RTC calibration, with the unit being ppb (parts per billion).
 *  @param dev Device instance
 *  @param calibration Calibration value
 * (unit: ppb, positive value indicates an increase in frequency, negative value indicates a decrease in frequency)
 *  @return 0 secessful
 *  eg： set calibration = 1000，increase 1000 ppb。
 *       set calibration = -1000，decrease 1000 ppb。
 *  main.c eg：int ret = rtc_ls_set_calibration(dev, 1);  // increase 1 ppb
 * Premise: First, calculate the ppb value based on the deviation of the crystal oscillator by calling the function
 * "rtc_calibration_from_frequency(uint32_t frequency)".
 * Then, pass the calculated ppb value into this function.
 *
*/
#if defined(CONFIG_RTC_CALIBRATION)

int32_t floor_custom_divisor(int32_t num, int32_t divisor) {
    // integer division
    int32_t result = num / divisor;

    // If it is a negative number and there is a remainder, round down.
    if (num < 0 && num % divisor != 0) {
        result -= 1;
    }

    return result;
}

static int rtc_ls_set_calibration(const struct device *dev, int32_t calibration){
    const struct rtc_ls_config *cfg = dev->config;

    if (calibration>=0){

        int64_t calibration_minute = (int64_t)calibration*60;
        int32_t calib_cyc =  (calibration_minute/RTC_CALIB_MAX_PPB);

        rtc_cycle_config(dev, cfg->cyc_1hz, calib_cyc, true);


    }
    else{

        uint32_t cyc_1hz_m1_act = cfg->cyc_1hz+(floor_custom_divisor(calibration,RTC_CALIB_MAX_PPB));
        // LOG_INF("cyc_1hz_m1_act: %d \n",cyc_1hz_m1_act);

        int64_t calib_cyc_pre = (calibration%RTC_CALIB_MAX_PPB);

        uint32_t calib_cyc_act = 1*60 + (calib_cyc_pre*60/RTC_CALIB_MAX_PPB);
        // LOG_INF("calib_cyc_act: %d \n",calib_cyc_act);

        rtc_cycle_config(dev, cyc_1hz_m1_act , calib_cyc_act, true);

    }

    return 0;
}

static int rtc_ls_get_calibration(const struct device *dev, int32_t *calibration){
    const struct rtc_ls_config *cfg = dev->config;

    // Ensure that the calibration pointer is not empty
    if (calibration == NULL) {
        return -EINVAL;
    }

    // Check whether the calibration function is supported
    if (!(RTC_REGS(dev)->CTRL & RTC_CTRL_CALIB_EN_MASK)) {
        *calibration = 0;
         return 0;
    }

    uint32_t calib_cyc = RTC_REGS(dev)->CALIB >> 24;  // read calib_cyc (31:24)
    uint32_t cyc_1hz_m1 = RTC_REGS(dev)->CALIB & 0xFFFFF; // read cyc_1hz_m1  (19:0)
    *calibration = ((uint64_t)(cyc_1hz_m1+1 - cfg->cyc_1hz))*RTC_CALIB_MAX_PPB+ ((uint64_t)calib_cyc) *RTC_CALIB_MAX_PPB/60;  // 计算校准值 (ppb)
    return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

static int rtc_ls_init(const struct device *dev){

    const struct rtc_ls_config *cfg = dev->config;
    const struct device *clk_dev =cfg->clk_cfg.cctl_dev;
    struct rtc_ls_data *data = dev->data;

    data->alarm_cb = NULL;
    data->alarm_cb_user_data = NULL;

    RTC_REGS(dev)->CTRL &= ~RTC_CTRL_ALARM_EN_MASK;//Alarm Disable
    if (device_is_ready(clk_dev)) {

        clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->clk_cfg);
    } else {

        LOG_ERR("Clock control device not ready!");
    }

    cfg->irq_config_func(dev);
    rtc_cycle_config(dev, cfg->cyc_1hz, cfg->calib_cyc,true);
    RTC_REGS(dev)->CTRL |= RTC_CTRL_ENABLE_MASK;
	return 0;
}

/**
 * @brief RTC driver API
 */
static const struct rtc_driver_api rtc_ls_api = {
    .get_time = rtc_ls_get_time,
    .set_time = rtc_ls_set_time,

#if defined(CONFIG_RTC_ALARM)
    .alarm_get_supported_fields = rtc_ls_alarm_get_supported_fields,
    .alarm_set_time = rtc_ls_alarm_set_time,
    .alarm_get_time = rtc_ls_alarm_get_time,
    .alarm_is_pending = rtc_ls_alarm_is_pending,
    .alarm_set_callback = rtc_ls_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
	.update_set_callback = rtc_ls_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */

#if defined(CONFIG_RTC_CALIBRATION)
    .set_calibration = rtc_ls_set_calibration,
    .get_calibration = rtc_ls_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */

};

#define RTC_LS_IRQ_INIT(n) \
static void rtc_ls_irq_config_func_##n(const struct device *dev) \
{ \
	IRQ_CONNECT(DT_INST_IRQN(n), \
		DT_INST_IRQ(n, priority), \
		rtc_irq_handler, \
		DEVICE_DT_INST_GET(n), 0); \
	irq_enable(DT_INST_IRQN(n)); \
}

#define RTC_LS_DEVICE(n) \
	RTC_LS_IRQ_INIT(n); \
	static struct rtc_ls_data rtc_ls_data_##n; \
	static const struct rtc_ls_config rtc_ls_config_##n = { \
        .clk_cfg = LS_DT_CLK_CFG_ITEM(n),  \
		.base = DT_INST_REG_ADDR(n), \
		.irq_config_func = rtc_ls_irq_config_func_##n, \
        .cyc_1hz = DT_INST_PROP(n, cyc_1hz), \
        .calib_cyc = DT_INST_PROP(n, calib_cyc), \
        .calib_enable = DT_INST_NODE_HAS_PROP(DT_DRV_INST(n), calib_enable), \
	}; \
	DEVICE_DT_INST_DEFINE(n, rtc_ls_init, NULL, \
				&rtc_ls_data_##n, &rtc_ls_config_##n, \
				POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
				&rtc_ls_api);

DT_INST_FOREACH_STATUS_OKAY(RTC_LS_DEVICE)
