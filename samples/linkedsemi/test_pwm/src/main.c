#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
int main(void)
{
	uint32_t max_period;
	uint32_t period;
	int ret;
	printk("PWM driver example\n");
	if (!pwm_is_ready_dt(&pwm_led0)) {
		printk("Error: PWM device %s is not ready\n", pwm_led0.dev->name);
		return 0;
	}
	max_period = PWM_USEC(2);
	period = PWM_USEC(1);
	ret = pwm_set_dt(&pwm_led0, max_period, period);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return 0;
	}
		return 0;
	k_msleep(100);
}