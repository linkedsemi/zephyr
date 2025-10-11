#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define PA_NODE DT_NODELABEL(gpioa)
#define PA1_PIN 1

void pulse_task(void *arg1, void *arg2, void *arg3)
{
	const struct device *gpioa = DEVICE_DT_GET(PA_NODE);
	gpio_pin_configure(gpioa, PA1_PIN, GPIO_OUTPUT_ACTIVE);
	int level = 0;
	while (1) {
		gpio_pin_set(gpioa, PA1_PIN, level);
		level = !level;
		k_msleep(5);
	}
}

K_THREAD_STACK_DEFINE(pulse_stack, 512);
struct k_thread pulse_thread;

void main(void)
{
	printk("Hello World! 1243\n");
	const struct device *cap_dev = DEVICE_DT_GET(DT_NODELABEL(cap));
	if (!device_is_ready(cap_dev)) {
		printk("CAP device not ready!\n");
		return;
	}
	printk("CAP device found and ready\n");

	k_thread_create(&pulse_thread, pulse_stack, K_THREAD_STACK_SIZEOF(pulse_stack), pulse_task,
			NULL, NULL, NULL, 7, 0, K_NO_WAIT);
#if defined(CONFIG_CAP_V2)
	for (int i = 0; i < 16; i++) {
#else
	for (int i = 0; i < 8; i++) {
#endif
		printk("[channel%d] cap_sample_fetch: waiting for data...\n", i);
		enum sensor_channel my_channel = (SENSOR_CHAN_CAP_01 + i);
		int ret = sensor_sample_fetch_chan(cap_dev, my_channel);
		if (ret) {
			printk("[channel%d] sensor_sample_fetch failed: %d\n", i, ret);
			continue;
		}
		printk("[channel%d] cap_sample_fetch: get data!\n", i);
		struct sensor_value val;
		ret = sensor_channel_get(cap_dev, my_channel, &val);
		if (ret) {
			printk("[channel%d] sensor_channel_get failed: %d\n", i, ret);
		} else {
			printk("[channel%d] frequency: %d Hz, val2: %d\n", i, val.val1, val.val2);
		}
		k_msleep(100);
	}

#if defined(CONFIG_CAP_V2)
	printk("CAP channel 0-15 test finished!\n");
#else
	printk("CAP channel 0-7 test finished!\n");
#endif
}
