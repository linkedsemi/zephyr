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


	k_thread_create(&pulse_thread, pulse_stack, K_THREAD_STACK_SIZEOF(pulse_stack),
					pulse_task, NULL, NULL, NULL, 7, 0, K_NO_WAIT);

	for (int i = 0; i < 8; i++) {
		printk("[通道%d] cap_sample_fetch: 等待数据...\n", i);
		enum sensor_channel my_channel = (SENSOR_CHAN_CAP_01 + i);
		int ret = sensor_sample_fetch_chan(cap_dev, my_channel);
		if (ret) {
			printk("[通道%d] sensor_sample_fetch 失败: %d\n", i, ret);
			continue;
		}
		printk("[通道%d] cap_sample_fetch: 获取到数据!\n", i);
		struct sensor_value val;
		ret = sensor_channel_get(cap_dev, my_channel, &val);
		if (ret) {
			printk("[通道%d] sensor_channel_get 失败: %d\n", i, ret);
		} else {
			printk("[通道%d] 频率: %d Hz, val2: %d\n", i, val.val1, val.val2);
		}
		k_msleep(100);
	}
	printk("CAP单线程通道0-7测试完成!\n");
}



