
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i3c.h>

#define DT_DRV_COMPAT vnd_i3c_device

static int i3c_senser_init(const struct device *dev)
{
	ARG_UNUSED(dev);
    /*if want to do something, must after controller initailed*/
	return 0;
}



/* define child node for the i3c controller*/
#define VND_I3C_TARGET_INIT(n) \
	DEVICE_DT_INST_DEFINE(n, i3c_senser_init, NULL, NULL, NULL, POST_KERNEL,                       \
				     CONFIG_I3C_CONTROLLER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(VND_I3C_TARGET_INIT) 
