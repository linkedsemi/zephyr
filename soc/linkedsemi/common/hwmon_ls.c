#include <zephyr/kernel.h>
#define DT_DRV_COMPAT linkedsemi_hwmon
#define HWMON_INIT_PRIORITY 91
enum hwmon_class
{
	HWMON_FAN,
	HWMON_ADC,
	HWMON_SENSOR,
};

struct hwmon_ref {
	const struct device *dev;
	uint8_t channel;
};

struct hwmon_ls_config {
	const struct hwmon_ref *in;
	const struct hwmon_ref *out;
	uint8_t in_num;
	uint8_t out_num;
	enum hwmon_class class;
};

static int hwmon_ls_init(const struct device *dev)
{
	return 0;
}

#define LS_HWMON_IN_INIT(node,prop,idx) \
	{\
		.dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node,hwmon_ins,0)),\
		.channel = DT_PHA(node,hwmon_ins,channel)\
	},

#define LS_HWMON_OUT_INIT(node,prop,idx) \
	{\
		.dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node,hwmon_outs,0))\
		.channel = DT_PHA(node,hwmon_outs,channel)\
	},

#define LS_HWMON_INIT(idx)\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(idx,hwmon_ins),(\
		static const struct hwmon_ref hwmon_ins_##idx[] = {\
			DT_INST_FOREACH_PROP_ELEM(idx,hwmon_ins,LS_HWMON_IN_INIT)\
		};\
	))\
	IF_ENABLED(DT_INST_NODE_HAS_PROP(idx,hwmon_outs),(\
		static const struct hwmon_ref hwmon_outs_##idx[] = {\
			DT_INST_FOREACH_PROP_ELEM(idx,hwmon_outs,LS_HWMON_OUT_INIT)\
		};\
	))\
	static const struct hwmon_ls_config hwmon_ls_cfg_##idx = {\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(idx,hwmon_ins),(\
			.in = hwmon_ins_##idx,\
			.in_num = ARRAY_SIZE(hwmon_ins_##idx),\
		))\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(idx,hwmon_outs),(\
			.out = hwmon_outs_##idx,\
			.out_num = ARRAY_SIZE(hwmon_outs_##idx),\
		))\
		.class = DT_INST_PROP(idx,class),\
	};\
	DEVICE_DT_INST_DEFINE(idx,hwmon_ls_init,NULL,NULL,\
		&hwmon_ls_cfg_##idx,POST_KERNEL,HWMON_INIT_PRIORITY,NULL);

DT_INST_FOREACH_STATUS_OKAY(LS_HWMON_INIT)
