ADC_TEST
###########

struct adc_channel_cfg 结构体的 channel_id 要和设备树中 “pinctrl-0” 节点所对应的通道一致；

测试温感时 channel_id 必须配置为13 (ADC1 和 ADC2 的13通道专用测温感), 设备树中 “pinctrl-0” 节点可以随意配置。

adc_drive_type = <BINBUF_DIRECT_DRIVE_ADC>;  // 测温感时设备树中的adc_drive_type只能配置为BINBUF_DIRECT_DRIVE_ADC

conversion_mode 可配置的选项为  <regular_mode>, <inject_mode>, <loop_mode>;

举例：
对于普通模数转换: 选用内部参考电压 (1.4v), 输入电压为 0.9v 的情况下, 转换结果为 2632 左右；
对于采集温感: 室温环境下转换结果为2102左右, 用风枪100℃吹, 随着时间的增加，转换结果逐渐减小。