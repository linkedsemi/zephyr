#define DT_DRV_COMPAT linkedsemi_ls_host_vuart
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/espi.h>
#include <stdint.h>
#include <espi_lpc_common.h>

#include "vuart_ls.h"

LOG_MODULE_REGISTER(host_vuart, CONFIG_ESPI_LOG_LEVEL);

#define IRQ_TYPE_NONE         0
#define IRQ_TYPE_EDGE_RISING  1
#define IRQ_TYPE_EDGE_FALLING 2
#define IRQ_TYPE_EDGE_BOTH    (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)
#define IRQ_TYPE_LEVEL_HIGH   4
#define IRQ_TYPE_LEVEL_LOW    8

#define PORT_NUM   8
#define LCR_DLAB   0x80
#define IIR_NOPEND 0x01
#define IIR_THRE   0x02
#define IIR_RDA    0x04

struct host_vuart_cfg {
	const struct device *parent; /* eSPI 控制器 */
	const struct device *vuart;
	const struct upstream_irq_type *up_irq;
};

struct host_vuart_reg {
	uint8_t dll;
	uint8_t dlh;
	uint8_t ier;
	uint8_t fcr;
	uint8_t lcr;
	uint8_t mcr;
	uint8_t scr;
};

struct host_vuart_data {
	struct host_vuart_reg data_reg;
	struct k_spinlock lock;
	atomic_t level_up_irq_active;
	struct peri_ioport ioport[PORT_NUM];
};

static uint8_t host_vuart_calc_iir(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;
	k_spinlock_key_t key;
	uint8_t iir = 0;

	key = k_spin_lock(&ptr_data->lock);

	iir = (ptr_data->data_reg.fcr & 0x1) ? 0xC0 : 0x00;

	if ((ptr_data->data_reg.ier & 0x01) && host_vuart_rx_available(cfg->vuart)) {
		iir |= IIR_RDA;
	} else if ((ptr_data->data_reg.ier & 0x02)) {
		iir |= IIR_THRE;
	} else {
		iir |= IIR_NOPEND;
	}

	k_spin_unlock(&ptr_data->lock, key);

	return iir;
}

static void host_vuart_report_inactive_level_up_irq(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;

	if ((host_vuart_calc_iir(dev) & 0xf) == IIR_NOPEND) {
		if (cfg->up_irq && cfg->up_irq->type) {
			if (atomic_test_and_clear_bit(&ptr_data->level_up_irq_active, 0)) {
				espi_lpc_set_level_irq(cfg->parent, cfg->up_irq->idx, 0);
			}
		}
	}
}

static void host_vuart_report_active_edge_level_up_irq(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;

	if ((host_vuart_calc_iir(dev) & 0xf)  == IIR_NOPEND) {
		return;
	}

	if (cfg->up_irq->type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH)) {
		if (!atomic_test_and_set_bit(&ptr_data->level_up_irq_active, 0)) {
			espi_lpc_set_level_irq(cfg->parent, cfg->up_irq->idx, 1);
		}
	} else {
		espi_lpc_raise_edge_irq(cfg->parent, cfg->up_irq->idx);
	}
}

static void host_vuart_rx_callback(void *dev)
{
	host_vuart_report_active_edge_level_up_irq((const struct device *)dev);
}

static void host_vuart_reg0_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	struct device *dev = ioport->ctx;
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;
	uint8_t *val = (uint8_t *)res;

	if (ptr_data->data_reg.lcr & LCR_DLAB) {
		*val = ptr_data->data_reg.dll;
	} else if (host_vuart_calc_iir(dev)) {
		ls_vuart_get_tx_char(cfg->vuart, val);
		host_vuart_report_inactive_level_up_irq(dev);
	}
}

static void host_vuart_reg0_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	struct device *dev = ioport->ctx;
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;

	if (ptr_data->data_reg.lcr & LCR_DLAB) {
		ptr_data->data_reg.dll = *val;
	} else {
		ls_vuart_put_rx_char(cfg->vuart, *val);
	}
}

static void host_vuart_reg1_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	uint8_t *val = (uint8_t *)res;

	if (ptr_data->data_reg.lcr & LCR_DLAB) {
		*val = ptr_data->data_reg.dlh;
	} else {
		*val = ptr_data->data_reg.ier;
	}
}

static void host_vuart_reg1_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;

	if (ptr_data->data_reg.lcr & LCR_DLAB) {
		ptr_data->data_reg.dlh = *val;
	} else {
		ptr_data->data_reg.ier = *val;
		host_vuart_report_active_edge_level_up_irq(dev);
	}
}

static void host_vuart_reg2_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	const struct device *dev = ioport->ctx;
	uint8_t *val = (uint8_t *)res;
	*val = host_vuart_calc_iir(dev);
}

static void host_vuart_reg2_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->data_reg.fcr = *val;
}

static void host_vuart_reg3_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	uint8_t *val = (uint8_t *)res;
	*val = ptr_data->data_reg.lcr;
}

static void host_vuart_reg3_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->data_reg.lcr = *val;
}

static void host_vuart_reg4_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	uint8_t *val = (uint8_t *)res;
	*val = ptr_data->data_reg.mcr;
}

static void host_vuart_reg4_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->data_reg.mcr = *val;
}

static void host_vuart_reg5_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	struct device *dev = ioport->ctx;
	const struct host_vuart_cfg *cfg = dev->config;
	uint8_t *val = res;
	*val = 0x60;
	if (host_vuart_rx_available(cfg->vuart)) {
		*val |= 0x01;
	}
}
static void host_vuart_reg5_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	__ASSERT(false, "Unreachable code path");
}

static void host_vuart_reg6_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	uint8_t *val = (uint8_t *)res;
	*val = 0x00; /* 调制解调器状态寄存器 */
}

static void host_vuart_reg6_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	__ASSERT(false, "Unreachable code path");
}

static void host_vuart_reg7_read(const struct peri_ioport_content *ioport, uint8_t size, void *res)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	uint8_t *val = (uint8_t *)res;
	*val = ptr_data->data_reg.scr;
}

static void host_vuart_reg7_write(const struct peri_ioport_content *ioport, uint8_t size,
				  uint8_t *val)
{
	const struct device *dev = ioport->ctx;
	struct host_vuart_data *ptr_data = dev->data;
	ptr_data->data_reg.scr = *val;
}

static int host_vuart_init(const struct device *dev)
{
	const struct host_vuart_cfg *cfg = dev->config;
	struct host_vuart_data *ptr_data = dev->data;

	if (!device_is_ready(cfg->parent) || !device_is_ready(cfg->vuart)) {
		LOG_ERR("parent or vuart not ready");
		return -ENODEV;
	}

	ptr_data->level_up_irq_active = ATOMIC_INIT(0);

	ls_vuart_register_tx_start_callback(cfg->vuart, host_vuart_rx_callback, (void *)dev);

	for (uint32_t i = 0; i < PORT_NUM; i++) {
		espi_lpc_add_ioport(cfg->parent, (struct peri_ioport *)(&ptr_data->ioport[i]));
	}
	return 0;
}

#define HOST_VUART_INIT(inst)                                                                      \
	IF_ENABLED(DT_HAS_UP_IRQ(inst), (UPSTREAM_IRQ_DT_INST_DEFINE(inst)))                                                                                 \
	static struct host_vuart_data host_vuart_data_##inst = {                                   \
		.ioport = {[0] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg0_read,                 \
						  .io_write = host_vuart_reg0_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port)}},              \
			   [1] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg1_read,                 \
						  .io_write = host_vuart_reg1_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 1}},          \
			   [2] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg2_read,                 \
						  .io_write = host_vuart_reg2_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 2  }},       \
			   [3] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg3_read,                 \
						  .io_write = host_vuart_reg3_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 3}},          \
			   [4] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg4_read,                 \
						  .io_write = host_vuart_reg4_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 4}},          \
			   [5] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg5_read,                 \
						  .io_write = host_vuart_reg5_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 5}},          \
			   [6] = {.content =                                                       \
					  &(struct peri_ioport_content){                           \
						  .io_read = host_vuart_reg6_read,                 \
						  .io_write = host_vuart_reg6_write,               \
						  .ctx = (void *)DEVICE_DT_INST_GET(inst),         \
						  .addr = DT_INST_PROP(inst, port) + 6}},          \
			   [7] = {.content = &(struct peri_ioport_content){                        \
					  .io_read = host_vuart_reg7_read,                         \
					  .io_write = host_vuart_reg7_write,                       \
					  .ctx = (void *)DEVICE_DT_INST_GET(inst),                 \
					  .addr = DT_INST_PROP(inst, port) + 7}}}};                \
	static const struct host_vuart_cfg host_vuart_cfg_##inst = {                               \
		.parent = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                     \
		.vuart = DEVICE_DT_GET(DT_INST_PHANDLE(inst, target)),                             \
		IF_ENABLED(DT_HAS_UP_IRQ(inst), (.up_irq = UPSTREAM_IRQ_DT_INST_CONFIG_GET(inst))) }; \
	DEVICE_DT_INST_DEFINE(inst, &host_vuart_init, NULL, &host_vuart_data_##inst,               \
			      &host_vuart_cfg_##inst, POST_KERNEL, CONFIG_ESPI_INIT_PRIORITY, 0);

DT_INST_FOREACH_STATUS_OKAY(HOST_VUART_INIT)
