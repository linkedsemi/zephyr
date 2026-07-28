/*
 * Copyright (c) 2026 Linkedsemi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Case-open (chassis intrusion) driver for Linkedsemi QSH.
 *
 * Hardware: a DEDICATED case-open module inside APP_PMU, register
 * CASEOPEN_CTRL @ 0x4006f044. This is NOT a GPIO device: its two input
 * channels (A/B) are hard-wired in silicon, and the module raises a
 * dedicated interrupt (PMU_CASEOPEN_IRQN == 21) whenever any enabled
 * channel is intruded. The driver must therefore be interrupt-driven,
 * not a GPIO poller.
 *
 * Field layout (CONFIRMED on HW + by vendor), per channel (bit0 = chA,
 * bit1 = chB):
 *   msk [1:0]  - interrupt enable: 1 = enabled, 0 = masked (R/W)
 *   clr [3:2]  - write 1 to clear the int edge-latch (and thus the IRQ);
 *               this is the ONLY way to clear it. (R/W, self-idle)
 *   int [5:4]  - EDGE-LATCH (RO): set by the trigger edge on the COPEN
 *               input and held until clr, REGARDLESS of msk (edges that
 *               happen while masked are still latched). The module is
 *               edge-triggered (vendor: falling edge at the detector;
 *               board level: closed = 0 V, intrusion = 3.3 V, and the
 *               closed->open transition is what triggers).
 *   stt [7:6]  - COMBINATIONAL, stt = int & msk (RO). This is the IRQ
 *               output, NOT an independent sticky latch. It reads 0
 *               whenever the channel is masked or int was cleared.
 *
 * Related: ext_intr @ 0x3c bit16 (RO) is the PMU-level external-IRQ
 * aggregator; it tracks stt and auto-clears when clr is written.
 *
 * Driver model:
 *   - On the trigger edge the module latches int; stt = int & msk raises
 *     IRQ 21.
 *   - The ISR snapshots int into a sticky SW copy (data->latched), masks
 *     the fired channel(s) to prevent an IRQ storm, and clears int via clr.
 *     Because of this, HW int/stt read 0 after the ISR; the sticky state
 *     lives in data->latched (see `caseopen status` / sample_fetch).
 *   - Edges that arrive while a channel is masked still latch int; they
 *     are folded into data->latched by sample_fetch().
 *   - rearm() clears int and restores the full mask. Being edge-triggered,
 *     a chassis that is ALREADY open will not re-trigger until it is
 *     closed and opened again.
 *   - Power-up fabricates a spurious trigger edge on both channels
 *     (int=0x3 with the chassis closed). init scrubs it via clr BEFORE
 *     enabling detection, so boot never reports a false intrusion — the
 *     tradeoff is that a genuine "intruded while powered off" latch
 *     cannot be distinguished and is scrubbed too.
 *   - Per vendor feedback caseopen does NOT depend on vbatmem_clk or the
 *     TIM/RTC clocks in CLKG_SRST (confirmed on HW with all of them
 *     gated off), so the driver does not touch any clock at all.
 */

#define DT_DRV_COMPAT linkedsemi_caseopen

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/caseopen_ls.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/spinlock.h>
#include "reg_app_pmu_rg.h"
#include "qsh.h" /* PMU_CASEOPEN_IRQN */

LOG_MODULE_REGISTER(caseopen_ls, CONFIG_SENSOR_LOG_LEVEL);

/* Both channels enabled by default; the HW table says which are wired. */
#define CASEOPEN_CH_MASK 0x3U

struct caseopen_config {
	volatile uint32_t *ctrl;
	void (*irq_config_func)(const struct device *dev);
};

struct caseopen_data {
	struct k_spinlock lock;
	struct k_sem intr_sem;
	uint8_t latched; /* sticky SW copy of stt (bit0=chA, bit1=chB) */
	uint8_t raw;     /* latest int snapshot */
	uint8_t enabled; /* current msk value (channels still armed) */
	uint32_t isr_count;   /* how many times the ISR actually ran */
	uint32_t isr_last_v;  /* CASEOPEN_CTRL snapshot at last ISR entry */
};

static inline uint32_t caseopen_read(const struct device *dev)
{
	const struct caseopen_config *cfg = dev->config;

	return *cfg->ctrl;
}

/* Write only the MASK field plus an optional CLR field.
 * clr_field is the per-channel mask (bit0=chA, bit1=chB); writing a
 * channel's bit clears that channel's latched stt + pending IRQ. int/stt
 * are RO and never written.
 */
static void caseopen_write_locked(const struct device *dev, uint32_t msk_field,
				  uint32_t clr_field)
{
	const struct caseopen_config *cfg = dev->config;
	uint32_t v;

	v = FIELD_PREP(APP_PMU_RG_CASEOPEN_MASK_MASK, msk_field);
	v |= FIELD_PREP(APP_PMU_RG_CASEOPEN_CLR_MASK, clr_field);
	*cfg->ctrl = v;
}

static void caseopen_isr(const void *arg)
{
	const struct device *dev = arg;
	struct caseopen_data *data = dev->data;
	uint32_t v = caseopen_read(dev);
	uint8_t stt = (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v);
	uint8_t intr = (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v);
	uint8_t msk_hw = (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK, v);
	uint8_t fired, new_msk;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->isr_count++;
	data->isr_last_v = v;
	data->raw = intr;
	/* int is the real edge-latch (stt is just int & msk): any set int
	 * bit means an intrusion edge occurred. Fold it into the SW latch.
	 */
	data->latched |= intr | stt;
	/* Mask the channels that are CURRENTLY triggering this interrupt so
	 * the IRQ cannot storm until rearm() restores the full mask.
	 * CRITICAL: compute from the HW mask snapshot, ONLY ever removing
	 * bits. Using data->enabled here would re-enable a channel that a
	 * previous ISR masked, and its next contact-bounce edge would
	 * re-fire -> ping-pong IRQ storm between channels (seen on HW as
	 * isr_count jumping by 7 on a dual intrusion).
	 */
	fired = intr & msk_hw; /* == stt by definition (stt = int & msk) */
	new_msk = msk_hw & (uint8_t)~fired;
	k_spin_unlock(&data->lock, key);

	/* Clear the latched status + pending IRQ for the fired channels. */
	caseopen_write_locked(dev, new_msk, fired);
	k_busy_wait(5);
	caseopen_write_locked(dev, new_msk, 0U);

	k_sem_give(&data->intr_sem);
}

static int caseopen_set_enable(const struct device *dev, bool enable)
{
	struct caseopen_data *data = dev->data;
	uint8_t msk = enable ? CASEOPEN_CH_MASK : 0U;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->enabled = msk;
	caseopen_write_locked(dev, msk, 0U);
	k_spin_unlock(&data->lock, key);

	LOG_INF("case-open detection %s (msk=0x%x)", enable ? "enabled" : "disabled", msk);
	return 0;
}

static int caseopen_rearm(const struct device *dev)
{
	struct caseopen_data *data = dev->data;
	uint8_t msk, still_open;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	msk = data->enabled;
	/* Restore full mask and clear both latches in one shot. */
	caseopen_write_locked(dev, msk, CASEOPEN_CH_MASK);
	k_busy_wait(5);
	caseopen_write_locked(dev, msk, 0U);
	data->latched = 0U;
	data->raw = 0U;

	/* The module is EDGE-triggered: an already-open chassis will NOT
	 * re-latch after clr (a new closed->open edge is required). This
	 * re-sample only catches an edge that raced the rearm itself; if
	 * one did, mirror it into the SW latch and keep that channel
	 * masked so the IRQ cannot storm.
	 */
	still_open = (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK,
					 caseopen_read(dev));
	if (still_open) {
		data->latched = still_open;
		msk = msk & (uint8_t)~still_open;
		caseopen_write_locked(dev, msk, 0U);
	}
	k_spin_unlock(&data->lock, key);

	LOG_INF("case-open latch rearmed (reg=0x%08x)", caseopen_read(dev));
	return 0;
}

static int caseopen_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct caseopen_data *data = dev->data;
	uint32_t v;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_INTRUSION) {
		return -ENOTSUP;
	}

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	v = caseopen_read(dev);
	/* int latches edges even on masked channels (no IRQ fires for
	 * those): pick them up here in addition to ISR captures.
	 */
	data->latched |= (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v) |
			 (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v);
	data->raw = (uint8_t)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v);
	k_spin_unlock(&data->lock, key);
	return 0;
}

static int caseopen_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct caseopen_data *data = dev->data;

	if (chan != SENSOR_CHAN_INTRUSION) {
		return -ENOTSUP;
	}

	/* val1: latched intrusion mask (non-zero = intrusion;
	 *       bit0 = chA, bit1 = chB)
	 * val2: current raw input mask (diagnostics)
	 */
	val->val1 = data->latched;
	val->val2 = data->raw;
	return 0;
}

static int caseopen_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_INTRUSION && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_CASEOPEN_REARM:
		return caseopen_rearm(dev);
	case SENSOR_ATTR_CASEOPEN_ENABLE:
		return caseopen_set_enable(dev, val->val1 != 0);
	default:
		return -ENOTSUP;
	}
}

static int caseopen_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_INTRUSION && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	switch ((int)attr) {
	case SENSOR_ATTR_CASEOPEN_ENABLE:
		val->val1 = FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK,
				      caseopen_read(dev)) ? 1 : 0;
		val->val2 = 0;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int caseopen_init(const struct device *dev)
{
	struct caseopen_data *data = dev->data;
	const struct caseopen_config *cfg = dev->config;
	uint32_t v = caseopen_read(dev);

	k_sem_init(&data->intr_sem, 0, 1);
	data->latched = 0U;
	data->raw = 0U;
	data->enabled = CASEOPEN_CH_MASK;

	/* Power-up itself fabricates int=0x3 (confirmed on HW with the
	 * chassis closed). A boot-time int snapshot therefore CANNOT be
	 * distinguished from a genuine "intruded while powered off" latch,
	 * so we do NOT fold it into the SW latch — pre-boot intrusion
	 * detection is not supported on this silicon; we just scrub.
	 * clr while msk is still 0 so enabling detection below cannot fire
	 * a false interrupt.
	 */
	caseopen_write_locked(dev, 0U, CASEOPEN_CH_MASK);
	k_busy_wait(5);
	caseopen_write_locked(dev, 0U, 0U);

	LOG_INF("caseopen init, CASEOPEN_CTRL=0x%08x (msk=%u clr=%u int=%u stt=%u)", v,
		(unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK, v),
		(unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_CLR_MASK, v),
		(unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v),
		(unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v));

	/* Hook the dedicated intrusion interrupt (PMU_CASEOPEN_IRQN). */
	cfg->irq_config_func(dev);

	/* Enable detection on both channels (starts clean; the boot glitch
	 * was scrubbed above).
	 */
	return caseopen_set_enable(dev, true);
}

static const struct sensor_driver_api caseopen_driver_api = {
	.sample_fetch = caseopen_sample_fetch,
	.channel_get = caseopen_channel_get,
	.attr_set = caseopen_attr_set,
	.attr_get = caseopen_attr_get,
};

#define CASEOPEN_DEFINE(inst)                                                                      \
	static struct caseopen_data caseopen_data_##inst;                                          \
	static void caseopen_irq_config_##inst(const struct device *dev)                          \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(PMU_CASEOPEN_IRQN, 3, caseopen_isr,                             \
			    DEVICE_DT_INST_GET(inst), 0);                                            \
		irq_enable(PMU_CASEOPEN_IRQN);                                                    \
	}                                                                                          \
	static const struct caseopen_config caseopen_config_##inst = {                             \
		.ctrl = (volatile uint32_t *)DT_INST_REG_ADDR(inst),                               \
		.irq_config_func = caseopen_irq_config_##inst,                                      \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, caseopen_init, NULL, &caseopen_data_##inst,             \
				     &caseopen_config_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &caseopen_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CASEOPEN_DEFINE)

/* ------------------------------------------------------------------ */
/* Shell debug commands: caseopen dump | status | rearm | enable 0|1   */
/* ------------------------------------------------------------------ */
#ifdef CONFIG_SHELL

#include <zephyr/shell/shell.h>
#include <stdlib.h>

static const struct device *caseopen_shell_dev(const struct shell *sh)
{
	const struct device *dev = DEVICE_DT_GET_ONE(linkedsemi_caseopen);

	if (!device_is_ready(dev)) {
		shell_error(sh, "caseopen device not ready");
		return NULL;
	}
	return dev;
}

static int cmd_caseopen_dump(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct device *dev = caseopen_shell_dev(sh);

	if (dev == NULL) {
		return -ENODEV;
	}

	const struct caseopen_config *cfg = dev->config;
	uint32_t v = *cfg->ctrl;
	/* ext_intr @ 0x3c is the PMU-level "external IRQ asserted" flag
	 * (bit16, RO); it tracks caseopen_int and auto-clears when clr is
	 * written. Useful to confirm the IRQ line is ours and released.
	 */
	volatile uint32_t *ext_intr = (volatile uint32_t *)(cfg->ctrl - 2);
	uint32_t ei = *ext_intr;

	shell_print(sh, "CASEOPEN_CTRL @0x%08lx = 0x%08x", (unsigned long)(uintptr_t)cfg->ctrl, v);
	shell_print(sh, "  msk[1:0]=0x%x (chA=%u chB=%u)  clr[3:2]=0x%x",
		    (unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK, v),
		    (unsigned int)(FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK, v) & 1),
		    (unsigned int)((FIELD_GET(APP_PMU_RG_CASEOPEN_MASK_MASK, v) >> 1) & 1),
		    (unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_CLR_MASK, v));
	shell_print(sh, "  int[5:4]=0x%x (chA=%u chB=%u)  stt[7:6]=0x%x (chA=%u chB=%u)",
		    (unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v),
		    (unsigned int)(FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v) & 1),
		    (unsigned int)((FIELD_GET(APP_PMU_RG_CASEOPEN_INT_MASK, v) >> 1) & 1),
		    (unsigned int)FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v),
		    (unsigned int)(FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v) & 1),
		    (unsigned int)((FIELD_GET(APP_PMU_RG_CASEOPEN_STT_MASK, v) >> 1) & 1));
	shell_print(sh, "  ext_intr@0x3c bit16=%u (PMU external IRQ line asserted)",
		    (unsigned int)((ei >> 16) & 1));
	{
		struct caseopen_data *data = dev->data;

		shell_print(sh, "  IRQ%u enabled=%u  isr_count=%u  isr_last_reg=0x%08x  sw_latched=0x%x",
			    (unsigned int)PMU_CASEOPEN_IRQN,
			    (unsigned int)irq_is_enabled(PMU_CASEOPEN_IRQN),
			    (unsigned int)data->isr_count, data->isr_last_v,
			    (unsigned int)data->latched);
	}
	shell_print(sh, "  NOTE: int[5:4] = edge-latch (set on trigger edge even if masked,");
	shell_print(sh, "        cleared only by clr). stt[7:6] = int & msk = the IRQ output,");
	shell_print(sh, "        NOT a latch. Sticky state = sw_latched (see `caseopen status`).");
	return 0;
}

static int cmd_caseopen_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct device *dev = caseopen_shell_dev(sh);
	struct sensor_value val;
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}

	ret = sensor_sample_fetch_chan(dev, SENSOR_CHAN_INTRUSION);
	if (ret == 0) {
		ret = sensor_channel_get(dev, SENSOR_CHAN_INTRUSION, &val);
	}
	if (ret != 0) {
		shell_error(sh, "read failed: %d", ret);
		return ret;
	}

	shell_print(sh, "latched(stt)=0x%x (chA=%u chB=%u) current(int)=0x%x",
		    val.val1, (unsigned int)(val.val1 & 1),
		    (unsigned int)((val.val1 >> 1) & 1), val.val2);
	return 0;
}

static int cmd_caseopen_rearm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct device *dev = caseopen_shell_dev(sh);
	struct sensor_value val = {0};
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}

	ret = sensor_attr_set(dev, SENSOR_CHAN_INTRUSION,
			      (enum sensor_attribute)SENSOR_ATTR_CASEOPEN_REARM, &val);
	if (ret != 0) {
		shell_error(sh, "rearm failed: %d", ret);
		return ret;
	}
	shell_print(sh, "rearmed");
	return cmd_caseopen_dump(sh, 0, NULL);
}

static int cmd_caseopen_enable(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = caseopen_shell_dev(sh);
	struct sensor_value val = {0};
	int ret;

	if (dev == NULL) {
		return -ENODEV;
	}

	val.val1 = strtol(argv[1], NULL, 0);
	ret = sensor_attr_set(dev, SENSOR_CHAN_INTRUSION,
			      (enum sensor_attribute)SENSOR_ATTR_CASEOPEN_ENABLE, &val);
	if (ret != 0) {
		shell_error(sh, "enable failed: %d", ret);
		return ret;
	}
	shell_print(sh, "detection %s", val.val1 ? "enabled" : "disabled");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_caseopen,
	SHELL_CMD_ARG(dump, NULL, "Dump raw CASEOPEN_CTRL register", cmd_caseopen_dump, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Show latched/current intrusion state", cmd_caseopen_status,
	              1, 0),
	SHELL_CMD_ARG(rearm, NULL, "Clear latched intrusion status", cmd_caseopen_rearm, 1, 0),
	SHELL_CMD_ARG(enable, NULL, "enable <0|1>: disable/enable detection",
		      cmd_caseopen_enable, 2, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(caseopen, &sub_caseopen, "Case-open (chassis intrusion) debug commands", NULL);

#endif /* CONFIG_SHELL */
