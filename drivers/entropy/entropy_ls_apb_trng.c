#define DT_DRV_COMPAT linkedsemi_ls_apb_trng
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <soc_clock.h>

struct trng_apb_regs {
	uint32_t trng_ctrl;          /* 0x000: [0]=fsm_rst, 1=复位 0=运行 */
	uint32_t cs_cnt_th_i;        /* 0x004: [19:0] 相干采样计数下界 */
	uint32_t cs_cnt_th_h;        /* 0x008: [19:0] 相干采样计数上界 */
	uint32_t good_th;            /* 0x00C: [27:16]=good_th_h(高阈值) [11:0]=good_th_l(低阈值) */
	uint32_t nb_smpl_lock_cyc;   /* 0x010: [27:16]=max_lock_cyc [11:0]=nb_smpl */
	uint32_t ro_sel_opt;         /* 0x014: [15:4]=ro_sel_init [0]=ro_sel_ctrl(1=自动扫描) */
	uint32_t ro0_dly_sel;        /* 0x018: RO0延迟线抽头, one-hot, 必须非零 */
	uint32_t ro1_dly_sel;        /* 0x01C: RO1延迟线抽头, one-hot, 必须非零 */
	uint32_t intr_msk;           /* 0x020: [3:0] 中断屏蔽: bit3=no_fnd bit2=lock bit1=no_match bit0=match */
	uint32_t intr_clr;           /* 0x024: [3:0] 写1清除中断, 布局同intr_msk */
	uint32_t intr_stat;          /* 0x028: [3:0] 经mask过滤的中断状态(只读) */
	uint32_t intr_raw;           /* 0x02C: [3:0] 原始中断状态(只读), 不受mask影响 */
	uint32_t trng_out;           /* 0x030: [31:16]=rand_dat [2]=no_fnd [1]=locked [0]=matched */
	uint32_t cur_ro_sel;         /* 0x034: [11:0] 当前RO索引(只读) */
};

#define TRNG_CTRL_FSM_RST        BIT(0)   
#define TRNG_OUT_RAND_DAT_SHF    16       
#define TRNG_IRQ_MATCH           BIT(0)
#define TRNG_IRQ_NO_MATCH        BIT(1)
#define TRNG_IRQ_LOCK            BIT(2)
#define TRNG_IRQ_NO_FOUND        BIT(3)
#define TRNG_IRQ_ALL             0xFU    
#define CFG_CS_TH_LO    		 0x00000U
#define CFG_CS_TH_HI    		 0xFFFFFU
#define CFG_GOOD_TH_H   		 0x000U
#define CFG_GOOD_TH_L   		 0xFFFU
#define CFG_MAX_LOCK_CYC 		 256U
#define CFG_NB_SMPL     		 4095U
#define CFG_RO_SEL_OPT  		 0x00000001U
#define CFG_RO0_DLY_SEL 		 0x00010000U
#define CFG_RO1_DLY_SEL 		 0x00010000U
#define GOOD_TH_PACK(good_th_h, good_th_l)       ((((good_th_h) & 0xFFFU) << 16) | ((good_th_l) & 0xFFFU))
#define NB_SMPL_LOCK_PACK(max_lock_cyc, nb_smpl) ((((max_lock_cyc) & 0xFFFU) << 16) | ((nb_smpl) & 0xFFFU))
#define TRNG_POLL_TIMEOUT_MS 	 200U

static void trng_init(volatile struct trng_apb_regs *r)
{
	r->trng_ctrl |= TRNG_CTRL_FSM_RST;
	r->cs_cnt_th_i = CFG_CS_TH_LO;
	r->cs_cnt_th_h = CFG_CS_TH_HI;
	r->good_th = GOOD_TH_PACK(CFG_GOOD_TH_H, CFG_GOOD_TH_L);
	r->nb_smpl_lock_cyc = NB_SMPL_LOCK_PACK(CFG_MAX_LOCK_CYC, CFG_NB_SMPL);
	r->ro_sel_opt = CFG_RO_SEL_OPT;
	r->ro0_dly_sel = CFG_RO0_DLY_SEL;  
	r->ro1_dly_sel = CFG_RO1_DLY_SEL;
	r->intr_msk = TRNG_IRQ_MATCH | TRNG_IRQ_NO_FOUND | TRNG_IRQ_LOCK;
	r->intr_clr = TRNG_IRQ_ALL;
}

static void trng_fsm_pulse(volatile struct trng_apb_regs *r)
{
	r->trng_ctrl |= TRNG_CTRL_FSM_RST;
	r->trng_ctrl &= ~TRNG_CTRL_FSM_RST;
}

static int trng_poll16(volatile struct trng_apb_regs *r, uint16_t *out)
{
	uint32_t deadline = k_cycle_get_32() +
		k_ms_to_cyc_ceil32(TRNG_POLL_TIMEOUT_MS);

	while ((int32_t)(k_cycle_get_32() - deadline) < 0) {
		uint32_t raw = r->intr_raw & TRNG_IRQ_ALL;

		if ((raw & TRNG_IRQ_NO_FOUND) != 0U) {
			trng_fsm_pulse(r);
			r->intr_clr = TRNG_IRQ_ALL;
			continue;
		}
		if ((raw & TRNG_IRQ_MATCH) != 0U) {
			*out = (uint16_t)(r->trng_out >> TRNG_OUT_RAND_DAT_SHF);
			r->intr_clr = TRNG_IRQ_ALL;
			trng_fsm_pulse(r);
			return 0;
		}
		if ((raw & TRNG_IRQ_LOCK) != 0U) {
			r->intr_clr = TRNG_IRQ_LOCK;
		}
	}
	return -EIO;
}

struct entropy_ls_apb_trng_data {
	struct k_mutex mutex;
	struct k_pipe pool;
	uint8_t pool_buf[CONFIG_ENTROPY_LS_APB_TRNG_POOL_SIZE];
};

struct entropy_ls_apb_trng_config {
	volatile struct trng_apb_regs *regs;      
	void (*irq_config_func)(const struct device *dev);  
#if defined(CONFIG_CLOCK_CONTROL)
	struct ls_clk_cfg ccfg;                   
#endif
#if defined(CONFIG_RESET)
	struct reset_dt_spec reset;               
#endif
};

static int trng_power_cycle(const struct entropy_ls_apb_trng_config *cfg)
{
#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev != NULL) {
		clock_control_off(cfg->ccfg.cctl_dev,
				  (clock_control_subsys_t)&cfg->ccfg);
	}
#endif
#if defined(CONFIG_RESET)
	if (cfg->reset.dev != NULL) {
		int rc = reset_line_toggle(cfg->reset.dev, cfg->reset.id);
		if (rc != 0) {
			return rc;
		}
	}
#endif
#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev != NULL) {
		clock_control_on(cfg->ccfg.cctl_dev,
				 (clock_control_subsys_t)&cfg->ccfg);
	}
#endif
	return 0;
}

static inline void trng_irq_enable(volatile struct trng_apb_regs *r)
{
	unsigned int key = irq_lock();
	r->intr_msk |= TRNG_IRQ_MATCH;
	irq_unlock(key);
}

static inline void trng_irq_disable(volatile struct trng_apb_regs *r)
{
	unsigned int key = irq_lock();
	r->intr_msk &= ~TRNG_IRQ_MATCH;
	irq_unlock(key);
}

static void ls_apb_trng_isr(const struct device *dev)
{
	const struct entropy_ls_apb_trng_config *cfg = dev->config;
	struct entropy_ls_apb_trng_data *data = dev->data;
	volatile struct trng_apb_regs *r = cfg->regs;
	uint32_t stat = r->intr_stat & TRNG_IRQ_ALL;

	if ((stat & TRNG_IRQ_NO_FOUND) != 0U) {
		trng_fsm_pulse(r);                   
		r->intr_clr = TRNG_IRQ_ALL;           
		return;                                
	}

	if ((stat & TRNG_IRQ_MATCH) != 0U) {
		uint16_t s = (uint16_t)(r->trng_out >> TRNG_OUT_RAND_DAT_SHF);
		uint8_t bytes[2] = {(uint8_t)(s & 0xFF), (uint8_t)(s >> 8)};
		size_t written = 0;
		r->intr_clr = TRNG_IRQ_ALL;
		k_pipe_put(&data->pool, bytes, 2, &written, 2, K_NO_WAIT);
		if (written == 0) {
			r->intr_msk &= ~TRNG_IRQ_MATCH;
			return;
		}
		trng_fsm_pulse(r);
		return;
	}

	if ((stat & TRNG_IRQ_LOCK) != 0U) {
		r->intr_clr = TRNG_IRQ_LOCK;
	}
}

static int trng_hw_recover(const struct entropy_ls_apb_trng_config *cfg)
{
	int rc = trng_power_cycle(cfg);
	if (rc != 0) {
		return rc;
	}
	trng_init(cfg->regs);
	cfg->regs->trng_ctrl &= ~TRNG_CTRL_FSM_RST;
	return 0;
}

static int ls_apb_trng_get_entropy(const struct device *dev,
				    uint8_t *buf, uint16_t len)
{
	const struct entropy_ls_apb_trng_config *cfg = dev->config;
	struct entropy_ls_apb_trng_data *data = dev->data;
	volatile struct trng_apb_regs *r = cfg->regs;
	uint16_t copied = 0;

	if (!buf || !len) {
		return -EINVAL;
	}
	k_mutex_lock(&data->mutex, K_FOREVER);
	while (copied < len) {
		size_t got = 0;
		trng_irq_enable(r);
		(void)k_pipe_get(&data->pool, &buf[copied], len - copied,
				 &got, 1, K_MSEC(200));
		if (got > 0) {
			copied += got;
			continue;
		}
		if (trng_hw_recover(cfg) != 0) {
			k_mutex_unlock(&data->mutex);
			return -EIO;
		}
	}
	k_mutex_unlock(&data->mutex);
	return 0;
}

static int ls_apb_trng_get_entropy_isr(const struct device *dev,
					uint8_t *buf, uint16_t len,
					uint32_t flags)
{
	const struct entropy_ls_apb_trng_config *cfg = dev->config;
	struct entropy_ls_apb_trng_data *data = dev->data;
	volatile struct trng_apb_regs *r = cfg->regs;
	size_t got = 0;
	uint16_t i;

	if (!buf || !len) {
		return -EINVAL;
	}

	(void)k_pipe_get(&data->pool, buf, len, &got, 1, K_NO_WAIT);

	if ((flags & ENTROPY_BUSYWAIT) == 0U) {
		trng_irq_enable(r);
		return got ? (int)got : -ENODATA;
	}

	i = (uint16_t)got;
	if (i == len) {
		return len;
	}
	trng_irq_disable(r);
	unsigned int key = irq_lock();
	int ret = len;
	r->intr_clr = TRNG_IRQ_ALL;
	trng_fsm_pulse(r);

	while (i < len) {
		uint16_t s;
		if (trng_poll16(r, &s) != 0) {
			ret = -EIO;
			break;
		}
		buf[i++] = (uint8_t)(s & 0xFF);
		if (i < len) {
			buf[i++] = (uint8_t)(s >> 8);
		}
	}
	irq_unlock(key);
	trng_irq_enable(r);
	return ret;
}

static int ls_apb_trng_init(const struct device *dev)
{
	const struct entropy_ls_apb_trng_config *cfg = dev->config;
	struct entropy_ls_apb_trng_data *data = dev->data;
#if defined(CONFIG_CLOCK_CONTROL)
	if (cfg->ccfg.cctl_dev != NULL) {
		if (!device_is_ready(cfg->ccfg.cctl_dev)) {
			return -ENODEV;
		}
	}
#endif
#if defined(CONFIG_RESET)
	if (cfg->reset.dev != NULL) {
		if (!device_is_ready(cfg->reset.dev)) {
			return -ENODEV;
		}
	}
#endif
	int rc = trng_power_cycle(cfg);
	if (rc != 0) {
		return rc;
	}
	k_mutex_init(&data->mutex);
	k_pipe_init(&data->pool, data->pool_buf, sizeof(data->pool_buf));
	trng_init(cfg->regs);
	cfg->irq_config_func(dev);
	cfg->regs->trng_ctrl &= ~TRNG_CTRL_FSM_RST;
	return 0;
}

static const struct entropy_driver_api ls_apb_trng_api = {
	.get_entropy = ls_apb_trng_get_entropy,
	.get_entropy_isr = ls_apb_trng_get_entropy_isr,
};

#define LS_APB_TRNG_INIT(inst)								\
	static void irq_cfg_##inst(const struct device *dev)			\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    ls_apb_trng_isr, DEVICE_DT_INST_GET(inst), 0);	\
		irq_enable(DT_INST_IRQN(inst));				\
	}									\
	static struct entropy_ls_apb_trng_data data_##inst;			\
	static const struct entropy_ls_apb_trng_config				\
		cfg_##inst = {							\
		.regs = (volatile struct trng_apb_regs *)			\
			DT_INST_REG_ADDR(inst),          	\
		.irq_config_func = irq_cfg_##inst,     	\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, clocks),			\
			(.ccfg = LS_DT_CLK_CFG_ITEM(inst),))  	\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, resets),			\
			(.reset = RESET_DT_SPEC_INST_GET(inst),)) 	\
	};									\
	DEVICE_DT_INST_DEFINE(inst,					\
			      ls_apb_trng_init,				\
			      NULL,				\
			      &data_##inst,			\
			      &cfg_##inst,			\
			      PRE_KERNEL_1,			\
			      CONFIG_ENTROPY_INIT_PRIORITY, 	\
			      &ls_apb_trng_api);			\

DT_INST_FOREACH_STATUS_OKAY(LS_APB_TRNG_INIT)
