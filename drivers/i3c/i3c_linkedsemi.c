/*
 * Copyright (c) 2025 Linkedsemi Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*132*/


#include <string.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>
#if defined(CONFIG_PINCTRL)
    #include <zephyr/drivers/pinctrl.h>
#endif
#if defined(CONFIG_RESET)
    #include <zephyr/drivers/reset.h>
#endif
#if defined(CONFIG_CLOCK_CONTROL)
    #include <zephyr/drivers/clock_control.h>
    #include <soc_clock.h>
#endif

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(i3c,CONFIG_I3C_LOG_LEVEL);
#include "ls_i3c/ls_i3c_misc.h"
#include "ls_i3c/ls_ll_i3c.h"
#include "ls_i3c/reg_i3c.h"
#include "reg_sysc_app_per.h"
#include "HAL_def.h"
#include <zephyr/sys/util.h>
#include "core_rv32.h"

#define DT_DRV_COMPAT linkedsemi_i3c

#define PINCTRL_STATE_PINMUX_I2C PINCTRL_STATE_PRIV_START

#define I3C_SCLH_I2C_MIN_FM_NS  600ull
#define I3C_SCLH_I2C_MIN_FMP_NS 260ull
#define I3C_SCLL_OD_MIN_FM_NS   1320ull
#define I3C_SCLL_OD_MIN_FMP_NS  500ull
#define I3C_SCLL_OD_MIN_I3C_NS  200ull

#define I3C_SCLL_PP_MIN_NS  32ull
#define I3C_SCLH_I3C_MIN_NS 32ull
// #define I3C_SCLH_I3C_MIN_NS 64ull
// #define I3C_SCLH_I3C_MIN_NS 128ull

#define I3C_TBUF_FMP_MIN_NS 500.0
#define I3C_TBUF_FM_MIN_NS  1300.0
#define I3C_TCAS_MIN_NS     38.4

/* i3c target default confige*/
#define I3C_TARGET_OFFLINE_EN 1
#define I3C_TARGET_ENBALE_RADDOM_PART 0
#define I3C_TARGET_SOUPPORT_HDR 0  		// 	The hdr mode is not supported
#define I3C_TARGET_IGNORE_SE0SE1 0
/* MATCHSS=1 时 START/STOP sticky flag 只在 MATCHED 时置位：写匹配
 * ISR 清 MATCHED 后，restart（repeated START）的 START flag 被抑制，
 * 读请求（STREQRD）无中断可触发，TX 供数窗口被错过。改 0 = flag
 * 总是置位（驱动 START 块有 state==WR/RD 守卫，非匹配消息无害）。 */
#define I3C_TARGET_MATCH_START_STOP 0
#define I3C_TARGET_NACK_REQUEST 0
#define I3C_TARGET_ONCE_WRITE_TX_LEN 4

/* target event status  */
#define STATUS_EVDET_NONE            0
#define STATUS_EVDET_REQ_NOT_SENT    1
#define STATUS_EVDET_REQ_SENT_NACKED 2
#define STATUS_EVDET_REQ_SENT_ACKED  3

/* IBI 完成等待超时：总线忙时请求要排队等仲裁（如主机正在 ENTDAA
 * 长帧链、或大读传输期间），仲裁等待可达数 ms~数十 ms；超时路径会
 * 撤销 SCTRL.EVENT 请求，过短会把仍在合法等仲裁的 IBI 误杀。
 * 100ms 覆盖 DAA 场景，也是主机不存在时 ibi_raise 不永久阻塞的兜底。 */
#define I3C_IBI_COMPLETE_TIMEOUT   K_MSEC(100)

/* Private define for CCC command */
#define I3C_BROADCAST_RSTDAA          (0x00000006U)
#define I3C_BROADCAST_ENTDAA          (0x00000007U)

#define __LS_I3C_GET_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->EVR) &\
                                                    (__FLAG__)) == (__FLAG__)) ? SET : RESET)
#define __LS_I3C_MASTER_GET_ERROR(__HANDLE__) 			((__HANDLE__)->SER)
#define __LS_I3C_SLAVE_GET_ERROR(__HANDLE__) 	((__HANDLE__)->SERRWARN)
#define I3C_CHECK_FLAG(__ISR__, __FLAG__) 		((((__ISR__) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

#define LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT  	k_ms_to_cyc_ceil64(1)
/* 帧完成等待兜底超时（只服务 i3c_transfer 私有传输；CCC 不走此
 * 信号量）。3ms 对 legacy I2C 大帧偏紧（400kHz 下 256B ≈ 5.8ms
 * 会误超时），取 100ms。 */
#define LS_I3C_TRANSFER_TIMEOUT  				K_MSEC(100)

/* I3C target PID parsing */
#define GET_PID_VENDOR_ID(pid) (((uint64_t)pid >> 33) & 0x7fff) /* PID[47:33] */
#define GET_PID_ID_TYP(pid)    (((uint64_t)pid >> 32) & 0x1)    /* PID[32] */
#define GET_PID_PARTNO(pid)    (pid & 0xffffffff)               /* PID[31:0] */

#define I3C_TGT_INTSET_MASK                                                                        \
	(I3C_SINTSET_START_MASK | I3C_SINTSET_MATCHED_MASK | I3C_SINTSET_STOP_MASK |   \
	 I3C_SINTSET_DACHG_MASK | I3C_SINTSET_CCC_MASK | I3C_SINTSET_ERRWARN_MASK |    \
	 I3C_SINTSET_RXPEND_MASK| I3C_SINTSET_CHANDLED_MASK |I3C_SINTSET_NOWCNTLR_MASK|  \
	 I3C_SINTSET_EVENT_MASK | I3C_SINTSET_SLVRST_MASK)

struct i3c_fifo_info
{
    uint32_t                   ControllerTxFifoSize;
    uint32_t                   ControllerRxFifoSize;
    uint32_t                   TargetTxFifoSize;
	uint32_t                   TargetRxFifoSize;
};

/*target operation type */
enum ls_i3c_target_oper_state {
	LS_I3C_OP_STATE_IDLE,
	LS_I3C_OP_STATE_WR,
	LS_I3C_OP_STATE_RD,
	LS_I3C_OP_STATE_IBI,
	LS_I3C_OP_STATE_MAX,
};

// enum ls_i3c_sf_state {
// 	LS_I3C_SF_DAA,    /* Dynamic addressing state */
// 	LS_I3C_SF_CCC,    /* First part of CCC command state*/
// 	LS_I3C_SF_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
// 	LS_I3C_SF,        /* Private msg state */
// 	LS_I2C_SF,        /* I2C legacy msg state */
// 	LS_I3C_SF_IDLE,   /* Idle bus state */
// 	LS_I3C_SF_ERR,    /* Error state */
// 	LS_I3C_SF_INVAL,  /* Invalid state */
// };

enum ls_i3c_msg_state {
	LS_I3C_MSG_DAA,    /* Dynamic addressing state */
	LS_I3C_MSG_CCC,    /* First part of CCC command state*/
	LS_I3C_MSG_CCC_P2, /* Second part of CCC command state (used for direct commands)*/
	LS_I3C_MSG,        /* Private msg state */
	LS_I3C_MSG_IDLE,   /* Idle bus state */
	LS_I3C_MSG_ERR,    /* Error state */
	LS_I3C_MSG_INVAL,  /* Invalid state */
};

enum i3c_role {
	I3C_ROLE_CONTROLLER,
	I3C_ROLE_TARGET,
	I3C_ROLE_NONE,
};

/* Struct to hold the information about the current message on the bus */
struct ls_i3c_msg {
	uint8_t target_addr;         /* Current target xfer address */
	struct i3c_msg *i3c_msg_ptr; /* Pointer to the current private message to send on the bus */
	struct i3c_msg *i3c_msg_ctrl_ptr; /* Pointer to the private message that will be used by the
					   * control FIFO
					   */
	struct i3c_msg *i3c_msg_status_ptr; /* Pointer to the private message that will be used by
					     * the status FIFO
					     */
	struct i2c_msg *i2c_msg_ptr; /* Pointer to the current legacy message to send on the bus */
	struct i2c_msg *i2c_msg_ctrl_ptr; /* Pointer to the I2C legavy message that will be used by
					   * the control FIFO
					   */
	size_t num_msgs;                  /* Number of messages */
	size_t ctrl_msg_idx;              /* Current control message index */
	size_t status_msg_idx;            /* Current status message index */
	size_t xfer_msg_idx;              /* Current trasnfer message index */
	uint32_t cur_num_xfer;			  /* Current transffered data number*/
	uint32_t msg_type;                /* Either LL_I3C_CONTROLLER_MTYPE_PRIVATE or
					   * LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C
					   */
};

/* CTRL register options */
#define CTRL_EVENT_NORMAL    0
#define CTRL_EVENT_IBI       1
#define CTRL_EVENT_CNTLR_REQ 2
#define CTRL_EVENT_HJ        3


/* Driver config */
struct ls_i3c_config {
	struct i3c_driver_config common;

    /* Pointer to controller registers. */
	I3C_TypeDef   *base;
	uint32_t clock_frequency;
    void (*irq_config_func)(const struct device *dev);
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct ls_i3c_data {
    /* Common i3c driver data */
	struct i3c_driver_data common;

	const struct device *dev; /* 反向指针，供工作队列处理函数取回设备句柄 */
	/* SERRWARN 记账：ISR 只置位原始值不入队 LOG（LOG 分配/入队耗时
	 * 在 12.5MHz SCL 下会破坏总线时序），由系统工作队列线程消费打印 */
	atomic_t serrwarn_pending;
	struct k_work serrwarn_work;

	struct i3c_target_config *target_config;
	struct i3c_config_target config_target;
	struct i3c_fifo_info fifo_info;
	volatile enum ls_i3c_target_oper_state state;
	enum i3c_role cur_role;
	/** Mutex to serialize access */
	struct k_mutex lock;
	struct k_sem target_event_lock_sem; /* IBI 完成通知信号量：ibi_raise 发起后等待，ISR 终态 give */
	volatile int ibi_status; /* IBI 结果：0=ACKed, -EIO=NACKed；ISR 先写本字段再 give */
	struct k_sem device_sync_sem; /* controller :Sync between device communication messages */

	struct ls_i3c_msg curr_msg;
	enum ls_i3c_msg_state msg_state;  /* Current I3C bus state */
#ifdef CONFIG_I3C_USE_IBI
	struct {
		/* List of addresses used in the DEVICEx[] register. */
		uint8_t addr;

		/*
		 * True if target devices require mandatory byte
		 * for IBI.
		 */
		bool has_mandatory_byte;

		bool already_enabled;
	} ibi_inifo[4];
#endif
};

static void ls_i3c_log_err_type(const struct device *dev);
static void ls_i3c_serrwarn_work_handler(struct k_work *work);
static inline void ls_i3c_xfer_reset(I3C_TypeDef *base);

static int ls_i3c_cntlr_wave_init(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	// struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	I3C_TypeDef *base = config->base;

	uint64_t scll_od = 0;
	uint64_t sclh_i2c = 0;
	uint64_t scll_pp = 0;
	uint64_t sclh_i3c = 0;
	uint32_t clk_wave = 0;
	LOG_DBG("config->clock_frequency = %d\r\n",config->clock_frequency);
	LOG_DBG("i2c hz= %d ,i3c hz= %d \r\n",data->common.ctrl_config.scl.i2c,data->common.ctrl_config.scl.i3c);

	// if(data->common.ctrl_config.scl.i2c > 0 && config->common.dev_list.num_i2c > 0)
	if (data->common.ctrl_config.scl.i2c) {
#ifdef CONFIG_SOC_SERIES_LSQSH
		if (data->common.ctrl_config.scl.i2c >= 1000000) {
			/* LSQSH: force FM+ 1 MHz for best mixed-bus SDR throughput
			 * 内部时钟 = 主频 × 2（上下边沿采样）：
			 * - 主频 50MHz → 内部时钟 100MHz：scll_od = sclh_i2c = 49
			 * - 主频 25MHz → 内部时钟 50MHz：scll_od = sclh_i2c = 24
			 * SCL 周期 1us = 内部时钟周期 × (scll_od + 1 + sclh_i2c + 1 + 2) */
			if (config->clock_frequency >= 100000000) {
				scll_od = 49;
				sclh_i2c = 49;
			} else {
				scll_od = 24;
				sclh_i2c = 24;
			}
		} else if (data->common.ctrl_config.scl.i2c >= 400000) {
			/* LSQSH: allow FM 400 kHz, but warn that SDR will be slow */
			LOG_WRN("I2C SCL %u Hz below 1 MHz on LSQSH; "
				 "SDR throughput will be significantly reduced",
				 data->common.ctrl_config.scl.i2c);
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_FM_NS * config->clock_frequency,
					      1000000000ull) - 1;
			sclh_i2c = DIV_ROUND_UP(config->clock_frequency,
					      data->common.ctrl_config.scl.i2c) - scll_od - 2;
			if (sclh_i2c <
			    DIV_ROUND_UP(I3C_SCLH_I2C_MIN_FM_NS * config->clock_frequency,
					 1000000000ull) - 1) {
				LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at current I3C "
					"clock "
					"frequency for FM I2C bus");
				return -EINVAL;
			}
		} else {
			LOG_ERR("LSQSH I3C does not support I2C SCL below 400 kHz, got %u Hz",
				data->common.ctrl_config.scl.i2c);
			return -EINVAL;
		}
#else
		if (data->common.ctrl_config.scl.i2c > 400000) {
			/* I2C bus is FM+ */
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_FMP_NS * config->clock_frequency, 1000000000ull) - 1;
			sclh_i2c = DIV_ROUND_UP(config->clock_frequency, data->common.ctrl_config.scl.i2c) - scll_od - 2;
			if (sclh_i2c <
			    DIV_ROUND_UP(I3C_SCLH_I2C_MIN_FMP_NS * config->clock_frequency, 1000000000ull) - 1) {
				LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at "
					"current I3C clock "
					"frequency for FM+ I2C bus");
				return -EINVAL;
			}
		} else {
			/* I2C bus is FM */
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_FM_NS * config->clock_frequency, 1000000000ull) - 1;
			sclh_i2c = DIV_ROUND_UP(config->clock_frequency, data->common.ctrl_config.scl.i2c) - scll_od - 2;
			if (sclh_i2c <
			    DIV_ROUND_UP(I3C_SCLH_I2C_MIN_FM_NS * config->clock_frequency, 1000000000ull) - 1) {
				LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at current I3C "
					"clock "
					"frequency for FM I2C bus");
				return -EINVAL;
			}
		}
#endif
	} else {
		if (config->common.dev_list.num_i2c > 0) {
			LOG_ERR("I2C devices on bus but no I2C SCL frequency configured");
			return -EINVAL;
		}

		/* Pure I3C bus: shorten open-drain to ~4 MHz while meeting I3C minimums */
		scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_I3C_NS * config->clock_frequency,
				      1000000000ull) - 1;
		sclh_i2c = DIV_ROUND_UP(I3C_SCLH_I3C_MIN_NS * config->clock_frequency,
				      1000000000ull) - 1;
	}

	sclh_i3c = DIV_ROUND_UP(I3C_SCLH_I3C_MIN_NS * config->clock_frequency, 1000000000ull) - 1;
	scll_pp = DIV_ROUND_UP(config->clock_frequency, data->common.ctrl_config.scl.i3c) - sclh_i3c - 2;
	if (scll_pp < DIV_ROUND_UP(I3C_SCLL_PP_MIN_NS * config->clock_frequency, 1000000000ull) - 1) {
		LOG_ERR("Cannot find a combination of SCLL_PP and SCLH_I3C at current I3C clock "
			"frequency for specified I3C bus speed");
		return -EINVAL;
	}
	LOG_DBG("sclh_i2c = %lld,  scll_od = %lld,  sclh_i3c = %lld,  scll_pp = %lld,  \r\n",sclh_i2c,scll_od,sclh_i3c,scll_pp);
	clk_wave = ((uint32_t)sclh_i2c << 24) | ((uint32_t)scll_od << 16) |
		   ((uint32_t)sclh_i3c << 8) | (scll_pp);
	WRITE_REG(base->TIMINGR0, clk_wave);
	
	uint8_t free_timing = 0;
	uint8_t aval = 0;
	// if(config->common.dev_list.num_i2c > 0){
	if(data->common.ctrl_config.scl.i2c) {
		if (data->common.ctrl_config.scl.i2c > 400000) {
			/* Mixed bus with I2C FM+ device */
			free_timing = (uint8_t)(
				(I3C_TBUF_FMP_MIN_NS * config->clock_frequency / 1e9 - 0.5) / 2);
		} else {
			/* Mixed bus with I2C FM device */
			free_timing = (uint8_t)(
				(I3C_TBUF_FM_MIN_NS * config->clock_frequency / 1e9 - 0.5) / 2);
		}
	}else
	{
		/* Pure I3C bus */
		free_timing =
			(uint8_t)((I3C_TCAS_MIN_NS * config->clock_frequency / 1e9 - 0.5) / 2);
	}


    // REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_FREE,free_timing+3);
	REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_FREE,0x7f);
	/* SDA_HD：SDA hold time 档位随 I3C 内部计数时钟（= 主频 × 2）配置：
	 * 50MHz → 0，100MHz 及以上 → 1（config->clock_frequency 即内部计数时钟） */
	if (config->clock_frequency >= 100000000) {
		REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_SDA_HD,1);
	} else {
		REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_SDA_HD,0);
	}
    REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_ASNCR,0);

	aval = (uint8_t)(DIV_ROUND_UP(1000ull * config->clock_frequency, 1000000000ull) - 1);
	REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_AVAL,aval);

    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALL,2);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLC,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLD,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLT,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLA,1);

	return 0;
}

static int ls_i3c_target_config(const struct device *dev)
{
	const struct ls_i3c_config *dev_config = dev->config;
	struct ls_i3c_data *dev_data = dev->data;	
	struct i3c_config_target *config_target =  &dev_data->config_target;
	I3C_TypeDef *base = dev_config->base;
	base->SMAXLIMITS = (config_target->max_read_len<<I3C_SMAXLIMITS_MAXRD_POS)
						|(config_target->max_write_len<<I3C_SMAXLIMITS_MAXWR_POS); 

	base->SIDEXT &= ~(I3C_SIDEXT_BCR_MASK | I3C_SIDEXT_DCR_MASK);
	base->SIDEXT |= (config_target->bcr << I3C_SIDEXT_BCR_POS) | (config_target->dcr << I3C_SIDEXT_DCR_POS);
	base->SCONFIG &= ~I3C_SCONFIG_IDRAND_MASK; 
    base->SVENDORID &= ~I3C_SIDVID_VID_MASK;
    base->SVENDORID |= (uint32_t)GET_PID_VENDOR_ID(config_target->pid);

	base->SIDPARTNO = (uint32_t)GET_PID_PARTNO(config_target->pid);

	base->SCONFIG &= ~I3C_SCONFIG_SADDR_MASK;
	/* static_addr 是 7 位地址值，必须左移到 SADDR 域（bit31:25）。
	 * 直接 OR 会落到 bit[6:0] 污染 SLVNACK/S0IGNORE 等使能位
	 * （static-address=0 时无感，非 0 即事故）。 */
	base->SCONFIG |= ((uint32_t)config_target->static_addr << I3C_SCONFIG_SADDR_POS) &
			 I3C_SCONFIG_SADDR_MASK;
	return 0;

}

static int ls_i3c_configure(const struct device *dev, enum i3c_config_type type, void *config)
{
	const struct ls_i3c_config *dev_config = dev->config;
	struct ls_i3c_data *dev_data = dev->data;
	I3C_TypeDef *base = dev_config->base;
	
	int ret = 0;
	k_mutex_lock(&dev_data->lock, K_FOREVER);
	if (type == I3C_CONFIG_CONTROLLER) {
		struct i3c_config_controller *cntlr_cfg = config;

		if ((cntlr_cfg->scl.i2c == 0U) || (cntlr_cfg->scl.i3c == 0U)) {
			return -EINVAL;
		}
		if(cntlr_cfg->supported_hdr != 0)
		{
			LOG_ERR("The I3c controller supports only SDR mode");
			return -EINVAL;
		}

		dev_data->common.ctrl_config.scl.i3c = cntlr_cfg->scl.i3c;
		dev_data->common.ctrl_config.scl.i2c = cntlr_cfg->scl.i2c;

		ls_i3c_xfer_reset(base);

		uint32_t contr_en = REG_FIELD_RD(base->CFGR,I3C_CFGR_MASTER_EN);
		REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,0);
		ret = ls_i3c_cntlr_wave_init(dev);
		if(contr_en)
		{
			REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,1);
		}
	}
	else
	{
		struct i3c_config_target *config_target = config;
		memcpy(&dev_data->config_target,config_target,sizeof(struct i3c_config_target));
		ret = ls_i3c_target_config(dev);
	}
	k_mutex_unlock(&dev_data->lock);
	return ret;
}

static int ls_i3c_config_get(const struct device *dev, enum i3c_config_type type, void *config)
{
	struct ls_i3c_data *data = dev->data;

	if (config == NULL) {
		return -EINVAL;
	}

	if (type == I3C_CONFIG_CONTROLLER) {
		(void)memcpy(config, &data->common.ctrl_config, sizeof(data->common.ctrl_config));
	}else
	{
		(void)memcpy(config, &data->config_target, sizeof(data->config_target));
	}
	
	return 0;
}

static void ls_i3c_enable_target_interrupt(const struct device *dev, bool enable)
{
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;

	/* Disable the target interrupt events */
	base->SINTCLR = base->SINTSET;

	/* Clear the target interrupt status */
	base->SSTATUS = base->SSTATUS;

	/* Enable the target interrupt events */
	if (enable) {
		base->SINTSET = I3C_TGT_INTSET_MASK;
	}
}

static void ls_i3c_dev_init(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	struct i3c_config_target *config_target = &data->config_target;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;

	data->dev = dev;
	k_work_init(&data->serrwarn_work, ls_i3c_serrwarn_work_handler);

	/*controller basic config*/
	REG_FIELD_WR(base->CFGR,I3C_CFGR_CE3_ENABLE,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_TXTHRES,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_EXITPTRN,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_RSTPTRN,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_SMODE,0);
	// REG_FIELD_WR(base->CFGR,I3C_CFGR_STOP_MODE,1);
#if defined(CONFIG_I3C_USE_IBI)
	/* 允许应答 Hot-Join：HJACK=0 时 controller 会 NACK 总线上所有
	 * HJ 请求。HJ 事件经 HJF 中断进 IBI workqueue 处理，故随
	 * CONFIG_I3C_USE_IBI 使能。 */
	REG_FIELD_WR(base->CFGR,I3C_CFGR_HJACK,1);
#endif
	
	REG_FIELD_WR(base->SCONFIG2,I3C_SCONFIG2_TARGET_CLOCK_EN,1);
	REG_FIELD_WR(base->SCONFIG2,I3C_SCONFIG2_CCRCCC_EN,0);	
	REG_FIELD_WR(base->SCONFIG2,I3C_SCONFIG2_CONTROLLER_RESET_PATTERN_DELAY,1);

	ls_i3c_target_config(dev);
	uint8_t matchCount = (uint8_t)(config->clock_frequency/1e6);
	uint32_t sconfigvalue = 0;
	sconfigvalue = base->SCONFIG;
	sconfigvalue &= 
		~(I3C_SCONFIG_SADDR_MASK|I3C_SCONFIG_BAMATCH_MASK
		|I3C_SCONFIG_OFFLINE_MASK|I3C_SCONFIG_IDRAND_MASK
		|I3C_SCONFIG_DDROK_MASK);
	sconfigvalue |= 
	(config_target->static_addr<<I3C_SCONFIG_SADDR_POS)
	|(matchCount << I3C_SCONFIG_BAMATCH_POS)|(I3C_TARGET_OFFLINE_EN <<I3C_SCONFIG_OFFLINE_POS)
	|(I3C_TARGET_ENBALE_RADDOM_PART <<I3C_SCONFIG_IDRAND_POS)|(I3C_TARGET_SOUPPORT_HDR << I3C_SCONFIG_DDROK_POS)
	|(I3C_TARGET_IGNORE_SE0SE1 <<I3C_SCONFIG_S0IGNORE_POS)|(I3C_TARGET_MATCH_START_STOP << I3C_SCONFIG_MATCHSS_POS)
	|(I3C_TARGET_NACK_REQUEST <<I3C_SCONFIG_SLVNACK_POS) ;// |I3C_SCONFIG_SLVENA_MASK
	base->SCONFIG = sconfigvalue;

	if (I3C_BCR_DEVICE_ROLE(config_target->bcr) == I3C_BCR_DEVICE_ROLE_I3C_CONTROLLER_CAPABLE)
	{
		uint32_t contr_en = REG_FIELD_RD(base->CFGR,I3C_CFGR_MASTER_EN);
		if(ls_i3c_cntlr_wave_init(dev) != 0)
		{
			LOG_ERR("i3c initialization failed : %s",__func__);
			return;
		}
		if(contr_en)
		{
			REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,1);
		}

		if(ctrl_config->is_secondary == true)
		{
			/*secondary controller(target)*/
			config_target->enable = true;
			data->cur_role = I3C_ROLE_TARGET;
			REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,0);
			REG_FIELD_WR(base->SCONFIG,I3C_SCONFIG_SLVENA,1);
		}else
		{
			/*controller*/
			config_target->enable = false;
			data->cur_role = I3C_ROLE_CONTROLLER;
			REG_FIELD_WR(base->SCONFIG,I3C_SCONFIG_SLVENA,0);
			REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,1);
		}
	}else
	{
		/*only taraget*/
		config_target->enable = true;
		data->cur_role = I3C_ROLE_TARGET;
		REG_FIELD_WR(base->SCONFIG,I3C_SCONFIG_SLVENA,1);
		REG_FIELD_WR(base->CFGR,I3C_CFGR_MASTER_EN,0);
	}
}

#include "ls_soc_gpio.h"
#define I3C10_SCL PJ00
#define I3C10_SDA PJ01

#define I3C9_SCL PJ02
#define I3C9_SDA PJ03
/**
 * @brief Initialize the hardware.
 *
 * @param dev Pointer to controller device driver instance.
 */
static int ls_i3c_init(const struct device *dev)
{
	const struct ls_i3c_config *dev_config = dev->config;
	struct ls_i3c_data *data = dev->data;
	struct i3c_config_controller *ctrl_config = &data->common.ctrl_config;
	I3C_TypeDef *base = (I3C_TypeDef *)dev_config->base;
    int ret = 0;

	ret = i3c_addr_slots_init(dev);
	if (ret != 0) {
		return ret;
	}

	(void)ctrl_config;

	/* Set I3C_PD operational */
	// ret = clock_control_on(clk_dev, (clock_control_subsys_t)&config->clock_subsys);
	// if (ret < 0) {
	// 	LOG_ERR("Turn on I3C clock fail %d", ret);
	// 	return ret;
	// }


#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        if (!device_is_ready(clk_dev)) {
            LOG_ERR("%s device not ready", clk_dev->name);
            return -ENODEV;
        }
        clock_control_off(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_RESET)
    if (dev_config->reset.dev != NULL) {
        if (!device_is_ready(dev_config->reset.dev)) {
            LOG_ERR("Reset controller device is not ready");
            return -ENODEV;
        }

        ret = reset_line_toggle(dev_config->reset.dev, dev_config->reset.id);
        if (ret != 0) {
            LOG_ERR("toggle reset line failed");
            return ret;
        }
    }
#endif

#if defined(CONFIG_CLOCK_CONTROL)
    if (dev_config->ccfg.cctl_dev) {
        const struct device *clk_dev = dev_config->ccfg.cctl_dev;
        clock_control_on(clk_dev, (clock_control_subsys_t)&dev_config->ccfg);
    }
#endif

#if defined(CONFIG_PINCTRL)
    ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("%s: Could not configure pins", dev->name);
    }
#endif

	data->state = LS_I3C_OP_STATE_IDLE;
	data->fifo_info.ControllerTxFifoSize = 16;
	data->fifo_info.ControllerRxFifoSize = 16;
	data->fifo_info.TargetTxFifoSize = 16;
	data->fifo_info.TargetRxFifoSize = 8;

#if defined(CONFIG_I3C_USE_IBI)
	/* IBIIE：target IBI；HJIE：Hot-Join（controller 侧 ACK 由 CFGR.HJACK 控制，
	 * 两者需同时使能 HJ 才能完整闭环：ACK → HJF 中断 → workqueue 重 DAA） */
	base->IER = I3C_IER_IBIIE_MASK | I3C_IER_HJIE_MASK;
#else
	base->IER = 0;
#endif
	/* Initial I3C device as controller or target */
	ls_i3c_dev_init(dev);

	k_sem_init(&data->target_event_lock_sem, 0, 1);
	data->ibi_status = 0;
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);
    k_mutex_init(&data->lock);

	dev_config->irq_config_func(dev);

	if(data->cur_role == I3C_ROLE_CONTROLLER)
	{
		LOG_DBG("I3C_ROLE_CONTROLLER started\n");
		/* Perform bus initialization */
		if(dev_config->common.dev_list.num_i3c > 0)
		{
			ret = i3c_bus_init(dev, &dev_config->common.dev_list);
		}
	}else
	{
		LOG_DBG("I3C_ROLE_TARGET started\n");
	}

	return ret;
}


static int i3c_ls_curr_msg_init(const struct device *dev, struct i3c_msg *i3c_msgs,
				   struct i2c_msg *i2c_msgs, uint8_t num_msgs, uint8_t tgt_addr)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;

	/* Either should be NULL */
	__ASSERT(!(i3c_msgs == NULL && i2c_msgs == NULL), "Both i3c_msgs and i2c_msgs are NULL");
	__ASSERT(!(i3c_msgs != NULL && i2c_msgs != NULL),
		 "Both i3c_msgs and i2c_msgs are not NULL");

	curr_msg->target_addr = tgt_addr;
	curr_msg->num_msgs = num_msgs;
	curr_msg->ctrl_msg_idx = 0;
	curr_msg->status_msg_idx = 0;
	curr_msg->xfer_msg_idx = 0;
	curr_msg->cur_num_xfer = 0;
	/* I3C private message */
	if (i2c_msgs == NULL) {
		curr_msg->msg_type = LL_I3C_CONTROLLER_MTYPE_PRIVATE;
		curr_msg->i3c_msg_ptr = i3c_msgs;
		curr_msg->i3c_msg_ctrl_ptr = i3c_msgs;
		curr_msg->i3c_msg_status_ptr = i3c_msgs;
	} else {
		/* Legacy I2C message */
		curr_msg->msg_type = LL_I3C_CONTROLLER_MTYPE_LEGACY_I2C;
		curr_msg->i2c_msg_ptr = i2c_msgs;
		curr_msg->i2c_msg_ctrl_ptr = i2c_msgs;
	}

	if(curr_msg->msg_type == LL_I3C_CONTROLLER_MTYPE_PRIVATE)
	{
		if(i3c_msgs->flags & I3C_MSG_NBCH)
		{
			/* Disable arbitration header */
			LL_I3C_DisableArbitrationHeader(base);
		}
		else
		{
			/* Enable arbitration header */
			LL_I3C_EnableArbitrationHeader(base);
		}
	}else
	{
		/* 验证时发现，有些I3C兼容I2C的主机，如果帧开始处添加了仲裁头，会导致与I2C通信失败*/
		LL_I3C_DisableArbitrationHeader(base);
	}
	return 0;
}

static bool ls_i3c_curr_msg_is_i3c(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	return (curr_msg->msg_type == LL_I3C_CONTROLLER_MTYPE_PRIVATE);
}

/**
 * @brief Fills the I3C TX FIFO from a given buffer
 *
 * @param buf The buffer to fill the TX FIFO from
 * @param len The total buffer length
 * @param offset Pointer to the offset from the beginning of buffer which will be incremented by the
 * number of bytes sent to the TX FIFO
 *
 * @return Returns true if last byte was sent (TXLAST flag was set)
 */
static bool ls_i3c_fill_tx_fifo(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	I3C_TypeDef *base = config->base;
	uint8_t *buf;
	size_t len;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		buf = curr_msg->i3c_msg_ptr->buf;
		len = curr_msg->i3c_msg_ptr->len;
	} else {
		buf = curr_msg->i2c_msg_ptr->buf;
		len = curr_msg->i2c_msg_ptr->len;
	}

	while (LL_I3C_IsActiveFlag_TXFNF(base)) {

		LL_I3C_TransmitData8(base, buf[curr_msg->cur_num_xfer]);
		// curr_msg->i3c_msg_ptr->num_xfer++;
		curr_msg->cur_num_xfer++;
		if(curr_msg->cur_num_xfer == len)
		{
			return true;
		}
	}

	return false;
}

/**
 * @brief Drains the I3C RX FIFO from a given buffer
 *
 * @param buf The buffer to drain the RX FIFO to
 * @param len The total buffer length
 * @param offset Pointer to the offset from the beginning of buffer which will be incremented by the
 * number of bytes drained from the RX FIFO
 *
 * @return Returns true if last byte was received (RXLAST flag was set)
 */
static bool ls_i3c_drain_rx_fifo(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	I3C_TypeDef *base = config->base;
	uint8_t *buf;
	size_t len;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		buf = curr_msg->i3c_msg_ptr->buf;
		len = curr_msg->i3c_msg_ptr->len;
	} else {
		buf = curr_msg->i2c_msg_ptr->buf;
		len = curr_msg->i2c_msg_ptr->len;
	}

	/* 读取条件：RXFNE 标志 或 RXCOUNT>0（帧末兜底路径下标志可能因
	 * word 阈值/同步滞后不置位，但 FIFO 里确实有字节） */
	uint32_t rxc = (base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >>
		       I3C_SDATACTRL_RXCOUNT_POS;
	if (LL_I3C_IsActiveFlag_RXFNE(base) || rxc > 0) {
		/* 防越界：帧末兜底排空可能因标志/计数滞后多读，
		 * 超出 len 的字节丢弃，不写 buf、不计数 */
		if (curr_msg->cur_num_xfer < len) {
			buf[curr_msg->cur_num_xfer] = LL_I3C_ReceiveData8(base);
			curr_msg->cur_num_xfer++;
		} else {
			(void)LL_I3C_ReceiveData8(base);
		}
		if(curr_msg->cur_num_xfer == len)
		{
			return true;
		}
	}

	return false;
}

static int ls_i3c_curr_msg_xfer_next(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	if (curr_msg->xfer_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_ptr->num_xfer = curr_msg->cur_num_xfer;
		curr_msg->i3c_msg_ptr++;
	} else {
		curr_msg->i2c_msg_ptr++;
	}

	curr_msg->xfer_msg_idx++;
	curr_msg->cur_num_xfer = 0;
	return 0;
}


static void ls_i3c_event_isr_tx(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	switch (data->msg_state) {
	case LS_I3C_MSG: {

		/* 读消息不填 TX FIFO：TXFNF 在读段期间恒置位（TX FIFO 总有
		 * 空间），不拦截会把读缓冲区（未初始化）的内容写进 TX FIFO
		 * ——幻影字节会在同帧下一个写段被先发出去，且 xfer_msg_idx
		 * 被误推进导致该读的 RX 字节错位。方向按 xfer 指针
		 * （control 指针跑在前面）。 */
		if (curr_msg->xfer_msg_idx < curr_msg->num_msgs) {
			bool is_read = ls_i3c_curr_msg_is_i3c(dev) ?
				((curr_msg->i3c_msg_ptr->flags & I3C_MSG_READ) != 0) :
				((curr_msg->i2c_msg_ptr->flags & I2C_MSG_READ) != 0);
			if (is_read) {
				break;
			}
		}

		if (ls_i3c_fill_tx_fifo(dev)) {
			ls_i3c_curr_msg_xfer_next(dev);
		}

		break;
	}
	case LS_I3C_MSG_DAA: {

		break;
	}
	case LS_I3C_MSG_CCC: {

		break;
	}
	case LS_I3C_MSG_CCC_P2: {

		break;
	}
	default:
		break;
	}
}

static void ls_i3c_event_isr_rx(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;

	switch (data->msg_state) {
	case LS_I3C_MSG: {

		if (ls_i3c_drain_rx_fifo(dev)) {
			ls_i3c_curr_msg_xfer_next(dev);
		}

		break;
	}
	case LS_I3C_MSG_DAA: {
		break;
	}
	case LS_I3C_MSG_CCC_P2: {
		break;
	}
	default:
		break;
	}
}

static int ls_i3c_curr_msg_status_next(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	if (curr_msg->status_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_status_ptr++;
	}else {
		curr_msg->i2c_msg_ctrl_ptr++;
	}
	curr_msg->status_msg_idx++;
	return 0;
}

static int ls_i3c_curr_msg_status_update_num_xfer(const struct device *dev, size_t num_xfer)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	if (curr_msg->status_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	/* Legacy I2C messages do not have num_xfer */
	if (ls_i3c_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_status_ptr->num_xfer = num_xfer;
	}

	return 0;
}

static int ls_i3c_curr_msg_control_get_dir(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		return (((curr_msg->i3c_msg_ctrl_ptr->flags & I3C_MSG_READ) == I3C_MSG_READ)
				? LL_I3C_DIRECTION_READ
				: LL_I3C_DIRECTION_WRITE);
	}

	return (((curr_msg->i2c_msg_ctrl_ptr->flags & I2C_MSG_READ) == I2C_MSG_READ)
			? LL_I3C_DIRECTION_READ
			: LL_I3C_DIRECTION_WRITE);
}

static int ls_i3c_curr_msg_control_get_len(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	return (ls_i3c_curr_msg_is_i3c(dev)) ? curr_msg->i3c_msg_ctrl_ptr->len
						: curr_msg->i2c_msg_ctrl_ptr->len;
}

static int ls_i3c_curr_msg_control_get_end(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	if (ls_i3c_curr_msg_is_i3c(dev))
	{
		return ((curr_msg->ctrl_msg_idx < (curr_msg->num_msgs - 1)) ? LL_I3C_GENERATE_RESTART
										: LL_I3C_GENERATE_STOP);
	}else
	{
		if (curr_msg->ctrl_msg_idx >= (curr_msg->num_msgs -1)) {
			return LL_I3C_GENERATE_STOP;
		}
		/* 获取下一个I2C执行中的flag信息，但是指针不能在此处变动 */
		struct i2c_msg *next_i2c_msg = &curr_msg->i2c_msg_ctrl_ptr[1];
		return (((next_i2c_msg->flags & I2C_MSG_RESTART) == I2C_MSG_RESTART) ? LL_I3C_GENERATE_RESTART
										: LL_I3C_GENERATE_STOP);

		// return ((curr_msg->ctrl_msg_idx < (curr_msg->num_msgs - 1)) ? LL_I3C_GENERATE_RESTART
										// : LL_I3C_GENERATE_STOP);
	}

}

static int ls_i3c_curr_msg_control_next(const struct device *dev)
{
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;

	if (curr_msg->ctrl_msg_idx >= curr_msg->num_msgs) {
		LOG_ERR("No more messages left");
		return -EFAULT;
	}

	if (ls_i3c_curr_msg_is_i3c(dev)) {
		curr_msg->i3c_msg_ctrl_ptr++;
	} else {
		curr_msg->i2c_msg_ctrl_ptr++;
	}

	curr_msg->ctrl_msg_idx++;

	return 0;
}

static void ls_i3c_event_isr_cf(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	I3C_TypeDef *base = config->base;

	switch (data->msg_state) {
	case LS_I3C_MSG: {
		/* 所有控制字发完后 CFNF 仍可能因 C-FIFO 腾空再次触发，
		 * 越界读 i3c_msg_ctrl_ptr 会发出幻影消息，必须拦截 */
		if (curr_msg->ctrl_msg_idx >= curr_msg->num_msgs) {
			break;
		}
		LL_I3C_ControllerHandleMessage(
			base, curr_msg->target_addr, ls_i3c_curr_msg_control_get_len(dev),
			ls_i3c_curr_msg_control_get_dir(dev), curr_msg->msg_type,
			ls_i3c_curr_msg_control_get_end(dev));

		ls_i3c_curr_msg_control_next(dev);
		break;
	}
	case LS_I3C_MSG_CCC:
	case LS_I3C_MSG_CCC_P2: {
		break;
	}
	default:
		break;
	}
}


static int ls_i3c_request_transfer_flag(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;

	data->msg_state = LS_I3C_MSG;
	// data->sf_state = LS_I3C_SF;

	/* 防上一帧 ERR+FCF 双 give 的信号量残留：错误帧（如地址 NACK）时
	 * ISR 的 ERR 分支和 FC 分支都会 give device_sync_sem，take 只消费
	 * 一个，残留计数会让下一笔传输的 k_sem_take 立即返回（不等帧完成
	 * 就上报成功）。每帧开始前重置计数。 */
	k_sem_reset(&data->device_sync_sem);

	LL_I3C_RequestTransfer(base);

	/* Wait for whole transfer to complete */
	if (k_sem_take(&data->device_sync_sem, LS_I3C_TRANSFER_TIMEOUT) != 0) {
		return -ETIMEDOUT;
	}

	/* 帧末残余字节收尾（线程上下文）：FCF 置位时接收流水线的最后几个
	 * 字节可能还没进 RX FIFO，且 RXFNE/RXCOUNT 跨时钟域同步滞后。
	 * 先驻留 ~5us 等同步再排空，并给提前终止（target T-bit）的在读
	 * 消息落账 num_xfer——这类消息不触发 xfer_next（没收满 len），
	 * 不落账应用会看到 actual=0。 */
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	if (data->msg_state != LS_I3C_MSG_ERR &&
	    curr_msg->xfer_msg_idx < curr_msg->num_msgs &&
	    ls_i3c_curr_msg_is_i3c(dev)) {
		size_t len = curr_msg->i3c_msg_ptr->len;

		if (curr_msg->cur_num_xfer < len) {
			k_busy_wait(5);
			/* 排空与 ISR 的 RXFNE 分支互斥：残余字节在本窗口内落地
			 * 时 ISR 可能也在 drain，cur_num_xfer/buf 是共享状态，
			 * 不互斥会重复读/错位 */
			unsigned int key = irq_lock();
			for (int drain_i = 0; drain_i < 64; drain_i++) {
				uint32_t rxc = (base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >>
					       I3C_SDATACTRL_RXCOUNT_POS;
				if (!LL_I3C_IsActiveFlag_RXFNE(base) && rxc == 0) {
					break;
				}
				if (ls_i3c_drain_rx_fifo(dev)) {
					break;
				}
			}
			/* 落账（含未收满的提前终止消息） */
			curr_msg->i3c_msg_ptr->num_xfer = curr_msg->cur_num_xfer;
			irq_unlock(key);
		}
	}

	/* 兜底：FC 中断先于 ERR 置位的竞态窗口（ISR 已复查，这里再保险
	 * 一次）。必须读 EVR/SER 活标志而非只看 msg_state：ERRF（EVR）
	 * 跨时钟域可能晚同步甚至不可见，msg_state 来不及更新；SER 是
	 * 错误详情寄存器（ANACK/DNACK 等），与 ERRF 独立同步，两个一起查。 */
	if (LL_I3C_IsActiveFlag_ERR(base) ||
	    (READ_REG(base->SER) & (I3C_SER_ANACK_MASK | I3C_SER_DNACK_MASK |
				    I3C_SER_COVR_MASK | I3C_SER_DOVR_MASK)) != 0) {
		LL_I3C_ClearFlag_ERR(base);
		ls_i3c_log_err_type(dev);
		data->msg_state = LS_I3C_MSG_ERR;
	}

	if (data->msg_state == LS_I3C_MSG_ERR) {
		return -EIO;
	}

	return 0;
}


/**
 * @brief Perform Dynamic Address Assignment.
 *
 * @see i3c_do_daa
 *
 * @param dev Pointer to controller device driver instance.
 *
 * @return @see i3c_do_daa
 */
static int ls_i3c_do_daa(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
    I3C_TypeDef *base = (I3C_TypeDef *)config->base;
    uint8_t rx_buf[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    int ret = 0;
    k_mutex_lock(&data->lock, K_FOREVER);

    /* Enable arbitration header */
    LL_I3C_EnableArbitrationHeader(base);
    LL_I3C_ControllerHandleCCC(base, I3C_BROADCAST_ENTDAA, 0U, LL_I3C_GENERATE_STOP);

	/* DAA 主循环只靠 FCF/ERRF 退出：总线上无设备应答或目标异常时
	 * 两个标志都可能永远不来，会死循环挂死。加 100ms 兜底超时。 */
	uint64_t daa_exp = arch_k_cycle_get_64() + k_ms_to_cyc_ceil64(100);
	do
	{
		if (arch_k_cycle_get_64() > daa_exp)
		{
			LOG_ERR("%s: DAA timeout", dev->name);
			ret = -ETIMEDOUT;
			goto out_daa;
		}
		if (__LS_I3C_GET_FLAG(base, I3C_EVR_TXFNFF) == SET)
		{
			for (uint32_t index = 0U; index < 8U; index++)
			{
			/* Retrieve payload byte by byte */
				rx_buf[index] = ((uint8_t)LL_I3C_ReceiveData8(base));
			}
			struct i3c_device_desc *target;
			uint16_t vendor_id;
			uint32_t part_no;
			uint64_t pid;
			uint8_t dyn_addr;
			/* Vendor ID portion of Provisioned ID */
			vendor_id = (((uint16_t)rx_buf[0] << 8U) | (uint16_t)rx_buf[1]) &
				    0xFFFEU;

			/* Part Number portion of Provisioned ID */
			part_no = (uint32_t)rx_buf[2] << 24U | (uint32_t)rx_buf[3] << 16U |
				  (uint32_t)rx_buf[4] << 8U | (uint32_t)rx_buf[5];

			/* ... and combine into one Provisioned ID */
			pid = (uint64_t)vendor_id << 32U | (uint64_t)part_no;

			/* assigned_okay=true：能应答 ENTDAA 的一定是已丢失动态地址的
			 * 设备（如 RSTDAA 后的 HJ 重连），其 desc 里可能还留着旧地址；
			 * 允许直接复用原地址，既避免 helper 误报 -EINVAL 中断 DAA，
			 * 也让重连后地址保持不变，上层无需重新发现设备 */
			ret = i3c_dev_list_daa_addr_helper(&data->common.attached_dev.addr_slots,
							   &config->common.dev_list, pid,
							   false, true,
							   &target, &dyn_addr);
			if (ret != 0) {
				LOG_ERR("TARGET device address dose not match");
				LOG_ERR("DAA: Rcvd PID 0x%04x%08x", vendor_id, part_no);
				goto out_daa;
			}

			if (target == NULL) {
				/* 设备树中没有注册对应的从机设备子节点，使用随机分配地址来保证后续的其他设备能够继续正常走通 */
				LOG_DBG("%s: PID 0x%04x%08x is not in registered device "
					"list, given dynamic address 0x%02x",
					dev->name, vendor_id, part_no, dyn_addr);
			} else {
				/* Update target descriptor */
				target->dynamic_addr = dyn_addr;
				target->bcr = rx_buf[6];
				target->dcr = rx_buf[7];
			}

			/* Mark the address as I3C device */
			i3c_addr_slots_mark_i3c(&data->common.attached_dev.addr_slots, dyn_addr);
			
			/*
			 * If the device has static address, after address assignment,
			 * the device will not respond to the static address anymore.
			 * So free the static one from address slots if different from
			 * newly assigned one.
			 */
			if ((target != NULL) &&(target->static_addr != 0U) && (dyn_addr != target->static_addr)) {
				i3c_addr_slots_mark_free(&data->common.attached_dev.addr_slots,
							 target->static_addr);
			}

			/* Check if Tx FIFO requests data */
			if (__LS_I3C_GET_FLAG(base, I3C_EVR_TXFNFF) == SET)
			{
				/* Write device address in the TDR register */
				LL_I3C_TransmitData8(base, dyn_addr<<1);
				LOG_INF("PID 0x%04x%08x assigned dynamic address 0x%02x",
					vendor_id, part_no, dyn_addr);
			}
			// else
			// {
			// 	ret = -EBUSY;
			// 	goto out_daa;
			// }
		}
	} while (__LS_I3C_GET_FLAG(base,I3C_EVR_FCF_MASK) != SET
		&& __LS_I3C_GET_FLAG(base,I3C_EVR_ERRF_MASK) != SET);
	

out_daa:
	/* 等待帧收尾：加有界等待，防止总线异常时永久挂死 */
	{
		uint64_t stop_exp = arch_k_cycle_get_64() + LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT * 4;
		while((READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0)
		{
			if (arch_k_cycle_get_64() > stop_exp) {
				LOG_ERR("%s: DAA wait-frame-complete timeout", dev->name);
				return -ETIMEDOUT;
			}
		}
	}

	if(__LS_I3C_GET_FLAG(base,I3C_EVR_ERRF_MASK))
	{
		ret = -EIO;
		ls_i3c_log_err_type(dev);
		LL_I3C_ClearFlag_ERR(base);
	}

	if(__LS_I3C_GET_FLAG(base,I3C_EVR_FCF_MASK))
	{
		/* Clear frame complete flag */
		LL_I3C_ClearFlag_FC(base);
	}
	ls_i3c_xfer_reset(base);
	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Prepare the controller for transfers.
 *
 * This is simply a wrapper to clear out status bits,
 * and error bits. Also this tells the controller to
 * flush both TX and RX FIFOs.
 *
 * @param base Pointer to controller registers.
 */
static inline void ls_i3c_xfer_reset(I3C_TypeDef *base)
{
    LL_I3C_RequestStatusFIFOFlush(base);
	while(LL_I3C_IsActiveFlag_RXFNE(base))
	{
    	LL_I3C_RequestRxFIFOFlush(base);
	}
	while(!LL_I3C_IsActiveFlag_TXFE(base))
	{
    	LL_I3C_RequestTxFIFOFlush(base);
	}
	while(!LL_I3C_IsActiveFlag_CFE(base))
	{
		LL_I3C_RequestControlFIFOFlush(base);
	}
	//i3c core_clk 和 pbus_clk之间可能存在时间差，需等待确保fifo状态为空
}

/**
 * @brief Send Common Command Code (CCC).
 *
 * @see i3c_do_ccc
 *
 * @param dev Pointer to controller device driver instance.
 * @param payload Pointer to CCC payload.
 *
 * @return @see i3c_do_ccc
 */
static int ls_i3c_do_ccc(const struct device *dev,
			   struct i3c_ccc_payload *payload)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;
	
	int ret = 0;
	uint32_t exit_condition = 0;

	uint64_t start_time; 
	uint64_t exp_time;
	uint8_t timeout = false;	

	if (payload == NULL) {
		return -EINVAL;
	}

	if(data->cur_role != I3C_ROLE_CONTROLLER)
	{
		LOG_ERR("%s: the i3c device current role is not controller", dev->name);
		return -EINVAL;
	}

	if (config->common.dev_list.num_i3c == 0) {
		/*
		 * No i3c devices in dev tree. Just return so
		 * we don't get errors doing cmds when there
		 * are no devices listening/responding.
		 */
		return 0;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	// unsigned int k = arch_irq_lock();

	LOG_DBG("CCC[0x%02x]", payload->ccc.id);
	LOG_DBG("CCC len  :  %d", payload->ccc.data_len);
	if(payload->targets.num_targets > 0)
	{
		LOG_DBG("payload->targets.payloads->data_len  :  %d", payload->targets.payloads->data_len);
		
	}
	/* Emit START */

	/* CCC 不做 device_sync_sem 等待：该信号量只在 FCF 中断里 give，
	 * 而 CCC 路径不开 FC 中断（开了反而会被 ISR 的 RXFNE 排空抢走
	 * CCC 读数据），等信号量必然白等满超时。CCC 是自含轮询路径
	 * （phase1/2 自己等 FCF/ERRF），控制字进 C-FIFO 后由硬件在总线
	 * 空闲时执行，无需软件握手。
	 * 注：i3c_transfer 路径仍走 ls_i3c_request_transfer_flag（它开 FC
	 * 中断，信号量有人给）。 */
	/* 先清 FIFO 再发起请求：C-FIFO 若有残留控制字，先 RequestTransfer
	 * 会把残留字先发出去；且 flush 与刚启动的帧可能打架。 */
	ls_i3c_xfer_reset(base);

	data->msg_state = LS_I3C_MSG;
	LL_I3C_RequestTransfer(base);
	
	if(i3c_ccc_is_payload_broadcast(payload) == true)
	{
		LL_I3C_ControllerHandleCCC(base,payload->ccc.id,payload->ccc.data_len,LL_I3C_GENERATE_STOP);
		start_time = arch_k_cycle_get_64(); 
		exp_time = LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT + start_time;	
		do{
			if((__LS_I3C_GET_FLAG(base, I3C_EVR_TXFNFF) == SET) && (payload->ccc.data_len !=0))
			{
				LL_I3C_TransmitData8(base, (uint8_t)*payload->ccc.data);
				payload->ccc.data++;
				payload->ccc.data_len--;
			}

			if(arch_k_cycle_get_64() > exp_time)
			{
				LOG_ERR("%s: CCC[0x%02x] broadcast timeout", dev->name, payload->ccc.id);
				timeout = true;
				ret = -ETIMEDOUT;
			}

			exit_condition = ((base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (exit_condition == 0U && timeout == false);

		if(exit_condition == I3C_EVR_ERRF)
		{
			ret = -EIO;
			LOG_ERR("%s: CCC[0x%02x] broadcast error, SER=0x%X", dev->name, payload->ccc.id, base->SER);
		}

		if(timeout == true)
		{
			LOG_ERR("%s: CCC[0x%02x] broadcast transfer timeout", dev->name, payload->ccc.id);
			ret = -ETIMEDOUT;
		}
		
		goto out_ccc_stop;
	}
	else
	{
		struct i3c_ccc_target_payload *new_tgt = payload->targets.payloads;
		struct i3c_ccc_target_payload *cur_tgt = new_tgt;
		uint8_t num_target = payload->targets.num_targets;
		uint8_t cur_tgt_idx = 1;
		uint32_t transfer_num = 0;
		if(num_target == 0)
		{
			LOG_ERR("%s: CCC[0x%02x] direct num_target error", dev->name, payload->ccc.id);
			ret = -EIO;
			goto out_ccc_stop;
		}
		/* 关键：num_xfer 是本驱动的收/发计数器，必须清零。
		 * i3c_ccc.c 各 helper 的 ccc_tgt_payload 是栈变量且从不 memset，
		 * num_xfer 带着栈垃圾进来，导致收数字节被判为"超出期望"丢弃、
		 * transfer_num 对不上报 amount mismatch。 */
		for (int i = 0; i < num_target; i++)
		{
			payload->targets.payloads[i].num_xfer = 0;
		}
		start_time = arch_k_cycle_get_64(); 
		exp_time = LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT * num_target + start_time;
		LL_I3C_ControllerHandleCCC(base,payload->ccc.id,payload->ccc.data_len,LL_I3C_GENERATE_RESTART);
		do{
			/*tx-fifo depth is 16 ,so broadcast tx data can be fully input, broadcast tx data is not too long*/
			if((__LS_I3C_GET_FLAG(base, I3C_EVR_TXFNFF) == SET) && (payload->ccc.data_len !=0))
			{
				LL_I3C_TransmitData8(base, (uint8_t)*payload->ccc.data);
				payload->ccc.data++;
				payload->ccc.data_len--;
			}

			if(arch_k_cycle_get_64() > exp_time)
			{
				LOG_ERR("%s: CCC[0x%02x] direct phase1 timeout", dev->name, payload->ccc.id);
				timeout = true;
				ret = -ETIMEDOUT;
			}
			exit_condition = ((base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (payload->ccc.data_len > 0 && exit_condition == 0);

		if(exit_condition == I3C_EVR_ERRF)
		{
			ret = -EIO;
			LOG_ERR("%s: CCC[0x%02x] direct phase1 error, SER=0x%X", dev->name, payload->ccc.id, base->SER);
			goto out_ccc_stop;
		}

		start_time = arch_k_cycle_get_64(); 
		exp_time = LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT * num_target + start_time;	
		uint32_t rnw;
		do{
			/* num_target==0 后不再发控制字：CFNFF（控制 FIFO 未满）
			 * 在最后一条消息发出后仍可能置位，不拦截会拿着已越界的
			 * new_tgt 再发一条垃圾消息并污染 transfer_num */
			if (__LS_I3C_GET_FLAG(base, I3C_EVR_CFNFF) == SET && num_target > 0)
			{
				if(new_tgt->rnw == 1)
				{
					rnw = LL_I3C_DIRECTION_READ;
				}else
				{
					rnw = LL_I3C_DIRECTION_WRITE;
				}
				if(num_target > 1)
				{
					LL_I3C_ControllerHandleMessage(base,
						new_tgt->addr,
						new_tgt->data_len,
						rnw,
						LL_I3C_CONTROLLER_MTYPE_DIRECT,
						LL_I3C_GENERATE_RESTART);
						
				}else if(num_target == 1)
				{
					LL_I3C_ControllerHandleMessage(base,
						new_tgt->addr,
						new_tgt->data_len,
						rnw,
						LL_I3C_CONTROLLER_MTYPE_DIRECT,
						LL_I3C_GENERATE_STOP);
				}else
				{
					LOG_ERR("%s: CCC[0x%02x] direct num_target error", dev->name, payload->ccc.id);
					goto out_ccc_stop;
				}
				transfer_num += new_tgt->data_len;
				new_tgt++;
				num_target--;
			}
			if ((__LS_I3C_GET_FLAG(base, LL_I3C_EVR_TXFNFF) == SET))
			{
				if(cur_tgt_idx > payload->targets.num_targets)
				{
					LOG_ERR("%s: CCC[0x%02x] direct num_target error", dev->name, payload->ccc.id);
					goto out_ccc_stop;
				}
				if(cur_tgt->data_len != 0)
				{
					LL_I3C_TransmitData8(base, (uint8_t)*cur_tgt->data);
					cur_tgt->data++;
					cur_tgt->num_xfer++;
					transfer_num --;
				}else
				{
					cur_tgt_idx++;
					if(cur_tgt_idx <= payload->targets.num_targets)
					{
						cur_tgt++;
					}
				}
			}
			/* RXFNEF 标志驱动接收（RXCOUNT 跨 core/pbus 时钟域同步，
			 * 热路径读到的值滞后，不能用作热路径判据）；
			 * 收满 num_xfer==data_len 后多余的字节丢弃，避免越界写。 */
			if ((__LS_I3C_GET_FLAG(base, LL_I3C_EVR_RXFNEF) == SET))
			{
				if(cur_tgt_idx > payload->targets.num_targets)
				{
					LOG_ERR("%s: CCC[0x%02x] direct num_target error", dev->name, payload->ccc.id);
					goto out_ccc_stop;
				}
				if(cur_tgt->data_len != 0)
				{
					uint8_t rx_byte = LL_I3C_ReceiveData8(base);
					if(cur_tgt->num_xfer < cur_tgt->data_len)
					{
						*cur_tgt->data = rx_byte;
						cur_tgt->data++;
						cur_tgt->num_xfer++;
						transfer_num --;
					}
					/* else: 超出期望的字节，丢弃 */
				}else
				{
					cur_tgt_idx++;
					if(cur_tgt_idx <= payload->targets.num_targets)
					{
						cur_tgt++;
					}
				}
			}

			if(arch_k_cycle_get_64() > exp_time)
			{
				LOG_ERR("%s: CCC[0x%02x] direct phase2 timeout", dev->name, payload->ccc.id);
				timeout = true;
				ret = -ETIMEDOUT;
				// break;
			}

			/* Calculate exit_condition value based on Frame complete and error flags */
			exit_condition = (READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (exit_condition == 0U && timeout == false);

		if(timeout == true)
		{
			LOG_ERR("%s: CCC[0x%02x]: timeout", dev->name, payload->ccc.id);
			ret = -ETIMEDOUT;
			goto out_ccc_stop;
		}

		/* Drain 残余字节。
		 * 分支条件：num_xfer < data_len 表示还缺字节（应收），
		 * 已收满后 FIFO 仍有字节才是 unexpected。
		 * RXFNEF 标志更新滞后，FIFO 读空后瞬间仍可能置位；
		 * 此处已脱离热路径，用 RXCOUNT 交叉验证：为 0 说明是
		 * 滞后标志，直接退出；非 0 才是真残留字节。 */
		while ((__LS_I3C_GET_FLAG(base, LL_I3C_EVR_RXFNEF) == SET))
		{
			if(cur_tgt->num_xfer < cur_tgt->data_len)
			{
				*cur_tgt->data = LL_I3C_ReceiveData8(base);
				cur_tgt->data++;
				cur_tgt->num_xfer++;
				transfer_num --;
			}
			else if ((((base->SDATACTRL) & I3C_SDATACTRL_RXCOUNT_MASK) >>
				  I3C_SDATACTRL_RXCOUNT_POS) == 0U)
			{
				break; /* FIFO 实际已空，RXFNEF 为滞后标志 */
			}
			else
			{
				(void)LL_I3C_ReceiveData8(base); /* 丢弃，避免越界写 */
				LOG_ERR("%s: CCC[0x%02x] target 0x%02x received unexpected data",
					dev->name, payload->ccc.id, cur_tgt->addr);
				ret = -EIO;
				break;
			}
		}

		/* CCC 读段补收窗口：FCF（帧完成）可能先于 RX FIFO 最后字节
		 * 落地（接收流水线延迟），上面的 drain 会扑空。帧完成后全程
		 * 驻留一个短窗口（~10us）持续轮询，有字节就收；不能用
		 * "连续 N 次空"提前退出——纯寄存器轮询两次只隔几十纳秒，
		 * 远小于字节落地延迟，会误判排空。已收满而 RXFNEF 仍置位时
		 * 用 RXCOUNT 交叉验证（滞后标志 or 真残留）。 */
		{
			uint64_t settle_start = arch_k_cycle_get_64();
			uint64_t settle_exp = settle_start + 10000;
			while (arch_k_cycle_get_64() < settle_exp)
			{
				if (__LS_I3C_GET_FLAG(base, LL_I3C_EVR_RXFNEF) == SET)
				{
					if(cur_tgt->num_xfer < cur_tgt->data_len)
					{
						*cur_tgt->data = LL_I3C_ReceiveData8(base);
						cur_tgt->data++;
						cur_tgt->num_xfer++;
						transfer_num --;
					}
					else if ((((base->SDATACTRL) & I3C_SDATACTRL_RXCOUNT_MASK) >>
						  I3C_SDATACTRL_RXCOUNT_POS) == 0U)
					{
						/* FIFO 实际已空，RXFNEF 为滞后标志，继续驻留 */
					}
					else
					{
						(void)LL_I3C_ReceiveData8(base); /* 丢弃，避免越界写 */
						LOG_ERR("%s: CCC[0x%02x] target 0x%02x received unexpected data",
							dev->name, payload->ccc.id, cur_tgt->addr);
						ret = -EIO;
						break;
					}
				}
			}
		}

		if(transfer_num != 0)
		{
			LOG_ERR("%s: CCC[0x%02x] target 0x%02x amount mismatch, remaining %d",
				dev->name, payload->ccc.id,
				(cur_tgt_idx <= payload->targets.num_targets) ? cur_tgt->addr : 0,
				transfer_num);
			ret = -EIO;
		}
	}

out_ccc_stop:
	/* 等待帧收尾：有界等待，防止硬件异常（如 target 掉线把 SDA
	 * 拉住）时永久挂死。 */
	{
		uint64_t stop_exp = arch_k_cycle_get_64() + LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT * 4;
		while((READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0)
		{
			if (arch_k_cycle_get_64() > stop_exp) {
				LOG_ERR("%s: CCC[0x%02x] wait-frame-complete timeout",
					dev->name, payload->ccc.id);
				ret = -ETIMEDOUT;
				break;
			}
		}
	}

	if (__LS_I3C_GET_FLAG(base, LL_I3C_EVR_FCF) == SET)
	{
		LL_I3C_ClearFlag_FC(base);
	}	

	/* Check on error flag */
	if (__LS_I3C_GET_FLAG(base, LL_I3C_EVR_ERRF) == SET)
	{
		/* Clear error flag */
		ls_i3c_log_err_type(dev);
		LL_I3C_ClearFlag_ERR(base);
		/* Update returned status value */
		ret = -EIO;
	}

	ls_i3c_xfer_reset(base);

	k_mutex_unlock(&data->lock);

	// arch_irq_unlock(k);

	return ret;
}


/*
 * brief:  Transfer messages in I3C mode.
 *
 * see i3c_transfer
 *
 * param[in] dev       Pointer to device driver instance.
 * param[in] target    Pointer to target device descriptor.
 * param[in] msgs      Pointer to I3C messages.
 * param[in] num_msgs  Number of messages to transfers.
 *
 * return  see i3c_transfer
 */
static int ls_i3c_transfer(const struct device *dev, struct i3c_device_desc *target,
			     struct i3c_msg *msgs, uint8_t num_msgs)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;
	int ret = 0;

	/* 入参防护：NULL 直接解引用会硬 fault，必须最先拦截 */
	if ((target == NULL) || (msgs == NULL)) {
		return -EINVAL;
	}

	if (target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	if(num_msgs < 1)
	{
		return -EINVAL;
	}

	for(uint8_t i=0;i < num_msgs;i++)  // error
	{
		msgs[i].num_xfer = 0;
		if(msgs[i].hdr_mode != 0)
		{
			LOG_ERR("%s: i3c_transfer to 0x%02x: HDR mode not supported", dev->name, target->dynamic_addr);
			return -EINVAL;
		}
	}
	k_mutex_lock(&data->lock, K_FOREVER);
	
	LL_I3C_EnableIT_FC(base);
	LL_I3C_EnableIT_CFNF(base);
	LL_I3C_EnableIT_SFNE(base);
	LL_I3C_EnableIT_RXFNE(base);
	LL_I3C_EnableIT_TXFNF(base);
	LL_I3C_ClearFlag_ERR(base);
	LL_I3C_EnableIT_ERR(base);

	ret = i3c_ls_curr_msg_init(dev, msgs, NULL, num_msgs, target->dynamic_addr);
	if (ret != 0) {
		/* curr_msg_init 失败（如非法参数）时不再发起传输 */
		goto out_transfer;
	}

	ret = ls_i3c_request_transfer_flag(dev);
	if(ret !=0){
		LOG_ERR("%s: i3c_transfer to 0x%02x, num_msgs=%u: request transfer failed, err=%d",
			dev->name, target->dynamic_addr, num_msgs, ret);
		/* 错误帧（如地址 NACK）可能留下未消费的 TX/C-FIFO 内容，
		 * 不清理会污染下一笔传输 */
		ls_i3c_xfer_reset(base);
	} else {
		/* 写消息必须完整发出：地址 NACK 等场景下中断路径可能"成功"
		 * 返回但 num_xfer=0（ERRF 跨时钟域晚同步漏检的兜底）。
		 * 读消息允许短读（target T-bit 提前终止是 I3C 合法行为）。 */
		for (uint8_t i = 0; i < num_msgs; i++) {
			if ((msgs[i].flags & I3C_MSG_READ) == 0 &&
			    msgs[i].num_xfer != msgs[i].len) {
				LOG_ERR("%s: i3c_transfer to 0x%02x: write msg %u short, %u/%u",
					dev->name, target->dynamic_addr, i,
					msgs[i].num_xfer, msgs[i].len);
				ret = -EIO;
				break;
			}
		}
		if (ret != 0) {
			ls_i3c_xfer_reset(base);
		}
	}
out_transfer:
	LL_I3C_DisableIT_FC(base);
	LL_I3C_DisableIT_CFNF(base);
	// LL_I3C_DisableIT_SFNE(base);
	LL_I3C_DisableIT_RXFNE(base);
	LL_I3C_DisableIT_TXFNF(base);
	LL_I3C_DisableIT_ERR(base);
	k_mutex_unlock(&data->lock);

	return ret;
}

/*
 * brief:  Find a registered I3C target device.
 *
 * This returns the I3C device descriptor of the I3C device
 * matching the incoming id.
 *
 * param[in] dev  Pointer to controller device driver instance.
 * param[in] id   Pointer to I3C device ID.
 *
 * return  see i3c_device_find.
 */
static inline struct i3c_device_desc *ls_i3c_device_find(const struct device *dev,
							   const struct i3c_device_id *id)
{
	const struct ls_i3c_config *config = dev->config;

	return i3c_dev_list_find(&config->common.dev_list, id);
}

static void ls_i3c_log_slave_serrwarn(const struct device *dev, I3C_TypeDef *base,
				      uint32_t err)
{
	struct ls_i3c_data *data = dev->data;

	if (!err) {
		return;
	}

	LOG_ERR("%s: SERRWARN raw=0x%08x, state=%d, txcount=%lu, rxcount=%lu",
		dev->name, err, data->state,
		(base->SDATACTRL & I3C_SDATACTRL_TXCOUNT_MASK) >> I3C_SDATACTRL_TXCOUNT_POS,
		(base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >> I3C_SDATACTRL_RXCOUNT_POS);

	if (err & I3C_SERRWARN_OWRITE_MASK) {
		LOG_ERR("  OWRITE: SWDATAB written when TX FIFO full / not in read phase");
	}
	if (err & I3C_SERRWARN_OREAD_MASK) {
		LOG_ERR("  OREAD: SRDATAB read when RX FIFO empty / not in write phase");
	}
	if (err & I3C_SERRWARN_S0S1_MASK) {
		LOG_ERR("  S0S1: S0/S1 mode error");
	}
	if (err & I3C_SERRWARN_HCRC_MASK) {
		LOG_ERR("  HCRC: CCC CRC error");
	}
	if (err & I3C_SERRWARN_HPAR_MASK) {
		LOG_ERR("  HPAR: CCC parity error");
	}
	if (err & I3C_SERRWARN_SPAR_MASK) {
		LOG_ERR("  SPAR: SDR data parity error");
	}
	if (err & I3C_SERRWARN_IBIURUN_MASK) {
		LOG_ERR("  IBIURUN: IBI payload underrun/overrun");
	}
	if (err & I3C_SERRWARN_INVSTART_MASK) {
		LOG_ERR("  INVSTART: invalid START condition");
	}
	if (err & I3C_SERRWARN_TERM_MASK) {
		LOG_ERR("  TERM: controller terminated read early");
	}
	if (err & I3C_SERRWARN_URUNNACK_MASK) {
		LOG_ERR("  URUNNACK: slave NACKed due to TX FIFO underrun");
	}
	if (err & I3C_SERRWARN_URUN_MASK) {
		LOG_ERR("  URUN: TX FIFO underrun during read");
	}
	if (err & I3C_SERRWARN_ORUN_MASK) {
		LOG_ERR("  ORUN: RX FIFO overrun during write");
	}
}

/* SERRWARN 延迟打印工作项：ISR 内 LOG 分配/入队耗时在 12.5MHz SCL
 * （0.8us/字节）下会错过整个总线窗口（实测导致 multi_msg_4 读段欠载），
 * 故 ISR 只把原始值记账到 serrwarn_pending，由系统工作队列线程打印。 */
static void ls_i3c_serrwarn_work_handler(struct k_work *work)
{
	struct ls_i3c_data *data = CONTAINER_OF(work, struct ls_i3c_data, serrwarn_work);
	const struct device *dev = data->dev;
	const struct ls_i3c_config *config = dev->config;
	uint32_t err = atomic_clear(&data->serrwarn_pending);

	if (err) {
		ls_i3c_log_slave_serrwarn(dev, config->base, err);
	}
}

static void ls_i3c_target_isr(const struct device *dev)
{

	int status = 0;
	struct ls_i3c_data *data = dev->data;
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *base = config->base;

	if(data->target_config == NULL)
	{
		LOG_ERR("%s: targer not register", __func__);
		ls_i3c_enable_target_interrupt(dev,false);
		return;
	}

	struct i3c_config_target *config_tgt = &data->config_target;
	struct i3c_target_config *target_config = data->target_config;
	const struct i3c_target_callbacks *target_cb = data->target_config->callbacks;

	/* 不做排空循环：若某事件标志位在当前分支结构下无法清除（如
	 * TXSEND 电平持续有效），电平触发会让 ISR 反复重入形成中断
	 * 风暴。保持单次通过。 */
	if (base->SINTMASKED) {

		/* 先交付 RX FIFO 中尚未取走的字节：字节中断与 STOP 常在同一次
		 * ISR 中同时挂起（最后一字节紧随 STOP），若先跑 STOP 分支，
		 * stop_cb 会把状态机/索引复位，尾字节就会被误当成下一笔消息的
		 * 开头。 */
		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SINTCLR_MATCHED_MASK))
		{
			if(data->state != LS_I3C_OP_STATE_IBI)
			{
				if((data->state != LS_I3C_OP_STATE_WR) && I3C_CHECK_FLAG(base->SSTATUS,I3C_SSTATUS_STREQWR_MASK))
				{
					uint8_t rx_data;
					data->state = LS_I3C_OP_STATE_WR;
					/*controller write quest*/
					if ((target_cb != NULL) &&
						(target_cb->write_requested_cb != 0))
					{
						target_cb->write_requested_cb(data->target_config);
					}
					/* 带超时的首字节等待：让从机尽早拿到写数据（为后续
					 * 读做准备），但首字节可能已被上方 RXPEND 块收走
					 * （MATCHED 与 RXPEND 同一次挂起），此时死等会一直
					 * 占用 ISR 直到下一笔传输，STOP 无法处理。100us
					 * 覆盖最慢速率档的地址→字节间隔，超时后字节改由
					 * RXPEND 中断交付。 */
					uint32_t wait_start = k_cycle_get_32();
					uint32_t wait_budget = k_us_to_cyc_ceil32(100);
					while(!(base->SSTATUS & I3C_SSTATUS_RXPEND_MASK)) {
						/* 首字节可能已被上方 RXPEND 块收走（MATCHED 与 RXPEND
						 * 同一次挂起）：RXCOUNT > 0 说明 FIFO 已有数据，无需再
						 * 等新字节，直接跳出避免空等 100us。 */
						if ((base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >>
						    I3C_SDATACTRL_RXCOUNT_POS) {
							break;
						}
						if ((k_cycle_get_32() - wait_start) > wait_budget) {
							break;
						}
					}
					{
						/* 同顶部 RXPEND 块：按 RXCOUNT 上限排空，防非写相位
						 * 读 SRDATAB 不弹出导致的死循环 */
						uint32_t budget = ((base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >>
								   I3C_SDATACTRL_RXCOUNT_POS) + 2U;
						while((base->SSTATUS & I3C_SSTATUS_RXPEND_MASK) && (budget-- != 0U))
						{
							rx_data = (uint8_t)base->SRDATAB;
							target_cb->write_received_cb(data->target_config, rx_data);
						}
					}
				}
				else if((data->state != LS_I3C_OP_STATE_RD) && I3C_CHECK_FLAG(base->SSTATUS,I3C_SSTATUS_STREQRD_MASK))
				{
					/*controller read requset*/
					uint8_t tx_data;
					// LOG_DBG("MATCHED read branch, state=%d, SSTATUS=0x%x",
					// 	data->state, base->SSTATUS);
					data->state = LS_I3C_OP_STATE_RD;
					base->SINTSET = I3C_SINTSET_TXSEND_MASK;
					if ((target_cb != NULL) &&
						target_cb->read_requested_cb)
					{
						if(I3C_CHECK_FLAG(base->SSTATUS,I3C_SSTATUS_TXNOTFULL_MASK))
						{
							/* 预填（write_requested_cb/stop_cb 预装）数据已在
							 * FIFO 中由硬件自动发出，此刻无需向总线注入字节：
							 * 注入会把 read_requested_cb 的 0x00/END0 混入
							 * 数据流，且 END0 会提前终止读段（master 侧 ABT）
							 * 导致读不足长度。等 FIFO 数据发空（TXCOUNT==0）
							 * 再由回调续供。 */
							uint32_t txcount =
								(base->SDATACTRL & I3C_SDATACTRL_TXCOUNT_MASK) >>
								I3C_SDATACTRL_TXCOUNT_POS;
							if (txcount == 0U) {
								status = target_cb->read_requested_cb(data->target_config,&tx_data);
								if(status == 0)
								{
									base->SWDATAB = tx_data;
								}
								else if(status == 1)
								{
									base->SWDATAB = tx_data|I3C_SWDATAB_END0_MASK;
								}
							}
						}
					}
				}
			}
			base->SSTATUS = I3C_SSTATUS_MATCHED_MASK;
			base->SINTCLR = I3C_SSTATUS_MATCHED_MASK;
		}
		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SSTATUS_RXPEND_MASK))
		{
			uint8_t rx_data;
			/* 以 RXCOUNT 为上限排空：非写相位读 SRDATAB 不会弹出 FIFO
			 * （OREAD），只盯 RXPEND 标志会死循环占住 ISR。 */
			uint32_t budget = ((base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >>
					   I3C_SDATACTRL_RXCOUNT_POS) + 2U;
			while((base->SSTATUS & I3C_SSTATUS_RXPEND_MASK) && (budget-- != 0U))
			{
				rx_data = (uint8_t)base->SRDATAB;
				target_cb->write_received_cb(data->target_config, rx_data);
			}
			if (base->SSTATUS & I3C_SSTATUS_RXPEND_MASK) {
				/* 非写相位读 SRDATAB 不弹出导致 budget 耗尽仍有剩字节：
				 * 只记计数（ISR 内不打印，避免影响时序），由外部观察 */
			}
			if(I3C_CHECK_FLAG(base->SSTATUS,I3C_SSTATUS_STREQRD_MASK))
			{
				base->SINTSET = I3C_SINTSET_TXSEND_MASK;
			}
		}

			/* Check error or warning has occurred */
			if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SSTATUS_STOP_MASK) || I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_ERRWARN_MASK)) {
				/* STOP 分支必须 FLUSHTB：STOP 时清空 TX FIFO，使 stop_cb
				 * 的 prefill 从 buf[0] 起写，数据流恒为 buf 干净前缀、
				 * END0 位置正确；不清会让残留与新 prefill 拼接、END0
				 * 位置漂移。 */
				base->SDATACTRL |= I3C_SDATACTRL_FLUSHTB_MASK;
				base->SSTATUS = I3C_SSTATUS_MATCHED_MASK;
				base->SINTSET = I3C_SINTSET_MATCHED_MASK;
				base->SINTCLR = I3C_SINTSET_TXSEND_MASK;
				base->SSTATUS = I3C_SSTATUS_STOP_MASK;
				if(base->SERRWARN)
				{
					/* 清 SERRWARN 中断标志。TERM/URUN 在读欠载
					 * 场景下是 controller 主动提前终止的合法行为，非异常。
					 * 原始值记账后经工作队列线程打印（ISR 内 LOG
					 * 分配/入队耗时破坏高速率总线时序，见
					 * ls_i3c_serrwarn_work_handler 注释）。 */
					atomic_or(&data->serrwarn_pending, base->SERRWARN);
					k_work_submit(&data->serrwarn_work);
					base->SERRWARN = base->SERRWARN;
				}
				data->state = LS_I3C_OP_STATE_IDLE;
				/* Notify upper layer a STOP condition received */
				if ((target_cb != NULL) && (target_cb->stop_cb != NULL)) {
					target_cb->stop_cb(data->target_config);
				}
			}

		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SINTMASK_TXSEND_MASK))
		{
			uint8_t byte;
			if(data->state == LS_I3C_OP_STATE_WR)
			{
				data->state = LS_I3C_OP_STATE_RD;
				/* 同 MATCHED 读分支：FIFO 尚有预填数据时不注入 */
				uint32_t txcount =
					(base->SDATACTRL & I3C_SDATACTRL_TXCOUNT_MASK) >>
					I3C_SDATACTRL_TXCOUNT_POS;
				if (txcount == 0U) {
					status = target_cb->read_requested_cb(data->target_config, &byte);
					if(status == 0)
					{
						base->SWDATAB = byte;
					}
					else if(status == 1)
					{
						base->SWDATAB = byte|I3C_SWDATAB_END0_MASK;
					}
				}
			}
			while((base->SSTATUS & I3C_SSTATUS_STREQRD_MASK) &&
								(base->SSTATUS & I3C_SSTATUS_TXNOTFULL_MASK))
			{
				/* 续装直到 FIFO 满（TXNOTFULL 自停）。不按 TXCOUNT==0
				 * 逐字节续装：TXSEND 在 FIFO 非空时也会触发，逐字节
				 * 续装会漏续。END0（status==1）注入后立即终止，防止
				 * 后续字节覆盖 END0 位置；status<0 表示已无数据可续装。 */
				status = target_cb->read_processed_cb(data->target_config, &byte);
				if(status == 0)
				{
					base->SWDATAB = byte;
				}
				else if(status == 1)
				{
					base->SWDATAB = byte|I3C_SWDATAB_END0_MASK;
					break;
				}
				else if(status < 0)
				{
					base->SINTCLR = I3C_SINTSET_TXSEND_MASK;
					break;
				}
				// LOG_ERR("tx :SSTATUS=0x%x", base->SSTATUS);
			}
			// if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SINTMASK_TXSEND_MASK))
			// {
			// 	if((base->SSTATUS && I3C_SSTATUS_STMSG_MASK) == 0)
			// 	{
			// 		base->SINTCLR = I3C_SINTCLR_TXSEND_MASK;
			// 	}
			// }
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_SLVRST_MASK)) {
			base->SSTATUS = I3C_SSTATUS_SLVRST_MASK;
		}

		/* Check START or Sr detected */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_START_MASK)) {
			/* The end of xfer is a Sr */
			if ((data->state == LS_I3C_OP_STATE_WR) ||
				(data->state == LS_I3C_OP_STATE_RD)) {
				/* SCONFIG.MATCHSS=1 keeps MATCHED set across restart.
				 * A repeated start does not generate a new MATCHED
				 * interrupt, so re-evaluate direction here.
				 */
				if ((data->state != LS_I3C_OP_STATE_RD) &&
				    I3C_CHECK_FLAG(base->SSTATUS, I3C_SSTATUS_STREQRD_MASK)) {
					uint8_t tx_data;
					// LOG_DBG("START read branch, state=%d, SSTATUS=0x%x",
					// 	data->state, base->SSTATUS);
					data->state = LS_I3C_OP_STATE_RD;
					base->SINTSET = I3C_SINTSET_TXSEND_MASK;
					if ((target_cb != NULL) &&
					    target_cb->read_requested_cb) {
						if (I3C_CHECK_FLAG(base->SSTATUS,
							I3C_SSTATUS_TXNOTFULL_MASK)) {
							/* 同 MATCHED 读分支：FIFO 尚有预填
							 * 数据时不注入（见上方注释） */
							uint32_t txcount =
								(base->SDATACTRL &
								 I3C_SDATACTRL_TXCOUNT_MASK) >>
								I3C_SDATACTRL_TXCOUNT_POS;
							if (txcount == 0U) {
								status = target_cb->read_requested_cb(
									data->target_config, &tx_data);
								if (status == 0) {
									base->SWDATAB = tx_data;
								} else if (status == 1) {
									base->SWDATAB = tx_data |
										I3C_SWDATAB_END0_MASK;
								}
							}
						}
					}
				} else if ((data->state != LS_I3C_OP_STATE_WR) &&
					   I3C_CHECK_FLAG(base->SSTATUS,
							  I3C_SSTATUS_STREQWR_MASK)) {
					// LOG_DBG("START write branch, state=%d, SSTATUS=0x%x",
					// 	data->state, base->SSTATUS);
					data->state = LS_I3C_OP_STATE_WR;
					if ((target_cb != NULL) &&
					    (target_cb->write_requested_cb != 0)) {
						target_cb->write_requested_cb(
							data->target_config);
					}
				}
			}

			base->SSTATUS = I3C_SSTATUS_START_MASK;
		}

		/* CCC 'not' automatically handled was received */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_CCC_MASK)) {
			base->SSTATUS = I3C_SSTATUS_CCC_MASK;
		}

		/* CCC handled (handled by IP) */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_CHANDLED_MASK)) {
			base->SSTATUS = I3C_SSTATUS_CHANDLED_MASK;
		}

		/* Dynamic address changed（SETDASA/SETAASA/ENTDAA 置位、RSTDAA 丢失，
		 * PDF programmers model SSTATUS bit13）。DACHG 在使能掩码里但此前
		 * ISR 从不清除：一旦硬件挂起该位（如 RSTDAA/SETAASA/SETDASA 测试）
		 * 电平触发会形成中断风暴。按 CHANDLED 同款模式 W1C 清掉。 */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_DACHG_MASK)) {
			base->SSTATUS = I3C_SSTATUS_DACHG_MASK;
		}

		/* Event requested. IBI, hot-join, bus control.
		 * IBI/HJ 请求的统一终态处理：先写结果、恢复状态机，最后 give 唤醒
		 * ibi_raise 中等待的线程（顺序保证 take 返回时 ibi_status 已生效）。 */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_EVENT_MASK)) {
			uint32_t evdet = (base->SSTATUS & I3C_SSTATUS_EVDET_MASK) >>
					 I3C_SSTATUS_EVDET_POS;

			base->SSTATUS = I3C_SSTATUS_EVENT_MASK;

			if (evdet == STATUS_EVDET_REQ_SENT_ACKED) {
				data->ibi_status = 0;
				data->state = LS_I3C_OP_STATE_IDLE;
				k_sem_give(&data->target_event_lock_sem);
			} else if (evdet == STATUS_EVDET_REQ_SENT_NACKED) {
				data->ibi_status = -EIO;
				data->state = LS_I3C_OP_STATE_IDLE;
				k_sem_give(&data->target_event_lock_sem);
			}
			/* STATUS_EVDET_REQ_NOT_SENT 为中间态（请求等待总线仲裁），继续
			 * 等待终态事件；ibi_raise 的 take 带超时兜底，不会永久阻塞 */
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_DACHG_MASK)) {
			base->SSTATUS = I3C_SSTATUS_DACHG_MASK;
			if (base->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK) {
				if (target_config != NULL) {
					config_tgt->dynamic_addr =
						(base->SDYNADDR&I3C_SDYNADDR_DADDR_MASK) >> I3C_SDYNADDR_DADDR_POS;
				}
			} else if (target_config != NULL) {
				/* RSTDAA 等导致动态地址丢失：同步清掉软件记账，
				 * 避免残留陈旧地址被应用误用 */
				config_tgt->dynamic_addr = 0;
			}
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTSET_NOWCNTLR_MASK)) {
			LOG_ERR("%s: crr not support", __func__);
			base->SSTATUS = I3C_SSTATUS_NOWCNTLR_MASK;
			// base->SCONFIG &= ~I3C_SCONFIG_SLVENA_MASK;
		}
	}
}

static void ls_i3c_log_err_type(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *i3c = config->base;

	if (LL_I3C_IsActiveFlag_ANACK(i3c)) {
		LOG_ERR("%s: Address NACK", dev->name);
	}

	if (LL_I3C_IsActiveFlag_COVR(i3c)) {
		LOG_ERR("%s: Control/Status FIFO underrun/overrun", dev->name);
	}

	if (LL_I3C_IsActiveFlag_DOVR(i3c)) {
		LOG_ERR("%s: TX/RX FIFO underrun/overrun", dev->name);
	}

	if (LL_I3C_IsActiveFlag_DNACK(i3c)) {
		LOG_ERR("%s: Data NACK by target", dev->name);
	}

	if (LL_I3C_IsActiveFlag_PERR(i3c)) {
		switch (LL_I3C_GetMessageErrorCode(i3c)) {
		case LL_I3C_CONTROLLER_ERROR_CE0:
			LOG_ERR("%s: Illegally formatted CCC detected", dev->name);
			break;
		case LL_I3C_CONTROLLER_ERROR_CE1:
			LOG_ERR("%s: Data on bus is not as expected", dev->name);
			break;
		case LL_I3C_CONTROLLER_ERROR_CE2:
			LOG_ERR("%s: No response to broadcast address", dev->name);
			break;
		default:
			LOG_ERR("%s: Unsupported error detected", dev->name);
			break;
		}
	}
}

static void ls_i3c_isr(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = config->base;
	struct i3c_device_desc *target = NULL;
	struct ls_i3c_msg *curr_msg = &data->curr_msg;
	int ret;
	
	if(READ_BIT(base->SCONFIG,I3C_SCONFIG_SLVENA_MASK) && data->cur_role == I3C_ROLE_TARGET)
	{
		ls_i3c_target_isr(dev);
		return;
	}

	uint32_t it_flags   = READ_REG(base->EVR);
	uint32_t it_sources = READ_REG(base->IER);

	uint32_t it_masks   = (uint32_t)(it_flags & it_sources);

	/* TX FIFO not full handler */
	if (LL_I3C_IsActiveFlag_ERR(base)) {
		LL_I3C_ClearFlag_ERR(base);
		ls_i3c_log_err_type(dev);
		data->msg_state = LS_I3C_MSG_ERR;
		k_sem_give(&data->device_sync_sem);
	}

	/* TX FIFO not full handler */
	if (LL_I3C_IsActiveFlag_TXFNF(base) && LL_I3C_IsEnabledIT_TXFNF(base)) {
		ls_i3c_event_isr_tx(dev);
	}


	/* TX FIFO not full handler */
	if (LL_I3C_IsActiveFlag_TXFNF(base) && LL_I3C_IsEnabledIT_TXFNF(base)) {
		ls_i3c_event_isr_tx(dev);
	}

	/* RX FIFO not empty handler */
	if (LL_I3C_IsActiveFlag_RXFNE(base) && LL_I3C_IsEnabledIT_RXFNE(base)) {
		ls_i3c_event_isr_rx(dev);
	}

	/* Control FIFO not full handler */
	if (LL_I3C_IsActiveFlag_CFNF(base) && LL_I3C_IsEnabledIT_CFNF(base)) {
		ls_i3c_event_isr_cf(dev);
	}

	/* Status FIFO not empty handler */
	if (LL_I3C_IsActiveFlag_SFNE(base) && LL_I3C_IsEnabledIT_SFNE(base)) {

		if (data->msg_state == LS_I3C_MSG) {
			size_t num_xfer = LL_I3C_GetXferDataCount(base);

			/* 提前终止（target T-bit 拉低）的消息：LL_I3C_GetXferDataCount
			 * 读的是 live SR.XDCNT，跨时钟域同步滞后，此刻可能还是 0，
			 * 直接覆盖会把已收字节数抹成 0（应用看到 actual=0 的假
			 * 空读）。仅当状态项对应的正是当前传输中的消息（索引相等
			 * =该消息被提前终止、xfer_next 尚未推进）时，与软件实际从
			 * RX FIFO 收进 buf 的字节数取大者；正常完成的消息维持
			 * xdcnt 语义。 */
			if (curr_msg->status_msg_idx == curr_msg->xfer_msg_idx &&
			    curr_msg->cur_num_xfer > num_xfer) {
				num_xfer = curr_msg->cur_num_xfer;
			}

			ls_i3c_curr_msg_status_update_num_xfer(dev, num_xfer);
			ls_i3c_curr_msg_status_next(dev);
		} else {
			/* Read and discard the status FIFO word since it will not be used */
			uint32_t status_reg = base->SR;

			ARG_UNUSED(status_reg);
		}
	}

	/* Target read early termination flag (only used during CCC commands)*/
	if (LL_I3C_IsActiveFlag_RXTGTEND(base) && LL_I3C_IsEnabledIT_RXTGTEND(base)) {
		/* A target ended a read request early during a CCC command, move the ptr to the
		 * next target
		 */
		LL_I3C_ClearFlag_RXTGTEND(base);
	}

	/* Frame complete handler */
	if (LL_I3C_IsActiveFlag_FC(base) && LL_I3C_IsEnabledIT_FC(base)) {
		LL_I3C_ClearFlag_FC(base);

		/* FCF 与 ERRF 可能同帧到达但有先后（跨时钟域）：FC 先到时
		 * 若不复查 ERR，会把错误帧（如地址 NACK 的读）当成功帧
		 * give 信号量 → 上层以 ret=0/actual=0 返回 */
		if (LL_I3C_IsActiveFlag_ERR(base)) {
			LL_I3C_ClearFlag_ERR(base);
			ls_i3c_log_err_type(dev);
			data->msg_state = LS_I3C_MSG_ERR;
		}

		if((data->msg_state == LS_I3C_MSG)  && (curr_msg->ctrl_msg_idx < curr_msg->num_msgs))
		{
			LL_I3C_RequestTransfer(base);
			return;
		}

		/* 每帧只 give 一次：ERR 分支（前面）已 give 过的错误帧不再
		 * give，否则残留计数会让下一帧的等待提前返回（假成功）。
		 * 帧末 RX 残余字节的排空与 num_xfer 落账已移到
		 * ls_i3c_request_transfer_flag 的等待返回后（线程上下文，
		 * 可安全驻留等跨时钟域同步，ISR 内立即排空会漏尾字节）。 */
		if (data->msg_state != LS_I3C_MSG_ERR) {
			k_sem_give(&data->device_sync_sem);
		}

		// (void)pm_device_runtime_put(dev);
		// pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

		/* Mark bus as idle after each frame complete */
		/* RX FIFO not empty handler（只处理标志可见部分；帧末可能滞后
		 * 落地的残余字节由 request_transfer_flag 等待返回后兜底） */
		while (LL_I3C_IsActiveFlag_RXFNE(base)) {
			ls_i3c_event_isr_rx(dev);
		}
		data->msg_state = LS_I3C_MSG_IDLE;
	}
#if defined(CONFIG_I3C_USE_IBI)
	if (I3C_CHECK_FLAG(it_masks, I3C_EVR_IBIF) != RESET)
	{
		/* Clear IBI request flag */
		LL_I3C_ClearFlag_IBI(base);
        uint8_t IBICRTgtAddr = LL_I3C_GetIBITargetAddr(base);
        uint8_t IBITgtNbPayload = LL_I3C_GetNbIBIAddData(base);
        uint32_t IBITgtPayload = LL_I3C_GetIBIPayload(base);

		/*将 ibi 的信息抛送给对应的从机设备*/
		target = i3c_dev_list_i3c_addr_find(dev, (uint8_t)IBICRTgtAddr);
		if (target != NULL) {
			ret = i3c_ibi_work_enqueue_target_irq(target,(uint8_t *)&IBITgtPayload,IBITgtNbPayload);
			if (ret < 0) {
				LOG_ERR("Enqueuing ibi work fail, ret %d", ret);
			}
		} else {
			LOG_ERR("IBI from unknown device addr 0x%x", IBICRTgtAddr);
		}
	}
	if (I3C_CHECK_FLAG(it_masks, I3C_EVR_CRF) != RESET)
	{
		/* Clear controller-role request flag */
		LL_I3C_ClearFlag_CR(base);
		/* TODO: LS I3C can support CR, but not implemented yet */
	}
	if (I3C_CHECK_FLAG(it_masks, I3C_EVR_HJF) != RESET)
	{
		/* Clear hot-join flag */
		LL_I3C_ClearFlag_HJ(base);
		ret = i3c_ibi_work_enqueue_hotjoin(dev);
		if (ret < 0) {
			LOG_ERR("Enqueuing ibi work fail, ret %d", ret);
		}
	}

#endif
}

#ifdef CONFIG_I3C_USE_IBI
static int ls_i3c_ibi_enable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = config->base;
	struct i3c_ccc_events i3c_events = {.events = 0};
	uint8_t set_idx = 0xff;
	int ret = 0;
	/* Check target IBI request capable */
	if (!i3c_device_is_ibi_capable(target)) {
		LOG_ERR("%s: device is not ibi capable", __func__);
		return -EINVAL;
	}

	for(uint8_t idx = 0; idx < ARRAY_SIZE(data->ibi_inifo); idx++ )
	{
		if (data->ibi_inifo[idx].addr == target->dynamic_addr && data->ibi_inifo[idx].already_enabled == true) {
			LOG_ERR("%s: selected target is already in the list", __func__);
			return -EINVAL;
		}
	}
	/* have DEVICEx register is not in used*/
	for(uint8_t idx = 0; idx < ARRAY_SIZE(data->ibi_inifo); idx++)
	{
		if(data->ibi_inifo[idx].already_enabled == false )
		{
			data->ibi_inifo[idx].already_enabled = true;
			data->ibi_inifo[idx].has_mandatory_byte = i3c_ibi_has_payload(target);
			data->ibi_inifo[idx].addr = target->dynamic_addr;
			set_idx = idx;
			break;
		}
	}
	/*暂时先把cr使能放到ibi使能中，两者共用一个DEVICEx 寄存器*/
	uint32_t controller_capable = i3c_device_is_controller_capable(target);
	if(controller_capable)
	{
		// 当前版本zephyr设备驱动框架还不支持crr功能
		i3c_events.events |= I3C_CCC_EVT_CR;
	}
	/* config DEVICEx register */
	uint32_t write_value = 0;
	write_value = ((uint32_t)data->ibi_inifo[set_idx].addr << I3C_DEVRX_DA_POS) |
				  ((uint32_t)I3C_DEVRX_IBIACK_MASK) 							|
				  ((uint32_t)controller_capable << I3C_DEVRX_CRACK_POS) 		|
				  ((uint32_t)data->ibi_inifo[set_idx].has_mandatory_byte << I3C_DEVRX_IBIDEN_POS);

	base->DEVRX[set_idx] = write_value;
	/* Enable target IBI event by ENEC command */
	i3c_events.events |= I3C_CCC_EVT_INTR;  //|I3C_CCC_EVT_CR;
	ret = i3c_ccc_do_events_set(target, true, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI ENEC for 0x%02x (%d)", target->dynamic_addr, ret);
	}

	return ret;
}


static int ls_i3c_ibi_disable(const struct device *dev, struct i3c_device_desc *target)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = config->base;
	struct i3c_ccc_events i3c_events;
	uint8_t idx = 0;
	int ret = 0;

	if (!i3c_device_is_ibi_capable(target)) {
		LOG_ERR("%s: device is not ibi capable", __func__);
		return -EINVAL;
	}

	for(idx = 0; idx < ARRAY_SIZE(data->ibi_inifo); idx++ )
	{
		if (data->ibi_inifo[idx].addr == target->dynamic_addr) {
			break;
		}
	}

	if (idx == ARRAY_SIZE(data->ibi_inifo)) {
		LOG_ERR("%s: target is not in list of registered addresses", __func__);
		return -ENODEV;
	}

	data->ibi_inifo[idx].already_enabled = false;
	data->ibi_inifo[idx].addr = 0;

	/* Disable disable target IBI */
	i3c_events.events = I3C_CCC_EVT_INTR | I3C_CCC_EVT_CR;
	ret = i3c_ccc_do_events_set(target, false, &i3c_events);
	if (ret != 0) {
		LOG_ERR("Error sending IBI DISEC for 0x%02x (%d)", target->dynamic_addr, ret);
	}

	/* config DEVICEx register */
	base->DEVRX[idx] = 0;

	return ret;
}	

static int ls_i3c_target_ibi_raise(const struct device *dev, struct i3c_ibi *request)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = config->base;
	uint32_t ctrlValue = 0;
	int ret;
	/* the request or the payload were not specific */
	if ((request == NULL) || ((request->payload_len) && (request->payload == NULL))) {
		return -EINVAL;
	}
	if(READ_BIT(base->SCONFIG,I3C_SCONFIG_SLVENA_MASK) != I3C_SCONFIG_SLVENA_MASK)
	{
		return -EINVAL;
	}
	switch (request->ibi_type)
	{
	case I3C_IBI_TARGET_INTR:
		if (request->payload_len == 0 || request->payload_len > 8) {
			LOG_ERR("%s: IBI invalid payload_len, len: %#x", __func__,
				request->payload_len);
			return -EINVAL;
		}
		/* DISEC 会在硬件锁存 IBIDIS，期间 SCTRL.EVENT 请求被直接忽略，
		 * 总线上不会有任何波形。提前检查，fail-fast 代替 100ms 超时。 */
		if (base->SSTATUS & I3C_SSTATUS_IBIDIS_MASK) {
			LOG_ERR("%s: IBI disabled by DISEC, need ENEC first", __func__);
			return -EACCES;
		}
		/* mutex 串行化 IBI 发起；完成信号量 init=0，仅用于等待终态 */
		k_mutex_lock(&data->lock, K_FOREVER);
		k_sem_reset(&data->target_event_lock_sem);
		data->ibi_status = -EINPROGRESS;
		data->state = LS_I3C_OP_STATE_IBI;

		ctrlValue = base->SCTRL;
		ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
		ctrlValue |= (CTRL_EVENT_IBI << I3C_SCTRL_EVENT_POS) & I3C_SCTRL_EVENT_MASK;
		uint8_t *ibi_payload = request->payload;
		ctrlValue |= (((uint32_t)*ibi_payload<<I3C_SCTRL_IBIDATA_POS) & I3C_SCTRL_IBIDATA_MASK);

		base->IBIEXTIDATA |= I3C_SIBIEXTIDATA_CLR_MASK;
		uint16_t remain_num = request->payload_len - 1;
		if(remain_num > 0)
		{
			ctrlValue |= I3C_SCTRL_EXTDATA_MASK;
			ibi_payload++;
			while((remain_num > 0) && (base->IBIEXTIDATA & I3C_SIBIEXTIDATA_FREE_MASK))
			{
				if(remain_num > 1)
				{
					base->IBIEXTIDATA = (uint32_t)*ibi_payload;
				}else
				{
					base->IBIEXTIDATA = (uint32_t)*ibi_payload|I3C_SIBIEXTIDATA_END1_MASK;
				}
				remain_num --;
				ibi_payload++;
			}
		}
		if(remain_num > 0)
		{
			/* 发起前失败：尚未占用信号量，恢复状态后直接返回 */
			base->IBIEXTIDATA |= I3C_SIBIEXTIDATA_CLR_MASK;
			LOG_ERR("%s : ibi paload too long , the ibi fifo is full", __func__);
			data->state = LS_I3C_OP_STATE_IDLE;
			k_mutex_unlock(&data->lock);
			return -EINVAL;
		}

		/* 发起 IBI，等待 ISR 上报终态（ACK/NACK），超时兜底防死锁 */
		base->SCTRL = ctrlValue;
		if (k_sem_take(&data->target_event_lock_sem, I3C_IBI_COMPLETE_TIMEOUT) != 0) {
			ctrlValue = base->SCTRL;
			ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
			base->SCTRL = ctrlValue;
			data->state = LS_I3C_OP_STATE_IDLE;
			k_mutex_unlock(&data->lock);
			LOG_ERR("%s: IBI complete timeout", __func__);
			return -ETIMEDOUT;
		}
		ret = data->ibi_status;
		k_mutex_unlock(&data->lock);
		return ret;
	case I3C_IBI_CONTROLLER_ROLE_REQUEST:
		LOG_ERR("not supported crr");
		return -ENOTSUP;
		break;
	case I3C_IBI_HOTJOIN:
		/* 同 IBIDIS：HJDIS 锁存期间请求不会上到总线 */
		if (base->SSTATUS & I3C_SSTATUS_HJDIS_MASK) {
			LOG_ERR("%s: Hot-Join disabled by DISEC, need ENEC first", __func__);
			return -EACCES;
		}
		/* HJ 仅供无动态地址的设备使用：DAVALID=1 时硬件会直接抑制
		 * HJ 请求，总线上不会出现任何波形（最终 100ms 超时）。
		 * 提前检查，提示调用方先让 controller 广播 RSTDAA。 */
		if (base->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK) {
			LOG_ERR("%s: dynamic address still valid, need RSTDAA before Hot-Join",
				__func__);
			return -EACCES;
		}
		k_mutex_lock(&data->lock, K_FOREVER);
		k_sem_reset(&data->target_event_lock_sem);
		data->ibi_status = -EINPROGRESS;
		data->state = LS_I3C_OP_STATE_IBI;

		ctrlValue = base->SCTRL;
		ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
		ctrlValue |= (CTRL_EVENT_HJ << I3C_SCTRL_EVENT_POS) & I3C_SCTRL_EVENT_MASK;

		base->SCTRL = ctrlValue;
		if (k_sem_take(&data->target_event_lock_sem, I3C_IBI_COMPLETE_TIMEOUT) != 0) {
			ctrlValue = base->SCTRL;
			ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
			base->SCTRL = ctrlValue;
			data->state = LS_I3C_OP_STATE_IDLE;
			k_mutex_unlock(&data->lock);
			LOG_ERR("%s: hotjoin complete timeout", __func__);
			return -ETIMEDOUT;
		}
		ret = data->ibi_status;
		k_mutex_unlock(&data->lock);
		return ret;
	default:
		break;
	}

	return 0;
}
#endif


/**
 * @brief Instructs the I3C Target device to register itself to the I3C Controller
 *
 * This routine instructs the I3C Target device to register itself to the I3C
 * Controller via its parent controller's i3c_target_register() API.
 *
 * @param dev Pointer to target device driver instance.
 * @param cfg Config struct with functions and parameters used by the I3C driver
 * to send bus events
 *
 * @return @see i3c_device_find.
 */
static int ls_i3c_target_register(const struct device *dev, struct i3c_target_config *cfg)
{
	struct ls_i3c_data *data = dev->data;
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *base = config->base;
	data->target_config = cfg;

	if(data->cur_role != I3C_ROLE_TARGET)
	{
		LOG_ERR("%s: device current role isn't target", __func__);
		return -EINVAL;
	}
	base->SERRWARN = base->SERRWARN;
	ls_i3c_enable_target_interrupt(dev,true);
	return 0;
}

static int ls_i3c_target_unregister(const struct device *dev, struct i3c_target_config *cfg)
{
	struct ls_i3c_data *data = dev->data;

	data->target_config = NULL;

	ls_i3c_enable_target_interrupt(dev,false);
	return 0;
}

static int ls_i3c_target_tx_write(const struct device *dev, uint8_t *buf, uint16_t len,uint8_t hdr_mode)
{
	struct ls_i3c_data *data = dev->data;
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *base = config->base;
	uint32_t i = 0;
	uint8_t *tx_buf = buf;

	if (hdr_mode != 0) {
		LOG_ERR("%s: HDR not supported", __func__);
		return -ENOSYS;
	}
	if ((buf == NULL) || (len == 0)) {
		LOG_ERR("%s: Data buffer configuration failed", __func__);
		return -EINVAL;
	}

	/* 临界区：关中断保证 state 检查、清 TXSEND、flush、预填不被 I3C ISR 打断。
	 * 否则判断通过后若 master 恰好发起读，MATCHED/TXSEND 分支会与本函数
	 * 交错写 SWDATAB，且 FLUSHTB 会抹掉 ISR 已写入的字节。临界区很短。 */
	unsigned int key = irq_lock();

	/* RD 态（master 正在读）拒绝填充：TX FIFO 正被硬件排空，
	 * 此时 FLUSHTB+重填会破坏进行中的读。 */
	if (data->state == LS_I3C_OP_STATE_RD) {
		irq_unlock(key);
		return 0;
	}

	/* 填充期间关闭 TXSEND：防止残留使能态在 FIFO 非满时触发 ISR，
	 * 导致 read_requested/read_processed 回调与本函数的写交错 */
	base->SINTCLR = I3C_SINTSET_TXSEND_MASK;

	/* IDLE 态不重复 FLUSHTB：ISR 的 STOP 分支已在每笔传输结束时清空
	 * TX FIFO（见 ls_i3c_target_isr），此处 TXCOUNT=0，续写即从 buf[0]
	 * 起装；重复 flush 属双重操作（原厂指引：传输中避免 FLUSHTB）。 */

	/* 以 FIFO 现存字节数（TXCOUNT）为起点续写 buf：
	 * - IDLE：flush 后 TXCOUNT=0，从 buf[0] 开始装；
	 * - WR（master 正在写本机、为随后的 restart+读备数据）：FLUSHTB 无效，
	 *   FIFO 中可能残留上次预填的字节。本驱动的预填模型里 FIFO 内容始终
	 *   是同一 buf 的前缀，因此从 TXCOUNT 处续写不会产生重复或错位；
	 *   若 TXCOUNT >= len（FIFO 中已有的前缀已覆盖本次长度），不写任何
	 *   字节，仅按需要开 TXSEND。
	 * 仅当整个 buf 写完时在末字节标 END0；装不下的部分由 TXSEND 中断
	 * 经 read_processed_cb 续发（ISR 为最后字节标 END0）。
	 * 返回 FIFO 中本 buf 的字节总数，调用方用它做续发记账（tx_idx）。 */
	i = (base->SDATACTRL & I3C_SDATACTRL_TXCOUNT_MASK) >>
	    I3C_SDATACTRL_TXCOUNT_POS;

	while ((i < len) && ((base->SDATACTRL & I3C_SDATACTRL_TXFULL_MASK) == 0)) {
		uint32_t byte = tx_buf[i++];
		if (i == len) {
			byte |= I3C_SWDATAB_END0_MASK;
		}
		base->SWDATAB = byte;
	}
	if (i < len) {
		base->SINTSET = I3C_SINTSET_TXSEND_MASK;
	}

	irq_unlock(key);
	return i;
}

static int ls_i3c_i2c_api_configure(const struct device *dev, uint32_t dev_config)
{
	return -ENOSYS;
}

static int ls_i3c_i2c_api_transfer(const struct device *dev,
				     struct i2c_msg *msgs,
				     uint8_t num_msgs,
				     uint16_t addr)
{
	const struct ls_i3c_config *config = dev->config;
	struct ls_i3c_data *data = dev->data;
	I3C_TypeDef *base = (I3C_TypeDef *)config->base;
	int ret = 0;

	if (msgs == NULL) {
		return -EINVAL;
	}

	if (addr == 0U) {
		return -EINVAL;
	}

	if(num_msgs < 1)
	{
		return -EINVAL;
	}

	if(data->common.ctrl_config.scl.i2c == 0)
	{
		LOG_ERR(" %s : I2C clock is not configured !", __func__);
		return -ENOSYS;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	LL_I3C_EnableIT_FC(base);
	LL_I3C_EnableIT_CFNF(base);
	LL_I3C_EnableIT_SFNE(base);
	LL_I3C_EnableIT_RXFNE(base);
	LL_I3C_EnableIT_TXFNF(base);
	LL_I3C_ClearFlag_ERR(base);
	LL_I3C_EnableIT_ERR(base);
	ret = i3c_ls_curr_msg_init(dev, NULL, msgs, num_msgs, addr);

	ret = ls_i3c_request_transfer_flag(dev);
	if(ret !=0){
		LOG_ERR("%s: i3c_i2c_transfer to 0x%02x, num_msgs=%u: request transfer failed, err=%d",
			dev->name, addr, num_msgs, ret);
	}
	LL_I3C_DisableIT_FC(base);
	LL_I3C_DisableIT_CFNF(base);
	// LL_I3C_DisableIT_SFNE(base);
	LL_I3C_DisableIT_RXFNE(base);
	LL_I3C_DisableIT_TXFNF(base);
	LL_I3C_DisableIT_ERR(base);

	k_mutex_unlock(&data->lock);

	return ret;
}
static const struct i3c_driver_api ls_i3c_driver_api = {
	.configure = ls_i3c_configure,
	.config_get = ls_i3c_config_get,
	// .recover_bus = ls_i3c_recover_bus,

	.do_daa = ls_i3c_do_daa,
	.do_ccc = ls_i3c_do_ccc,

	.target_tx_write = ls_i3c_target_tx_write,
	.i3c_device_find = ls_i3c_device_find,

	.i3c_xfers = ls_i3c_transfer,

	.target_register = ls_i3c_target_register,
	.target_unregister = ls_i3c_target_unregister,

	.i2c_api.configure = ls_i3c_i2c_api_configure,
	.i2c_api.transfer = ls_i3c_i2c_api_transfer,
	// .i2c_api.recover_bus = ls_i3c_recover_bus,
#ifdef CONFIG_I3C_USE_IBI
	.ibi_enable = ls_i3c_ibi_enable,
	.ibi_disable = ls_i3c_ibi_disable,
	.ibi_raise = ls_i3c_target_ibi_raise,
#endif 
};



#define DT_INST_TGT_PID_PROP_OR(id, prop, idx)                                                     \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx), (DT_INST_PROP_BY_IDX(id, prop, idx)), (0))
#define DT_INST_TGT_PID_RAND_PROP_OR(id, prop, idx)                                                \
	COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),                                           \
			((DT_INST_PROP_BY_IDX(id, prop, 0) & (0x1)) != 0), (0))				


#define I3C_LS_DEVICE(id)                                                                        \
    IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_INST_DEFINE(id)));                                  \
	static void ls_i3c_config_func_##id(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(id), DT_INST_IRQ(id, priority), ls_i3c_isr,             \
			    DEVICE_DT_INST_GET(id), 0);                                            \
		irq_enable(DT_INST_IRQN(id));                                                      \
	};																						\
	static struct i3c_device_desc ls_i3c_device_array_##id[] = I3C_DEVICE_ARRAY_DT_INST(id); \
	static struct i3c_i2c_device_desc ls_i2c_device_array_##id[] =                       \
		I3C_I2C_DEVICE_ARRAY_DT_INST(id);                                                  \
	static const struct ls_i3c_config ls_i3c_config_##id = {                               \
		.base = (I3C_TypeDef *)DT_INST_REG_ADDR(id),                                    \
		.clock_frequency = (DT_INST_PROP(id, clock_frequency))*2,\
		.irq_config_func = ls_i3c_config_func_##id,                                      \
		.common.dev_list.i3c = ls_i3c_device_array_##id,                                 \
		.common.dev_list.num_i3c = ARRAY_SIZE(ls_i3c_device_array_##id),                 \
		.common.dev_list.i2c = ls_i2c_device_array_##id,                             \
		.common.dev_list.num_i2c = ARRAY_SIZE(ls_i2c_device_array_##id),             \
        IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id), )) \
        IF_ENABLED(DT_HAS_CLOCKS(id), (.ccfg = LS_DT_CLK_CFG_ITEM(id), )) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(id, resets), (.reset = RESET_DT_SPEC_INST_GET(id), )) \
	};                                                                                         \
	static struct ls_i3c_data ls_i3c_data_##id = {                                            \
		.common.ctrl_config.scl.i3c = DT_INST_PROP_OR(id, i3c_scl_hz, 0),                   \
		.common.ctrl_config.scl.i2c = DT_INST_PROP_OR(id, i2c_scl_hz, 0),					\
		.common.ctrl_config.is_secondary = DT_INST_PROP_OR(id, is_secondary, false),          \
		.config_target.static_addr = DT_INST_PROP_OR(id, static_address, 0),               \
		.config_target.pid = ((uint64_t)DT_INST_TGT_PID_PROP_OR(id, tgt_pid, 0) << 32) |   \
				     DT_INST_TGT_PID_PROP_OR(id, tgt_pid, 1),                      \
		.config_target.pid_random = DT_INST_TGT_PID_RAND_PROP_OR(id, tgt_pid, 0),          \
		.config_target.bcr = DT_INST_PROP(id, bcr),                                        \
		.config_target.dcr = DT_INST_PROP_OR(id, dcr, 0),                                  \
		.config_target.supported_hdr = false,												\
		.config_target.max_read_len = DT_INST_PROP_OR(id, maximum_read, 0),                \
		.config_target.max_write_len = DT_INST_PROP_OR(id, maximum_write, 0),              \
	};																						\
	DEVICE_DT_INST_DEFINE(id, ls_i3c_init, NULL, &ls_i3c_data_##id, &ls_i3c_config_##id, \
			      POST_KERNEL, CONFIG_I3C_CONTROLLER_INIT_PRIORITY,                    \
			      &ls_i3c_driver_api);													\
								

DT_INST_FOREACH_STATUS_OKAY(I3C_LS_DEVICE)