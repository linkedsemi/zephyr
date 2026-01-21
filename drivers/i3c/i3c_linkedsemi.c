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
// #include "field_manipulate.h"

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
#define I3C_TARGET_MATCH_START_STOP 1
#define I3C_TARGET_NACK_REQUEST 0
#define I3C_TARGET_ONCE_WRITE_TX_LEN 4

/* target event status  */
#define STATUS_EVDET_NONE            0
#define STATUS_EVDET_REQ_NOT_SENT    1
#define STATUS_EVDET_REQ_SENT_NACKED 2
#define STATUS_EVDET_REQ_SENT_ACKED  3

/* Private define for CCC command */
#define I3C_BROADCAST_RSTDAA          (0x00000006U)
#define I3C_BROADCAST_ENTDAA          (0x00000007U)

#define __LS_I3C_GET_FLAG(__HANDLE__, __FLAG__) (((((__HANDLE__)->EVR) &\
                                                    (__FLAG__)) == (__FLAG__)) ? SET : RESET)
#define __LS_I3C_MASTER_GET_ERROR(__HANDLE__) 			((__HANDLE__)->SER)
#define __LS_I3C_SLAVE_GET_ERROR(__HANDLE__) 	((__HANDLE__)->SERRWARN)
#define I3C_CHECK_FLAG(__ISR__, __FLAG__) 		((((__ISR__) & (__FLAG__)) == (__FLAG__)) ? SET : RESET)

#define LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT  	k_ms_to_cyc_ceil64(1)
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

	struct i3c_target_config *target_config;
	struct i3c_config_target config_target;
	struct i3c_fifo_info fifo_info;
	enum ls_i3c_target_oper_state state;
	enum i3c_role cur_role;
	/** Mutex to serialize access */
	struct k_mutex lock;
	struct k_sem target_event_lock_sem; /* Semaphore used for i3c target ibi_raise() */	
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
	if(data->common.ctrl_config.scl.i2c)
	{
		if(data->common.ctrl_config.scl.i2c >= 400000)
		{
			/* I2C bus is FM+ */
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_FMP_NS * config->clock_frequency,1000000000ull) - 1;
			sclh_i2c = DIV_ROUND_UP(config->clock_frequency, data->common.ctrl_config.scl.i2c) - scll_od - 2;
			if (sclh_i2c <
			DIV_ROUND_UP(I3C_SCLH_I2C_MIN_FMP_NS * config->clock_frequency, 1000000000ull) - 1) {
				LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at "
					"current I3C clock "
					"frequency for FM+ I2C bus");
				return -EINVAL;
			}
		}
		else
		{
			/* I2C bus is FM */
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_FM_NS * config->clock_frequency,1000000000ull) - 1;
			sclh_i2c = DIV_ROUND_UP(config->clock_frequency, data->common.ctrl_config.scl.i2c) - scll_od - 2;
			if (sclh_i2c <
		    DIV_ROUND_UP(I3C_SCLH_I2C_MIN_FM_NS * config->clock_frequency, 1000000000ull) - 1) {
			LOG_ERR("Cannot find a combination of SCLL_OD and SCLH_I2C at current I3C "
				"clock "
				"frequency for FM I2C bus");
			return -EINVAL;
			}
		}
		/* 临时性修改，后续看用户对i2c的波形有什么需求。 按I3C spec中计算的波形，占空比很大， 并且scll_od会对I3C波形开漏模式下的速率存在影响*/
		while(scll_od < sclh_i2c)
		{
			scll_od++;
			sclh_i2c--;
		}
		sclh_i2c = 255;
		scll_od = 255;
	}else
	{
		if(config->common.dev_list.num_i2c > 0)
		{
			__ASSERT(0, "have i2c device on bus,but not set frequence\n");
			return -EINVAL;
		}else
		{
			/* Assume no I2C devices on the bus */
			scll_od = DIV_ROUND_UP(I3C_SCLL_OD_MIN_I3C_NS * config->clock_frequency,
						1000000000ull) -
				   1;
			sclh_i2c = 0;	
		}

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


    REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_FREE,free_timing+3);
    REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_SDA_HD,1);
    REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_ASNCR,0);

	aval = (uint8_t)(DIV_ROUND_UP(1000ull * config->clock_frequency, 1000000000ull) - 1);
	REG_FIELD_WR(base->TIMINGR1,I3C_TIMINGR1_AVAL,aval);

    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALL,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLC,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLD,0);
    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLT,0);

    REG_FIELD_WR(base->TIMINGR2,I3C_TIMINGR2_STALLA,0);

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
	base->SCONFIG |= config_target->static_addr;
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

	/*controller basic config*/
	REG_FIELD_WR(base->CFGR,I3C_CFGR_CE3_ENABLE,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_TXTHRES,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_RXTHRES,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_EXITPTRN,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_RSTPTRN,1);
	REG_FIELD_WR(base->CFGR,I3C_CFGR_SMODE,0);
	// REG_FIELD_WR(base->CFGR,I3C_CFGR_STOP_MODE,1);
	
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
	base->IER = I3C_IER_IBIIE_MASK;
#else
	base->IER = 0;
#endif
	/* Initial I3C device as controller or target */
	ls_i3c_dev_init(dev);

	k_sem_init(&data->target_event_lock_sem, 1, 1);
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

	if (LL_I3C_IsActiveFlag_RXFNE(base)) {
		buf[curr_msg->cur_num_xfer] = LL_I3C_ReceiveData8(base);
		// curr_msg->i3c_msg_ptr->num_xfer++;
		curr_msg->cur_num_xfer++;
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

	switch (data->msg_state) {
	case LS_I3C_MSG: {

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

	LL_I3C_RequestTransfer(base);

	/* Wait for whole transfer to complete */
	if (k_sem_take(&data->device_sync_sem, LS_I3C_TRANSFER_TIMEOUT) != 0) {
		return -ETIMEDOUT;
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

	do
	{
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

			ret = i3c_dev_list_daa_addr_helper(&data->common.attached_dev.addr_slots,
							   &config->common.dev_list, pid,
							   false, false,
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
	// wait transfer complete
	while((READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0);

	if(__LS_I3C_GET_FLAG(base,I3C_EVR_ERRF_MASK))
	{
		ret = -EIO;
		LOG_ERR("ENTDAA ERROR , ERROR CODE = 0x%x",__LS_I3C_MASTER_GET_ERROR(base));
		LL_I3C_ClearFlag_ERR(base);
		ls_i3c_xfer_reset(base);
	}

	if(__LS_I3C_GET_FLAG(base,I3C_EVR_FCF_MASK))
	{
		/* Clear frame complete flag */
		LL_I3C_ClearFlag_FC(base);
	}

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
		LOG_ERR("the i3c device currnet role is not controller \n");
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

	if (ls_i3c_request_transfer_flag(dev) == ETIMEDOUT) {
		ret = -ETIMEDOUT;
		LOG_ERR("CCC[0x%02x] %s START error (%d)",
			payload->ccc.id,
			i3c_ccc_is_payload_broadcast(payload) ? "broadcast" : "direct",
			ret);

		goto out_ccc_stop;
	}

	ls_i3c_xfer_reset(base);
	
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
				LOG_ERR("%s : timeout ",__func__);
				timeout = true;
				ret = -ETIMEDOUT;
			}

			exit_condition = ((base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (exit_condition == 0U && timeout == false);

		if(exit_condition == I3C_EVR_ERRF)
		{
			ret = -EIO;
			LOG_ERR(" ERROR bccc , error code = 0X%X",base->SER);
		}
		
		if(timeout == true)
		{
			LOG_ERR(" ERROR bccc: transfer timeout ");
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
			LOG_ERR("%s : error num_target ",__func__);
			ret = -EIO;
			goto out_ccc_stop;
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
				LOG_ERR("%s : timeout ",__func__);
				timeout = true;
				ret = -ETIMEDOUT;
			}
			exit_condition = ((base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (payload->ccc.data_len > 0 && exit_condition == 0);

		if(exit_condition == I3C_EVR_ERRF)
		{
			ret = -EIO;
			LOG_ERR(" ERROR bccc , error code = 0X%X",base->SER);
			goto out_ccc_stop;
		}

		start_time = arch_k_cycle_get_64(); 
		exp_time = LS_I3C_TRANSFER_POLLING_MODE_TIMEOUT * num_target + start_time;	
		uint32_t rnw;
		do{
			if (__LS_I3C_GET_FLAG(base, I3C_EVR_CFNFF) == SET)
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
					LOG_ERR("%s : error num_target ",__func__);
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
					LOG_ERR("%s : error num_target ",__func__);
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
			if ((__LS_I3C_GET_FLAG(base, LL_I3C_EVR_RXFNEF) == SET))
			{
				if(cur_tgt_idx > payload->targets.num_targets)
				{
					LOG_ERR("%s : error num_target ",__func__);
					goto out_ccc_stop;
				}
				if(cur_tgt->data_len != 0)
				{
					*cur_tgt->data = LL_I3C_ReceiveData8(base);
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

			if(arch_k_cycle_get_64() > exp_time)
			{
				LOG_ERR("%s : timeout ",__func__);
				timeout = true;
				ret = -ETIMEDOUT;
				// break;
			}

			/* Calculate exit_condition value based on Frame complete and error flags */
			exit_condition = (READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF));
		} while (exit_condition == 0U && timeout == false);

		if(timeout == true)
		{
			LOG_ERR("CCC[0x%02x]: timeout",payload->ccc.id);
			ret = -ETIMEDOUT;
			goto out_ccc_stop;
		}

		while ((__LS_I3C_GET_FLAG(base, LL_I3C_EVR_RXFNEF) == SET))
		{
			if(cur_tgt->num_xfer < cur_tgt->data_len)
			{
				LOG_ERR("CCC[0x%02x]: received unexpected data",payload->ccc.id);
				ret = -EIO;
				break;
			}
			else
			{
				*cur_tgt->data = LL_I3C_ReceiveData8(base);
				cur_tgt->data++;
				cur_tgt->num_xfer++;
				transfer_num --;
			}
		}

		if(transfer_num != 0)
		{
			LOG_ERR("CCC[0x%02x]: the amount of data transmitted does not match, the remaining amount of data is %d ",payload->ccc.id, transfer_num);
			ret = -EIO;
		}
	}

out_ccc_stop:
	// wait transfer complete
	while((READ_REG(base->EVR) & (I3C_EVR_FCF | I3C_EVR_ERRF)) == 0);
	
	if (__LS_I3C_GET_FLAG(base, LL_I3C_EVR_FCF) == SET)
	{
		LL_I3C_ClearFlag_FC(base);
	}	

	/* Check on error flag */
	if (__LS_I3C_GET_FLAG(base, LL_I3C_EVR_ERRF) == SET)
	{
		/* Clear error flag */

		LOG_ERR("ENTDAA ERROR , ERROR CODE = 0x%x",__LS_I3C_MASTER_GET_ERROR(base));
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

	if (msgs == NULL) {
		return -EINVAL;
	}

	if (target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	if(num_msgs < 1)
	{
		return -EINVAL;
	}

	if (target->dynamic_addr == 0U) {
		return -EINVAL;
	}

	for(uint8_t i=0;i < num_msgs;i++)  // error
	{
		msgs[i].num_xfer = 0;
		if(msgs[i].hdr_mode != 0)
		{
			LOG_ERR("%s : not soupport hdr mode ",__func__);
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

	ret = ls_i3c_request_transfer_flag(dev);
	if(ret !=0){
		LOG_ERR("Failed to transfer messages, err=%d", ret);
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


	

	if(base->SINTMASKED){

		/* Check error or warning has occurred */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_ERRWARN_MASK)) {
			base->SINTCLR = I3C_SINTSET_TXSEND_MASK;
			LOG_ERR("%s: Error %#x", __func__, base->SERRWARN);
			base->SERRWARN = base->SERRWARN;
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTSET_SLVRST_MASK)) {
			LOG_ERR("%s : Slave reset",__func__);
			base->SSTATUS = I3C_SSTATUS_SLVRST_MASK;
		}
		
		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SINTCLR_TXSEND_MASK))
		{
			uint8_t byte;

			status = target_cb->read_processed_cb(data->target_config,
									&byte);
			if(status == 0)
			{
				base->SWDATAB = byte;
			}
			else if(status == 1)
			{
				base->SWDATAB = byte|I3C_SWDATAB_END0_MASK;
			}
		}

		/* Check incoming header matched target dynamic address */
		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SINTCLR_MATCHED_MASK))
		{
			if(data->state != LS_I3C_OP_STATE_IBI)
			{
				if(I3C_CHECK_FLAG(base->SSTATUS,I3C_SSTATUS_STREQRD_MASK))
				{
					/*controller read requset*/
					uint8_t tx_data;
					data->state = LS_I3C_OP_STATE_RD;
					base->SINTSET = I3C_SINTSET_TXSEND_MASK;
					if ((target_cb != NULL) && 
						target_cb->read_requested_cb)
					{
						if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SSTATUS_TXNOTFULL_MASK))
						{
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
				else 
				{
					data->state = LS_I3C_OP_STATE_WR;
					
					/*controller write quest*/
					if ((target_cb != NULL) &&
						(target_cb->write_requested_cb != 0))
					{
						target_cb->write_requested_cb(data->target_config);
					}
				}
			}


			/* If SCONFIG.MATCHSS=1, MATCHED bit must remain 1 to detect next start
				* or stop.
				*
				* Clear the status bit in STOP or START handler.
				*/
			if (I3C_CHECK_FLAG(base->SCONFIG, I3C_SCONFIG_MATCHSS_MASK)) { 
				base->SINTCLR = I3C_SINTCLR_MATCHED_MASK;
			} else {
				base->SSTATUS = I3C_SSTATUS_MATCHED_MASK;
			}
		}

		if (I3C_CHECK_FLAG(base->SCONFIG, I3C_SINTMASK_SLVRST_MASK)) {
			base->SSTATUS = I3C_SINTMASK_SLVRST_MASK;
		}

		if(I3C_CHECK_FLAG(base->SINTMASKED,I3C_SSTATUS_RXPEND_MASK))
		{
			uint8_t rx_count = (base->SDATACTRL & I3C_SDATACTRL_RXCOUNT_MASK) >> I3C_SDATACTRL_RXCOUNT_POS;
			uint8_t rx_data; 
			for (int j = 0; j < rx_count; j++)
			{
				rx_data = (uint8_t)base->SRDATAB;
				target_cb->write_received_cb(data->target_config, rx_data);
			}
		}

		/* Check START or Sr detected */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_START_MASK)) {
			/* The end of xfer is a Sr */
			if ((data->state == LS_I3C_OP_STATE_WR) ||
				(data->state == LS_I3C_OP_STATE_RD)) {
			}

			base->SSTATUS = I3C_SSTATUS_START_MASK;
		}

		/* CCC 'not' automatically handled was received */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_CCC_MASK)) {
			base->SSTATUS = I3C_SSTATUS_CCC_MASK;
		}

		/* CCC handled (handled by IP) */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_CHANDLED_MASK)) {
			base->SSTATUS = I3C_SINTMASK_CHANDLED_MASK;
		}

		/* Event requested. IBI, hot-join, bus control */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_EVENT_MASK)) {
			base->SSTATUS = I3C_SINTMASK_EVENT_MASK;

			if (((base->SSTATUS & I3C_SSTATUS_EVDET_MASK) >> I3C_SSTATUS_EVDET_POS)  ==
				STATUS_EVDET_REQ_SENT_ACKED) {
				k_sem_give(&data->target_event_lock_sem);
			}
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTMASK_DACHG_MASK)) {
			base->SSTATUS = I3C_SSTATUS_DACHG_MASK;
			if(base->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK)
			{
				if(target_config != NULL)
				{
					config_tgt->dynamic_addr =
						(base->SDYNADDR&I3C_SDYNADDR_DADDR_MASK) >> I3C_SDYNADDR_DADDR_POS;
				}
			}
		}

		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SINTSET_NOWCNTLR_MASK)) {
			LOG_ERR("%s: crr not support", __func__);
			base->SSTATUS = I3C_SSTATUS_NOWCNTLR_MASK;
			// base->SCONFIG &= ~I3C_SCONFIG_SLVENA_MASK;
		}
	
		
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SSTATUS_STOP_MASK)) {
			base->SSTATUS = I3C_SSTATUS_STOP_MASK;
			data->state = LS_I3C_OP_STATE_IDLE;
			/* Notify upper layer a STOP condition received */
			if ((target_cb != NULL) && (target_cb->stop_cb != NULL)) {
				target_cb->stop_cb(data->target_config);
			}
			/* Clear DA matched status and re-enable interrupt */
			base->SSTATUS = I3C_SSTATUS_MATCHED_MASK;
			base->SINTSET = I3C_SSTATUS_MATCHED_MASK;
			base->SINTCLR = I3C_SINTSET_TXSEND_MASK;
		}

		/* Event requested. IBI, hot-join, bus control */
		if (I3C_CHECK_FLAG(base->SINTMASKED, I3C_SSTATUS_EVENT_MASK)) {
			base->SSTATUS = I3C_SSTATUS_EVENT_MASK;

			if ((base->SSTATUS & I3C_SSTATUS_EVDET_MASK) ==
			    I3C_SSTATUS_EVDET_MASK) {
				k_sem_give(&data->target_event_lock_sem);
				data->state = LS_I3C_OP_STATE_IDLE;
			}
		}
	}
	
}

static void ls_i3c_log_err_type(const struct device *dev)
{
	const struct ls_i3c_config *config = dev->config;
	I3C_TypeDef *i3c = config->base;

	if (LL_I3C_IsActiveFlag_ANACK(i3c)) {
		LOG_ERR("Address NACK");
	}

	if (LL_I3C_IsActiveFlag_COVR(i3c)) {
		LOG_ERR("Control/Status FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DOVR(i3c)) {
		LOG_ERR("TX/RX FIFO underrun/overrun");
	}

	if (LL_I3C_IsActiveFlag_DNACK(i3c)) {
		LOG_ERR("Data NACK by target");
	}

	if (LL_I3C_IsActiveFlag_PERR(i3c)) {
		switch (LL_I3C_GetMessageErrorCode(i3c)) {
		case LL_I3C_CONTROLLER_ERROR_CE0:
			LOG_ERR("Illegally formatted CCC detected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE1:
			LOG_ERR("Data on bus is not as expected");
			break;
		case LL_I3C_CONTROLLER_ERROR_CE2:
			LOG_ERR("No response to broadcast address");
			break;
		default:
			LOG_ERR("Unsupported error detected");
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

		if((data->msg_state == LS_I3C_MSG)  && (curr_msg->ctrl_msg_idx < curr_msg->num_msgs))
		{
			LL_I3C_RequestTransfer(base);
			return;
		}


		k_sem_give(&data->device_sync_sem);

		// (void)pm_device_runtime_put(dev);
		// pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

		/* Mark bus as idle after each frame complete */
		/* RX FIFO not empty handler */
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
		ret = i3c_ibi_work_enqueue_target_irq(target,(uint8_t *)&IBITgtPayload,IBITgtNbPayload);
		if (ret < 0) {
			LOG_ERR("Enqueuing ibi work fail, ret %d", ret);
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
	i3c_ccc_do_events_set(target, true, &i3c_events);
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
		k_sem_take(&data->target_event_lock_sem, K_FOREVER);
		data->state = LS_I3C_OP_STATE_IBI;

		ctrlValue = base->SCTRL;
		ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
		ctrlValue |= (CTRL_EVENT_IBI << I3C_SCTRL_EVENT_POS) & I3C_SCTRL_EVENT_MASK;
		uint8_t *ibi_payload = request->payload;
		ctrlValue |= (((uint32_t)*ibi_payload<<I3C_SCTRL_IBIDATA_POS) & I3C_SCTRL_IBIDATA_MASK);

		base->IBIEXTIDATA |= I3C_SIBIEXTIDATA_CLR_MASK;
		uint16_t remain_num = request->payload_len - 1;
		if(remain_num > 1)
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
			base->IBIEXTIDATA |= I3C_SIBIEXTIDATA_CLR_MASK;
			LOG_ERR("%s : ibi paload too long , the ibi fifo is full", __func__);
			return -EINVAL;
		}
		base->SCTRL = ctrlValue;
		break;
	case I3C_IBI_CONTROLLER_ROLE_REQUEST:
		LOG_ERR("not supported crr");
		return -ENOTSUP;
		break;
	case I3C_IBI_HOTJOIN:
		k_sem_take(&data->target_event_lock_sem, K_FOREVER);
		data->state = LS_I3C_OP_STATE_IBI;

		ctrlValue = base->SCTRL;
		ctrlValue &= ~I3C_SCTRL_EVENT_MASK;
		ctrlValue |= (CTRL_EVENT_HJ << I3C_SCTRL_EVENT_POS) & I3C_SCTRL_EVENT_MASK;

		base->SCTRL = ctrlValue;
		break;
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
	
	base->SDATACTRL |= I3C_SDATACTRL_FLUSHTB_MASK;
	/* The data want to send is less than the fifo length , So the last data is configured in end mode*/
	if(len <= data->fifo_info.TargetTxFifoSize)
	{
		for(i = 0; i< len-1;)
		{
			base->SWDATAB = tx_buf[i++];
		}
		base->SWDATAB = tx_buf[i++] | I3C_SWDATAB_END0_MASK;
	}else
	{
		while((base->SDATACTRL & I3C_SDATACTRL_TXFULL_MASK) != I3C_SDATACTRL_TXFULL_MASK)
		{
			base->SWDATAB = tx_buf[i++];
		}
	}

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
		LOG_ERR("Failed to transfer messages, err=%d", ret);
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