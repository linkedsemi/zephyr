#define DT_DRV_COMPAT linkedsemi_ls_espi
#include <zephyr/drivers/espi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <stdarg.h>
#include "espi_utils.h"
#include "reg_espi_type.h"
#include "espi_lpc_common.h"

LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);

#define ESPI_SYS_EVT_2_SLP_S5_N	BIT(2)
#define ESPI_SYS_EVT_2_SLP_S4_N BIT(1)
#define ESPI_SYS_EVT_2_SLP_S3_N BIT(0)

#define ESPI_SYS_EVT_3_OOB_RST_WARN BIT(2)
#define ESPI_SYS_EVT_3_PLTRST_N BIT(1)
#define ESPI_SYS_EVT_3_SUS_STAT_N BIT(0)

#define ESPI_SYS_EVT_4_PME_N_VLD BIT(7)
#define ESPI_SYS_EVT_4_WAKE_N_VLD BIT(6)
#define ESPI_SYS_EVT_4_OOB_RST_ACK_VLD BIT(4)
#define ESPI_SYS_EVT_4_PME_N BIT(3)
#define ESPI_SYS_EVT_4_WAKE_N BIT(2)
#define ESPI_SYS_EVT_4_OOB_RST_ACK BIT(0)

#define ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_STATUS_VLD BIT(7)
#define ESPI_SYS_EVT_5_ERROR_NONFATAL_VLD BIT(6)
#define ESPI_SYS_EVT_5_ERROR_FATAL_VLD BIT(5)
#define ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_DONE_VLD BIT(4)
#define ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_STATUS BIT(3)
#define ESPI_SYS_EVT_5_ERROR_NONFATAL BIT(2)
#define ESPI_SYS_EVT_5_ERROR_FATAL BIT(1)
#define ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_DONE BIT(0)

#define ESPI_SYS_EVT_6_HOST_RST_ACK_VLD BIT(7)
#define ESPI_SYS_EVT_6_RCIN_N_VLD BIT(6)
#define ESPI_SYS_EVT_6_SMI_N_VLD BIT(5)
#define ESPI_SYS_EVT_6_SCI_N_VLD BIT(4)
#define ESPI_SYS_EVT_6_HOST_RST_ACK BIT(3)
#define ESPI_SYS_EVT_6_RCIN_N BIT(2)
#define ESPI_SYS_EVT_6_SMI_N BIT(1)
#define ESPI_SYS_EVT_6_SCI_N BIT(0)

#define ESPI_SYS_EVT_7_NMIOUT_N BIT(2)
#define ESPI_SYS_EVT_7_SMIOUT_N BIT(1)
#define ESPI_SYS_EVT_7_HOST_RST_WARN BIT(0)

#ifdef CONFIG_SOC_SERIES_LS101X
static uint8_t pc_rx_buf[0x100];
static uint8_t np_rx_buf[0x100];
static uint8_t pc_tx_buf[0x100];
static uint8_t vw_tx_buf[0x100];
static inline uint8_t *pc_rx_buf_get(const struct device *dev)
{
    return pc_rx_buf;
}

static inline uint8_t *np_rx_buf_get(const struct device *dev)
{
    return np_rx_buf;
}

static inline uint8_t *pc_tx_buf_get(const struct device *dev)
{
    return pc_tx_buf;
}

static inline uint8_t *vw_tx_buf_get(const struct device *dev)
{
    return vw_tx_buf;
}

static inline uint32_t espi_buf_addr_to_reg(const struct device *dev,uint8_t *addr)
{
    return (uint32_t)addr;
}

#else

#define ESPI_NP_RX_BUF_OFFSET	0x800
#define ESPI_PC_RX_BUF_OFFSET	0x900
#define ESPI_PC_TX_BUF_OFFSET	0xa00
#define ESPI_VW_TX_BUF_OFFSET   0xb00

static inline uint8_t *pc_rx_buf_get(const struct device *dev)
{
    struct espi_lpc_ls_config *cfg = dev->config;
    return (uint8_t *)cfg->reg + ESPI_PC_RX_BUF_OFFSET;
}

static inline uint8_t *np_rx_buf_get(const struct device *dev)
{
    struct espi_lpc_ls_config *cfg = dev->config;
    return (uint8_t *)cfg->reg + ESPI_NP_RX_BUF_OFFSET;
}

static inline uint8_t *pc_tx_buf_get(const struct device *dev)
{
    struct espi_lpc_ls_config *cfg = dev->config;
    return (uint8_t *)cfg->reg + ESPI_PC_TX_BUF_OFFSET;
}

static inline uint8_t *vw_tx_buf_get(const struct device *dev)
{
    struct espi_lpc_ls_config *cfg = dev->config;
    return (uint8_t *)cfg->reg + ESPI_VW_TX_BUF_OFFSET;
}

static inline uint32_t espi_buf_addr_to_reg(const struct device *dev,uint8_t *addr)
{
    struct espi_lpc_ls_config *cfg = dev->config;
    return (uint32_t)addr - (uint32_t)cfg->reg;
}
#endif

static int espi_ls_configure(const struct device *dev, struct espi_cfg *cfg)
{
    const struct espi_lpc_ls_config *dev_cfg = dev->config;
    reg_espi_t *reg = dev_cfg->reg;
    uint8_t op_freq_supp = 0;
    switch(cfg->max_freq)
    {
    case 20:
    
    break;
    case 25:

    break;
    case 66:

    break;
    default:
        __ASSERT(0,"illegal espi freq\n");
    break;
    }
    reg->GEN_CFG = cfg->io_caps<<ESPI_GEN_CFG_IO_MODE_SUPP_POS|op_freq_supp<<ESPI_GEN_CFG_OP_FREQ_SUPP_POS|cfg->channel_caps<<ESPI_GEN_CFG_CH_SUPP_POS;
    return 0;
}

static bool espi_ls_get_channel_status(const struct device *dev,enum espi_channel ch)
{
    bool ready = false;
    const struct espi_lpc_ls_config *cfg = dev->config;
    reg_espi_t *reg = cfg->reg;
    switch(ch)
    {
    case ESPI_CHANNEL_PERIPHERAL:
        if(reg->PER_CH0_CFG&ESPI_CH0_CFG_PER_CH_RDY_MASK)
        {
            ready = true;
        }
    break;
    case ESPI_CHANNEL_VWIRE:
        if(reg->VWIR_CH1_CFG&ESPI_CH1_CFG_VWIR_CH_RDY_MASK)
        {
            ready = true;
        }
    break;
    case ESPI_CHANNEL_OOB:
        if(reg->OOB_CH2_CFG&ESPI_CH2_CFG_OOB_CH_RDY_MASK)
        {
            ready = true;
        }
    break;
    case ESPI_CHANNEL_FLASH:
        if(reg->FLS_CH3_CFG&ESPI_CH3_CFG_FLS_CH_RDY_MASK)
        {
            ready = true;
        }
    break;
    default:
        __ASSERT(0,"illegal espi channel\n");
    break;
    }
    return ready;
}

static int espi_ls_read_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{
    switch(op)
    {
    case E8042_OBF_HAS_CHAR:

    break;
    case E8042_IBF_HAS_CHAR:

    break;
    case E8042_READ_KB_STS:

    break;
    case EACPI_OBF_HAS_CHAR:

    break;
    case EACPI_IBF_HAS_CHAR:

    break;
    case EACPI_READ_STS:

    break;
    default:
        __ASSERT(0,"illegal espi read lpc request op\n");
    break;
    }
    return 0;
}

static int espi_ls_write_lpc_request(const struct device *dev,enum lpc_peripheral_opcode op,uint32_t *data)
{

    switch(op)
    {
    case E8042_WRITE_KB_CHAR:

    break;
    case E8042_WRITE_MB_CHAR:

    break;
    case E8042_RESUME_IRQ:

    break;
    case E8042_PAUSE_IRQ:

    break;
    case E8042_CLEAR_OBF:

    break;
    case E8042_SET_FLAG:

    break;
    case E8042_CLEAR_FLAG:

    break;
    default:
        __ASSERT(0,"illegal espi write lpc request op\n");
    break;
    }
    return 0;
}

static int espi_ls_send_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t level)
{
    switch(vw)
    {
    case ESPI_VWIRE_SIGNAL_PME:

    break;
    case ESPI_VWIRE_SIGNAL_WAKE:

    break;
    case ESPI_VWIRE_SIGNAL_OOB_RST_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_TARGET_BOOT_STS:

    break;
    case ESPI_VWIRE_SIGNAL_ERR_NON_FATAL:

    break;
    case ESPI_VWIRE_SIGNAL_ERR_FATAL:

    break;
    case ESPI_VWIRE_SIGNAL_TARGET_BOOT_DONE:

    break;
    case ESPI_VWIRE_SIGNAL_HOST_RST_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_RST_CPU_INIT:

    break;
    case ESPI_VWIRE_SIGNAL_SMI:

    break;
    case ESPI_VWIRE_SIGNAL_SCI:

    break;
    case ESPI_VWIRE_SIGNAL_DNX_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_ACK:

    break;
    default:
        __ASSERT(0,"illegal espi send vw\n");
    break;
    }
    return 0;
}

static int espi_ls_receive_vwire(const struct device *dev,enum espi_vwire_signal vw,uint8_t *level)
{
    switch(vw)
    {
    case ESPI_VWIRE_SIGNAL_SLP_S3:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_S4:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_S5:

    break;
    case ESPI_VWIRE_SIGNAL_OOB_RST_WARN:

    break;
    case ESPI_VWIRE_SIGNAL_PLTRST:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_STAT:

    break;
    case ESPI_VWIRE_SIGNAL_NMIOUT:

    break;
    case ESPI_VWIRE_SIGNAL_SMIOUT:

    break;
    case ESPI_VWIRE_SIGNAL_HOST_RST_WARN:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_A:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_PWRDN_ACK:

    break;
    case ESPI_VWIRE_SIGNAL_SUS_WARN:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_WLAN:

    break;
    case ESPI_VWIRE_SIGNAL_SLP_LAN:

    break;
    case ESPI_VWIRE_SIGNAL_HOST_C10:

    break;
    case ESPI_VWIRE_SIGNAL_DNX_WARN:

    break;
    default:
        __ASSERT(0,"illegal espi receive vw\n");
    break;
    }
    return 0;
}

static int espi_ls_send_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_receive_oob(const struct device *dev,struct espi_oob_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_read(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_write(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_flash_erase(const struct device *dev,struct espi_flash_packet *pckt)
{
    return 0;
}

static int espi_ls_manage_callback(const struct device *dev,struct espi_callback *callback,bool set)
{
    struct espi_lpc_ls_data *data = dev->data;
    return espi_manage_callback(&data->callbacks,callback,set);
}

static const struct espi_driver_api espi_ls_driver_api = {
    .config = espi_ls_configure,
    .get_channel_status = espi_ls_get_channel_status,
    .read_lpc_request = espi_ls_read_lpc_request,
    .write_lpc_request = espi_ls_write_lpc_request,
    .send_vwire = espi_ls_send_vwire,
    .receive_vwire = espi_ls_receive_vwire,
    .send_oob = espi_ls_send_oob,
    .receive_oob = espi_ls_receive_oob,
    .flash_read = espi_ls_flash_read,
    .flash_write = espi_ls_flash_write,
    .flash_erase = espi_ls_flash_erase,
    .manage_callback = espi_ls_manage_callback,
};

static void espi_vw_tx(const struct device *dev,uint8_t count,uint8_t idx,uint8_t val,...)
{
	const struct espi_lpc_ls_config *cfg = dev->config;
	struct espi_lpc_ls_data *data = dev->data;
    reg_espi_t *reg = cfg->reg;
	va_list vargs;
	uint32_t espi_stat;
	uint32_t vw_first_word = val<<16|idx<<8|(count - 1);
	uint8_t *vw_tx = vw_tx_buf_get(dev);
	k_spinlock_key_t flags = k_spin_lock(&data->u.espi.vw_tx_lock);
	do{
		espi_stat = reg->STATUS;
	}while(espi_stat & ESPI_STATUS_SET_VWIRE_AVAIL_MASK);
    reg->UP_VWIR_LEN_M1 = count*2;
	count--;
	va_start(vargs,val);
	if(count>0)
	{
		idx = va_arg(vargs,unsigned);
		val = va_arg(vargs,unsigned);
		vw_first_word |= idx<<24;
		*vw_tx++ = val;
		count--;
	}
	while(count--)
	{
		idx = va_arg(vargs,unsigned);
		val = va_arg(vargs,unsigned);
		*vw_tx++ = idx;
		*vw_tx++ = val;
	}
	va_end(vargs);
    reg->UP_VWIR_DAT = vw_first_word;
    reg->STATUS_SET = ESPI_STATUS_ALERT_SET_MASK|ESPI_STATUS_SET_VWIRE_AVAIL_MASK;
	k_spin_unlock(&data->u.espi.vw_tx_lock,flags);
}

static void mem_io_rd_rsp_pkt_build(uint8_t len,uint32_t data,uint8_t *up_data_buf,uint32_t *up_first_word,uint16_t *up_len)
{
	*up_first_word = 0x0f| len<<16|(data & 0xff)<<24;
	*up_len = 3+len;
	up_data_buf[0] = data>>8;
	up_data_buf[1] = data>>16;
	up_data_buf[2] = data>>24;
}

static void espi_send_oob_rst_ack(const struct device *dev)
{
	espi_vw_tx(dev,1,4,ESPI_SYS_EVT_4_OOB_RST_ACK_VLD|ESPI_SYS_EVT_4_OOB_RST_ACK);
}

static void espi_send_host_rst_ack(const struct device *dev)
{
	espi_vw_tx(dev,1,6,ESPI_SYS_EVT_6_HOST_RST_ACK_VLD|ESPI_SYS_EVT_6_HOST_RST_ACK);
}

static void ls_espi_isr(void *arg)
{
	struct device *dev = (struct device *) arg;
	const struct espi_lpc_ls_config *cfg = dev->config;
	struct espi_lpc_ls_data *data = dev->data;
    reg_espi_t *reg = cfg->reg;
    uint32_t stt = reg->INTERRUPT_STT;
    if(stt&ESPI_INTR_STT_PSTC_FREE_MASK)
    {
		uint8_t *rx = pc_rx_buf_get(dev);
		uint8_t opcode = rx[0];
		reg->INTERRUPT_CLEAR = ESPI_INTR_STT_PSTC_FREE_MASK;
		if((opcode&~0x3)==0x4c) //PUT_MEMWR32_SHORT
		{
			uint8_t len = (opcode & 0x3) + 1;
			uint32_t addr = rx[1]<<24|rx[2]<<16|rx[3]<<8|rx[4];
			memwr_short(data,len,addr,&rx[5]);
		}
        reg->STATUS_SET = ESPI_STATUS_SET_PC_FREE_MASK;
    }
    if(stt&ESPI_INTR_STT_NPST_FREE_MASK)
    {
		uint8_t *rx = np_rx_buf_get(dev);
		uint8_t opcode = rx[0];
        reg->INTERRUPT_CLEAR = ESPI_INTR_STT_NPST_FREE_MASK;
		if((opcode&~0xf)==0x40)
		{
			uint32_t up_first_word = 0;
			uint16_t up_len = 1;
			uint8_t len = (opcode&0x3) + 1;
			uint8_t *up_data_remain = pc_tx_buf_get(dev); 
			uint32_t res;
			if(opcode&0x8)	//PUT_MEMRD32_SHORT
			{
				uint32_t addr = rx[1]<<24|rx[2]<<16|rx[3]<<8|rx[4];
				memrd_short(data,len,addr,&res);
				mem_io_rd_rsp_pkt_build(len,res,up_data_remain,&up_first_word,&up_len);
			}else
			{
				uint16_t addr = rx[1]<<8 | rx[2];
				if(opcode&0x4) //PUT_IOWR_SHORT
				{
					iowr_short(data,len,addr,&rx[3]);
					up_first_word = 6;
					up_len = 3;
				}else	//PUT_IORD_SHORT
				{
					iord_short(data,len,addr,&res);
					mem_io_rd_rsp_pkt_build(len,res,up_data_remain,&up_first_word,&up_len);
				}
			}
            reg->UP_PSTC_DAT = up_first_word;
            reg->UP_PSTC_LEN_M1 = up_len - 1;
		}
        reg->STATUS_SET = ESPI_STATUS_ALERT_SET_MASK | ESPI_STATUS_SET_PC_AVAIL_MASK | ESPI_STATUS_SET_NP_FREE_MASK;
    }
    if(stt&ESPI_INTR_STT_VWIR_FREE_MASK)
    {

    }
    if(stt&ESPI_INTR_STT_OOBP_FREE_MASK)
    {

    }
    if(stt&ESPI_INTR_STT_DN_CNFG_MASK)
    {
        
    }
    if(stt&ESPI_INTR_STT_DN_VWIR_02_MASK)
    {
        uint32_t dn_vwir_sys0 = reg->DN_VWIR_SYS0;
        uint8_t s02 = dn_vwir_sys0>>16&0xf;
        (void)s02;
        reg->INTERRUPT_CLEAR = ESPI_INTR_STT_DN_VWIR_02_MASK;

    }
    if(stt&ESPI_INTR_STT_DN_VWIR_03_MASK)
    {
        uint32_t dn_vwir_sys0 = reg->DN_VWIR_SYS0;
        uint8_t s03 = dn_vwir_sys0>>24&0xf;
        if(s03&ESPI_SYS_EVT_3_OOB_RST_WARN)
        {
			espi_send_oob_rst_ack(dev);
        }
        reg->INTERRUPT_CLEAR = ESPI_INTR_STT_DN_VWIR_03_MASK;

    }
    if(stt&ESPI_INTR_STT_DN_VWIR_07_MASK)
    {
        uint32_t dn_vwir_sys1 = reg->DN_VWIR_SYS1;
		uint8_t s07 = dn_vwir_sys1>>24&0xf;
		if(s07&ESPI_SYS_EVT_7_HOST_RST_WARN)
		{
			espi_send_host_rst_ack(dev);
		}
        reg->INTERRUPT_CLEAR = ESPI_INTR_STT_DN_VWIR_07_MASK;
    }
    if(stt&ESPI_INTR_STT_DN_REST_MASK)
    {
        reg->INTERRUPT_CLEAR = ESPI_INTR_STT_DN_REST_MASK;
        struct espi_event evt = {
            .evt_type = ESPI_BUS_RESET,
            .evt_details = 0,
            .evt_data = 0,
        };
        espi_send_callbacks(&data->callbacks,dev,evt);
    }
}


static void espi_send_boot_done(const struct device *dev)
{
	espi_vw_tx(dev,1,5,ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_DONE_VLD|ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_DONE|
						ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_STATUS_VLD|ESPI_SYS_EVT_5_SLAVE_BOOT_LOAD_STATUS);
}

static void espi_send_edge_irq(const struct device *dev,uint8_t idx)
{
	espi_vw_tx(dev,2,idx&0x80?1:0,1<<7|(idx&0x7f),idx&0x80?1:0,idx&0x7f);
}

static void espi_send_level_irq(const struct device *dev,uint8_t idx,uint8_t active)
{
	espi_vw_tx(dev,1,idx&0x80?1:0,active<<7|(idx&0x7f));
}

static void espi_reg_init(const struct device *dev)
{
	const struct espi_lpc_ls_config *const cfg = dev->config;
    reg_espi_t *reg = cfg->reg;
    reg->GEN_CFG = 0<<ESPI_GEN_CFG_IO_MODE_SUPP_POS|0<<ESPI_GEN_CFG_OP_FREQ_POS|0xf<<ESPI_GEN_CFG_CH_SUPP_POS;
    reg->PER_CH0_CFG = 1<<ESPI_CH0_CFG_PER_MAX_PLOAD_SUPP_POS|ESPI_CH0_CFG_PER_CH_RDY_MASK;
    reg->VWIR_CH1_CFG = 0x1f<<ESPI_CH1_CFG_VWIR_MAX_CNT_SUPP_POS|ESPI_CH1_CFG_VWIR_CH_RDY_MASK;
    reg->OOB_CH2_CFG = 1<<ESPI_CH2_CFG_OOB_MAX_PLOAD_SUPP_POS|ESPI_CH2_CFG_OOB_CH_RDY_MASK;
    reg->FLS_CH3_CFG = 3<<ESPI_CH3_CFG_FLS_SHARE_MODE_POS|1<<ESPI_CH3_CFG_FLS_MAX_PLOAD_SUPP_POS|ESPI_CH3_CFG_FLS_CH_RDY_MASK;
    reg->STATUS_SET = ESPI_STATUS_ALERT_CLEAR_MASK|ESPI_STATUS_SET_VWIRE_FREE_MASK
                    |ESPI_STATUS_SET_OOB_FREE_MASK|ESPI_STATUS_SET_NP_FREE_MASK
                    |ESPI_STATUS_SET_PC_FREE_MASK|ESPI_STATUS_SET_FLASH_C_FREE_MASK
                    |ESPI_STATUS_SET_FLASH_NP_FREE_MASK;
    reg->DW_NPST_ADR = espi_buf_addr_to_reg(dev,np_rx_buf_get(dev));
    reg->DW_PSTC_ADR = espi_buf_addr_to_reg(dev,pc_rx_buf_get(dev));
    reg->UP_PSTC_ADR = espi_buf_addr_to_reg(dev,pc_tx_buf_get(dev));
    reg->UP_VWIR_ADR = espi_buf_addr_to_reg(dev,vw_tx_buf_get(dev));

    reg->INTERRUPT_CLEAR = ESPI_INTR_STT_DN_REST_MASK|ESPI_INTR_STT_PSTC_FREE_MASK|ESPI_INTR_STT_NPST_FREE_MASK
                        |ESPI_INTR_STT_DN_VWIR_02_MASK|ESPI_INTR_STT_DN_VWIR_03_MASK|ESPI_INTR_STT_DN_VWIR_07_MASK;
    reg->INTERRUPT_MASK = ESPI_INTR_STT_DN_REST_MASK|ESPI_INTR_STT_PSTC_FREE_MASK|ESPI_INTR_STT_NPST_FREE_MASK
                        |ESPI_INTR_STT_DN_VWIR_02_MASK|ESPI_INTR_STT_DN_VWIR_03_MASK|ESPI_INTR_STT_DN_VWIR_07_MASK;
}

static int espi_ls_init(const struct device *dev)
{
	const struct espi_lpc_ls_config *const cfg = dev->config;
	struct espi_lpc_ls_data *const data = dev->data;
    int ret;
	sys_slist_init(&data->peri_io);
	sys_slist_init(&data->peri_mem);
	sys_slist_init(&data->callbacks);
    cfg->irq_config_func(dev);
    if(cfg->cctl_cfg.cctl_dev)
    {
		const struct device *clk_dev = cfg->cctl_cfg.cctl_dev;
		if (!device_is_ready(clk_dev)) {
			LOG_DBG("%s device not ready", clk_dev->name);
			return -ENODEV;
		}
		clock_control_on(clk_dev, (clock_control_subsys_t)&cfg->cctl_cfg);
    }
    ret = pinctrl_apply_state(cfg->pcfg,PINCTRL_STATE_DEFAULT);
    if(ret)
    {
        return ret;
    }
    espi_reg_init(dev);
    espi_send_boot_done(dev);
    return 0;
}

#define LS_ESPI_INIT(idx)\
	IF_ENABLED(CONFIG_PINCTRL,(PINCTRL_DT_INST_DEFINE(idx);))\
    static void espi_ls_irq_config_func_##idx(const struct device *dev)	\
    {\
        IRQ_CONNECT(DT_INST_IRQN(idx),DT_INST_IRQ(idx, priority),\
                ls_espi_isr,DEVICE_DT_INST_GET(idx), 0);\
        irq_enable(DT_INST_IRQN(idx));\
    }\
    static const struct espi_lpc_ls_config espi_ls_cfg_##idx = {\
        .reg = (reg_espi_t *)DT_INST_REG_ADDR(idx),\
        .irq_config_func = espi_ls_irq_config_func_##idx,\
        .raise_edge_irq = espi_send_edge_irq,\
        .set_level_irq = espi_send_level_irq,\
    	IF_ENABLED(CONFIG_PINCTRL, (.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),)) \
    	IF_ENABLED(DT_HAS_CLOCKS(idx), (.cctl_cfg = LS_DT_CLK_CFG_ITEM(idx),))	 \
    };\
    static struct espi_lpc_ls_data espi_ls_data_##idx;\
    DEVICE_DT_INST_DEFINE(idx,espi_ls_init,NULL,&espi_ls_data_##idx,\
        &espi_ls_cfg_##idx,PRE_KERNEL_2,CONFIG_ESPI_INIT_PRIORITY,\
        &espi_ls_driver_api);
            
DT_INST_FOREACH_STATUS_OKAY(LS_ESPI_INIT)