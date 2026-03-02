#ifndef ESPI_LPC_COMMON_H_
#define ESPI_LPC_COMMON_H_
#include <zephyr/sys/slist.h>
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/sys/atomic.h>
#include "fifo.h"
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

struct peri_ioport_content {
    void (*io_read)(const struct peri_ioport_content *ioport,uint8_t size,void *res);
    void (*io_write)(const struct peri_ioport_content *ioport,uint8_t size,uint8_t *data);
    struct device *ctx;
    uint16_t addr;
};

struct peri_ioport {
    sys_snode_t node;
    const struct peri_ioport_content *content;
};

struct peri_mem_content {
    bool (*mem_read)(struct peri_mem_content *mem,uint32_t addr,uint8_t size,void *res);
    bool (*mem_write)(struct peri_mem_content *mem,uint32_t addr,uint8_t size,uint8_t *data);
    void *ctx;
};

struct peri_mem {
    sys_snode_t node;
    struct peri_mem_content *content;
};

struct upstream_irq_type {
    uint8_t idx;
    uint8_t type;
};
#define DT_HAS_UP_IRQ(inst) DT_NODE_HAS_PROP(DT_DRV_INST(inst), up_irq)

#define UP_IRQ_CONFIG_NAME(node_id) _CONCAT(__up_irq,DEVICE_DT_NAME_GET(node_id))

#define UPSTREAM_IRQ_DT_DEFINE(node_id) \
    static const struct upstream_irq_type UP_IRQ_CONFIG_NAME(node_id) = { \
        .idx = DT_PROP_BY_IDX(node_id,up_irq,0),\
        .type = DT_PROP_BY_IDX(node_id,up_irq,1),\
    };

#define UPSTREAM_IRQ_DT_INST_DEFINE(inst) UPSTREAM_IRQ_DT_DEFINE(DT_DRV_INST(inst))

#define UPSTREAM_IRQ_DT_CONFIG_GET(node_id) &UP_IRQ_CONFIG_NAME(node_id)

#define UPSTREAM_IRQ_DT_INST_CONFIG_GET(inst) UPSTREAM_IRQ_DT_CONFIG_GET(DT_DRV_INST(inst))


#define ESPI_RECOVER_MAGIC {'E','S','P','I','R','C','V','R'}

#define UP_IRQ_EDGE_TYPE 0

struct espi_cfg_recover {
    uint8_t magic[8];
    uint32_t gen_cfg;
    uint32_t per_ch0_cfg;
    uint32_t vwir_ch1_cfg;
    uint32_t oob_ch2_cfg;
    uint32_t fls_ch3_cfg;
};

struct espi_sysevent_base {
    uint8_t s02_ms;
    uint8_t s03_ms;
    uint8_t s04_sm;
    uint8_t s05_sm;
    uint8_t s06_sm;
    uint8_t s07_ms;
};

struct espi_lpc_ls_config {
	void (*irq_config_func)(const struct device *);
    void *reg;
    void (*raise_edge_irq)(const struct device *,uint8_t);
    void (*set_level_irq)(const struct device *,uint8_t,uint8_t);
    struct host_bmc_msg_exch hb_exch;
    struct espi_sysevent_base *sysevent_base;
    struct espi_cfg_recover *recover_data;
    struct gpio_dt_spec cs;
    IF_ENABLED(CONFIG_PINCTRL, (const struct pinctrl_dev_config *pcfg;))
    IF_ENABLED(CONFIG_CLOCK_CONTROL, (struct ls_clk_cfg ccfg;))
    IF_ENABLED(CONFIG_RESET, (struct reset_dt_spec reset;))
};

struct espi_lpc_ls_data {
    const struct espi_lpc_ls_config *cfg;
    sys_slist_t peri_io;
    sys_slist_t peri_mem;
    union{
        struct espi_data{
            struct k_spinlock vw_tx_lock;
            struct gpio_callback cs_cb;
        }espi;
        struct {
            struct k_spinlock serirq_src_lock;
            struct k_spinlock int_msk_lock;
            uint32_t serirq_edge_mask;
        }lpc;
    }u;
};

#define VUART_FIFO_SIZE 16
struct host_vuart_fifo {
    struct fifo_env b2h;
    struct fifo_env h2b;
    uint8_t b2h_buf[VUART_FIFO_SIZE];
    uint8_t h2b_buf[VUART_FIFO_SIZE];
    bool host_rx_from_vuart;
    bool host_tx_to_vuart;
};

enum vuart_hb_msg_type {
    B_RX_AVAIL,
    B_TX_EMPTY,
    H_RX_AVAIL,
    H_TX_EMPTY,
    VUART_MODE_SET,
};

struct vuart_hb_msg {
    enum vuart_hb_msg_type type;
    bool host_rx_from_vuart;
    bool host_tx_to_vuart;
};

struct host_kcs_env {
    atomic_t lock;
    uint8_t status;
    uint8_t data_out;
    uint8_t data_in;
};

enum kcs_hb_msg_type {
    KCS_IBF_EVENT,
    KCS_OBF_EVENT,
};

struct espi_vwire_msg {
    uint8_t vw_idx;
};

#ifdef CONFIG_ESPI_LPC_MBOX
struct host_bmc_msg_exch {
    const struct device *dev;
    void (*rx_callback)(const struct device *dev,void *msg);
    const struct mbox_dt_spec mbox_tx;
    const struct mbox_dt_spec mbox_rx;
};
#define HOST_BMC_MSG_EXCH_INIT(idx,rx_cb,peer_rx_cb) {\
    .dev = DEVICE_DT_GET(DT_DRV_INST(idx)),     \
    .rx_callback = rx_cb,\
    .mbox_tx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), tx),\
    .mbox_rx = MBOX_DT_SPEC_GET(DT_INST_PHANDLE(idx, mbox), rx),\
    }
#else
struct host_bmc_msg_exch {
    const struct device *peer;
    void (*peer_rx_callback)(const struct device *dev,void *msg);
};

#define GET_PEER_DEV(node_id,local_node_id,...)     \
    (DT_SAME_NODE(node_id,local_node_id)?0:(uint32_t)DEVICE_DT_GET(node_id))
#define HOST_BMC_MSG_EXCH_INIT(idx,rx_cb,peer_rx_cb) {\
    .peer = (const struct device *)(DT_FOREACH_CHILD_STATUS_OKAY_SEP_VARGS(DT_INST_PARENT(idx),GET_PEER_DEV,(+),DT_DRV_INST(idx))),  \
    .peer_rx_callback = peer_rx_cb, \
    }
#endif

void espi_lpc_raise_edge_irq(const struct device *dev,uint8_t idx);

void espi_lpc_add_ioport(const struct device *dev,struct peri_ioport *ioport);

void espi_lpc_remove_ioport(const struct device *dev,struct peri_ioport *ioport);

void espi_lpc_add_mem(const struct device *dev,struct peri_mem *mem);

void espi_lpc_remove_mem(const struct device *dev,struct peri_mem *mem);

bool iord_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,void *res);

bool iowr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,uint8_t *data);

bool memwr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,uint8_t *data);

bool memrd_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,void *res);

void host_bmc_msg_exch_init(const struct host_bmc_msg_exch *exch);

void vuart_status_send(const struct host_bmc_msg_exch *exch,enum vuart_hb_msg_type vuart_msg_type);

void vuart_b2h_mode_set(const struct host_bmc_msg_exch *exch,bool host_rx_from_vuart,bool host_tx_to_vuart);

int get_host_vuart_mode_setting(const struct device *dev, uint8_t *rx_enable, uint8_t *tx_enable);

void kcs_env_lock(struct host_kcs_env *env);

void kcs_env_unlock(struct host_kcs_env *env);

void kcs_h2b_send_ibf(const struct host_bmc_msg_exch *exch);

void kcs_b2h_send_obf(const struct host_bmc_msg_exch *exch);

void espi_vwire_msg_send(const struct host_bmc_msg_exch *exch,uint8_t vw_idx);

void host_vuart_rx_callback(const struct device *dev,void *msg);

void bmc_vuart_rx_callback(const struct device *dev, void *msg);

void bmc_kcs_rx_callback(const struct device *dev,void *msg);

void host_kcs_rx_callback(const struct device *dev,void *msg);

void host_espi_rx_callback(const struct device *dev,void *msg);

void bmc_espi_rx_callback(const struct device *dev,void *msg);


#endif
