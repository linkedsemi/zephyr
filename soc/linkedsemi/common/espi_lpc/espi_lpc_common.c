#include <zephyr/sys/util.h>
#include "espi_lpc_common.h"


void espi_lpc_raise_edge_irq(const struct device *dev,uint8_t idx)
{
    const struct espi_lpc_ls_config *cfg = dev->config;
    cfg->raise_edge_irq(dev,idx);
}

bool iord_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,void *res)
{
	sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_io,ptr){
		struct peri_ioport *io = CONTAINER_OF(ptr,struct peri_ioport,node);
		if(addr == io->content->addr)
		{
			io->content->io_read(io->content,size,res);
			return true;
		}
	}
	return false;
}

bool iowr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,uint8_t *data)
{
	sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_io,ptr){
		struct peri_ioport *io = CONTAINER_OF(ptr,struct peri_ioport,node);
		if(addr == io->content->addr)
		{
			io->content->io_write(io->content,size,data);
			return true;
		}
	}
	return false;
}

bool memwr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,uint8_t *data)
{
	sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_mem,ptr){
		struct peri_mem *mem = CONTAINER_OF(ptr,struct peri_mem,node);
		if(mem->content->mem_write(mem->content,addr,size,data))
		{
			return true;
		}
	}
	return false;
}

bool memrd_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,void *res)
{
	sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_mem,ptr){
		struct peri_mem *mem = CONTAINER_OF(ptr,struct peri_mem,node);
		if(mem->content->mem_read(mem->content,addr,size,res))
		{
			return true;
		}
	}
	return false;
}

void espi_lpc_add_ioport(const struct device *dev,struct peri_ioport *ioport)
{
    struct espi_lpc_ls_data *data = dev->data;
	sys_slist_append(&data->peri_io,&ioport->node);
}

void espi_lpc_add_mem(const struct device *dev,struct peri_mem *mem)
{
    struct espi_lpc_ls_data *data = dev->data;
	sys_slist_append(&data->peri_mem,&mem->node);
}

void espi_lpc_remove_ioport(const struct device *dev,struct peri_ioport *ioport)
{
    struct espi_lpc_ls_data *data = dev->data;
	sys_slist_find_and_remove(&data->peri_io,&ioport->node);
}

void espi_lpc_remove_mem(const struct device *dev,struct peri_mem *mem)
{
    struct espi_lpc_ls_data *data = dev->data;
	sys_slist_find_and_remove(&data->peri_mem,&mem->node);
}

void kcs_env_lock(struct host_kcs_env *env)
{
    while(atomic_inc(&env->lock))
    {
        atomic_dec(&env->lock);
    }
}

void kcs_env_unlock(struct host_kcs_env *env)
{
	atomic_dec(&env->lock);
}

#ifdef CONFIG_ESPI_LPC_MBOX
static void espi_lpc_mbox_msg_send(const struct mbox_dt_spec *mbox,void *msg_ptr,size_t size)
{
    struct mbox_msg msg = {
        .data = msg_ptr,
        .size = size,
    };
    mbox_send_dt(mbox, &msg);
}

static void vuart_mbox_status_send(const struct mbox_dt_spec *mbox,enum vuart_hb_msg_type vuart_msg_type)
{
	struct vuart_hb_msg vuart_msg = {
		.type = vuart_msg_type,
	};
	espi_lpc_mbox_msg_send(mbox,&vuart_msg,sizeof(struct vuart_hb_msg));
}

static void hb_exch_rx_callback(const struct device *dev,
				mbox_channel_id_t channel_id, void *user_data,
				struct mbox_msg *data)

{
	const struct host_bmc_msg_exch *exch = user_data;
	exch->rx_callback(exch->dev, (void *)data->data);
}

void host_bmc_msg_exch_init(const struct host_bmc_msg_exch *exch)
{
    mbox_register_callback_dt(&exch->mbox_rx,hb_exch_rx_callback, (void *)exch);
    mbox_set_enabled_dt(&exch->mbox_tx, true);
    mbox_set_enabled_dt(&exch->mbox_rx, true);
}

void vuart_status_send(const struct host_bmc_msg_exch *exch,enum vuart_hb_msg_type vuart_msg_type)
{
	vuart_mbox_status_send(&exch->mbox_tx,vuart_msg_type);
}

void vuart_b2h_mode_set(const struct host_bmc_msg_exch *exch,bool host_rx_from_vuart,bool host_tx_to_vuart)
{
	struct vuart_hb_msg vuart_msg = {
		.type = VUART_MODE_SET,
		.host_rx_from_vuart = host_rx_from_vuart,
		.host_tx_to_vuart = host_tx_to_vuart,
	};
	espi_lpc_mbox_msg_send(&exch->mbox_tx,&vuart_msg,sizeof(struct vuart_hb_msg));
}

void kcs_h2b_send_ibf(const struct host_bmc_msg_exch *exch)
{
	enum kcs_hb_msg_type kcs_msg = KCS_IBF_EVENT;
	espi_lpc_mbox_msg_send(&exch->mbox_tx,&kcs_msg,sizeof(kcs_msg));
}

void kcs_b2h_send_obf(const struct host_bmc_msg_exch *exch)
{
	enum kcs_hb_msg_type kcs_msg = KCS_OBF_EVENT;
	espi_lpc_mbox_msg_send(&exch->mbox_tx,&kcs_msg,sizeof(kcs_msg));
}

void espi_vwire_msg_send(const struct host_bmc_msg_exch *exch,uint8_t vw_idx)
{
	struct espi_vwire_msg vw_msg = {.vw_idx = vw_idx,};
	espi_lpc_mbox_msg_send(&exch->mbox_tx,&vw_msg,sizeof(vw_msg));
}

#else
void host_bmc_msg_exch_init(const struct host_bmc_msg_exch *exch)
{

}

void vuart_status_send(const struct host_bmc_msg_exch *exch,enum vuart_hb_msg_type vuart_msg_type)
{
	struct vuart_hb_msg vuart_msg = {
		.type = vuart_msg_type,
	};
	exch->peer_rx_callback(exch->peer,&vuart_msg);
}

void vuart_b2h_mode_set(const struct host_bmc_msg_exch *exch,bool host_rx_from_vuart,bool host_tx_to_vuart)
{
	struct vuart_hb_msg vuart_msg = {
		.type = VUART_MODE_SET,
		.host_rx_from_vuart = host_rx_from_vuart,
		.host_tx_to_vuart = host_tx_to_vuart,
	};
	exch->peer_rx_callback(exch->peer,&vuart_msg);
}

void kcs_h2b_send_ibf(const struct host_bmc_msg_exch *exch)
{
	enum kcs_hb_msg_type kcs_msg = KCS_IBF_EVENT;
	exch->peer_rx_callback(exch->peer,&kcs_msg);
}

void kcs_b2h_send_obf(const struct host_bmc_msg_exch *exch)
{
	enum kcs_hb_msg_type kcs_msg = KCS_OBF_EVENT;
	exch->peer_rx_callback(exch->peer,&kcs_msg);
}

void espi_vwire_msg_send(const struct host_bmc_msg_exch *exch,uint8_t vw_idx)
{
	struct espi_vwire_msg vw_msg = {.vw_idx = vw_idx,};
	exch->peer_rx_callback(exch->peer,&vw_msg);
}

#endif
