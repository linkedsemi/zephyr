#include "espi_lpc_common.h"


void espi_lpc_raise_edge_irq(const struct device *dev,uint8_t idx)
{
    const struct espi_lpc_ls_config *cfg = dev->config;
    cfg->raise_edge_irq(dev,idx);
}

void espi_lpc_set_level_irq(const struct device *dev,uint8_t idx,uint8_t active)
{
    const struct espi_lpc_ls_config *cfg = dev->config;
    cfg->set_level_irq(dev,idx,active);
}

bool iord_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,void *res)
{
	struct sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi->peri_io,ptr){
		struct peri_ioport *io = SYS_SLIST_CONTAINER(ptr,struct peri_ioport,node);
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
	struct sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_io,ptr){
		struct peri_ioport *io = SYS_SLIST_CONTAINER(ptr,struct peri_ioport,node);
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
	struct sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi_lpc->peri_mem,ptr){
		struct peri_mem *mem = SYS_SLIST_CONTAINER(ptr,struct peri_mem,node);
		if(mem->content->mem_write(mem->content,addr,size,data))
		{
			return true;
		}
	}
	return false;
}

bool memrd_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,void *res)
{
	struct sys_snode_t *ptr;
	SYS_SLIST_FOR_EACH_NODE(&espi->peri_mem,ptr){
		struct peri_mem *mem = SYS_SLIST_CONTAINER(ptr,struct peri_mem,node);
		if(mem->content->mem_read(mem->content,addr,size,res))
		{
			return true;
		}
	}
	return false;
}

void espi_lpc_add_ioport(const struct device *dev,struct peri_ioport *ioport)
{
    const struct espi_lpc_ls_data *data = dev->data;
	sys_slist_append(&data->peri_io,&ioport->node);
}

void espi_lpc_add_mem(const struct device *dev,struct peri_mem *mem)
{
    const struct espi_lpc_ls_data *data = dev->data;
	sys_slist_append(&data->peri_mem,&mem->node);
}

void espi_lpc_remove_ioport(const struct device *dev,struct peri_ioport *ioport)
{
    const struct espi_lpc_ls_data *data = dev->data;
	sys_slist_find_and_remove(&data->peri_io,&ioport->node);
}

void espi_lpc_remove_mem(const struct device *dev,struct peri_mem *mem)
{
    const struct espi_lpc_ls_data *data = dev->data;
	sys_slist_find_and_remove(&data->peri_mem,&mem->node);
}
