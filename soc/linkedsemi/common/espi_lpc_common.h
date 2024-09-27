#ifndef ESPI_LPC_COMMON_H_
#define ESPI_LPC_COMMON_H_
#include <zephyr/sys/slist.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/spinlock.h>
#include "soc_clock.h"

struct peri_ioport_content {
    void (*io_read)(struct peri_ioport_content *ioport,uint8_t size,void *res);
    void (*io_write)(struct peri_ioport_content *ioport,uint8_t size,uint8_t *data);
    void *ctx;
    uint16_t addr;
};

struct peri_ioport {
    sys_snode_t node;
    struct peri_ioport_content *content;
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

struct espi_lpc_ls_config {
	void (*irq_config_func)(const struct device *);
    void *reg;
    void (*raise_edge_irq)(const struct device *,uint8_t);
    void (*set_level_irq)(const struct device *,uint8_t,uint8_t);
    const struct pinctrl_dev_config *pcfg;
    struct ls_clk_cfg cctl_cfg;
};

struct espi_lpc_ls_data {
    sys_slist_t peri_io;
    sys_slist_t peri_mem;
	sys_slist_t callbacks;
    union{
        struct {
            struct k_spinlock vw_tx_lock;
        }espi;
        struct {

        }lpc;
    }u;
};

void espi_lpc_raise_edge_irq(const struct device *dev,uint8_t idx);

void espi_lpc_set_level_irq(const struct device *dev,uint8_t idx,uint8_t active);

void espi_lpc_add_ioport(const struct device *dev,struct peri_ioport *ioport);

void espi_lpc_remove_ioport(const struct device *dev,struct peri_ioport *ioport);

void espi_lpc_add_mem(const struct device *dev,struct peri_mem *mem);

void espi_lpc_remove_mem(const struct device *dev,struct peri_mem *mem);

bool iord_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,void *res);

bool iowr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint16_t addr,uint8_t *data);

bool memwr_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,uint8_t *data);

bool memrd_short(struct espi_lpc_ls_data *espi_lpc,uint8_t size,uint32_t addr,void *res);

#endif