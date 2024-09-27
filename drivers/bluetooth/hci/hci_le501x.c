/*
 * Copyright (c) 2023 Linkedsemi.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <ls_hal_flash.h>

#include <zephyr/sys/util.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_hci_driver_le501x);

#define DT_DRV_COMPAT linkedsemi_bt_hci

/* Place random static address in 0x18025ff0(zephyr application starts at 0x18026000) */
#define BT_ADDR_OFFSET 0x25ff0
#define BT_ADDR_LEN 6

#define LL_THREAD_STACK_SIZE  0x400
#define LL_THREAD_PRIORITY 0

#define HCI_CMD                 0x01
#define HCI_ACL                 0x02
#define HCI_SCO                 0x03
#define HCI_EVT                 0x04
#define HCI_ISO                 0x05

static K_SEM_DEFINE(hci_send_sem, 0, 1);
static K_SEM_DEFINE(ll_thread_sem, 0, 1);
K_THREAD_STACK_DEFINE(rx_thread_stack, LL_THREAD_STACK_SIZE);
struct k_thread rx_thread_data;

uint32_t get_trng_value(void);
int cmd_handle(const struct device *dev, struct net_buf *buf);
void (*ll_hci_write_callback)(void *,uint8_t);
void *ll_hci_write_param;
void (*ll_hci_read_callback)(void *,uint8_t);
void *ll_hci_read_param;
uint8_t *ll_hci_read_buf;
uint16_t ll_hci_read_size;

struct net_buf *host_send_buf;
uint8_t *host_send_buf_current_ptr;
uint16_t host_send_buf_valid_len;
bool ll_hci_done_read;

struct hci_data {
	bt_hci_recv_t recv;
};

void ble_ll_task_event_set()
{
    k_sem_give(&ll_thread_sem);
}

void ble_sched();
static void rx_thread(void *p1,void *p2,void *p3)
{
    while(1)
    {
        k_sem_take(&ll_thread_sem,K_FOREVER);
        ble_sched();
    }
}

void aos_swint_set();
static void hci_copy_from_send_buf()
{
    if(ll_hci_read_size == 0) return;
    if(host_send_buf_valid_len == 0) 
    {
        if(host_send_buf)
        {
            net_buf_unref(host_send_buf);
        }
        k_sem_give(&hci_send_sem);
        return;
    }
    memcpy(ll_hci_read_buf,host_send_buf_current_ptr,ll_hci_read_size);
    host_send_buf_current_ptr += ll_hci_read_size;
    host_send_buf_valid_len -= ll_hci_read_size;
    ll_hci_read_size = 0;
    ll_hci_done_read = true;
    aos_swint_set();
}

void ll_hci_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *,uint8_t), void* dummy)
{
    ll_hci_read_buf = bufptr;
    ll_hci_read_size = size;
    ll_hci_read_callback = callback;
    ll_hci_read_param = dummy;
    hci_copy_from_send_buf();
}

void ll_hci_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *,uint8_t), void* dummy)
{
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));
    struct hci_data *hci = dev->data;
    ll_hci_write_callback = callback;
    ll_hci_write_param = dummy;
	uint8_t pkt_indicator = bufptr[0];
    bufptr += 1;
    size -= 1;
    struct net_buf *buf = NULL;
    switch(pkt_indicator)
    {
    case HCI_EVT:
    {
	    struct bt_hci_evt_hdr hdr;
        memcpy((void *)&hdr, bufptr, sizeof(hdr));
        bufptr += sizeof(hdr);
        size -= sizeof(hdr);
        __ASSERT_NO_MSG(hdr.len == size);
        buf = bt_buf_get_evt(hdr.evt,false,K_NO_WAIT);
        __ASSERT_NO_MSG(buf);
        net_buf_add_mem(buf,&hdr,sizeof(hdr));
    }
    break;
    case HCI_ACL:
    {
        struct bt_hci_acl_hdr hdr;
		memcpy((void *)&hdr, bufptr, sizeof(hdr));
		bufptr += sizeof(hdr);
		size -= sizeof(hdr);
        __ASSERT_NO_MSG(hdr.len == size);
	    buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_NO_WAIT);
        __ASSERT_NO_MSG(buf);
		net_buf_add_mem(buf, &hdr, sizeof(hdr));
    }
    break;
    default:
    	LOG_ERR("Unknown HCI type %u", pkt_indicator);
        while(1);
    }
    size_t buf_tailroom = net_buf_tailroom(buf);
    (void)(buf_tailroom);
    __ASSERT_NO_MSG(buf_tailroom>=size);
    net_buf_add_mem(buf,bufptr,size);
    /* Provide the buffer to the host */
	hci->recv(dev, buf);
    ll_hci_done_read = false;
    aos_swint_set();
}

void ll_hci_flow_on()
{

}

bool ll_hci_flow_off()
{
    return true;
}

static int bt_le501x_send(const struct device *dev, struct net_buf *buf)
{
    uint8_t pkt_indicator;
    struct bt_hci_cmd_hdr *chdr = (void *)buf->data;
    uint16_t _opcode = sys_le16_to_cpu(chdr->opcode);

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);
	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		pkt_indicator = HCI_ACL;
		break;
	case BT_BUF_CMD:
		pkt_indicator = HCI_CMD;
        if (BT_OGF(_opcode) == BT_OGF_VS) {
            cmd_handle(dev, buf);
            net_buf_unref(buf);
            return 0;
        }
		break;
	case BT_BUF_ISO_OUT:
		pkt_indicator = HCI_ISO;
		break;
	default:
		LOG_ERR("Unknown type %u", bt_buf_get_type(buf));
		while(1);
	}
    net_buf_push_u8(buf,pkt_indicator);
	LOG_HEXDUMP_DBG(buf->data, buf->len, "Final HCI buffer:");
	if (k_sem_take(&hci_send_sem, K_FOREVER) == 0) {
        host_send_buf = buf;
        host_send_buf_current_ptr = buf->data;
        host_send_buf_valid_len = buf->len;
        hci_copy_from_send_buf();
	} else {
		LOG_ERR("Send packet timeout error");
		while(1);
	}
    return 0;
}

static void generate_random_static_bt_addr(uint8_t * bt_addr) {
    srand(get_trng_value());

    for (uint8_t i = 0; i < BT_ADDR_LEN; i++) {
        bt_addr[i] = rand() & 0xFF;
    }

    bt_addr[BT_ADDR_LEN - 1] |= 0xC0;
}

static void le501x_bt_addr_init(void)
{
    uint8_t bt_addr[BT_ADDR_LEN], default_random_addr[BT_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    hal_flash_quad_io_read(BT_ADDR_OFFSET, bt_addr, BT_ADDR_LEN);
    if (memcmp(default_random_addr, bt_addr, BT_ADDR_LEN) == 0) {
        generate_random_static_bt_addr(bt_addr);
        hal_flash_page_program(BT_ADDR_OFFSET, bt_addr, BT_ADDR_LEN);
    }
}

static int bt_le501x_open(const struct device *dev, bt_hci_recv_t recv)
{
    le501x_bt_addr_init();

    struct hci_data *hci = dev->data;
	k_tid_t tid;

	tid = k_thread_create(&rx_thread_data, rx_thread_stack,
			      K_KERNEL_STACK_SIZEOF(rx_thread_stack),
			      rx_thread, (void *)dev, NULL, NULL,
			      K_PRIO_COOP(LL_THREAD_PRIORITY),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "bt_rx_thread");

	hci->recv = recv;

	return 0;
}

static int bt_le501x_close(const struct device *dev)
{
    struct hci_data *hci = dev->data;

	hci->recv = NULL;

    return 0;
}

static const struct bt_hci_driver_api drv = {
	.open	= bt_le501x_open,
	.close	= bt_le501x_close,
	.send	= bt_le501x_send,
};

static void swint_handler()
{
    if(ll_hci_done_read)
    {
        ll_hci_read_callback(ll_hci_read_param,0);
    }else
    {
        ll_hci_write_callback(ll_hci_write_param,0);
    }
    ble_ll_task_event_set();
    z_arm_int_exit();
}

void rco_freq_counting_init();
void rco_freq_counting_start();
uint16_t get_lsi_cnt_val();
void aos_swint_init(void (*isr)());

static int bt_le501x_init(const struct device *dev)
{
    rco_freq_counting_init();
    rco_freq_counting_start();
    while (get_lsi_cnt_val() == 0);
    aos_swint_init(swint_handler);

    return 0;
}

#define HCI_DEVICE_INIT(inst) \
	static struct hci_data hci_data_##inst = { \
	}; \
	DEVICE_DT_INST_DEFINE(inst, bt_le501x_init, NULL, &hci_data_##inst, NULL, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &drv)

/* Only one instance supported right now */
HCI_DEVICE_INIT(0)