#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "eth.h"
#include "reg_sysc_cpu.h"
#include "reg_sysc_awo.h"
#include "ls_soc_gpio.h"

#define ETH                   ((reg_eth_t *)(0x4001c000))
#define DDR4_OFFSET           0x00000
#define DDR4_BUFFER_BASE_VIRT (0x10080000 + DDR4_OFFSET)
#define DDR4_BUFFER_BASE_PHY  (0x10080000 + DDR4_OFFSET)
int loop_cnt;
unsigned int case_idx = 0;
int txdesc = 0;
char prnt_info_0[100];

/* 192.168.1.123 send "hello world" to 192.168.1.255 */
uint8_t frame[0x40] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                        0xff, 0xff, 0xff, 0xff, 0x8, 0x0, 0x45, 0x0,
                        0x0, 0x27, 0x60, 0xb, 0x40, 0x0, 0x80, 0x11,
                        0x15, 0x6c, 0xc0, 0xa8, 0x1, 0xff, 0xc0,
                        0xa8, 0x1, 0xff, 0x1f, 0x9a, 0x1f, 0x90,
                        0x0, 0x13, 0xa9, 0x80, 0x68, 0x65, 0x6c,
                        0x6c, 0x6f, 0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64,
};

typedef struct
{
    volatile uint8_t tx_mem_buff[64]; //0x0
} reg_sram_mem_t;

typedef struct
{
    volatile uint8_t rx_mem_buff[64]; //0x0
} reg_sram_mem1_t;

void c_delay(int s)
{
    for (; s >= 0; s--) {
        ;
    }
}

void init(void)
{
    /* pinmux */
    io_cfg_input(PT01);
    io_cfg_input(PT08);
    io_cfg_input(PT09);
    io_cfg_input(PT10);
    io_cfg_input(PT11);
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0xbc) = 0x2f3b;
    /* clock */
    *(volatile uint32_t *)(QSH_SYSC_AWO_ADDR + 0x58) = 0x206C80;

    memset((void *)(DDR4_BUFFER_BASE_VIRT + 0x0), 0, 0x2000);
    memcpy((void *)(DDR4_BUFFER_BASE_VIRT + 0x1000), frame, sizeof(frame));
    //tx disc
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x00000000) = DDR4_BUFFER_BASE_PHY + 0x00001000;
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x00000004) = DDR4_BUFFER_BASE_PHY + 0x00001300;
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x00000008) = 0x00000040;
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x0000000c) = 0xb0000040;
    //rx_disc
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x00000100) = DDR4_BUFFER_BASE_PHY + 0x00001100;
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x0000010c) = 0xc1000000;
    *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x0000013c) = 0x41000000;
}

int main(void)
{
    printf("version: %#x\n", *(volatile uint32_t *)0x4001F3FC);
    init(); //enable io function and clk

    ETH->DMA_MODE = 0x01; //D000
    c_delay(10);
    while (ETH->DMA_MODE)
        ;

    ETH->MAC_EXT_CONFIGURATION = 0x1008;
    ETH->DMA_CH0_TXDESC_RING_LENGTH = 0x04;
    ETH->DMA_CH0_RX_CONTROL2 = 0x04;
    ETH->DMA_CH0_TXDESC_LIST_ADDRESS = DDR4_BUFFER_BASE_PHY + 0x00000000;
    ETH->DMA_CH0_RXDESC_LIST_ADDRESS = DDR4_BUFFER_BASE_PHY + 0x00000100;
    ETH->DMA_CH0_TXDESC_TAIL_POINTER = 0x10;
    ETH->DMA_CH0_RXDESC_TAIL_POINTER = 0x10;
    ETH->DMA_CH0_INTERRUPT_ENABLE = 0xFFFF;
    ETH->DMA_CH0_RX_CONTROL = 0X01;
    ETH->DMA_CH0_TX_CONTROL = 0X01;
    ETH->MAC_INTERRUPT_ENABLE = 0X7FFFF;
#if 0
    // ETH->MAC_CONFIGURATION = 0xb003; //loopback
#else
    // ETH->MAC_CONFIGURATION = 0xa003;
    ETH->MAC_CONFIGURATION = 0xe003;
#endif
    c_delay(60);

    printf("end\n");
    for (volatile uint32_t delay = 0; delay < 0xfffff; delay++) {
        ;
    }
    printf("dma ch0 status %#x\n", ETH->DMA_CH0_STATUS);

    printf("memcpy:\n");
    for (volatile uint8_t i = 0; i < 0x40; i += 4) {
        printf("%#8.8x  %#8.8x  %#8.8x  %#8.8x  %#8.8x\n",
               *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x1000 + i),
               *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x1100 + i),
               *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x1900 + i),
               *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x1a00 + i),
               *(volatile uint32_t *)(DDR4_BUFFER_BASE_VIRT + 0x1b00 + i));
    }
    while (1)
        ;
    return (0);
}
