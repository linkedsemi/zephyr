#ifndef REG_ETH_TYPE_H_
#define REG_ETH_TYPE_H_
#include <stdint.h>

typedef struct
{
    volatile uint32_t MAC_CONFIGURATION; //0x0
    volatile uint32_t MAC_EXT_CONFIGURATION; //0x4
    volatile uint32_t MAC_PACKET_FILTER; //0x8
    volatile uint32_t MAC_WATCHDOG_TIMEOUT; //0xc
    volatile uint32_t RESERVED0[16];
    volatile uint32_t MAC_VLAN_TAG; //0x50
    volatile uint32_t RESERVED1[7];
    volatile uint32_t MAC_Q0_TX_FLOW_CTRL; //0x70
    volatile uint32_t RESERVED2[7];
    volatile uint32_t MAC_RX_FLOW_CTRL; //0x90
    volatile uint32_t RESERVED3[3];
    volatile uint32_t MAC_RXQ_CTRL0; //0xa0
    volatile uint32_t MAC_RXQ_CTRL1; //0xa4
    volatile uint32_t MAC_RXQ_CTRL2; //0xa8
    volatile uint32_t MAC_RXQ_CTRL3; //0xac
    volatile uint32_t MAC_INTERRUPT_STATUS; //0xb0
    volatile uint32_t MAC_INTERRUPT_ENABLE; //0xb4
    volatile uint32_t MAC_RX_TX_STATUS; //0xb8
    volatile uint32_t RESERVED4[15];
    volatile uint32_t MAC_PHYIF_CONTROL_STATUS; //0xf8
    volatile uint32_t RESERVED5[5];
    volatile uint32_t MAC_VERSION; //0x110
    volatile uint32_t MAC_DEBUG; //0x114
    volatile uint32_t RESERVED6[1];
    volatile uint32_t MAC_HW_FEATURE0; //0x11c
    volatile uint32_t MAC_HW_FEATURE1; //0x120
    volatile uint32_t MAC_HW_FEATURE2; //0x124
    volatile uint32_t MAC_HW_FEATURE3; //0x128
    volatile uint32_t RESERVED7[53];
    volatile uint32_t MAC_MDIO_ADDRESS; //0x200
    volatile uint32_t MAC_MDIO_DATA; //0x204
    volatile uint32_t RESERVED8[10];
    volatile uint32_t MAC_CSR_SW_CTRL; //0x230
    volatile uint32_t RESERVED9[51];
    volatile uint32_t MAC_ADDRESS0_HIGH; //0x300    
    volatile uint32_t MAC_ADDRESS0_LOW;  //0x304
    volatile uint32_t MAC_ADDRESS1_HIGH; //0x308    
    volatile uint32_t MAC_ADDRESS1_LOW;  //0x30c
    volatile uint32_t MAC_ADDRESS2_HIGH; //0x310    
    volatile uint32_t MAC_ADDRESS2_LOW;  //0x314
    volatile uint32_t RESERVED10[570];
    volatile uint32_t MTL_OPERATION_MODE; //0xc00
    volatile uint32_t RESERVED11[7];
    volatile uint32_t MTL_INTERRUPT_STATUS; //0xc20
    volatile uint32_t RESERVED12[3];
    volatile uint32_t MTL_RXQ_DMA_MAP0; //0xc30
    volatile uint32_t MTL_RXQ_DMA_MAP1; //0xc34
    volatile uint32_t RESERVED13[50];
    volatile uint32_t MTL_TXQ0_OPERATION_MODE; //0xd00
    volatile uint32_t MTL_TXQ0_UNDERFLOW; //0xd04
    volatile uint32_t MTL_TXQ0_DEBUG; //0xd08
    volatile uint32_t RESERVED14[8];
    volatile uint32_t MTL_Q0_INTERRUPT_CONTROL_STATUS; //0xd2c
    volatile uint32_t MTL_RXQ0_OPERATION_MODE; //0xd30
    volatile uint32_t MTL_RXQ0_MISSED_PACKET_OVERFLOW_CNT; //0xd34
    volatile uint32_t MTL_RXQ0_DEBUG; //0xd38
    volatile uint32_t MTL_RXQ0_CONTROL; //0xd3c
    volatile uint32_t RESERVED15[176];
    volatile uint32_t DMA_MODE; //0x1000
    volatile uint32_t DMA_SYSBUS_MODE; //0x1004
    volatile uint32_t DMA_INTERRUPT_STATUS; //0x1008
    volatile uint32_t DMA_DEBUG_STATUS0; //0x100c
    volatile uint32_t RESERVED16[60];
    volatile uint32_t DMA_CH0_CONTROL; //0x1100
    volatile uint32_t DMA_CH0_TX_CONTROL; //0x1104
    volatile uint32_t DMA_CH0_RX_CONTROL; //0x1108
    volatile uint32_t RESERVED17[2];
    volatile uint32_t DMA_CH0_TXDESC_LIST_ADDRESS; //0x1114
    volatile uint32_t RESERVED18[1];
    volatile uint32_t DMA_CH0_RXDESC_LIST_ADDRESS; //0x111c
    volatile uint32_t DMA_CH0_TXDESC_TAIL_POINTER; //0x1120
    volatile uint32_t RESERVED19[1];
    volatile uint32_t DMA_CH0_RXDESC_TAIL_POINTER; //0x1128
    volatile uint32_t DMA_CH0_TXDESC_RING_LENGTH; //0x112c
    volatile uint32_t DMA_CH0_RX_CONTROL2; //0x1130
    volatile uint32_t DMA_CH0_INTERRUPT_ENABLE; //0x1134
    volatile uint32_t DMA_CH0_RX_INTERRUPT_WATCHDOG_TIMER; //0x1138
    volatile uint32_t RESERVED20[2];
    volatile uint32_t DMA_CH0_CURRENT_APP_TXDESC; //0x1144
    volatile uint32_t RESERVED21[1];
    volatile uint32_t DMA_CH0_CURRENT_APP_RXDESC; //0x114c
    volatile uint32_t RESERVED22[1];
    volatile uint32_t DMA_CH0_CURRENT_APP_TXBUFFER; //0x1154
    volatile uint32_t RESERVED23[1];
    volatile uint32_t DMA_CH0_CURRENT_APP_RXBUFFER; //0x115c
    volatile uint32_t DMA_CH0_STATUS; //0x1160
    volatile uint32_t DMA_CH0_MISS_FRAME_CNT; //0x1164
} reg_eth_t;

enum EQOS_DMA_CH0_REG_DMA_CH0_STATUS_FIELD
{
    EQOS_DMA_CH0_REB_MASK = (int)0x380000,
    EQOS_DMA_CH0_REB_POS = 19,
    EQOS_DMA_CH0_TEB_MASK = (int)0x70000,
    EQOS_DMA_CH0_TEB_POS = 16,
    EQOS_DMA_CH0_NIS_MASK = (int)0x8000,
    EQOS_DMA_CH0_NIS_POS = 15,
    EQOS_DMA_CH0_AIS_MASK = (int)0x4000,
    EQOS_DMA_CH0_AIS_POS = 14,
    EQOS_DMA_CH0_CDE_MASK = (int)0x2000,
    EQOS_DMA_CH0_CDE_POS = 13,
    EQOS_DMA_CH0_FBE_MASK = (int)0x1000,
    EQOS_DMA_CH0_FBE_POS = 12,
    EQOS_DMA_CH0_ERI_MASK = (int)0x800,
    EQOS_DMA_CH0_ERI_POS = 11,
    EQOS_DMA_CH0_ETI_MASK = (int)0x400,
    EQOS_DMA_CH0_ETI_POS = 10,
    EQOS_DMA_CH0_RWT_MASK = (int)0x200,
    EQOS_DMA_CH0_RWT_POS = 9,
    EQOS_DMA_CH0_RPS_MASK = (int)0x100,
    EQOS_DMA_CH0_RPS_POS = 8,
    EQOS_DMA_CH0_RBU_MASK = (int)0x80,
    EQOS_DMA_CH0_RBU_POS = 7,
    EQOS_DMA_CH0_RI_MASK = (int)0x40,
    EQOS_DMA_CH0_RI_POS = 6,
    EQOS_DMA_CH0_TBU_MASK = (int)0x4,
    EQOS_DMA_CH0_TBU_POS = 2,
    EQOS_DMA_CH0_TPS_MASK = (int)0x2,
    EQOS_DMA_CH0_TPS_POS = 1,
    EQOS_DMA_CH0_TI_MASK = (int)0x1,
    EQOS_DMA_CH0_TI_POS = 0,
};

#endif 
