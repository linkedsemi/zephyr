#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA_

#include <stdint.h>

#define SHA_START     0x0
#define SHA_CTRL      0x4
#define SHA_INTR_M    0x20
#define SHA_INTR_C    0x24
#define SHA_INTR_S    0x28
#define SHA_INTR_R    0x2c
#define SHA_FIFO_DAT  0x30
#define SHA_FIFO_STAT 0x34
#define SHA_FSM_STAT  0x38
#define SHA_RSLT0     0x40
#define SHA_RSLT1     0x44
#define SHA_RSLT2     0x48
#define SHA_RSLT3     0x4c
#define SHA_RSLT4     0x50
#define SHA_RSLT5     0x54
#define SHA_RSLT6     0x58
#define SHA_RSLT7     0x5c

union sha_reg_start {
    volatile uint32_t value;
    struct {
        volatile uint32_t FSM_START : 1,  /* [0] */
                          RESERVED0 : 31; /* [1-31] */
    } field;
} __attribute__((packed));

union sha_reg_ctrl {
    volatile uint32_t value;
    struct {
        volatile uint32_t FST_DAT     : 1,   /*[0]*/
                          RESERVED0   : 3,   /*[1-3]*/
                          CALC_SHA224 : 1,   /*[4]*/
                          CALC_SM3    : 1,   /*[5]*/
                          RESERVED1   : 2,   /*[6-7]*/
                          LEN         : 14,  /*[8-21]*/
                          RESERVED2   : 10;  /*[22-31]*/
    } field;
} __attribute__((packed));

union sha_reg_intr {
    volatile uint32_t value;
    struct {
        volatile uint32_t FSM_END  : 1,  /*[0]*/
                          FSM_EMPT : 1,  /*[1]*/
                          RESERVED0     : 30; /*[2-31]*/
    } field;
} __attribute__((packed));

union sha_reg_fifo_stat {
    volatile uint32_t value;
    struct {
        volatile uint32_t FIFO_FLVL : 4,  /*[0-3]*/
                          RESERVED0 : 28; /*[4-31]*/
    } field;
} __attribute__((packed));

union sha_reg_fsm_stat {
    volatile uint32_t value;
    struct {
        volatile uint32_t FSM_IDLE  : 1,  /*[0]*/
                          RESERVED0 : 31; /*[1-31]*/
    } field;
} __attribute__((packed));

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_SHA_ */
