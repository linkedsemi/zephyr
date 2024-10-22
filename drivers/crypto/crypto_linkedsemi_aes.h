#ifndef ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_AES_
#define ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_AES_

#include <stdint.h>

#define CRYPT_DATA0 0x0
#define CRYPT_DATA1 0x4
#define CRYPT_DATA2 0x8
#define CRYPT_DATA3 0xc
#define CRYPT_KEY0 0x10
#define CRYPT_KEY1 0x14
#define CRYPT_KEY2 0x18
#define CRYPT_KEY3 0x1c
#define CRYPT_KEY4 0x20
#define CRYPT_KEY5 0x24
#define CRYPT_KEY6 0x28
#define CRYPT_KEY7 0x2c
#define CRYPT_IVR0 0x30
#define CRYPT_IVR1 0x34
#define CRYPT_IVR2 0x38
#define CRYPT_IVR3 0x3c
#define CRYPT_RES0 0x40
#define CRYPT_RES1 0x44
#define CRYPT_RES2 0x48
#define CRYPT_RES3 0x4c
#define CRYPT_CR   0x50
#define CRYPT_SR   0x54
#define CRYPT_ICFR 0x58
#define CRYPT_FIFO 0x5c

union aes_reg_cr {
    volatile uint32_t value;
    struct {
        volatile uint32_t GO: 1,         /*[0]    */
                          ENCS: 1,       /*[1]    */
                          AESKS: 2,      /*[2-3]  */
                          MODE: 2,       /*[4-5]  */
                          IVREN: 1,      /*[6]    */
                          IE: 1,         /*[7]    */
                          TYPE: 2,       /*[8-9]  */
                          TDES: 1,       /*[10]   */
                          DESKS: 1,      /*[11]   */
                          FIFOEN: 1,     /*[12]   */
                          FIFOODR: 1,    /*[13]   */
                          DMAEN: 1,      /*[14]   */
                          RESERVED0: 16, /*[15-30]*/
                          CRYSEL: 1;     /*[31]   */
    } field;
} __attribute__((packed));

union aes_reg_sr {
    volatile uint32_t value;
    struct {
        volatile uint32_t AESRIF: 1,       /*[0]*/
                          DESRIF: 1,       /*[1]*/
                          RESERVED0 : 6,   /*[2-7]*/
                          DONE: 1,         /*[8]*/
                          RESERVED1 : 23;  /*[9-31]*/
    } field;
} __attribute__((packed));

union aes_reg_icfr {
    volatile uint32_t value;
    struct {
    volatile uint32_t AESIF: 1,       /*[0]*/
                      DESIF: 1,       /*[1]*/
                      MULIF: 1,       /*[2]*/
                      RESERVED0 : 29; /*[3-31]*/
    } field;
} __attribute__((packed));

#endif /* ZEPHYR_DRIVERS_CRYPTO_CRYPTO_LINKEDSEMIH_AES_ */
