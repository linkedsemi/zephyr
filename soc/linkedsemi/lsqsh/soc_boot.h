
#ifndef _SOC_BOOT_H_
#define _SOC_BOOT_H_

#define TEST_WORD0 0xa5a53c3c
#define TEST_WORD1 0x5a5ac3c3

typedef struct image_header
{
    uint32_t test_word[2];
    uint32_t offset;
    uint32_t length;
    uint32_t version;
    uint8_t decrypt_key_id;
    uint8_t public_key_id;
    uint8_t key_derive_algo;
    uint8_t realtime_decrypt_lock;
    uint8_t iv[0x10];
    uint8_t encrypt_key[0x90];
    uint8_t sign[0x40];
    uint32_t exe_addr;
    uint32_t header_crc;
} image_header_t;

#endif /* _SOC_BOOT_H_ */
