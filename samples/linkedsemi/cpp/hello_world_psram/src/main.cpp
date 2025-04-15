#include <iostream>
#include <cstdio>
#include <cstdint>
#include <reg_base_addr.h>
#include <zephyr/kernel.h>

using namespace std;

int main() {
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

    uint32_t *arr = (uint32_t *)PSRAM_ADDR;
    uint32_t num = KB(50) / 4;

    for(uint32_t i = 0; i < num; i++) {
        arr[i] = i;
    }
    for(uint32_t r = 0; r < 3; r++) {
        printf("r: %d\n", r);
        for(uint32_t i = 0; i < num >> 3; i++) {
            uint32_t tmp = rand() % num;
            arr[tmp] = tmp;
            printf("arr[tmp]: %d  tmp: %d\n", arr[tmp], tmp);
        }
        uint32_t tmp = 10168;
        __ASSERT(arr[tmp] == tmp, "arr[i]: %d  i: %d", arr[tmp], tmp);
        for(uint32_t k = 0; k < 10; k++) {
            for(uint32_t i = 0; i < num >> 3; i++) {
                uint32_t tmp = rand() % num;
                __ASSERT(arr[tmp] == tmp, "arr[i]: %d  i: %d", arr[tmp], tmp);
            }
        }
    }
    printf("test pass\n");

    return 0;
}
