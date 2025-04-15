#include <iostream>
#include <cstdio>
#include <cstdint>
#include <reg_base_addr.h>
#include <zephyr/kernel.h>

using namespace std;

int main() {
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    const uint32_t blk_sz = KB(100);
    const uint32_t blk_num = 80;

    for (uint32_t i = 0; i < 0xffffffff; i++) {
        do {
            volatile uint32_t *test_data = (volatile uint32_t *)malloc(blk_sz);
            if (NULL == test_data) {
                printf("heap pool empty\n");
                __ASSERT(i >= blk_num, "i: %d  blk_num: %d", i, blk_num);
                printf("test pass\n");
                return 0;
            }
#if 1
            else {
                printf("%p\n", test_data);
                for (uint32_t j = 0; j < 40 / sizeof(uint32_t); j++) {
                    test_data[j] = j;
                    printf("%d ", test_data[j]);
                }
                printf("\n");
            }
#endif
        } while (0);
    }

    return 0;
}
