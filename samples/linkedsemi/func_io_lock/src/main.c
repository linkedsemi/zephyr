/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include "ls_soc_gpio.h"
#include "reg_sysc_sec_awo.h"
#include "reg_sysc_app_awo.h"
#include "reg_sysc_sec_per.h"
#include "reg_sysc_app_per.h"
#include "per_func_mux.h"

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    for(uint32_t i = PA00; i <= PT00; i++) {
        for(uint32_t func_num = PINMUX_FUNC_START; i <= PINMUX_FUNC_END; i++) {
            gpio_port_pin_t *x = (gpio_port_pin_t *)&i;
            /* save */
            uint32_t stat = SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1];
            /* unlock && clear */
            SYSC_SEC_AWO->FUNC_IO_LOCK[i / 16] &= ~(1 << (x->num));
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] &= ~(1 << (((x->port % 2) * 16)+ x->num));

            /* lock && set */
            SYSC_SEC_AWO->FUNC_IO_LOCK[i / 16] = 1 << (x->num);
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = 1 << (((x->port % 2) * 16)+ x->num);
            if (SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] & (1 << (((x->port % 2) * 16)+ x->num))) {
                __ASSERT(0, "lock test fail");
            }

            /* unlock && set */
            SYSC_SEC_AWO->FUNC_IO_LOCK[i / 16] &= ~(1 << (x->num));
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = 1 << (((x->port % 2) * 16)+ x->num);
            if (!(SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] & (1 << (((x->port % 2) * 16)+ x->num)))) {
                __ASSERT(0, "unlock test fail");
            }
            /* reload */
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = stat;
        }
    }

    for(uint32_t i = PA00; i <= PT00; i++) {
        for(uint32_t func_num = PINMUX_FUNC_START; i <= PINMUX_FUNC_END; i++) {
            gpio_port_pin_t *x = (gpio_port_pin_t *)&i;
            /* save */
            uint32_t stat = SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1];
            uint32_t stat_per = SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4];

            uint8_t peek_val;
            uint8_t test_val;
            do {
                test_val = rand() % 32;
            } while ((test_val == 0) || (test_val == stat_per));

            /* unlock && clear */
            SYSC_SEC_AWO->FUNC_IO_LOCK[i / 16] &= ~(1 << (x->num));
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] &= ~(1 << (((x->port % 2) * 16)+ x->num));

            /* unlock && clear */
            SYSC_SEC_PER->IO_FUNC_LOCK[i / 16] &= ~(1 << (x->num));
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] &= ~(0x3f << ((x->num % 4) * 8));

            /* lock && set */
            SYSC_SEC_PER->IO_FUNC_LOCK[i / 16] = 1 << (x->num);
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] |= test_val << ((x->num % 4) * 8);
            peek_val = (SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] >> ((x->num % 4) * 8)) & 0x3f;
            if (peek_val != 0) {
                __ASSERT(0, "unlock test fail. expect: %#x  but got: %#x", 0, peek_val);
            }

            /* unlock && set */
            SYSC_SEC_PER->IO_FUNC_LOCK[i / 16] &= ~(1 << (x->num));
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] |= test_val << ((x->num % 4) * 8);
            peek_val = (SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] >> ((x->num % 4) * 8)) & 0x3f;
            if (peek_val != test_val) {
                __ASSERT(0, "unlock test fail. expect: %#x  but got: %#x", test_val, peek_val);
            }

            /* reload */
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = stat;
        }
    }
    printf("pass\n");

    return 0;
}
