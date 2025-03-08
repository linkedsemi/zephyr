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

#if defined(CONFIG_GPIO)
#error CONFIG_GPIO=n
#endif

// lock:
//     func
//     config(except ien)
//     din
//     intr clr

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    for (uint32_t pin = PA00; pin <= PT00; pin++) {
        for (uint32_t func_num = PINMUX_FUNC_START; pin <= PINMUX_FUNC_END; pin++) {
            gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
            /* save */
            uint32_t stat = SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1];
            /* unlock && clear */
            io_func_cfg_lock(pin, false);
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] &= ~(1 << (((x->port % 2) * 16) + x->num));

            /* lock && set */
            io_func_cfg_lock(pin, true);
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = 1 << (((x->port % 2) * 16) + x->num);
            if (SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] & (1 << (((x->port % 2) * 16) + x->num))) {
                __ASSERT(0, "lock test fail");
            }

            /* unlock && set */
            io_func_cfg_lock(pin, false);
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = 1 << (((x->port % 2) * 16) + x->num);
            if (!(SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] & (1 << (((x->port % 2) * 16) + x->num)))) {
                __ASSERT(0, "unlock test fail");
            }
            /* reload */
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = stat;
        }
    }

    for (uint32_t pin = PA00; pin <= PT00; pin++) {
        for (uint32_t func_num = PINMUX_FUNC_START; pin <= PINMUX_FUNC_END; pin++) {
            gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
            /* save */
            uint32_t stat = SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1];
            uint32_t stat_per = SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4];

            uint8_t peek_val;
            uint8_t test_val;
            do {
                test_val = rand() % 32;
            } while ((test_val == 0) || (test_val == stat_per));

            /* unlock && clear */
            io_func_cfg_lock(pin, false);
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] &= ~(1 << (((x->port % 2) * 16) + x->num));

            /* unlock && clear */
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] &= ~(0x3f << ((x->num % 4) * 8));

            /* lock && set */
            io_func_cfg_lock(pin, true);
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] |= test_val << ((x->num % 4) * 8);
            peek_val = (SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] >> ((x->num % 4) * 8)) & 0x3f;
            if (peek_val != 0) {
                __ASSERT(0, "unlock test fail. expect: %#x  but got: %#x", 0, peek_val);
            }

            /* unlock && set */
            io_func_cfg_lock(pin, false);
            SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] |= test_val << ((x->num % 4) * 8);
            peek_val = (SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4] >> ((x->num % 4) * 8)) & 0x3f;
            if (peek_val != test_val) {
                __ASSERT(0, "unlock test fail. expect: %#x  but got: %#x", test_val, peek_val);
            }

            /* reload */
            SYSC_APP_AWO->IO_FUNC[func_num][x->port >> 1] = stat;
        }
    }

    for (uint32_t pin = PA00; pin <= PT00; pin++) {
        // unlock
        io_cfg_lock(pin, false);
        // set
        io_cfg_output(pin);
        // check pass
        __ASSERT_NO_MSG(io_is_output(pin));

        //clear
        io_cfg_disable(pin);

        // lock
        io_cfg_lock(pin, true);
        // set
        io_cfg_output(pin);
        // check fail
        __ASSERT_NO_MSG(!io_is_output(pin));

        break; //do not test all usless safe
    }

    for (uint32_t pin = PA00; pin <= PT00; pin++) {
        // unlock
        io_cfg_lock(pin, false);
        // set
        io_cfg_input(pin);
        // check pass
        __ASSERT_NO_MSG(io_is_input(pin));

        //clear
        io_cfg_disable(pin);

        // lock
        io_cfg_lock(pin, true);
        // set
        io_cfg_input(pin);
        // check fail
        __ASSERT_NO_MSG(!io_is_input(pin));
    }

    for (uint32_t pin = PA00; pin <= PT00; pin++) {
        io_cfg_lock(pin, false);
        io_cfg_input(pin);
        io_cfg_output(pin);

        // unlock
        io_cfg_app_input_lock(pin, false);

        // set
        io_set_pin(pin);
        // check pass
        __ASSERT_NO_MSG(io_sec_get_input_val(pin) == 1);
        __ASSERT_NO_MSG(io_app_get_input_val(pin) == 1);

        // set
        io_clr_pin(pin);
        // check pass
        __ASSERT_NO_MSG(io_sec_get_input_val(pin) == 0);
        __ASSERT_NO_MSG(io_app_get_input_val(pin) == 0);

        // lock
        io_cfg_app_input_lock(pin, true);

        // set
        io_set_pin(pin);
        // check fail
        __ASSERT_NO_MSG(io_sec_get_input_val(pin) == 1);
        __ASSERT_NO_MSG(io_app_get_input_val(pin) == 0);

        // set
        io_clr_pin(pin);
        // check
        __ASSERT_NO_MSG(io_sec_get_input_val(pin) == 0);
        __ASSERT_NO_MSG(io_app_get_input_val(pin) == 0);

        break; //do not test all usless safe
    }

    do {
        io_cfg_lock(PT15, false);
        io_cfg_lock(PK13, false);
        io_func_cfg_lock(PT15, false);
        io_func_cfg_lock(PK13, false);

        // unlock
        io_exti_clr_cfg_lock(PK13, INT_EDGE_FALLING, false);

        io_cfg_output(PT15);
        io_cfg_input(PK13);
        io_set_pin(PT15);
        __ASSERT(1 == io_sec_get_input_val(PK13), "please connect PT15 && PK13");
        io_clr_pin(PT15);
        __ASSERT(0 == io_sec_get_input_val(PK13), "please connect PT15 && PK13");

        io_sec_exti_config(PK13, INT_EDGE_FALLING);

        // set
        io_set_pin(PT15);
        io_clr_pin(PT15);

        // check pass
        __ASSERT_NO_MSG(true == io_sec_get_exti_status(PK13, INT_EDGE_FALLING));
        io_clr_exti(PK13, INT_EDGE_FALLING);
        __ASSERT_NO_MSG(false == io_sec_get_exti_status(PK13, INT_EDGE_FALLING));

        // lock
        io_exti_clr_cfg_lock(PK13, INT_EDGE_FALLING, true);

        // set
        io_set_pin(PT15);
        io_clr_pin(PT15);

        __ASSERT_NO_MSG(true == io_sec_get_exti_status(PK13, INT_EDGE_FALLING));
        // clr intr
        io_clr_exti(PK13, INT_EDGE_FALLING);
        // check fail
        __ASSERT_NO_MSG(true == io_sec_get_exti_status(PK13, INT_EDGE_FALLING));

        // unlock
        io_exti_clr_cfg_lock(PK13, INT_EDGE_FALLING, false);

        // clr intr
        io_clr_exti(PK13, INT_EDGE_FALLING);
        // check pass
        __ASSERT_NO_MSG(false == io_sec_get_exti_status(PK13, INT_EDGE_FALLING));

        break; //do not test all usless safe
    } while(0);

    printf("pass\n");

    return 0;
}
