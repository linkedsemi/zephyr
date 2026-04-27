#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <platform.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

static int pinctrl_pin_show(uint8_t pin)
{
    gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
    uint32_t mask = (1<<x->num<<16) | (1<<x->num);
    LOG_INF("%s  "
            "pin: %#x  "
            "%p: PU1_PU0: %#x  "
            "%p: PD_PU2: %#x  "
            "%p: IEN1_IEN0: %#x  "
            "%p: DS1_DS0: %#x  "
            "%p: AE_DS2: %#x  "
            "%p: OD_FIR: %#x  "
            "%p: SL_ST: %#x  "

            "%p: OE_DIN: %#x  "
            "%p: DOC_DOS: %#x  "

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
            "%p: FUNC_IO_LOCK: %#x  "
            "%p: IO_FUNC_LOCK: %#x  "
            "%p: SEC GPIO_INTR_MSK: %#x  "
            "%p: SEC GPIO_INTR_STT: %#x  "
            "%p: SEC GPIO_INTR_RAW: %#x  "
#endif
            "%p: APP GPIO_INTR_MSK: %#x  "
            "%p: APP GPIO_INTR_STT: %#x  "
            "%p: APP GPIO_INTR_RAW: %#x  "

            "%p: PINMUX_FUNC1 enable: %#x %p: FUNC: %#x  "
            "%p: PINMUX_FUNC2 enable: %#x  "
            "%p: PINMUX_FUNC3 enable: %#x  "
            "%p: PINMUX_FUNC4 enable: %#x  "
            ,

            __func__,
            pin,

            &APP_PMU->IO_CFG[x->port].PU1_PU0,
            APP_PMU->IO_CFG[x->port].PU1_PU0 & mask,
            &APP_PMU->IO_CFG[x->port].PD_PU2,
            APP_PMU->IO_CFG[x->port].PD_PU2 & mask,
            &APP_PMU->IO_CFG[x->port].IEN1_IEN0,
            APP_PMU->IO_CFG[x->port].IEN1_IEN0 & mask,
            &APP_PMU->IO_CFG[x->port].DS1_DS0,
            APP_PMU->IO_CFG[x->port].DS1_DS0 & mask,
            &APP_PMU->IO_CFG[x->port].AE_DS2,
            APP_PMU->IO_CFG[x->port].AE_DS2 & mask,
            &APP_PMU->IO_CFG[x->port].OD_FIR,
            APP_PMU->IO_CFG[x->port].OD_FIR & mask,
            &APP_PMU->IO_CFG[x->port].SL_ST,
            APP_PMU->IO_CFG[x->port].SL_ST & mask,

            &APP_PMU->IO_VAL[x->port].OE_DIN,
            APP_PMU->IO_VAL[x->port].OE_DIN & mask,
            &APP_PMU->IO_VAL[x->port].DOC_DOS,
            APP_PMU->IO_VAL[x->port].DOC_DOS & mask,

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
            &SYSC_SEC_AWO->FUNC_IO_LOCK[pin>>4],
            SYSC_SEC_AWO->FUNC_IO_LOCK[pin>>4] & BIT(x->num),
            &SYSC_SEC_PER->IO_FUNC_LOCK[pin>>4],
            SYSC_SEC_PER->IO_FUNC_LOCK[pin>>4] & BIT(x->num),

            &SEC_PMU->GPIO_INTR_MSK[x->port],
            SEC_PMU->GPIO_INTR_MSK[x->port] & mask,
            &SEC_PMU->GPIO_INTR_STT[x->port],
            SEC_PMU->GPIO_INTR_STT[x->port] & mask,
            &SEC_PMU->GPIO_INTR_RAW[x->port],
            SEC_PMU->GPIO_INTR_RAW[x->port] & mask,
#endif
            &APP_PMU->GPIO_INTR_MSK[x->port],
            APP_PMU->GPIO_INTR_MSK[x->port] & mask,
            &APP_PMU->GPIO_INTR_STT[x->port],
            APP_PMU->GPIO_INTR_STT[x->port] & mask,
            &APP_PMU->GPIO_INTR_RAW[x->port],
            APP_PMU->GPIO_INTR_RAW[x->port] & mask,

            &SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC1][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC1),
            &SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4],
            per_func0_alt_get(pin),
            &SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC2][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC2),
            &SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC3][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC3),
            &SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC4][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC4)
    );

    return 0;
}

static int pin2code(char port, uint8_t num)
{
    int ret = -EINVAL;
    char convert_tab[] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'm', 'n', 'q', 't', };
    if (((port >= 'a') && (port <= 'z')) || ((port >= 'A') && (port <= 'Z'))) {
        if (((port >= 'A') && (port <= 'Z'))) {
            port += 'a' - 'A';
        }
        for (int i = 0; i < ARRAY_SIZE(convert_tab); i++) {
            if (convert_tab[i] == port) {
                ret = 0;
                gpio_port_pin_t *x = (gpio_port_pin_t *)&ret;
                x->port = i;
                x->num = num;
            }
        }
    }
    return ret;
}

static int cmd_pinctrl_pin_show(const struct shell *sh, size_t argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        int err = 0;
        char port = argv[i][0];
        int num = shell_strtoul(&argv[i][1], 10, &err);
        if (err) {
            shell_print(sh, "Invalid Arguments: %s",  argv[i]);
        } else {
            pinctrl_pin_show(pin2code(port, num));
        }
    }

    return 0;
}

SHELL_CMD_REGISTER(pinctrl_pin_show, NULL, "pinctrl_pin_show A10 A11 A12 A13 ...", cmd_pinctrl_pin_show);
