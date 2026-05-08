#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <platform.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

static int pinmux_show(uint8_t pin)
{
    gpio_port_pin_t *x = (gpio_port_pin_t *)&pin;
    uint32_t mask = (1<<x->num<<16) | (1<<x->num);
    LOG_INF("%s  "
            "pin: %#x  "
            "%#lx: PU1_PU0: %#x  "
            "%#lx: PD_PU2: %#x  "
            "%#lx: IEN1_IEN0: %#x  "
            "%#lx: DS1_DS0: %#x  "
            "%#lx: AE_DS2: %#x  "
            "%#lx: OD_FIR: %#x  "
            "%#lx: SL_ST: %#x  "

            "%#lx: OE_DIN: %#x  "
            "%#lx: DOC_DOS: %#x  "

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
            "%#lx: FUNC_IO_LOCK: %#lx  "
            "%#lx: IO_FUNC_LOCK: %#lx  "
            "%#lx: SEC GPIO_INTR_MSK: %#x  "
            "%#lx: SEC GPIO_INTR_STT: %#x  "
            "%#lx: SEC GPIO_INTR_RAW: %#x  "
#endif
            "%#lx: APP GPIO_INTR_MSK: %#x  "
            "%#lx: APP GPIO_INTR_STT: %#x  "
            "%#lx: APP GPIO_INTR_RAW: %#x  "

            "%#lx: PINMUX_FUNC1 enable: %#x  %#lx: FUNC: %#x  "
            "%#lx: PINMUX_FUNC2 enable: %#x  "
            "%#lx: PINMUX_FUNC3 enable: %#x  "
            "%#lx: PINMUX_FUNC4 enable: %#x  "
            ,

            __func__,
            pin,

            (uintptr_t)&APP_PMU->IO_CFG[x->port].PU1_PU0,
            APP_PMU->IO_CFG[x->port].PU1_PU0 & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].PD_PU2,
            APP_PMU->IO_CFG[x->port].PD_PU2 & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].IEN1_IEN0,
            APP_PMU->IO_CFG[x->port].IEN1_IEN0 & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].DS1_DS0,
            APP_PMU->IO_CFG[x->port].DS1_DS0 & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].AE_DS2,
            APP_PMU->IO_CFG[x->port].AE_DS2 & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].OD_FIR,
            APP_PMU->IO_CFG[x->port].OD_FIR & mask,
            (uintptr_t)&APP_PMU->IO_CFG[x->port].SL_ST,
            APP_PMU->IO_CFG[x->port].SL_ST & mask,

            (uintptr_t)&APP_PMU->IO_VAL[x->port].OE_DIN,
            APP_PMU->IO_VAL[x->port].OE_DIN & mask,
            (uintptr_t)&APP_PMU->IO_VAL[x->port].DOC_DOS,
            APP_PMU->IO_VAL[x->port].DOC_DOS & mask,

#if (DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay))
            (uintptr_t)&SYSC_SEC_AWO->FUNC_IO_LOCK[pin>>4],
            SYSC_SEC_AWO->FUNC_IO_LOCK[pin>>4] & BIT(x->num),
            (uintptr_t)&SYSC_SEC_PER->IO_FUNC_LOCK[pin>>4],
            SYSC_SEC_PER->IO_FUNC_LOCK[pin>>4] & BIT(x->num),

            (uintptr_t)&SEC_PMU->GPIO_INTR_MSK[x->port],
            SEC_PMU->GPIO_INTR_MSK[x->port] & mask,
            (uintptr_t)&SEC_PMU->GPIO_INTR_STT[x->port],
            SEC_PMU->GPIO_INTR_STT[x->port] & mask,
            (uintptr_t)&SEC_PMU->GPIO_INTR_RAW[x->port],
            SEC_PMU->GPIO_INTR_RAW[x->port] & mask,
#endif
            (uintptr_t)&APP_PMU->GPIO_INTR_MSK[x->port],
            APP_PMU->GPIO_INTR_MSK[x->port] & mask,
            (uintptr_t)&APP_PMU->GPIO_INTR_STT[x->port],
            APP_PMU->GPIO_INTR_STT[x->port] & mask,
            (uintptr_t)&APP_PMU->GPIO_INTR_RAW[x->port],
            APP_PMU->GPIO_INTR_RAW[x->port] & mask,

            (uintptr_t)&SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC1][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC1),
            (uintptr_t)&SYSC_APP_PER->FUNC_SEL[x->port][x->num / 4],
            per_func0_alt_get(pin),
            (uintptr_t)&SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC2][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC2),
            (uintptr_t)&SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC3][x->port >> 1],
            per_func_en_get(pin, PINMUX_FUNC3),
            (uintptr_t)&SYSC_APP_AWO->IO_FUNC[PINMUX_FUNC4][x->port >> 1],
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

static int cmd_pinmux_show(const struct shell *sh, size_t argc, char **argv)
{
    if (1 == argc) {
        shell_print(sh, "Usage: %s <pin> [pin] [pin] ...", argv[0]);
        return 0;
    }
    for (int i = 1; i < argc; i++) {
        int err = 0;
        char port = argv[i][0];
        int num = shell_strtoul(&argv[i][1], 10, &err);
        if (err) {
            shell_print(sh, "Invalid Arguments: %s",  argv[i]);
        } else {
            pinmux_show(pin2code(port, num));
        }
    }

    return 0;
}

SHELL_CMD_REGISTER(soc_pinmux, NULL, "pinmux_show A10 A11 A12 A13 ...", cmd_pinmux_show);
