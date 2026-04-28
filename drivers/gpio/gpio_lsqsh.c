/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_lsqsh_gpio

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dt-bindings/gpio/linkedsemi-ls-gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <errno.h>
#include <soc.h>
#include <platform.h>

LOG_MODULE_REGISTER(gpio_lsqsh, CONFIG_GPIO_LOG_LEVEL);

/* except qspi1 */
#define NUMBER_OF_PORTS 15

// use BIT(9) for GPIO DS field
#define LS_GPIO_DS_POS  9
#define LS_GPIO_DS_MASK (0x3U << LS_GPIO_DS_POS)

/* GPIO driver will use 1/4 out drive capability if DS is 0 or 1 */
#define LS_GPIO_DS_QUARTER_DRIVE (0x0U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_HALF_DRIVE    (0x1U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_MAX_DRIVE     (0x3U << LS_GPIO_DS_POS)

#define LSPIN(_port, _pin) (_port << 4 | _pin)

#define GPIO_LS_PULL_DOWN_MASK BIT(3)
#define GPIO_LS_PULL_UP_MASK   (BIT(2) | BIT(1) | BIT(0))

static struct gpio_ls_exti_data gpio_ls_exti_data;

struct gpio_ls_exti_data {
    /* a list of all ports */
    const struct device *ports[NUMBER_OF_PORTS];
    size_t count;
};

struct gpio_ls_config {
    struct gpio_driver_config common;
    uint32_t *base_io_cfg;
    uint32_t *base_io_val;
};

struct gpio_ls_data {
    struct gpio_driver_data common;
    sys_slist_t callbacks;
};
extern void io_vcore_exti_config(uint8_t pin, exti_edge_t edge);
void io_wkup_en_clr_set(uint8_t pin);
static int gpio_ls_port_set_bits_raw(const struct device *dev,
                                     gpio_port_pins_t pins);
static int gpio_ls_port_clear_bits_raw(const struct device *dev,
                                       gpio_port_pins_t pins);

static inline void gpio_ls_add_port(struct gpio_ls_exti_data *data,
                                    const struct device *dev)
{
    __ASSERT(dev, "No port device!");
    data->ports[data->count++] = dev;
}

static int get_gpio_port_id(uint32_t port)
{
    uint8_t port_id;

    switch (port) {
    /* port A base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioa), io_cfg):
        port_id = 0;
        break;
    /* port B base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiob), io_cfg):
        port_id = 1;
        break;
    /* port C base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioc), io_cfg):
        port_id = 2;
        break;
    /* port D base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiod), io_cfg):
        port_id = 3;
        break;
    /* port E base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioe), io_cfg):
        port_id = 4;
        break;
    /* port F base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiof), io_cfg):
        port_id = 5;
        break;
    /* port G base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiog), io_cfg):
        port_id = 6;
        break;
    /* port H base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioh), io_cfg):
        port_id = 7;
        break;
    /* port I base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioi), io_cfg):
        port_id = 8;
        break;
    /* port J base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioj), io_cfg):
        port_id = 9;
        break;
    /* port K base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiok), io_cfg):
        port_id = 10;
        break;
    /* port M base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiom), io_cfg):
        port_id = 11;
        break;
    /* port N base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpion), io_cfg):
        port_id = 12;
        break;
    /* port Q base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpioq), io_cfg):
        port_id = 13;
        break;
    /* port T base */
    case DT_REG_ADDR_BY_NAME(DT_NODELABEL(gpiot), io_cfg):
        port_id = 14;
        break;
    default:
        return -ENOTSUP;
    }
    return port_id;
}

static int gpio_ls_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    const struct gpio_ls_config *cfg = dev->config;
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    uint8_t pincode;
    uint8_t port;
    bool ret;

    port = get_gpio_port_id((uint32_t)gpio_cfg);
    pincode = LSPIN(port, pin);

    if ((GPIO_OUTPUT | GPIO_LINE_OPEN_DRAIN | GPIO_OUTPUT_INIT_HIGH)
        == (flags & (GPIO_OUTPUT | GPIO_LINE_OPEN_DRAIN | GPIO_OUTPUT_INIT_HIGH))) {
        flags &= ~GPIO_OUTPUT;
        flags |= GPIO_INPUT;
    }

    switch (flags & (GPIO_INPUT | GPIO_OUTPUT)) {
    case GPIO_OUTPUT:
#if defined(CONFIG_GPIO_CFG_LOCK)
        do {
            io_cfg_lock(pincode, false);
            io_cfg_output(pincode);
            io_cfg_lock(pincode, true);
            ret = io_is_output(pincode);
        } while (!ret);
#else
        io_cfg_output(pincode);
        ret = io_is_output(pincode);
        if (!ret) {
            DEV_ERR(dev, "%s:%d: operation fail", __func__, __LINE__);
        }
#endif
        break;
    case GPIO_DISCONNECTED:
#if defined(CONFIG_GPIO_CFG_LOCK)
        do {
            io_cfg_lock(pincode, false);
            io_pull_write(pincode, IO_PULL_DISABLE);
            io_cfg_disable(pincode);
            io_cfg_lock(pincode, true);
            ret = (!io_is_output(pincode)) && (!io_is_input(pincode));
        } while (!ret);
#else
        io_pull_write(pincode, IO_PULL_DISABLE);
        io_cfg_disable(pincode);
        ret = (!io_is_output(pincode)) && (!io_is_input(pincode));
#endif
        break;
    case GPIO_INPUT:
#if defined(CONFIG_GPIO_CFG_LOCK)
        do {
            io_cfg_lock(pincode, false);
            io_cfg_input(pincode);
            io_cfg_lock(pincode, true);
            io_cfg_app_input_lock(pincode, true);
            ret = io_is_input(pincode);
        } while (!ret);
#else
        io_cfg_input(pincode);
        ret = io_is_input(pincode);
#endif
        break;
    default:
        return -ENOTSUP;
    }

    switch (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
    case 0:
        io_pull_write(pincode, IO_PULL_DISABLE);
        break;
    case GPIO_PULL_UP:
        io_pull_write(pincode, IO_PULL_UP);
        break;
    case GPIO_PULL_DOWN:
        io_pull_write(pincode, IO_PULL_DOWN);
        break;
    default:
        return -EINVAL;
    }

    // switch (flags & LS_GPIO_DS_MASK) {
    // case LS_GPIO_DS_QUARTER_DRIVE:
    //     io_drive_capacity_write(pincode, IO_OUTPUT_QUARTER_DRIVER);
    //     break;
    // case LS_GPIO_DS_HALF_DRIVE:
    //     io_drive_capacity_write(pincode, IO_OUTPUT_HALF_DRIVER);
    //     break;
    // case LS_GPIO_DS_MAX_DRIVE:
    //     io_drive_capacity_write(pincode, IO_OUTPUT_MAX_DRIVER);
    //     break;
    // default:
    //     return -ENOTSUP;
    // }

    if ((flags & GPIO_OUTPUT) != 0) {
        if ((flags & GPIO_SINGLE_ENDED) != 0) {
            if (flags & GPIO_LINE_OPEN_DRAIN) {
                io_cfg_opendrain(pincode);
            } else {
                /* Output can't be open source */
                return -ENOTSUP;
            }
        } else {
            io_cfg_pushpull(pincode);
        }

        if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
            gpio_ls_port_set_bits_raw(dev, BIT(pin));
        } else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
            gpio_ls_port_clear_bits_raw(dev, BIT(pin));
        }
    }

    return 0;
}

#if defined(CONFIG_GPIO_GET_CONFIG)
static int gpio_ls_pin_get_config(const struct device *dev,
                                    gpio_pin_t pin,
                                    gpio_flags_t *out_flags)
{
    const struct gpio_ls_config *cfg = dev->config;
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    uint8_t pincode;
    uint8_t port;
    int ret = 0;

    port = get_gpio_port_id((uint32_t)gpio_cfg);
    pincode = LSPIN(port, pin);

    *out_flags = 0;
    if (per_func_en_get(pincode, PINMUX_FUNC1)
        || per_func_en_get(pincode, PINMUX_FUNC2)
        || per_func_en_get(pincode, PINMUX_FUNC3)
        || per_func_en_get(pincode, PINMUX_FUNC4)) {
        DEV_INF(dev, "pin %d not configured as gpio", pin);
        ret = -EINVAL;
        goto err;
    }

    uint8_t val = io_pull_read(pincode);
    if (val & GPIO_LS_PULL_DOWN_MASK) {
        *out_flags |= GPIO_PULL_DOWN;
    }
    if (val & GPIO_LS_PULL_UP_MASK) {
        *out_flags |= GPIO_PULL_UP;
    }

    if (io_is_output_enabled(pincode)) {
        *out_flags |= GPIO_OUTPUT;
        if (io_is_opendrain(pincode)) {
            *out_flags |= GPIO_OPEN_DRAIN;
        }
        if (io_get_output_val(pincode)) {
            *out_flags |= GPIO_OUTPUT_HIGH;
        } else {
            *out_flags |= GPIO_OUTPUT_LOW;
        }
    } else if (io_is_input_enabled(pincode)) {
        *out_flags |= GPIO_INPUT;
    }

    if (0 == (*out_flags & (GPIO_INPUT | GPIO_OUTPUT))) {
        *out_flags |= GPIO_DISCONNECTED;
    }

err:
    return ret;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

#ifdef CONFIG_GPIO_GET_DIRECTION
static int gpio_ls_port_get_direction(const struct device *dev,
                                        gpio_port_pins_t map,
                                        gpio_port_pins_t *inputs,
                                        gpio_port_pins_t *outputs)
{
    const struct gpio_ls_config *cfg = dev->config;
    reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;

    *inputs = (~gpio_cfg->IEN1_IEN0) & 0xffff;
    *outputs = gpio_val->OE_DIN >> 16;

    return 0;
}
#endif /* CONFIG_GPIO_GET_DIRECTION */

static void gpio_vcore_isr(const struct device *dev)
{
    struct gpio_ls_exti_data *data = (struct gpio_ls_exti_data *)DEVICE_DT_GET(DT_INST(0, linkedsemi_lsqsh_pinctrl))->data;
    const struct device *port_dev;
    struct gpio_ls_data *port_data;
    uint32_t pin_mask[NUMBER_OF_PORTS] = {0};
    uint32_t port_mask = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    volatile uint32_t *INT_STAT_BASE = SEC_PMU->GPIO_INTR_STT;
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay)
    volatile uint32_t *INT_STAT_BASE = APP_PMU->GPIO_INTR_STT;
#endif
    volatile uint32_t *INT_CLR_BASE = APP_PMU->GPIO_INTR_CLR;
    for (uint8_t i = 0; i < NUMBER_OF_PORTS; ++i) {
        volatile uint32_t *INT_STAT_REG = &INT_STAT_BASE[i];
        volatile uint32_t *INT_CLR_REG = &INT_CLR_BASE[i];
        IF_ENABLED(CONFIG_GPIO_INTR_LOCK,
                   (volatile uint32_t *INT_LOCK_REG = (volatile uint32_t *)((uint32_t)(&INT_CLR_BASE[i]) - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));))
        uint32_t int_stat = *INT_STAT_REG;
        if (int_stat == 0) {
            continue;
        }
        IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (uint32_t intr_lock_stat = (*INT_LOCK_REG) & int_stat;))
        IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (CLEAR_BIT(*INT_LOCK_REG, intr_lock_stat);))
        for (uint8_t j = 0; j < 16; ++j) {
            exti_edge_t edge = INT_EDGE_NONE;
            if ((1 << j) & int_stat) {
                *INT_CLR_REG = 1 << j;
                pin_mask[i] |= 1 << j;
                port_mask |= 1 << i;
                edge |= INT_EDGE_RISING;
            }
            if ((1 << 16 << j) & int_stat) {
                *INT_CLR_REG = 1 << 16 << j;
                pin_mask[i] |= 1 << j;
                port_mask |= 1 << i;
                edge |= INT_EDGE_FALLING;
            }
            if (edge) {
                *INT_CLR_REG = 0;
            }
        }
        IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (SET_BIT(*INT_LOCK_REG, intr_lock_stat);))
    }

    while (port_mask) {
        const struct gpio_ls_config *cfg;
        int bit = __builtin_ctz(port_mask);
        for (int i = 0; i < NUMBER_OF_PORTS; i++) {
            port_dev = data->ports[i];
            cfg = port_dev->config;
            if (get_gpio_port_id((uint32_t)cfg->base_io_cfg) == bit)
                break;
        }
        port_data = port_dev->data;
        gpio_fire_callbacks(&port_data->callbacks, port_dev, pin_mask[bit]);
        port_mask &= (port_mask - 1);
    }
}

static int gpio_ls_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
    const struct gpio_ls_config *cfg = dev->config;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)((uint32_t)cfg->base_io_cfg - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)((uint32_t)cfg->base_io_val - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
#else
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
#endif

    *value = gpio_val->OE_DIN & 0xffff;

    return 0;
}

static int gpio_ls_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
    const struct gpio_ls_config *cfg = dev->config;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    __maybe_unused reg_sec_io_cfg_t *sec_gpio_cfg = (reg_sec_io_cfg_t *)((uint32_t)cfg->base_io_cfg - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
    __maybe_unused reg_sec_io_val_t *sec_gpio_val = (reg_sec_io_val_t *)((uint32_t)cfg->base_io_val - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
#endif
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    const uint16_t target_pins = value & mask;
    const uint16_t port_value = gpio_val->DOC_DOS; // & 0xffff;
    const uint16_t dos = target_pins | (port_value & ~mask);
    bool ret;

#if defined(CONFIG_GPIO_CFG_LOCK)
    do {
        CLEAR_BIT(sec_gpio_cfg->LOCK, mask & 0xffff);
        gpio_val->DOC_DOS = dos | (~dos << 16);
        SET_BIT(sec_gpio_cfg->LOCK, mask & 0xffff);
        ret = ((gpio_val->DOC_DOS & target_pins) == target_pins);
    } while (!ret);
#else
    gpio_val->DOC_DOS = dos | (~dos << 16);
    ret = ((gpio_val->DOC_DOS & target_pins) == target_pins);
    if (!ret) {
        DEV_ERR(dev, "%s:%d: operation fail", __func__, __LINE__);
    }
#endif

    return 0;
}

static int gpio_ls_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    __maybe_unused reg_sec_io_cfg_t *sec_gpio_cfg = (reg_sec_io_cfg_t *)((uint32_t)cfg->base_io_cfg - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
    __maybe_unused reg_sec_io_val_t *sec_gpio_val = (reg_sec_io_val_t *)((uint32_t)cfg->base_io_val - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
#endif
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    bool ret;

#if defined(CONFIG_GPIO_CFG_LOCK)
    do {
        CLEAR_BIT(sec_gpio_cfg->LOCK, pins & 0xffff);
        if ((pins << 16) & gpio_cfg->OD_FIR) {
            gpio_val->OE_DIN &= ~((pins << 16) & gpio_cfg->OD_FIR);
        }
        gpio_val->DOC_DOS = pins & 0xffff;
        SET_BIT(sec_gpio_cfg->LOCK, pins & 0xffff);
        ret = ((gpio_val->DOC_DOS & pins) == pins);
    } while (!ret);
#else
    if ((pins << 16) & gpio_cfg->OD_FIR) {
        gpio_val->OE_DIN &= ~((pins << 16) & gpio_cfg->OD_FIR);
    }
    gpio_val->DOC_DOS = pins & 0xffff;
    ret = ((gpio_val->DOC_DOS & pins) == pins);
    if (!ret) {
        DEV_ERR(dev, "%s:%d: operation fail", __func__, __LINE__);
    }
#endif

    return 0;
}

static int gpio_ls_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    __maybe_unused reg_sec_io_cfg_t *sec_gpio_cfg = (reg_sec_io_cfg_t *)((uint32_t)cfg->base_io_cfg - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
    __maybe_unused reg_sec_io_val_t *sec_gpio_val = (reg_sec_io_val_t *)((uint32_t)cfg->base_io_val - (APP_PMU_RG_APP_ADDR - SEC_PMU_RG_SEC_ADDR));
#endif
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    bool ret;

#if defined(CONFIG_GPIO_CFG_LOCK)
    do {
        CLEAR_BIT(sec_gpio_cfg->LOCK, pins & 0xffff);
        gpio_val->DOC_DOS = pins << 16;
        if ((pins << 16) & gpio_cfg->OD_FIR) {
            gpio_val->OE_DIN |= (pins << 16) & gpio_cfg->OD_FIR;
        }
        SET_BIT(sec_gpio_cfg->LOCK, pins & 0xffff);
        ret = ((gpio_val->DOC_DOS & pins) == 0);
    } while (!ret);
#else
    gpio_val->DOC_DOS = pins << 16;
    if ((pins << 16) & gpio_cfg->OD_FIR) {
        gpio_val->OE_DIN |= (pins << 16) & gpio_cfg->OD_FIR;
    }
    ret = ((gpio_val->DOC_DOS & pins) == 0);
    if (!ret) {
        DEV_ERR(dev, "%s:%d: operation fail", __func__, __LINE__);
    }
#endif

    return 0;
}

static int gpio_ls_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;

    if ((gpio_val->DOC_DOS & 0xffff) & pins) {
        gpio_ls_port_clear_bits_raw(dev, pins);
    } else {
        gpio_ls_port_set_bits_raw(dev, pins);
    }
    return 0;
}

static int gpio_ls_pin_interrupt_configure(const struct device *dev,
                                           gpio_pin_t pin,
                                           enum gpio_int_mode mode,
                                           enum gpio_int_trig trig)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    reg_sec_pmu_rg_t *PMU = SEC_PMU;
#else
    reg_app_pmu_rg_t *PMU = APP_PMU;
#endif
    const struct gpio_ls_config *cfg = dev->config;
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    uint8_t pincode;
    uint8_t port;

    port = get_gpio_port_id((uint32_t)gpio_cfg);
    pincode = LSPIN(port, pin);

    if (!(mode & GPIO_INT_DISABLE)) {
        if (mode & GPIO_INT_EDGE) {
            if (trig == GPIO_INT_TRIG_BOTH) {
                IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (CLEAR_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<port|1<<16<<port);))
                if (!(mode & GPIO_INT_ENABLE_DISABLE_ONLY)) {
                    SET_BIT(APP_PMU->GPIO_INTR_CLR[port], 1<<pin|1<<16<<pin);
                }
                SET_BIT(PMU->GPIO_INTR_MSK[port], 1<<pin|1<<16<<pin);
            } else if (trig == GPIO_INT_TRIG_HIGH) {
                IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (CLEAR_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<port);))
                if (!(mode & GPIO_INT_ENABLE_DISABLE_ONLY)) {
                    SET_BIT(APP_PMU->GPIO_INTR_CLR[port], 1<<pin);
                }
                MODIFY_REG(PMU->GPIO_INTR_MSK[port], 1<<16<<pin, 1<<pin);
            } else {
                IF_ENABLED(CONFIG_GPIO_INTR_LOCK, (CLEAR_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<16<<port);))
                if (!(mode & GPIO_INT_ENABLE_DISABLE_ONLY)) {
                    SET_BIT(APP_PMU->GPIO_INTR_CLR[port], 1<<16<<pin);
                }
                MODIFY_REG(PMU->GPIO_INTR_MSK[port], 1<<pin, 1<<16<<pin);
            }
        } else {
            DEV_ERR(dev, "level interrupt is not support");
            return -ENOTSUP;
        }
    } else {
        CLEAR_BIT(PMU->GPIO_INTR_MSK[port], 1<<pin|1<<16<<pin);
    }

    if (!(mode & GPIO_INT_ENABLE_DISABLE_ONLY)) {
        WRITE_REG(APP_PMU->GPIO_INTR_CLR[port], 0);
    }

#if defined(CONFIG_GPIO_INTR_LOCK)
    if (mode & GPIO_INT_EDGE) {
        if (trig == GPIO_INT_TRIG_BOTH) {
            SET_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<port|1<<16<<port);
        } else if (trig == GPIO_INT_TRIG_HIGH) {
            SET_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<port);
        } else {
            SET_BIT(SEC_PMU->GPIO_INTR_LOCK[port], 1<<16<<port);
        }
    }
#endif

    return 0;
}

static int gpio_ls_exti_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    gpio_ls_exti_data.count = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu1, irq),
                DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu1, priority),
                gpio_vcore_isr,
                NULL,
                0);
    irq_enable(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu1, irq));
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay)
    IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu2, irq),
                DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu2, priority),
                gpio_vcore_isr,
                NULL,
                0);
    irq_enable(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), cpu2, irq));
#endif

    return 0;
}

static int gpio_ls_manage_callback(const struct device *dev,
                                   struct gpio_callback *callback,
                                   bool set)
{
    struct gpio_ls_data *data = dev->data;

    return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_ls_get_pending_int(const struct device *dev)
{
    const struct gpio_ls_config *cfg = dev->config;
    __maybe_unused reg_io_cfg_t *gpio_cfg = (reg_io_cfg_t *)cfg->base_io_cfg;
    __maybe_unused reg_io_val_t *gpio_val = (reg_io_val_t *)cfg->base_io_val;
    uint8_t port;

    port = get_gpio_port_id((uint32_t)gpio_cfg);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(cpu1), okay)
    volatile uint32_t *INT_STAT_BASE = SEC_PMU->GPIO_INTR_STT;
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(cpu2), okay)
    volatile uint32_t *INT_STAT_BASE = APP_PMU->GPIO_INTR_STT;
#endif

    return INT_STAT_BASE[port];
}

static const struct gpio_driver_api gpio_ls_driver_api = {
    .pin_configure = gpio_ls_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
    .pin_get_config = gpio_ls_pin_get_config,
#endif
    .port_get_raw = gpio_ls_port_get_raw,
    .port_set_masked_raw = gpio_ls_port_set_masked_raw,
    .port_set_bits_raw = gpio_ls_port_set_bits_raw,
    .port_clear_bits_raw = gpio_ls_port_clear_bits_raw,
    .port_toggle_bits = gpio_ls_port_toggle_bits,
    .pin_interrupt_configure = gpio_ls_pin_interrupt_configure,
    .manage_callback = gpio_ls_manage_callback,
    .get_pending_int = gpio_ls_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
    .port_get_direction = gpio_ls_port_get_direction,
#endif /* CONFIG_GPIO_GET_DIRECTION */
};

static const struct gpio_driver_api gpio_ls_exti_driver_api = {
    .manage_callback = gpio_ls_manage_callback,
};

DEVICE_DT_DEFINE(DT_INST(0, linkedsemi_lsqsh_pinctrl),
                 gpio_ls_exti_init,
                 NULL,
                 &gpio_ls_exti_data,
                 NULL,
                 PRE_KERNEL_1,
                 CONFIG_GPIO_INIT_PRIORITY,
                 &gpio_ls_exti_driver_api);

#define GPIO_LS_DEFINE(index)                                                 \
    static struct gpio_ls_data ls_data_##index;                               \
                                                                              \
    static const struct gpio_ls_config ls_config_##index = {                  \
        .base_io_cfg = (uint32_t *)DT_INST_REG_ADDR_BY_NAME(index, io_cfg),   \
        .base_io_val = (uint32_t *)DT_INST_REG_ADDR_BY_NAME(index, io_val),   \
        .common = { .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index) } \
    };                                                                        \
                                                                              \
    static int gpio_ls_port_init_##index(const struct device *dev)            \
    {                                                                         \
        gpio_ls_add_port(&gpio_ls_exti_data, dev);                            \
        return 0;                                                             \
    }                                                                         \
                                                                              \
    DEVICE_DT_INST_DEFINE(index,                                              \
                          gpio_ls_port_init_##index,                          \
                          NULL,                                               \
                          &ls_data_##index,                                   \
                          &ls_config_##index,                                 \
                          PRE_KERNEL_1,                                        \
                          CONFIG_GPIO_INIT_PRIORITY,                          \
                          &gpio_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_LS_DEFINE)
