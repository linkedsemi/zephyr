/*
 * Copyright (c) 2023 Linkedsemi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT linkedsemi_lsqsh_gpio

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/linkedsemi-ls-gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#include <zephyr/irq.h>
#include <errno.h>

#include <ls_soc_gpio.h>
#include <reg_exti_type.h>
#include <reg_pmu.h>

#define NUMBER_OF_PORTS 15

// use BIT(9) for GPIO DS field
#define LS_GPIO_DS_POS                  9
#define LS_GPIO_DS_MASK                 (0x3U << LS_GPIO_DS_POS)

/* GPIO driver will use 1/4 out drive capability if DS is 0 or 1 */
#define LS_GPIO_DS_QUARTER_DRIVE        (0x0U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_HALF_DRIVE           (0x1U << LS_GPIO_DS_POS)
#define LS_GPIO_DS_MAX_DRIVE            (0x3U << LS_GPIO_DS_POS)


#define LSPIN(_port, _pin) (_port << 4 | _pin)

static struct gpio_ls_common_data gpio_ls_common_data;

struct gpio_ls_common_config {
};

struct gpio_ls_common_data {
    /* a list of all ports */
    const struct device *ports[NUMBER_OF_PORTS];
    size_t count;
};

struct gpio_ls_config {
    struct gpio_driver_config common;
    uint32_t *base;
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

static inline void gpio_ls_add_port(struct gpio_ls_common_data *data,
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
    case DT_REG_ADDR(DT_NODELABEL(gpioa)):
        port_id = 0;
        break;
    /* port B base */
    case DT_REG_ADDR(DT_NODELABEL(gpiob)):
        port_id = 1;
        break;
    /* port C base */
    case DT_REG_ADDR(DT_NODELABEL(gpioc)):
        port_id = 2;
        break;
    /* port D base */
    case DT_REG_ADDR(DT_NODELABEL(gpiod)):
        port_id = 3;
        break;
    /* port E base */
    case DT_REG_ADDR(DT_NODELABEL(gpioe)):
        port_id = 4;
        break;
    /* port F base */
    case DT_REG_ADDR(DT_NODELABEL(gpiof)):
        port_id = 5;
        break;
    /* port G base */
    case DT_REG_ADDR(DT_NODELABEL(gpiog)):
        port_id = 6;
        break;
    /* port H base */
    case DT_REG_ADDR(DT_NODELABEL(gpioh)):
        port_id = 7;
        break;
    default:
        return -ENOTSUP;
    }
    return port_id;
}

static int gpio_ls_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;

    uint8_t pinval, port;

    port = get_gpio_port_id((uint32_t)gpio);
    pinval = LSPIN(port, pin);

    switch (flags & (GPIO_INPUT | GPIO_OUTPUT)) {
    case GPIO_INPUT:
        io_cfg_input(pinval);
        break;
    case GPIO_OUTPUT:
        io_cfg_output(pinval);
        break;
    case GPIO_DISCONNECTED:
        io_pull_write(pinval, IO_PULL_DISABLE);
        io_cfg_disable(pinval);
        break;
    default:
        return -ENOTSUP;
    }

    switch (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
    case 0:
        io_pull_write(pinval, IO_PULL_DISABLE);
        break;
    case GPIO_PULL_UP:
        io_pull_write(pinval, IO_PULL_UP);
        break;
    case GPIO_PULL_DOWN:
        io_pull_write(pinval, IO_PULL_DOWN);
        break;
    default:
        return -EINVAL;
    }

    switch (flags & LS_GPIO_DS_MASK) {
    case LS_GPIO_DS_QUARTER_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_QUARTER_DRIVER);
        break;
    case LS_GPIO_DS_HALF_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_HALF_DRIVER);
        break;
    case LS_GPIO_DS_MAX_DRIVE:
        io_drive_capacity_write(pinval, IO_OUTPUT_MAX_DRIVER);
        break;
    default:
        return -ENOTSUP;
    }

    if ((flags & GPIO_OUTPUT) != 0) {
        if ((flags & GPIO_SINGLE_ENDED) != 0) {
            if (flags & GPIO_LINE_OPEN_DRAIN) {
                io_cfg_opendrain(pinval);
            } else {
                /* Output can't be open source */
                return -ENOTSUP;
            }
        } else {
            io_cfg_pushpull(pinval);
        }

        if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
            gpio_ls_port_set_bits_raw(dev, BIT(pin));
        } else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
            gpio_ls_port_clear_bits_raw(dev, BIT(pin));
        }
    }

    return 0;
}

static void gpio_vcore_isr(const struct device *dev)
{
    struct gpio_ls_common_data *data = (struct gpio_ls_common_data *)DEVICE_DT_GET(DT_INST(0, linkedsemi_lsqsh_pinctrl))->data;
    const struct device *port_dev;
    struct gpio_ls_data *port_data;
    uint32_t interrupt_status = 0;
    volatile uint32_t *INT_STAT_BASE = PMU->GPIO_INTR_STT;
    volatile uint32_t *INT_CLR_BASE = PMU->GPIO_INTR_CLR;
    for (uint8_t i = 0; i < 8; ++i) {
        volatile uint32_t *INT_STAT_REG = &INT_STAT_BASE[i];
        volatile uint32_t *INT_CLR_REG = &INT_CLR_BASE[i];
        uint32_t int_stat = *INT_STAT_REG;
        if (int_stat == 0) {
            continue;
        }
        for (uint8_t j = 0; j < 16; ++j) {
            exti_edge_t edge = INT_EDGE_NONE;
            if (1 << j & int_stat) {
                *INT_CLR_REG = 1 << j;
                interrupt_status = 1 << j;
                edge |= INT_EDGE_RISING;
            }
            if (1 << 16 << j & int_stat) {
                *INT_CLR_REG = 1 << 16 << j;
                interrupt_status = 1 << j;
                edge |= INT_EDGE_FALLING;
            }
            if (edge) {
                *INT_CLR_REG = 0;
            }
        }
    }
    for (uint8_t i = 0; i < data->count; i++) {
        port_dev = data->ports[i];
        port_data = port_dev->data;
        gpio_fire_callbacks(&port_data->callbacks, port_dev, interrupt_status);
    }
}

static int gpio_ls_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;
    *value = gpio->DIN;
    return 0;
}

static int gpio_ls_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask, gpio_port_value_t value)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;
    uint32_t port_value;
    port_value = gpio->DOT;
    gpio->DOT = (value & mask) | (port_value & ~mask);
    return 0;
}

static int gpio_ls_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;

    gpio->DOT = pins;

    return 0;
}

static int gpio_ls_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;

    gpio->DOT = pins << 16;

    return 0;
}

static int gpio_ls_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;
    if (gpio->DOT & pins) {
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
    const struct gpio_ls_config *cfg = dev->config;
    pmu_io_reg_t *gpio = (pmu_io_reg_t *)cfg->base;
    uint8_t config, pinval, port;

    port = get_gpio_port_id((uint32_t)gpio);
    pinval = LSPIN(port, pin);

    if (mode != GPIO_INT_MODE_DISABLED) {
        if (mode == GPIO_INT_MODE_EDGE) {
            if (trig == GPIO_INT_TRIG_BOTH) {
                config = INT_EDGE_BOTH;
            } else if (trig == GPIO_INT_TRIG_HIGH) {
                config = INT_EDGE_RISING;
            } else {
                config = INT_EDGE_FALLING;
            }
        } else {
            return -ENOTSUP;
        }
    } else {
        config = INT_EDGE_DISABLE;
    }

    io_vcore_exti_config(pinval, config);
    return 0;
}

static int gpio_ls_common_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    gpio_ls_common_data.count = 0;

    IRQ_CONNECT(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), exti_gpio_vcore, irq),
                DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), exti_gpio_vcore, priority),
                gpio_vcore_isr,
                NULL,
                0);

    irq_enable(DT_IRQ_BY_NAME(DT_INST(0, linkedsemi_lsqsh_pinctrl), exti_gpio_vcore, irq));

    return 0;
}

static int gpio_ls_manage_callback(const struct device *dev,
                                   struct gpio_callback *callback,
                                   bool set)
{
    struct gpio_ls_data *data = dev->data;

    return gpio_manage_callback(&data->callbacks, callback, set);
}

static const struct gpio_driver_api gpio_ls_driver_api = {
    .pin_configure = gpio_ls_pin_configure,
    .port_get_raw = gpio_ls_port_get_raw,
    .port_set_masked_raw = gpio_ls_port_set_masked_raw,
    .port_set_bits_raw = gpio_ls_port_set_bits_raw,
    .port_clear_bits_raw = gpio_ls_port_clear_bits_raw,
    .port_toggle_bits = gpio_ls_port_toggle_bits,
    .pin_interrupt_configure = gpio_ls_pin_interrupt_configure,
    .manage_callback = gpio_ls_manage_callback,
};

static const struct gpio_driver_api gpio_ls_common_driver_api = {
    .manage_callback = gpio_ls_manage_callback,
};

static const struct gpio_ls_common_config gpio_ls_common_config = {};

DEVICE_DT_DEFINE(DT_INST(0, linkedsemi_lsqsh_pinctrl),
                 gpio_ls_common_init,
                 NULL,
                 &gpio_ls_common_data,
                 &gpio_ls_common_config,
                 PRE_KERNEL_1,
                 CONFIG_GPIO_INIT_PRIORITY,
                 &gpio_ls_common_driver_api);

#define GPIO_LS_DEFINE(index)                                                 \
    static struct gpio_ls_data ls_data_##index;                               \
                                                                              \
    static const struct gpio_ls_config ls_config_##index = {                  \
        .base = (uint32_t *)DT_INST_REG_ADDR(index),                          \
        .common = { .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index) } \
    };                                                                        \
                                                                              \
    static int gpio_ls_port_init_##index(const struct device *dev)            \
    {                                                                         \
        gpio_ls_add_port(&gpio_ls_common_data, dev);                          \
        return 0;                                                             \
    }                                                                         \
                                                                              \
    DEVICE_DT_INST_DEFINE(index, gpio_ls_port_init_##index, NULL, &ls_data_##index, &ls_config_##index, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &gpio_ls_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_LS_DEFINE)
