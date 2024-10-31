#include <zephyr/kernel.h>

#include <le501x/integration/reg_rcc.h>
#include <le501x/integration/reg_gpio.h>
#include <le501x/platform.h>
#include <field_manipulate.h>

void sys_init_ls_controller(void);

void io_init(void)
{
    RCC->AHBEN |= RCC_GPIOA_MASK | RCC_GPIOB_MASK | RCC_GPIOC_MASK;
    LSGPIOA->MODE = 0;
    LSGPIOA->IE = 0;
    LSGPIOA->OE = 0;
    LSGPIOA->PUPD = 0;
    LSGPIOB->MODE &= 0x3c00;
    LSGPIOB->IE = 0;
    LSGPIOB->OE = 0;
    LSGPIOB->PUPD = 0x2800;
}

#if defined(CONFIG_SPI_DW)
static void spi_dw_rcc_config(void)
{
    REG_FIELD_WR(RCC->APB2RST, RCC_SPI1, 1);
    REG_FIELD_WR(RCC->APB2RST, RCC_SPI1, 0);
    REG_FIELD_WR(RCC->APB2EN, RCC_SPI1, 1);
}
#endif

static int le501x_init(void)
{
    sys_init_ls_controller();
#if defined(CONFIG_SPI_DW)
    spi_dw_rcc_config();
#endif

    return 0;
}

SYS_INIT(le501x_init, PRE_KERNEL_1, 0);
