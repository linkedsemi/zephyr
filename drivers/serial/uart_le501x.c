#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

//....................................borad:le501x...........................................

#include <string.h>
#include <stdlib.h>
#include <ls_hal_uart.h>
#include <ls_ll_uart.h>
#include <ls_soc_gpio.h>

#include <field_manipulate.h>
#include <reg_rcc.h>
// #include <log.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/pm/policy.h>

#define DT_DRV_COMPAT le501x_uart

struct uart_le501x_data_t
{
	uint8_t uart_txd;
	uint8_t uart_rxd;
	uart_irq_callback_user_data_t user_cb;
	void *user_parm;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif		
#ifdef CONFIG_PM
	bool pm_policy_state_on;
#endif
};

static int uart_le501x_poll_in(const struct device *dev, unsigned char *p_char)
{
	// LOG_I("uart_le501x_poll_in");
	UART_HandleTypeDef *uart_handle  = (UART_HandleTypeDef *)dev->config;
	if (LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_OE)) {
	}

	if (!LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE)) {
		return -1;
	}
	*p_char = LL_UART_ReceiveData((reg_uart_t *)uart_handle->UARTX);
	return 0;
}

static void uart_le501x_poll_out(const struct device *dev, unsigned char p_char)
{
	UART_HandleTypeDef *uart_handle  = (UART_HandleTypeDef *)dev->config;

	while (1) {
		if (LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFNF))
		{
			uint32_t key = irq_lock();
			LL_UART_TransmitData((reg_uart_t *)uart_handle->UARTX,p_char);
			irq_unlock(key);
			return;
		}
	}
}

#ifdef CONFIG_PM
static void uart_le501x_pm_policy_state_lock_get(const struct device *dev)
{
	struct uart_le501x_data_t *data = dev->data;

	if (!data->pm_policy_state_on) {
		data->pm_policy_state_on = true;
		// pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

// static void uart_le501x_pm_policy_state_lock_put(const struct device *dev)
// {
// 	struct uart_le501x_data_t *data = dev->data;

// 	if (data->pm_policy_state_on) {
// 		data->pm_policy_state_on = false;
// 		// pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
// 		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
// 	}
// }
#endif /* CONFIG_PM */

void uart1_msp_init(void)
{
    REG_FIELD_WR(RCC->APB2RST, RCC_UART1, 1);
    REG_FIELD_WR(RCC->APB2RST, RCC_UART1, 0);
    REG_FIELD_WR(RCC->APB2EN, RCC_UART1, 1);	
}

void uart2_msp_init(void)
{
    REG_FIELD_WR(RCC->APB1RST, RCC_UART2, 1);
    REG_FIELD_WR(RCC->APB1RST, RCC_UART2, 0);
    REG_FIELD_WR(RCC->APB1EN, RCC_UART2, 1);
}

void uart3_msp_init(void)
{
    REG_FIELD_WR(RCC->APB1RST, RCC_UART3, 1);
    REG_FIELD_WR(RCC->APB1RST, RCC_UART3, 0);
    REG_FIELD_WR(RCC->APB1EN, RCC_UART3, 1);
}
static int uart_le501x_init(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;

	switch((uint32_t)uart_handle->UARTX)
	{
		case (uint32_t)UART1:
			uart1_msp_init();
			break;
		case (uint32_t)UART2:
			uart2_msp_init();
			break;
		case (uint32_t)UART3:
			uart3_msp_init();
			break;
		default:
			break;
	}

    REG_FIELD_WR(uart_handle->UARTX->LCR,UART_LCR_BRWEN,1);
    uart_handle->UARTX->BRR  =  uart_handle->Init.BaudRate;
    REG_FIELD_WR(uart_handle->UARTX->LCR,UART_LCR_BRWEN,0);
    uart_handle->UARTX->FCR = UART_FCR_TFRST_MASK | UART_FCR_RFRST_MASK | UART_FCR_FIFOEN_MASK;
    uart_handle->UARTX->LCR = FIELD_BUILD(UART_LCR_DLS,uart_handle->Init.WordLength)|FIELD_BUILD(UART_LCR_STOP,uart_handle->Init.StopBits)
                                  |FIELD_BUILD(UART_LCR_PARITY,uart_handle->Init.Parity)|FIELD_BUILD(UART_LCR_MSB,uart_handle->Init.MSBEN)
                                  |FIELD_BUILD(UART_LCR_RXEN,1)|FIELD_BUILD(UART_LCR_BRWEN,0);

	// LOG_I("uart addr : %x",(uint32_t)uart_handle->UARTX);
    pinmux_uart3_init(PB00,PB01);
    io_pull_write(PB01, IO_PULL_UP);

    pinmux_uart1_init(PA08,PA09);
    io_pull_write(PA09, IO_PULL_UP);

	#ifdef CONFIG_PM
	uart_le501x_pm_policy_state_lock_get(dev);
	#endif
	
	#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	struct uart_le501x_data_t *data = (struct uart_le501x_data_t *)dev->data;
	data->irq_config_func(dev);
	#endif
	return 0; 
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_le501x_irq_tx_enable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;	
	LL_UART_EnableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TXS); //transmission complete interrupt enable
}

void uart_le501x_irq_tx_disable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	LL_UART_DisableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TC);
}

int uart_le501x_irq_tx_ready(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFNF) &&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_TC);
}

void uart_le501x_irq_rx_enable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;	
	LL_UART_EnableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD); 	
	// #ifdef CONFIG_PM
	// 	uart_le501x_pm_policy_state_lock_get(dev);
	// #endif	
}

void uart_le501x_irq_rx_disable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	LL_UART_DisableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD);
	// #ifdef CONFIG_PM
	// 	uart_le501x_pm_policy_state_lock_put(dev);
	// #endif	
}

int uart_le501x_irq_tx_complete(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFEM);
}

int uart_le501x_irq_rx_ready(const struct device *dev)  //fifo no empty：Returen 1
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE);
}

void uart_le501x_irq_err_enable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

void uart_le501x_irq_err_disable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

int uart_le501x_irq_is_pending(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;

	return ((LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE)	&&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_RXRD))||
		(LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFNF) &&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_TC)));
}

int uart_le501x_irq_update(const struct device *dev)
{
	return 1;
}

void uart_le501x_irq_callback_set(const struct device *dev,uart_irq_callback_user_data_t cb,void *user_data)
{
	struct uart_le501x_data_t *data = (struct uart_le501x_data_t *)dev->data;
	data->user_cb = cb;
	data->user_parm = user_data;
}

void uart_le501x_isr(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	struct uart_le501x_data_t *data = (struct uart_le501x_data_t *)dev->data;
    uint8_t irq_flag =  false;
	if (LL_UART_IsActiveFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD) && LL_UART_IsEnabledIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD))
    {
        LL_UART_ClearFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD);
		irq_flag = true;
    }
    else if (LL_UART_IsActiveFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TXS) && LL_UART_IsEnabledIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TXS))
    {
        LL_UART_ClearFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TXS);
		irq_flag = true;
    }
    else if (LL_UART_IsActiveFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TC) && LL_UART_IsEnabledIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TC))
    {
        LL_UART_ClearFlagIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TC);
		irq_flag = true;
    }
	
	if(irq_flag == true && data->user_cb !=NULL)
	{
		data->user_cb(dev,data->user_parm);
	}
}

int uart_le501x_fifo_fill(const struct device *dev, const uint8_t *tx_data,int len)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	uint8_t num_tx = 0U;
	unsigned int key;

	if (!LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX, UART_SR_TFNF)) {
		return num_tx;
	}

	/* Lock interrupts to prevent nested interrupts or thread switch */
	key = irq_lock();

	while ((len - num_tx > 0) &&
	       LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX, UART_SR_TFNF)) {
		/* TXE flag will be cleared with byte write to DR|RDR register */

		/* Send a character (8bit , parity none) */
		LL_UART_TransmitData((reg_uart_t *)uart_handle->UARTX,tx_data[num_tx++]);
	}

	irq_unlock(key);

	return num_tx;
}

int uart_le501x_fifo_read(const struct device *dev, uint8_t *rx_data,const int size)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	uint8_t num_rx = 0U;

	while ((size - num_rx > 0) &&
	       LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX, UART_SR_RFNE)) {
		/* RXNE flag will be cleared upon read from DR|RDR register */

		/* Receive a character (8bit , parity none) */
		rx_data[num_rx++] = LL_UART_ReceiveData((reg_uart_t *)uart_handle->UARTX);

		/* Clear overrun error flag */
		// if (LL_USART_IsActiveFlag_ORE(config->usart)) {
		// 	LL_USART_ClearFlag_ORE(config->usart);
		// }
	}

	return num_rx;
}


#endif

static const  struct uart_driver_api uart_le501x_api = {
	.poll_in = uart_le501x_poll_in,
	.poll_out = uart_le501x_poll_out,
	.err_check = uart_le501x_init,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_le501x_fifo_fill,
	.fifo_read = uart_le501x_fifo_read,
	.irq_tx_enable = uart_le501x_irq_tx_enable,
	.irq_tx_disable = uart_le501x_irq_tx_disable,
	.irq_tx_ready = uart_le501x_irq_tx_ready,
	.irq_rx_enable = uart_le501x_irq_rx_enable,
	.irq_rx_disable = uart_le501x_irq_rx_disable,
	.irq_tx_complete = uart_le501x_irq_tx_complete,
	.irq_rx_ready = uart_le501x_irq_rx_ready,
	.irq_err_enable = uart_le501x_irq_err_enable,
	.irq_err_disable = uart_le501x_irq_err_disable,
	.irq_is_pending = uart_le501x_irq_is_pending,
	.irq_update = uart_le501x_irq_update,
	.irq_callback_set = uart_le501x_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
};



#if defined(CONFIG_UART_INTERRUPT_DRIVEN)	
#define LE501X_UART_IRQ_HANDLER_DECL(index)				\
	static void uart_le501x_irq_config_func_##index(const struct device *dev);\

#define LE501X_UART_IRQ_HANDLER(index)					\
static void uart_le501x_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		uart_le501x_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}
#else
#define LE501X_UART_IRQ_HANDLER_DECL(index) /* Not used */
#define LE501X_UART_IRQ_HANDLER(index) /* Not used */
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = uart_le501x_irq_config_func_##index,
#else
#define UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif

#define GET_UART_BAUDRATE(index) UART_BUADRATE_ENUM_GEN(DT_INST_PROP(index, current_speed))

#define LE501X_UART_INIT(index)	\
LE501X_UART_IRQ_HANDLER_DECL(index)	\
static UART_HandleTypeDef uart_handle_##index = {	\
	.UARTX = (reg_uart_t *)DT_INST_REG_ADDR(index),	\
	.Init.BaudRate = GET_UART_BAUDRATE(index),	\
	.Init.MSBEN = DT_INST_PROP_OR(index, MSBEN, 0),	\
	.Init.Parity = DT_INST_ENUM_IDX_OR(index, Parity, UART_NOPARITY),	\
	.Init.StopBits = DT_INST_ENUM_IDX_OR(index, StopBits, UART_STOPBITS1),	\
	.Init.WordLength =	DT_INST_ENUM_IDX_OR(index, WordLength, UART_BYTESIZE8),	\
};	\
static struct uart_le501x_data_t uart_le501x_data##index = {	\
	.user_cb = NULL,	\
	.user_parm = NULL,	\
	UART_IRQ_HANDLER_FUNC(index)	\
};	\
	\
DEVICE_DT_INST_DEFINE(index,	\
		    &uart_le501x_init,	\
		    NULL,	\
		    &uart_le501x_data##index, &uart_handle_##index,	\
		    PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,	\
		    &uart_le501x_api);	\
	\
LE501X_UART_IRQ_HANDLER(index)\


DT_INST_FOREACH_STATUS_OKAY(LE501X_UART_INIT)