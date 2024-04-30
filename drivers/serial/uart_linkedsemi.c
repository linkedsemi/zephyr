#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

//....................................borad:ls...........................................

#include <string.h>
#include <stdlib.h>
#include <ls_hal_uart.h>
#include <ls_ll_uart.h>
#include <ls_soc_gpio.h>

#include <field_manipulate.h>
// #include <log.h>
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/pm/policy.h>
#include "reg_sysc_per.h"
#include "ls_soc_gpio.h"

#ifdef SOC_LE5010
#include <reg_rcc.h>
#endif


#define DT_DRV_COMPAT linkedsemi_uart

struct uart_ls_data_t
{
	const struct pinctrl_dev_config *pcfg;
	uart_irq_callback_user_data_t user_cb;
	void *user_parm;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif		
#ifdef CONFIG_PM
	bool pm_policy_state_on;
#endif
};

static int uart_ls_poll_in(const struct device *dev, unsigned char *p_char)
{
	// LOG_I("uart_ls_poll_in");
	UART_HandleTypeDef *uart_handle  = (UART_HandleTypeDef *)dev->config;
	if (LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_OE)) {
	}

	if (!LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE)) {
		return -1;
	}
	*p_char = LL_UART_ReceiveData((reg_uart_t *)uart_handle->UARTX);
	return 0;
}

static void uart_ls_poll_out(const struct device *dev, unsigned char p_char)
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
#ifdef SOC_LE5010
#ifdef CONFIG_PM
static void uart_ls_pm_policy_state_lock_get(const struct device *dev)
{
	struct uart_ls_data_t *data = dev->data;

	if (!data->pm_policy_state_on) {
		data->pm_policy_state_on = true;
		// pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
		pm_policy_state_lock_get(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
	}
}

// static void uart_ls_pm_policy_state_lock_put(const struct device *dev)
// {
// 	struct uart_ls_data_t *data = dev->data;

// 	if (data->pm_policy_state_on) {
// 		data->pm_policy_state_on = false;
// 		// pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
// 		pm_policy_state_lock_put(PM_STATE_STANDBY, PM_ALL_SUBSTATES);
// 	}
// }
#endif /* CONFIG_PM */
#endif /*SOC_LE5010*/

void uart1_msp_init(void)
{
#if defined(SOC_LE5010)
    REG_FIELD_WR(RCC->APB2RST, RCC_UART1, 1);
    REG_FIELD_WR(RCC->APB2RST, RCC_UART1, 0);
    REG_FIELD_WR(RCC->APB2EN, RCC_UART1, 1);	
#elif defined(SOC_LS1010)
	REG_FIELD_WR(SYSC_PER->PD_PER_CLKG1, SYSC_PER_CLKG_SET_UART1, 1);
#endif
}

void uart2_msp_init(void)
{
#if defined(SOC_LE5010)
    REG_FIELD_WR(RCC->APB1RST, RCC_UART2, 1);
    REG_FIELD_WR(RCC->APB1RST, RCC_UART2, 0);
    REG_FIELD_WR(RCC->APB1EN, RCC_UART2, 1);
#elif defined(SOC_LS1010)
	REG_FIELD_WR(SYSC_PER->PD_PER_CLKG1, SYSC_PER_CLKG_SET_UART2, 1);
#endif
}

void uart3_msp_init(void)
{
#if defined(SOC_LE5010)
    REG_FIELD_WR(RCC->APB1RST, RCC_UART3, 1);
    REG_FIELD_WR(RCC->APB1RST, RCC_UART3, 0);
    REG_FIELD_WR(RCC->APB1EN, RCC_UART3, 1);
#elif defined(SOC_LS1010)
	REG_FIELD_WR(SYSC_PER->PD_PER_CLKG1, SYSC_PER_CLKG_SET_UART3, 1);
#endif
}

static void uart_msp_init(UART_HandleTypeDef *uart_handle)
{
	switch((uint32_t)uart_handle->UARTX)
	{
		case (uint32_t)UART1_BASE_ADDR:
			uart1_msp_init();
			break;
		case (uint32_t)UART2_BASE_ADDR:
			uart2_msp_init();
			break;
		case (uint32_t)UART3_BASE_ADDR:
			uart3_msp_init();
			break;
		default:
			break;
	}
}

static int uart_ls_init(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	struct uart_ls_data_t *data = (struct uart_ls_data_t *)dev->data;
	int ret = 0;
	(void)data;
	REG_FIELD_WR(SYSC_PER->PD_PER_CLKG1, SYSC_PER_CLKG_SET_UART1, 1);
	uart_msp_init(uart_handle);

	pinmux_uart1_init(PC14,PC12);
    REG_FIELD_WR(uart_handle->UARTX->LCR,UART_LCR_BRWEN,1);
    uart_handle->UARTX->BRR  =  uart_handle->Init.BaudRate;
    REG_FIELD_WR(uart_handle->UARTX->LCR,UART_LCR_BRWEN,0);
    uart_handle->UARTX->FCR = UART_FCR_TFRST_MASK | UART_FCR_RFRST_MASK | UART_FCR_FIFOEN_MASK;
    uart_handle->UARTX->LCR = FIELD_BUILD(UART_LCR_DLS,uart_handle->Init.WordLength)|FIELD_BUILD(UART_LCR_STOP,uart_handle->Init.StopBits)
                                  |FIELD_BUILD(UART_LCR_PARITY,uart_handle->Init.Parity)|FIELD_BUILD(UART_LCR_MSB,uart_handle->Init.MSBEN)
                                  |FIELD_BUILD(UART_LCR_RXEN,1)|FIELD_BUILD(UART_LCR_BRWEN,0);

	// LOG_I("uart addr : %x",(uint32_t)uart_handle->UARTX);
	/* Configure dt provided device signals when available */
	// ret = pinctrl_apply_state(data->pcfg, PINCTRL_STATE_DEFAULT);   //pin
	// if (ret < 0) {
	// 	// LOG_ERR("UART pinctrl setup failed (%d)", ret);
	// 	return ret;
	// }
#ifdef SOC_LE5010
	#ifdef CONFIG_PM
	uart_ls_pm_policy_state_lock_get(dev);
	#endif
#endif
	#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	data->irq_config_func(dev);
	#endif

	return ret; 
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_ls_irq_tx_enable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;	
	LL_UART_EnableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TXS); //transmission complete interrupt enable
}

void uart_ls_irq_tx_disable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	LL_UART_DisableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_TC);
}

int uart_ls_irq_tx_ready(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFNF) &&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_TC);
}

void uart_ls_irq_rx_enable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;	
	LL_UART_EnableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD); 	
	// #ifdef CONFIG_PM
	// 	uart_ls_pm_policy_state_lock_get(dev);
	// #endif	
}

void uart_ls_irq_rx_disable(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	LL_UART_DisableIT((reg_uart_t *)uart_handle->UARTX, UART_IT_RXRD);
	// #ifdef CONFIG_PM
	// 	uart_ls_pm_policy_state_lock_put(dev);
	// #endif	
}

int uart_ls_irq_tx_complete(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFEM);
}

int uart_ls_irq_rx_ready(const struct device *dev)  //fifo no empty：Returen 1
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	return LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE);
}

void uart_ls_irq_err_enable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

void uart_ls_irq_err_disable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

int uart_ls_irq_is_pending(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;

	return ((LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_RFNE)	&&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_RXRD))||
		(LL_UART_IsActiveFlag((reg_uart_t *)uart_handle->UARTX,UART_SR_TFNF) &&
		LL_UART_IsMaskIT((reg_uart_t *)uart_handle->UARTX,UART_IT_TC)));
}

int uart_ls_irq_update(const struct device *dev)
{
	return 1;
}

void uart_ls_irq_callback_set(const struct device *dev,uart_irq_callback_user_data_t cb,void *user_data)
{
	struct uart_ls_data_t *data = (struct uart_ls_data_t *)dev->data;
	data->user_cb = cb;
	data->user_parm = user_data;
}

void uart_ls_isr(const struct device *dev)
{
	UART_HandleTypeDef *uart_handle = (UART_HandleTypeDef *)dev->config;
	struct uart_ls_data_t *data = (struct uart_ls_data_t *)dev->data;
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

int uart_ls_fifo_fill(const struct device *dev, const uint8_t *tx_data,int len)
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

int uart_ls_fifo_read(const struct device *dev, uint8_t *rx_data,const int size)
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

static const  struct uart_driver_api uart_ls_api = {
	.poll_in = uart_ls_poll_in,
	.poll_out = uart_ls_poll_out,
	.err_check = uart_ls_init,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_ls_fifo_fill,
	.fifo_read = uart_ls_fifo_read,
	.irq_tx_enable = uart_ls_irq_tx_enable,
	.irq_tx_disable = uart_ls_irq_tx_disable,
	.irq_tx_ready = uart_ls_irq_tx_ready,
	.irq_rx_enable = uart_ls_irq_rx_enable,
	.irq_rx_disable = uart_ls_irq_rx_disable,
	.irq_tx_complete = uart_ls_irq_tx_complete,
	.irq_rx_ready = uart_ls_irq_rx_ready,
	.irq_err_enable = uart_ls_irq_err_enable,
	.irq_err_disable = uart_ls_irq_err_disable,
	.irq_is_pending = uart_ls_irq_is_pending,
	.irq_update = uart_ls_irq_update,
	.irq_callback_set = uart_ls_irq_callback_set,
#endif  /* CONFIG_UART_INTERRUPT_DRIVEN */
};



#if defined(CONFIG_UART_INTERRUPT_DRIVEN)	
#define LS_UART_IRQ_HANDLER_DECL(index)				\
	static void uart_ls_irq_config_func_##index(const struct device *dev);\

#define LS_UART_IRQ_HANDLER(index)					\
static void uart_ls_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		uart_ls_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}
#else
#define LS_UART_IRQ_HANDLER_DECL(index) /* Not used */
#define LS_UART_IRQ_HANDLER(index) /* Not used */
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = uart_ls_irq_config_func_##index,
#else
#define UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif

#define GET_UART_BAUDRATE(index) UART_BUADRATE_ENUM_GEN(DT_INST_PROP(index, current_speed))

#define LS_UART_INIT(index)	\
LS_UART_IRQ_HANDLER_DECL(index)	\
PINCTRL_DT_INST_DEFINE(index);						\
static UART_HandleTypeDef uart_handle_##index = {	\
	.UARTX = (reg_uart_t *)DT_INST_REG_ADDR(index),	\
	.Init.BaudRate = GET_UART_BAUDRATE(index),	\
	.Init.MSBEN = DT_INST_PROP_OR(index, MSBEN, 0),	\
	.Init.Parity = DT_INST_ENUM_IDX_OR(index, Parity, UART_NOPARITY),	\
	.Init.StopBits = DT_INST_ENUM_IDX_OR(index, StopBits, UART_STOPBITS1),	\
	.Init.WordLength =	DT_INST_ENUM_IDX_OR(index, WordLength, UART_BYTESIZE8),	\
};	\
static struct uart_ls_data_t uart_ls_data##index = {	\
	.user_cb = NULL,	\
	.user_parm = NULL,	\
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),			\
	UART_IRQ_HANDLER_FUNC(index)	\
};	\
	\
DEVICE_DT_INST_DEFINE(index,	\
		    &uart_ls_init,	\
		    NULL,	\
		    &uart_ls_data##index, &uart_handle_##index,	\
		    PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,	\
		    &uart_ls_api);	\
	\
LS_UART_IRQ_HANDLER(index)


DT_INST_FOREACH_STATUS_OKAY(LS_UART_INIT)