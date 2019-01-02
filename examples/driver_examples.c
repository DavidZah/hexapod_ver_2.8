/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

static void convert_cb_ADC_0(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

/**
 * Example of using ADC_0 to generate waveform.
 */
void ADC_0_example(void)
{
	adc_async_register_callback(&ADC_0, 0, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_0);
	adc_async_enable_channel(&ADC_0, 0);
	adc_async_start_conversion(&ADC_0);
}

/**
 * Example of using USART_LORA to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_LORA[12] = "Hello World!";

static void tx_cb_USART_LORA(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_LORA_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_LORA, USART_ASYNC_TXC_CB, tx_cb_USART_LORA);
	/*usart_async_register_callback(&USART_LORA, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_LORA, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_LORA, &io);
	usart_async_enable(&USART_LORA);

	io_write(io, example_USART_LORA, 12);
}

void I2C_0_example(void)
{
	struct io_descriptor *I2C_0_io;

	i2c_m_sync_get_io_descriptor(&I2C_0, &I2C_0_io);
	i2c_m_sync_enable(&I2C_0);
	i2c_m_sync_set_slaveaddr(&I2C_0, 0x12, I2C_M_SEVEN);
	io_write(I2C_0_io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using USART_BT to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_USART_BT[12] = "Hello World!";

static void tx_cb_USART_BT(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void USART_BT_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&USART_BT, USART_ASYNC_TXC_CB, tx_cb_USART_BT);
	/*usart_async_register_callback(&USART_BT, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&USART_BT, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&USART_BT, &io);
	usart_async_enable(&USART_BT);

	io_write(io, example_USART_BT, 12);
}

/**
 * Example of using Debug to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_Debug[12] = "Hello World!";

static void tx_cb_Debug(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void Debug_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&Debug, USART_ASYNC_TXC_CB, tx_cb_Debug);
	/*usart_async_register_callback(&Debug, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&Debug, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&Debug, &io);
	usart_async_enable(&Debug);

	io_write(io, example_Debug, 12);
}

/**
 * Example of using PWM_1.
 */
void PWM_1_example(void)
{
	pwm_set_parameters(&PWM_1, 10000, 5000);
	pwm_enable(&PWM_1);
}

/**
 * Example of using PWM_0.
 */
void PWM_0_example(void)
{
	pwm_set_parameters(&PWM_0, 10000, 10);
	pwm_enable(&PWM_0);
}

/**
 * Example of using TIMER_0.
 */
struct timer_task TIMER_0_task1, TIMER_0_task2;

static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}
