/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

/* The channel amount for ADC */
#define ADC_0_CH_AMOUNT 1

/* The buffer size for ADC */
#define ADC_0_BUFFER_SIZE 16

/* The maximal channel number of enabled channels */
#define ADC_0_CH_MAX 0

/*! The buffer size for USART */
#define USART_LORA_BUFFER_SIZE 16

/*! The buffer size for USART */
#define USART_BT_BUFFER_SIZE 16

/*! The buffer size for USART */
#define DEBUG_BUFFER_SIZE 16

struct adc_async_descriptor         ADC_0;
struct adc_async_channel_descriptor ADC_0_ch[ADC_0_CH_AMOUNT];
struct usart_async_descriptor       USART_LORA;
struct usart_async_descriptor       USART_BT;
struct usart_async_descriptor       Debug;

static uint8_t ADC_0_buffer[ADC_0_BUFFER_SIZE];
static uint8_t ADC_0_map[ADC_0_CH_MAX + 1];
static uint8_t USART_LORA_buffer[USART_LORA_BUFFER_SIZE];
static uint8_t USART_BT_buffer[USART_BT_BUFFER_SIZE];
static uint8_t Debug_buffer[DEBUG_BUFFER_SIZE];

struct i2c_m_sync_desc I2C_0;

struct pwm_descriptor PWM_1;

struct pwm_descriptor PWM_0;

struct timer_descriptor TIMER_0;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
void ADC_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
	adc_async_init(&ADC_0, ADC, ADC_0_map, ADC_0_CH_MAX, ADC_0_CH_AMOUNT, &ADC_0_ch[0], (void *)NULL);
	adc_async_register_channel_buffer(&ADC_0, 0, ADC_0_buffer, ADC_0_BUFFER_SIZE);

	// Disable digital pin circuitry
	gpio_set_pin_direction(PA04, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(PA04, PINMUX_PA04B_ADC_AIN4);
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_LORA_CLOCK_init()
{

	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_LORA_PORT_init()
{

	gpio_set_pin_function(PA05, PINMUX_PA05D_SERCOM0_PAD1);

	gpio_set_pin_function(PA06, PINMUX_PA06D_SERCOM0_PAD2);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_LORA_init(void)
{
	USART_LORA_CLOCK_init();
	usart_async_init(&USART_LORA, SERCOM0, USART_LORA_buffer, USART_LORA_BUFFER_SIZE, (void *)NULL);
	USART_LORA_PORT_init();
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(PA08,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA08, PINMUX_PA08D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(PA09,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PA09, PINMUX_PA09D_SERCOM2_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	_gclk_enable_channel(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_m_sync_init(&I2C_0, SERCOM2);
	I2C_0_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void USART_BT_CLOCK_init()
{

	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void USART_BT_PORT_init()
{

	gpio_set_pin_function(PB13, PINMUX_PB13C_SERCOM4_PAD1);

	gpio_set_pin_function(PB14, PINMUX_PB14C_SERCOM4_PAD2);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void USART_BT_init(void)
{
	USART_BT_CLOCK_init();
	usart_async_init(&USART_BT, SERCOM4, USART_BT_buffer, USART_BT_BUFFER_SIZE, (void *)NULL);
	USART_BT_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void Debug_CLOCK_init()
{

	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void Debug_PORT_init()
{

	gpio_set_pin_function(PB30, PINMUX_PB30D_SERCOM5_PAD0);

	gpio_set_pin_function(PB31, PINMUX_PB31D_SERCOM5_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void Debug_init(void)
{
	Debug_CLOCK_init();
	usart_async_init(&Debug, SERCOM5, Debug_buffer, DEBUG_BUFFER_SIZE, (void *)NULL);
	Debug_PORT_init();
}

void PWM_1_PORT_init(void)
{

	gpio_set_pin_function(PB08, PINMUX_PB08E_TC4_WO0);
}

void PWM_1_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC4);
	_gclk_enable_channel(TC4_GCLK_ID, CONF_GCLK_TC4_SRC);
}

void PWM_1_init(void)
{
	PWM_1_CLOCK_init();
	PWM_1_PORT_init();
	pwm_init(&PWM_1, TC4, _tc_get_pwm());
}

void PWM_0_PORT_init(void)
{

	gpio_set_pin_function(PB10, PINMUX_PB10E_TC5_WO0);
}

void PWM_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC5);
	_gclk_enable_channel(TC5_GCLK_ID, CONF_GCLK_TC5_SRC);
}

void PWM_0_init(void)
{
	PWM_0_CLOCK_init();
	PWM_0_PORT_init();
	pwm_init(&PWM_0, TC5, _tc_get_pwm());
}

void TIMER_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TCC0);
	_gclk_enable_channel(TCC0_GCLK_ID, CONF_GCLK_TCC0_SRC);
}

void TIMER_0_init(void)
{
	TIMER_0_CLOCK_init();
	timer_init(&TIMER_0, TCC0, _tcc_get_timer());
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA02

	gpio_set_pin_level(PWM_OUT,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PWM_OUT, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PWM_OUT, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA12

	gpio_set_pin_level(PIN_LED_RED,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PIN_LED_RED, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PIN_LED_RED, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA13

	gpio_set_pin_level(PIN_LED_BLUE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PIN_LED_BLUE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PIN_LED_BLUE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA15

	// Set pin direction to input
	gpio_set_pin_direction(PIN_BT_WAKE, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(PIN_BT_WAKE,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(PIN_BT_WAKE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB15

	gpio_set_pin_level(PIN_LED_GREEN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(PIN_LED_GREEN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PIN_LED_GREEN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB17

	gpio_set_pin_level(PIN_BT_RST,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(PIN_BT_RST, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(PIN_BT_RST, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();

	USART_LORA_init();

	I2C_0_init();
	USART_BT_init();
	Debug_init();

	PWM_1_init();

	PWM_0_init();

	TIMER_0_init();
}
