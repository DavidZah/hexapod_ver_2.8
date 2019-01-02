/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define PWM_OUT GPIO(GPIO_PORTA, 2)
#define PA04 GPIO(GPIO_PORTA, 4)
#define PA05 GPIO(GPIO_PORTA, 5)
#define PA06 GPIO(GPIO_PORTA, 6)
#define PA08 GPIO(GPIO_PORTA, 8)
#define PA09 GPIO(GPIO_PORTA, 9)
#define PIN_LED_RED GPIO(GPIO_PORTA, 12)
#define PIN_LED_BLUE GPIO(GPIO_PORTA, 13)
#define PIN_BT_WAKE GPIO(GPIO_PORTA, 15)
#define PB08 GPIO(GPIO_PORTB, 8)
#define PB10 GPIO(GPIO_PORTB, 10)
#define PB13 GPIO(GPIO_PORTB, 13)
#define PB14 GPIO(GPIO_PORTB, 14)
#define PIN_LED_GREEN GPIO(GPIO_PORTB, 15)
#define PIN_BT_RST GPIO(GPIO_PORTB, 17)
#define PB30 GPIO(GPIO_PORTB, 30)
#define PB31 GPIO(GPIO_PORTB, 31)

#endif // ATMEL_START_PINS_H_INCLUDED
