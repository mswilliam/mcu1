/*
 * 007_i2c_master_tx_testing.c
 *
 *  Created on: 5 nov. 2020
 *      Author: msw
 */

#include <string.h>
#include "stm32f407xx.h"
void delay(void);
void i2c_app_callback(i2c_handle_t *arg_poi2c_handler, uint8_t arg_u8evt);

#define SLAVE_ADDR	(0x68U)
#define MY_ADDR	(SLAVE_ADDR)

usart_config_t gl_ouart_config = { USART_MODE_TXRX, USART_STD_BAUD_9600,
USART_STOP_BITS_1, USART_WORD_LEN_8BITS, USART_PARITY_DI,
USART_HW_FLWO_CTRL_NONE };
uint8_t gl_au8rx_buffer[30];
uint8_t gl_au8tx_buffer[30] = "We are testing I2C slave Tx\n";
usart_handle_t gl_ousart_handle = { USART2, &gl_ouart_config, USART_STATE_IDLE,
		gl_au8tx_buffer, gl_au8rx_buffer, 0U, 0U, 0U };

uint8_t gl_u8exti0_priority = 0x5U;
uint8_t gl_u8i2c1_priority = 0x4U;

void uart_gpio_init() {
	/*!< PA2 => TX*/
	gpio_pin_config_t loc_oi2c_pin_config = {
	GPIO_PIN_2,
	GPIO_MODER_AF,
	GPIO_OSPEEDR_VERY_HIGH,
	GPIO_OTYPER_PP,
	GPIO_PUPDR_PU,
	GPIO_AF7 };

	gpio_handle_t loc_ogpioa_handle = { GPIOA, &loc_oi2c_pin_config };
	gpio_init(&loc_ogpioa_handle);

	/*!< PA3 => RX*/
	loc_oi2c_pin_config.pin_number = GPIO_PIN_7;
	gpio_init(&loc_ogpioa_handle);
}

void uart2_init() {
	usart_init(&gl_ousart_handle);
}

void init_button() {
	gpio_pin_config_t loc_button_pin_config = { GPIO_PIN_0, GPIO_MODER_IT_FT,
	GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
	gpio_handle_t loc_sgpio_handle_button = { GPIOA, &loc_button_pin_config };

	gpio_peri_clock_control(loc_sgpio_handle_button.ptr_sgpio, ENABLE);
	gpio_init(&loc_sgpio_handle_button);

	gpio_irq_config(EXTI0_IRQ_NUMBER, gl_u8exti0_priority, ENABLE);
}

void EXTI0_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_0);
	/*!< send some data*/
	usart_send_data(&gl_ousart_handle, gl_au8tx_buffer, strlen((const char*) gl_au8tx_buffer));
}

void I2C1_EV_IRQHandler(void) {
	//i2c_ev_irq_handler(&gl_ousart_handle);
}

void I2C1_ER_IRQHandler(void) {
	//i2c_er_irq_handler(&gl_ousart_handle);
}

int main() {
	/*!< user button init*/
	init_button();

	/*!< uart pin init*/
	uart_gpio_init();

	/*!< uart peropheral init*/
	uart2_init();

	/*!< uart enable*/
	usart_control_peripheral(USART2, ENABLE);


	for (;;)
		;

	return 0;
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFU; ++loc_u32index)
		;
}
