/*
 * 005_spi_txonly_arduino.c
 *
 *  Created on: Oct 14, 2020
 *      Author: mangwi01
 */

/*
 * SPI2_MOSI : PB15
 * SPI2_MISO : PB14
 * SPI2_SCK  : PB10
 * SPI2_NSS  : PB9
 * Alternate function AF5
 */
#include <string.h>
#include "stm32f407xx.h"

uint8_t gl_u8exti0_priority = 0x5U;

void spi_gpio_init();
void spi2_init();
void init_button();
void delay(void);

int main() {
	spi_gpio_init();
	spi2_init();
	spi_soe_config(SPI2, ENABLE);
	init_button();
	for (;;)
		;
	return 0;
}

void EXTI0_IRQHandler(void) {
	delay();
	gpio_irq_handling(GPIO_PIN_0);
	uint8_t loc_au8tx_buffer[] = { 'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r',
			'l', 'd', '\n' };

	spi_control(SPI2, ENABLE);
	/*!< First send length information*/
	uint8_t loc_u8data_len = (uint8_t) sizeof(loc_au8tx_buffer);
	spi_send_data(SPI2, &loc_u8data_len, sizeof(loc_u8data_len));

	/*!< Send data*/
	spi_send_data(SPI2, loc_au8tx_buffer, sizeof(loc_au8tx_buffer));

	/*!< Let's confirm that spi is not busy*/
	while (FLAG_SET == spi_get_flag_status(SPI2, SPI_BSY_FLAG))
		;
	spi_control(SPI2, DISABLE);
}

void spi_gpio_init() {
	/*!< SPI2_SCK  : PB13*/
	gpio_pin_config_t loc_sspi2_pin_config = {
	GPIO_PIN_13,
	GPIO_MODER_AF,
	GPIO_OSPEEDR_VERY_HIGH,
	GPIO_OTYPER_PP,
	GPIO_PUPDR_PU,
	GPIO_AF5 };

	gpio_handle_t loc_sgpiof_handle = { GPIOB, &loc_sspi2_pin_config };
	gpio_init(&loc_sgpiof_handle);

	/*!< SPI2_MOSI : PB15*/
	loc_sspi2_pin_config.pin_number = GPIO_PIN_15;
	gpio_init(&loc_sgpiof_handle);

	/*!< SPI2_MISO : PB14*/
	//loc_sspi2_pin_config.pin_number = GPIO_PIN_14;
	//gpio_init (&loc_sgpiof_handle);
	/*!< SPI2_NSS  : PB12*/
	loc_sspi2_pin_config.pin_number = GPIO_PIN_12;
	gpio_init(&loc_sgpiof_handle);
}

void spi2_init() {
	spi_config_t loc_sspi_config = {
	SPI_MSTR_MASTER,
	SPI_DIRECTION_FD,
	SPI_BR_DIV4,
	SPI_DFF_8BITS,
	SPI_CPOL_CLCK_LOW,
	SPI_CPHA_CAPTURE_ON_TRANSIT1,
	SPI_SSM_DI, 0 };
	spi_handle_t loc_sspi2_handle = { SPI2, &loc_sspi_config };
	spi_init(&loc_sspi2_handle);
}

void init_button() {
	gpio_pin_config_t loc_button_pin_config = { GPIO_PIN_0, GPIO_MODER_IT_FT,
	GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
	gpio_handle_t loc_sgpio_handle_button = { GPIOA, &loc_button_pin_config };

	gpio_peri_clock_control(loc_sgpio_handle_button.ptr_sgpio, ENABLE);
	gpio_init(&loc_sgpio_handle_button);

	gpio_irq_config(EXTI0_IRQ_NUMBER, gl_u8exti0_priority, ENABLE);
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFU; ++loc_u32index)
		;
}
