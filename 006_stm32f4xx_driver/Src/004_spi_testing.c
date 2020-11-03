/*
 * 004_spi_testing.c
 *
 *  Created on: Oct 13, 2020
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

void spi_gpio_init ();
void spi2_init ();

int main () {
	spi_gpio_init ();
	spi2_init ();
	/*!< this make the NSSsignal internally high and avoid MODF error*/
	spi_ssi_config (SPI2, ENABLE);
	spi_control (SPI2, ENABLE);

	uint8_t loc_au8tx_buffer [] = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '\n'};
	spi_send_data(SPI2, loc_au8tx_buffer, sizeof(loc_au8tx_buffer));
	/*!< Let's confirm that spi is not busy*/
	while (FLAG_SET == spi_get_flag_status(SPI2, SPI_BSY_FLAG));
	spi_control (SPI2, DISABLE);
	for (;;);
	return 0;
}

void spi_gpio_init () {
	/*!< SPI2_SCK  : PB10*/
	gpio_pin_config_t loc_sspi2_pin_config = {
			GPIO_PIN_10,
			GPIO_MODER_AF,
			GPIO_OSPEEDR_LOW,
			GPIO_OTYPER_PP,
			GPIO_PUPDR_NO,
			GPIO_AF5
	};

	gpio_handle_t loc_sgpiof_handle = {GPIOB, &loc_sspi2_pin_config};
	gpio_init (&loc_sgpiof_handle);

	/*!< SPI2_MOSI : PB15*/
	loc_sspi2_pin_config.pin_number = GPIO_PIN_15;
	gpio_init (&loc_sgpiof_handle);

	/*!< SPI2_MISO : PB14*/
	//loc_sspi2_pin_config.pin_number = GPIO_PIN_14;
	//gpio_init (&loc_sgpiof_handle);

	/*!< SPI2_NSS  : PB9*/
	//loc_sspi2_pin_config.pin_number = GPIO_PIN_9;
	//gpio_init (&loc_sgpiof_handle);
}

void spi2_init () {
	spi_config_t loc_sspi_config = {
			SPI_MSTR_MASTER,
			SPI_DIRECTION_FD,
			SPI_BR_DIV256,
			SPI_DFF_8BITS,
			SPI_CPOL_CLCK_LOW,
			SPI_CPHA_CAPTURE_ON_TRANSIT1,
			SPI_SSM_EN,
			0
	};
	spi_handle_t loc_sspi2_handle = {SPI2, &loc_sspi_config};
	spi_init (&loc_sspi2_handle);
}
