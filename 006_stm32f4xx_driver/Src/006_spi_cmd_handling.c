/*
 * 006_spi_cmd_handling.c
 *
 *  Created on: Oct 28, 2020
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

/*!< command codes*/
#define CMD_LED_CTRL		(0x50U)
#define CMD_SEMSOR_READ		(0x51U)
#define CMD_LED_READ		(0x52U)
#define CMD_PRINT			(0x53U)
#define CMD_ID_READ			(0x54U)

#define LED_ON		(0x01U)
#define LED_OFF		(0x00U)

/*!< arduino analog pins*/
#define ANALOG_PIN0		(0U)
#define ANALOG_PIN1		(1U)
#define ANALOG_PIN2		(2U)
#define ANALOG_PIN3		(3U)
#define ANALOG_PIN4		(4U)

/*!< arduino led*/
#define LED_PIN		(9U)

#define ACK_BYTE		(0xF5U)

#define MAX_NUMBER_OF_HANDLER		(5U)
typedef void (*handler_func_t)(void);

uint8_t gl_u8exti0_priority = 0x5U;

void spi_gpio_init();
void spi2_init();
void init_button();
void delay(void);
uint8_t spi_verify_response(uint8_t arg_u8ack);

void spi_send_led_on_cmd(void);
void spi_send_sensor_read_cmd(void);
void spi_send_led_read_cmd(void);
void spi_send_print_cmd(void);
void spi_send_id_read_cmd(void);

handler_func_t gl_apfhandler_func[MAX_NUMBER_OF_HANDLER] = {
		spi_send_led_on_cmd, spi_send_sensor_read_cmd, spi_send_led_read_cmd,
		spi_send_print_cmd, spi_send_id_read_cmd };

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
	static uint8_t loc_su8current_handler = 0U;
	gpio_irq_handling(GPIO_PIN_0);
	gl_apfhandler_func[loc_su8current_handler]();
	loc_su8current_handler = (loc_su8current_handler + 1)
			% MAX_NUMBER_OF_HANDLER;
}

uint8_t spi_verify_response(uint8_t arg_u8ack) {
	uint8_t loc_u8result = 0x00U;
	if (ACK_BYTE == arg_u8ack) {
		loc_u8result = 0x01;
	}
	return loc_u8result;
}

void spi_send_led_read_cmd(void) {

}

void spi_send_print_cmd(void) {

}

void spi_send_id_read_cmd(void) {

}

void spi_send_sensor_read_cmd(void) {
	delay();
	uint8_t loc_u8dummy_write = 0xFFU;
	uint8_t loc_u8dummy_read = 0U;
	uint8_t loc_u8cmd_code = CMD_SEMSOR_READ;

	spi_control(SPI2, ENABLE);

	/*!< CMD_LED_CTRL <pin_no(1)> <value(1)>*/
	spi_send_data(SPI2, &loc_u8cmd_code, sizeof(loc_u8cmd_code));

	/*!< do dummy read to clear off the RXNE*/
	spi_receive_data(SPI2, &loc_u8dummy_read, sizeof(loc_u8cmd_code));

	/*!< send some dummy bit (1 byte) to fetch the response from slave*/
	spi_send_data(SPI2, &loc_u8dummy_write, sizeof(loc_u8dummy_write));

	uint8_t loc_u8ack = 0U;
	spi_receive_data(SPI2, &loc_u8ack, sizeof(loc_u8ack));
	if (spi_verify_response(loc_u8ack)) {
		// send arguments
		uint8_t loc_au8args[] = { ANALOG_PIN0 };
		spi_send_data(SPI2, loc_au8args, sizeof(loc_au8args));

		/*!< do dummy read to clear off the RXNE*/
		spi_receive_data(SPI2, &loc_u8dummy_read, sizeof(loc_au8args));

		/*!< insert some delay so that slave can be ready*/
		delay();

		/*!< send some dummy bit (1 byte) to fetch the response from slave*/
		spi_send_data(SPI2, &loc_u8dummy_write, sizeof(loc_u8dummy_write));

		uint8_t loc_u8analo_read = 0U;
		spi_receive_data(SPI2, &loc_u8analo_read, sizeof(loc_u8analo_read));
	}

	/*!< Let's confirm that spi is not busy*/
	while (FLAG_SET == spi_get_flag_status(SPI2, SPI_FLAG_BSY))
		;
	spi_control(SPI2, DISABLE);
}

void spi_send_led_on_cmd(void) {
	delay();
	uint8_t loc_u8dummy_write = 0xFFU;
	uint8_t loc_u8dummy_read = 0U;
	uint8_t loc_u8cmd_code = CMD_LED_CTRL;

	spi_control(SPI2, ENABLE);

	/*!< CMD_LED_CTRL <pin_no(1)> <value(1)>*/
	spi_send_data(SPI2, &loc_u8cmd_code, sizeof(loc_u8cmd_code));

	/*!< do dummy read to clear off the RXNE*/
	spi_receive_data(SPI2, &loc_u8dummy_read, sizeof(loc_u8cmd_code));

	/*!< send some dummy bit (1 byte) to fetch the response from slave*/
	spi_send_data(SPI2, &loc_u8dummy_write, sizeof(loc_u8dummy_write));
	uint8_t loc_u8ack = 0U;
	spi_receive_data(SPI2, &loc_u8ack, sizeof(loc_u8ack));
	if (spi_verify_response(loc_u8ack)) {
		// send arguments
		uint8_t loc_au8args[] = { LED_PIN, LED_ON };
		spi_send_data(SPI2, loc_au8args, sizeof(loc_au8args));
	}

	/*!< Let's confirm that spi is not busy*/
	while (FLAG_SET == spi_get_flag_status(SPI2, SPI_FLAG_BSY))
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
