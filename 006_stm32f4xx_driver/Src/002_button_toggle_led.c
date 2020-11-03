/*
 * 002_button_toggle_led.c
 *
 *  Created on: 1 oct. 2020
 *      Author: mangwi01
 */

#include "stm32f407xx.h"

void delay(void);

int main(void) {
	gpio_pin_config_t loc_sled3_pin_config = { GPIO_PIN_13, GPIO_MODER_OUT,
			GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
	gpio_handle_t loc_sgpio_handle_led3 = { GPIOD, &loc_sled3_pin_config };

	gpio_pin_config_t loc_button_pin_config = { GPIO_PIN_0, GPIO_MODER_IT_FT,
			GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
	gpio_handle_t loc_sgpio_handle_button = { GPIOA, &loc_button_pin_config };

	gpio_peri_clock_control(loc_sgpio_handle_led3.ptr_sgpio, ENABLE);
	gpio_init(&loc_sgpio_handle_led3);

	gpio_peri_clock_control(loc_sgpio_handle_button.ptr_sgpio, ENABLE);
	gpio_init(&loc_sgpio_handle_button);

	gpio_write_to_output_pin(loc_sgpio_handle_led3.ptr_sgpio,
			loc_sgpio_handle_led3.ptr_sconfig->pin_number, GPIO_PIN_RESET);
	for (;;) {
		if (GPIO_PIN_SET
				== gpio_read_from_input_pin(loc_sgpio_handle_button.ptr_sgpio,
						loc_sgpio_handle_button.ptr_sconfig->pin_number)) {
			gpio_toggle_output_pin(loc_sgpio_handle_led3.ptr_sgpio,
					loc_sgpio_handle_led3.ptr_sconfig->pin_number);
			delay();
		} else {
			gpio_write_to_output_pin(loc_sgpio_handle_led3.ptr_sgpio,
					loc_sgpio_handle_led3.ptr_sconfig->pin_number,
					GPIO_PIN_RESET);
		}
	}
	return 0;
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFFU; ++loc_u32index)
		;
}

