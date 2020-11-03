/*
 * 003_button_interrupt_toggle_led.c
 *
 *  Created on: 4 oct. 2020
 *      Author: mangwi01
 */

#include "stm32f407xx.h"

void delay(void);

gpio_pin_config_t gl_sled3_pin_config = { GPIO_PIN_13, GPIO_MODER_OUT,
		GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
gpio_handle_t gl_sgpio_handle_led3 = { GPIOD, &gl_sled3_pin_config };
uint8_t gl_u8exti0_priority = 0x5U;

void EXTI0_IRQHandler(void) {
	delay();
	gpio_irq_handling(GPIO_PIN_0);
	gpio_toggle_output_pin(gl_sgpio_handle_led3.ptr_sgpio,
			gl_sgpio_handle_led3.ptr_sconfig->pin_number);
}

void EXTI1_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_4);
}

int main(void) {
	gpio_pin_config_t loc_button_pin_config = { GPIO_PIN_0, GPIO_MODER_IT_FT,
			GPIO_OSPEEDR_VERY_HIGH, GPIO_OTYPER_PP, GPIO_PUPDR_NO, GPIO_AF0 };
	gpio_handle_t loc_sgpio_handle_button = { GPIOA, &loc_button_pin_config };

	gpio_peri_clock_control(gl_sgpio_handle_led3.ptr_sgpio, ENABLE);
	gpio_init(&gl_sgpio_handle_led3);

	gpio_peri_clock_control(loc_sgpio_handle_button.ptr_sgpio, ENABLE);
	gpio_init(&loc_sgpio_handle_button);

	gpio_irq_config(EXTI0_IRQ_NUMBER, gl_u8exti0_priority, ENABLE);

	gpio_write_to_output_pin(gl_sgpio_handle_led3.ptr_sgpio,
			gl_sgpio_handle_led3.ptr_sconfig->pin_number, GPIO_PIN_RESET);

	for (;;)
		;
	return 0;
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFU; ++loc_u32index)
		;
}

