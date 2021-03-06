/*
 * 007_i2c_master_tx_testing.c
 *
 *  Created on: 5 nov. 2020
 *      Author: msw
 */

#include <string.h>
#include "stm32f407xx.h"
void delay(void);

#define MY_ADDR	(0x61U)
#define SLAVE_ADDR	(0x68U)

/*
 * PB6 => SCL
 * PB7 => SDA
 */

i2c_config_t gl_oi2c_config = { I2C_SCL_SPEED_SM, I2C_FM_DUTY_2, MY_ADDR,
I2C_ACK_EN };
i2c_handle_t gl_oi2c1_handle = { I2C1, &gl_oi2c_config };

uint8_t gl_u8exti0_priority = 0x5U;

void i2c_gpio_init() {
	/*!< PB6 => SCL*/
	gpio_pin_config_t loc_oi2c_pin_config = {
	GPIO_PIN_6,
	GPIO_MODER_AF,
	GPIO_OSPEEDR_VERY_HIGH,
	GPIO_OTYPER_OD,
	GPIO_PUPDR_PU,
	GPIO_AF4 };

	gpio_handle_t loc_ogpiof_handle = { GPIOB, &loc_oi2c_pin_config };
	gpio_init(&loc_ogpiof_handle);

	/*!< PB7 => SDA*/
	loc_oi2c_pin_config.pin_number = GPIO_PIN_7;
	gpio_init(&loc_ogpiof_handle);
}

void i2c1_init() {
	i2c_init(&gl_oi2c1_handle);
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
	uint8_t loc_au8some_data[] = "We are testing I2C master Tx\n";
	i2c_master_send_data(&gl_oi2c1_handle, loc_au8some_data,
			strlen((char*) loc_au8some_data), SLAVE_ADDR);

}

int main() {
	/*!< user button init*/
	init_button();

	/*!< i2c pin init*/
	i2c_gpio_init();

	/*!< i2c peropheral init*/
	i2c1_init();

	/*!< enable the i2c peripheral*/
	i2c_control(I2C1, ENABLE);

	for (;;)
		;
	return 0;
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFU; ++loc_u32index)
		;
}
