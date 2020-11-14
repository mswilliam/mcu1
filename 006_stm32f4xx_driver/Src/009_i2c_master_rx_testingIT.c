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
uint8_t gl_u8i2c1_priority = 0x4U;

void i2c_app_callback(i2c_handle_t *arg_poi2c_handler, uint8_t arg_u8evt) {
	if (I2C_ERROR_AF == arg_u8evt) {
		/*!< master ACK failed >*/
		i2c_close_send_data(arg_poi2c_handler);

		/*!< generate stop condition >*/
		i2c_generate_stop_condition(arg_poi2c_handler->m_poi2c_reg);

		/*!< Hang infinitr loop >*/
		for(;;);
	}
}



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

void I2C1_EV_IRQHandler (void) {
	i2c_ev_irq_handler(&gl_oi2c1_handle);
}

void I2C1_ER_IRQHandler (void) {
	i2c_er_irq_handler(&gl_oi2c1_handle);
}

void EXTI0_IRQHandler(void) {
	gpio_irq_handling(GPIO_PIN_0);

	uint8_t loc_u8commande_code = 0x51U;

	uint8_t loc_u8len = 0x0U;

	uint8_t loc_au8buffer[256] = "We are testing I2C master Tx\n";

	while(I2C_READY != i2c_master_send_data_it(&gl_oi2c1_handle, &loc_u8commande_code, 1, SLAVE_ADDR, I2C_REPEAT_START_YES));

	while(I2C_READY != i2c_master_receive_data_it(&gl_oi2c1_handle, &loc_u8len, 1, SLAVE_ADDR, I2C_REPEAT_START_YES));

	loc_u8commande_code = 0x52;
	while(I2C_READY != i2c_master_send_data_it(&gl_oi2c1_handle, &loc_u8commande_code, 1, SLAVE_ADDR, I2C_REPEAT_START_YES));


	while(I2C_READY != i2c_master_receive_data_it(&gl_oi2c1_handle, loc_au8buffer, loc_u8len, SLAVE_ADDR, I2C_REPEAT_START_NO));
}

int main() {
	/*!< user button init*/
	init_button();

	/*!< i2c pin init*/
	i2c_gpio_init();

	/*!< i2c peropheral init*/
	i2c1_init();

	/*!< i2c interrupt configuration*/
	i2c_irq_config(I2C1_EV_IRQ_NUMBER, gl_u8i2c1_priority, ENABLE);
	i2c_irq_config(I2C1_ER_IRQ_NUMBER, gl_u8i2c1_priority, ENABLE);

	/*!< enable the i2c peripheral*/
	i2c_control(I2C1, ENABLE);

	/*!< anable ack*/
	i2c_manage_ack(I2C1, ENABLE);
	for (;;)
		;
	return 0;
}

void delay(void) {
	for (uint32_t loc_u32index = 0; loc_u32index < 0xFFFFU; ++loc_u32index)
		;
}
