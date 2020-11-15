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

/*
 * PB6 => SCL
 * PB7 => SDA
 */

i2c_config_t gl_oi2c_config = { I2C_SCL_SPEED_SM, I2C_FM_DUTY_2, MY_ADDR,
I2C_ACK_EN };
i2c_handle_t gl_oi2c1_handle = { I2C1, &gl_oi2c_config };
uint8_t gl_au8buffer[30] = "We are testing I2C slave Tx\n";

uint8_t gl_u8exti0_priority = 0x5U;
uint8_t gl_u8i2c1_priority = 0x4U;

void i2c_app_callback(i2c_handle_t *arg_poi2c_handler, uint8_t arg_u8evt) {
	static uint8_t gl_u8count = 0U;
	static uint8_t gl_u8commande_code = 0U;
	if(I2C_EV_DATA_REQ == arg_u8evt) {
		/*!< master want some data, the slave has to send it>*/
		if (0x51U == gl_u8commande_code) {
					/*!< send the length information to master>*/
					i2c_slave_send_data(arg_poi2c_handler->m_poi2c_reg, strlen((const char *)gl_au8buffer));
				} else if (0x52U == gl_u8commande_code) {
					/*!< send the the content of the tx buffer>*/
					i2c_slave_send_data(arg_poi2c_handler->m_poi2c_reg, gl_au8buffer[gl_u8count++]);
				}
	} else if(I2C_EV_DATA_RCV == arg_u8evt) {
		/*!< data is waiting for the slave to read. slave has to read it>*/
		gl_u8commande_code = i2c_slave_receive_data(arg_poi2c_handler->m_poi2c_reg);
	} else if (I2C_ERROR_AF == arg_u8evt) {
		/*!< this happed only during slave txing.
		 * Master has sent the NACK. slave should
		 * understand that slave does not need moe data>*/
		gl_u8commande_code = 0xFFU;
		gl_u8count = 0;
	} else if(I2C_EV_STOP== arg_u8evt) {
		/*!< This happened only during slave reception.
		 * Master has ended the I2C communication with the slave.>*/
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

void I2C1_EV_IRQHandler(void) {
	i2c_ev_irq_handler(&gl_oi2c1_handle);
}

void I2C1_ER_IRQHandler(void) {
	i2c_er_irq_handler(&gl_oi2c1_handle);
}

int main() {
	/*!< i2c pin init*/
	i2c_gpio_init();

	/*!< i2c peropheral init*/
	i2c1_init();

	/*!< i2c interrupt configuration*/
	i2c_irq_config(I2C1_EV_IRQ_NUMBER, gl_u8i2c1_priority, ENABLE);
	i2c_irq_config(I2C1_ER_IRQ_NUMBER, gl_u8i2c1_priority, ENABLE);

	i2c_irq_slave_manage_callback_events (I2C1, ENABLE);

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
