/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 2 nov. 2020
 *      Author: ange
 */

#include "stm32f407xx_i2c_driver.h"

void i2c_peri_clock_control(i2c_reg_t *arg_ptr_i2c,
		uint8_t arg_u8enable_or_disable) {
	switch ((int32_t) arg_ptr_i2c) {
	case (int32_t) I2C1:
		if (ENABLE == arg_u8enable_or_disable) {
			I2C1_PCLK_EN();
		} else if (DISABLE == arg_u8enable_or_disable) {
			I2C1_PCLK_DI();
		}
		break;
	case (int32_t) I2C2:
		if (ENABLE == arg_u8enable_or_disable) {
			I2C2_PCLK_EN();
		} else if (DISABLE == arg_u8enable_or_disable) {
			I2C2_PCLK_DI();
		}
		break;
	case (int32_t) I2C3:
		if (ENABLE == arg_u8enable_or_disable) {
			I2C3_PCLK_EN();
		} else if (DISABLE == arg_u8enable_or_disable) {
			I2C3_PCLK_DI();
		}
		break;
	default:
		break;
	}
}

void i2c_de_init(i2c_reg_t *arg_ptr_i2c) {
	switch ((uint32_t) arg_ptr_i2c) {
	case (uint32_t) I2C1:
		I2C1_RST();
		break;
	case (uint32_t) I2C2:
		I2C2_RST();
		break;
	case (uint32_t) I2C3:
		I2C3_RST();
		break;
	default:
		break;
	}
}

void i2c_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
		uint8_t arg_enable_or_disable) {
	uint8_t loc_u8nvic_iser_index = arg_irq_number / 32U;
	uint8_t loc_u8nvic_iser_shift = arg_irq_number % 32U;
	if (ENABLE == arg_enable_or_disable) {
		NVIC_ISER[loc_u8nvic_iser_index] |= (SET << loc_u8nvic_iser_shift);
		uint8_t loc_u8nvic_iser_ipr_index = arg_irq_number / 4U;
		uint8_t loc_u8nvic_ipr_shift = (arg_irq_number % 4U) * 8U + 4U;
		NVIC_IPR[loc_u8nvic_iser_ipr_index] |= (arg_irq_priority
				<< loc_u8nvic_ipr_shift);
	} else if (DISABLE == arg_enable_or_disable) {
		NVIC_ICER[loc_u8nvic_iser_index] |= (SET << loc_u8nvic_iser_shift);
	}
}

void i2c_control(i2c_reg_t *arg_ptr_i2c, uint8_t arg_u8enable_or_disable) {
	if (ENABLE == arg_u8enable_or_disable) {
		arg_ptr_i2c->CR[1] |= (SET << I2C_CR1_PE);
	} else if (DISABLE == arg_u8enable_or_disable) {
		arg_ptr_i2c->CR[1] &= ~(SET << I2C_CR1_PE);
	}
}
