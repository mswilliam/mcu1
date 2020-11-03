/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 24, 2020
 *      Author: mangwi01
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Perioheral Clock setup
 */
void gpio_peri_clock_control(gpio_reg_t *arg_ptr_gpio,
		uint8_t arg_enable_or_disable) {

	switch ((uint32_t) arg_ptr_gpio) {
	case (uint32_t) GPIOA: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOA_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOA_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOB: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOB_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOB_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOC: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOC_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOC_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOD: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOD_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOD_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOE: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOE_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOE_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOF: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOF_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOF_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOG: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOG_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOG_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOH: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOH_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOH_PCLK_DI();
		}
	}
		break;
	case (uint32_t) GPIOI: {
		if (ENABLE == arg_enable_or_disable) {
			GPIOI_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			GPIOI_PCLK_DI();
		}
	}
		break;
	}
}

/*
 * Init and De-init
 */
void gpio_init(gpio_handle_t *arg_ptr_gpio_handle) {
	/*!< Enables peripheral clock for the gpio*/
	gpio_peri_clock_control(arg_ptr_gpio_handle->ptr_sgpio, ENABLE);

	/*!< 1. configure the mode of the gpio pin*/
	switch (arg_ptr_gpio_handle->ptr_sconfig->pin_mode) {
	case GPIO_MODER_IN:
	case GPIO_MODER_OUT:
	case GPIO_MODER_AF:
	case GPIO_MODER_ANALOG: {
		arg_ptr_gpio_handle->ptr_sgpio->MODER &= ~(TWO_BIT
				<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));
		arg_ptr_gpio_handle->ptr_sgpio->MODER |=
				(arg_ptr_gpio_handle->ptr_sconfig->pin_mode
						<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));
	}
		break;
	case GPIO_MODER_IT_FT:
	case GPIO_MODER_IT_RT:
	case GPIO_MODER_RFT: {
		if (GPIO_MODER_IT_FT == arg_ptr_gpio_handle->ptr_sconfig->pin_mode) {
			/*!< 1. Configure the FTSR*/
			EXTI->FTSR |= (SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
			EXTI->RTSR &=
					~(SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
		} else if (GPIO_MODER_IT_RT
				== arg_ptr_gpio_handle->ptr_sconfig->pin_mode) {
			/*!< 1. Configure the RTSR*/
			EXTI->RTSR |= (SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
			EXTI->FTSR &=
					~(SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
		} else if (GPIO_MODER_IT_RT
				== arg_ptr_gpio_handle->ptr_sconfig->pin_mode) {
			/*!< 1. Configure the RTSR & RTSR*/
			EXTI->FTSR |= (SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
			EXTI->RTSR |= (SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
		}
		/*!< 2. Configure the gpio port selection in SYSCFG_EXTICR*/
		uint8_t loc_u8index_syscfg_reg =
				arg_ptr_gpio_handle->ptr_sconfig->pin_number / 0x04U;
		uint8_t loc_u8shift = (arg_ptr_gpio_handle->ptr_sconfig->pin_number
				% 0x04U) * 0x04U;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[loc_u8index_syscfg_reg] =
				(GPIO_ADDR_TO_PORT(arg_ptr_gpio_handle->ptr_sgpio)
						<< loc_u8shift);
		/*!< 3. Enable the EXTI interrupt delivery using IMR: Interrupt Mask Register*/
		EXTI->IMR |= (SET << arg_ptr_gpio_handle->ptr_sconfig->pin_number);
	}
		break;
	}

	if (GPIO_MODER_OUT == arg_ptr_gpio_handle->ptr_sconfig->pin_mode) {
		// 2. configure the speed
		arg_ptr_gpio_handle->ptr_sgpio->OSPEEDR &= ~(TWO_BIT
				<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));
		arg_ptr_gpio_handle->ptr_sgpio->OSPEEDR |=
				(arg_ptr_gpio_handle->ptr_sconfig->pin_mode
						<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));

		// 3. configure the otype
		arg_ptr_gpio_handle->ptr_sgpio->OTYPER &= ~(ONE_BIT
				<< arg_ptr_gpio_handle->ptr_sconfig->pin_number);
		arg_ptr_gpio_handle->ptr_sgpio->OTYPER |=
				(arg_ptr_gpio_handle->ptr_sconfig->pin_otype
						<< arg_ptr_gpio_handle->ptr_sconfig->pin_number);
	}

	// 4. configure the pupd setting
	arg_ptr_gpio_handle->ptr_sgpio->PUPDR &= ~(TWO_BIT
			<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));
	arg_ptr_gpio_handle->ptr_sgpio->PUPDR |=
			(arg_ptr_gpio_handle->ptr_sconfig->pin_pupd
					<< (2 * arg_ptr_gpio_handle->ptr_sconfig->pin_number));

	// 5. configure the alt functionality.
	uint8_t loc_index = arg_ptr_gpio_handle->ptr_sconfig->pin_number / 8;
	uint8_t loc_pin_number_in_af_array =
			arg_ptr_gpio_handle->ptr_sconfig->pin_number % 8;
	arg_ptr_gpio_handle->ptr_sgpio->AFR[loc_index] &= ~(FOUR_BIT
			<< (4 * loc_pin_number_in_af_array));
	arg_ptr_gpio_handle->ptr_sgpio->AFR[loc_index] |=
			(arg_ptr_gpio_handle->ptr_sconfig->pin_afr
					<< (4 * loc_pin_number_in_af_array));
}

void gpio_de_init(gpio_reg_t *arg_ptr_gpio) {
	switch ((uint32_t) arg_ptr_gpio) {
	case (uint32_t) GPIOA: {
		GPIOA_RST();
	}
		break;
	case (uint32_t) GPIOB: {
		GPIOB_RST();
	}
		break;
	case (uint32_t) GPIOC: {
		GPIOC_RST();
	}
		break;
	case (uint32_t) GPIOD: {
		GPIOD_RST();
	}
		break;
	case (uint32_t) GPIOE: {
		GPIOE_RST();
	}
		break;
	case (uint32_t) GPIOF: {
		GPIOF_RST();
	}
		break;
	case (uint32_t) GPIOG: {
		GPIOG_RST();
	}
		break;
	case (uint32_t) GPIOH: {
		GPIOH_RST();
	}
		break;
	case (uint32_t) GPIOI: {
		GPIOI_RST();
	}
		break;
	}
}

/*
 * Read and write
 */
uint8_t gpio_read_from_input_pin(gpio_reg_t *arg_ptr_gpio,
		uint8_t arg_pin_number) {
	return (uint8_t) ((arg_ptr_gpio->IDR >> arg_pin_number) & (0x01U));
}

uint16_t gpio_read_from_input_port(gpio_reg_t *arg_ptr_gpio) {
	return (uint16_t) arg_ptr_gpio->IDR;
}

void gpio_write_to_output_pin(gpio_reg_t *arg_ptr_gpio, uint8_t arg_pin_number,
		uint8_t arg_value) {
	if (GPIO_PIN_RESET == arg_value) {
		arg_ptr_gpio->ODR &= ~(GPIO_PIN_SET << arg_pin_number);
	} else {
		arg_ptr_gpio->ODR |= (GPIO_PIN_SET << arg_pin_number);
	}
}

void gpio_write_to_output_port(gpio_reg_t *arg_ptr_gpio, uint16_t arg_data) {
	arg_ptr_gpio->ODR = arg_data;
}

void gpio_toggle_output_pin(gpio_reg_t *arg_ptr_gpio, uint8_t arg_pin_number) {
	arg_ptr_gpio->ODR ^= (GPIO_PIN_SET << arg_pin_number);
}

/*
 * IRQ Configuration and ISR handling
 */
void gpio_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
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

void gpio_irq_handling(uint8_t arg_pin_number) {
	EXTI->PR |= (SET << arg_pin_number);
}
