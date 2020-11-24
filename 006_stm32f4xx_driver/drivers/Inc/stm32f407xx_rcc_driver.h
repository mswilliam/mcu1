/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 23 nov. 2020
 *      Author: msw
 */
#include "stm32f407xx.h"

/*
 * @apb_bus_number
 * Definition of the possible values for APB bus number
 */
#define APB1	(0U)
#define APB2	(1U)

uint32_t rcc_get_pll_output_clk();

/***************************************************************************************
 * @fn					- rcc_get_pclk_value
 *
 * @brief				- get the clock frequence value of the given APH bus number
 *
 * @param[in]			- APH bus number, possible value from @apb_bus_number
 * @param[in]			- Base address of the data to be sent
 * @param[in]			- size of the data to be sent
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
uint32_t rcc_get_pclk_value(uint8_t arg_u8apb_bus_number);

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_



#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
