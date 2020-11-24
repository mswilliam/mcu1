/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 23 nov. 2020
 *      Author: msw
 */

#include "stm32f407xx_rcc_driver.h"

uint32_t rcc_get_pll_output_clk() {
	uint32_t loc_u32pll_output_clk = 0U;
	return loc_u32pll_output_clk;
}

uint32_t rcc_get_pclk_value(uint8_t arg_u8apb_bus_number) {
	uint32_t loc_u32pclk1 = 0U;
	uint8_t loc_u8clk_src = (uint8_t) ((RCC->CFGR >> RCC_CFGR_SW1) & TWO_BIT);
	/*!< find the clock source*/
	if (IS_HSE_SYSTEM_CLK == loc_u8clk_src) {
		loc_u32pclk1 = 8000000U; // 8MHZ
	} else if (IS_HSI_SYSTEM_CLK == loc_u8clk_src) {
		loc_u32pclk1 = 16000000U; //16MHZ
	} else if (IS_PLL_SYSTEM_CLK == loc_u8clk_src) {
		loc_u32pclk1 = rcc_get_pll_output_clk();
	}

	uint8_t loc_u8AHB_prescaler = (uint8_t) ((RCC->CFGR >> RCC_CFGR_HPRE)
			& FOUR_BIT);
	if (loc_u8AHB_prescaler > 7U) {
		uint16_t loc_au16AHBprescaler_div[] = { 2U, 4U, 8U, 16U, 64U, 128U,
				256U, 512U };
		loc_u32pclk1 /=
				loc_au16AHBprescaler_div[loc_u8AHB_prescaler & THREE_BIT];
	}

	uint8_t loc_u8APB_prescaler = 0U;
	if(APB1 == arg_u8apb_bus_number) {
		loc_u8APB_prescaler = (uint8_t) ((RCC->CFGR >> RCC_CFGR_PPRE1)
				& THREE_BIT);
	} else if(APB2 == arg_u8apb_bus_number) {
		loc_u8APB_prescaler = (uint8_t) ((RCC->CFGR >> RCC_CFGR_PPRE2)
				& THREE_BIT);
	}

	if (loc_u8APB_prescaler > 3U) {
		uint8_t loc_u8APB1_prescaler_div[] = { 2U, 4U, 8U, 16U };
		loc_u32pclk1 /= loc_u8APB1_prescaler_div[loc_u8APB_prescaler & TWO_BIT];
	}

	return loc_u32pclk1;
}
