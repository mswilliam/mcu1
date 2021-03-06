/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include <stdint.h>

#define TWO_BITS (0x03UL)
#define THREE_BITS (0x07UL)
#define FOUR_BITS (0x0FUL)

#define RCC_BASE_ADDR (0x40023800UL)

#define RCC_CR_OFFSET (0x0UL)
#define RCC_CR_ADDR ((RCC_BASE_ADDR) + (RCC_CR_OFFSET))
#define RCC_CR_HSION_BIT (16UL)
#define RCC_CR_HSION_VALUE (1UL) // 1: HSE oscillator ON
#define RCC_CR_HSERDY_BIT (17UL) // Bit 17 HSERDY: HSE clock ready flag
#define RCC_CR_HSERDY_VALUE (1UL) // 1: HSE oscillator ready

#define RCC_CFGR_OFFSET (0x08UL)
#define RCC_CFGR_ADDR ((RCC_BASE_ADDR) + (RCC_CFGR_OFFSET))
#define RCC_CFGR_SW_BIT (0UL)
#define RCC_CFGR_SW_VALUE (0x01UL) // 01: HSE oscillator selected as system clock

#define RCC_CFGR_MCO1_BIT (21UL)
#define RCC_CFGR_MCO1_VALUE (0x02UL) // 10: HSE oscillator clock selected

#define RCC_CFGR_MCO1PRE_BIT (24UL) // Bits 24:26 MCO1PRE: MCO1 prescaler
#define RCC_CFGR_MCO1PRE_VALUE (0x06UL) // 0xx: no division; 100: division by 2; 101: division by 3;
									   // 110: division by 4 //	111: division by 5

#define RCC_AHB1ENR_OFFSET (0x30UL)
#define RCC_AHB1ENR_ADDR ((RCC_BASE_ADDR) + (RCC_AHB1ENR_OFFSET))
#define RCC_AHB1ENR_PORT8_BIT (0UL)
#define RCC_AHB1ENR_PORT8_VALUE (1UL) // 1: IO port A clock enabled

#define GPIOA_BASE_ADDR (0x40020000UL)

#define GPIOA8_MODER_OFFSET (0x0UL)
#define GPIOA8_MODER_ADDR ((GPIOA_BASE_ADDR) + (GPIOA8_MODER_OFFSET))
#define GPIOA8_MODER_BIT (16UL)
#define GPIOA8_MODER_VALUE (0x02UL) // 10: Alternate function mode

#define GPIOA_AFRH_OFFSET (0x24UL)
#define GPIOA_AFRH_ADDR ((GPIOA_BASE_ADDR) + (GPIOA_AFRH_OFFSET))
#define GPIOA_AFRH_BIT (0UL)
#define GPIOA_AFRH_VALUE (0UL) // 0000: AF0

int main(void)
{
	// Configure the Microcontroller clock output 1
	volatile uint32_t * loc_ptr_RCC_CFGR_ADDR = (uint32_t *) RCC_CFGR_ADDR;
	*loc_ptr_RCC_CFGR_ADDR &= ~(TWO_BITS << RCC_CFGR_MCO1_BIT); // clear
	*loc_ptr_RCC_CFGR_ADDR |= (RCC_CFGR_MCO1_VALUE << RCC_CFGR_MCO1_BIT); // set

	// Configure the Microcontroller clock output 1 prescalar
	*loc_ptr_RCC_CFGR_ADDR &= ~(THREE_BITS << RCC_CFGR_MCO1PRE_BIT); // clear
	*loc_ptr_RCC_CFGR_ADDR |= (RCC_CFGR_MCO1PRE_VALUE << RCC_CFGR_MCO1PRE_BIT); // set


	// Select HSE as the system clock source.
	*loc_ptr_RCC_CFGR_ADDR &= ~(TWO_BITS << RCC_CFGR_SW_BIT); // clear
	*loc_ptr_RCC_CFGR_ADDR |= (RCC_CFGR_SW_VALUE << RCC_CFGR_SW_BIT); // set

	// HSE clock enable
	volatile uint32_t * loc_ptr_RCC_CR = (uint32_t *) RCC_CR_ADDR;
	*loc_ptr_RCC_CR |= (RCC_CR_HSION_VALUE << RCC_CR_HSION_BIT);
	while( ! (*loc_ptr_RCC_CR & ( RCC_CR_HSERDY_VALUE << RCC_CR_HSERDY_BIT) ) );

	// Configure PA8 as alternate function AF0
	volatile uint32_t * loc_RCC_AHB1ENR_ADDR = (uint32_t *) RCC_AHB1ENR_ADDR;
	*loc_RCC_AHB1ENR_ADDR |= (RCC_AHB1ENR_PORT8_VALUE << RCC_AHB1ENR_PORT8_BIT); // IO port A clock enable

	volatile uint32_t * loc_GPIOA_MODER_ADDR = (uint32_t *) GPIOA8_MODER_ADDR;
	*loc_GPIOA_MODER_ADDR &= ~(TWO_BITS << GPIOA8_MODER_BIT); // clear mode
	*loc_GPIOA_MODER_ADDR |= (GPIOA8_MODER_VALUE << GPIOA8_MODER_BIT); // set alternate function

	volatile uint32_t * loc_GPIOA_AFRH_ADDR = (uint32_t *) GPIOA_AFRH_ADDR;
	*loc_GPIOA_AFRH_ADDR &= ~(FOUR_BITS << GPIOA_AFRH_BIT); // clear alternate function selection
	*loc_GPIOA_AFRH_ADDR |= (GPIOA_AFRH_VALUE << GPIOA_AFRH_BIT); // set PA8 as alternate function AF0

    /* Loop forever */
	for(;;);
}
