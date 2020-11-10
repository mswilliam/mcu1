/*
 * stm32f407xx.h
 *
 *  Created on: Sep 23, 2020
 *      Author: mangwi01
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR                         (0x08000000UL)
#define SRAM1_BASE_ADDR                         (0x02000000UL)
#define SRAM2_BASE_ADDR                         (0x2001C000UL)
#define ROM_BASE_ADDR                           (0x1FFF0000UL)
#define SRAM_BASE_ADDR                          (SRAM1_BASE_ADDR)

/*
 * NVIC base addresses
 */
#define NVIC_ISER_BASE_ADDR                   (0xE000E100UL)
#define NVIC_ICER_BASE_ADDR                   (0XE000E180UL)
#define NVIC_ISPR_BASE_ADDR                   (0XE000E200UL)
#define NVIC_ICPR_BASE_ADDR                   (0XE000E280UL)
#define NVIC_IABR_BASE_ADDR                   (0xE000E300UL)
#define NVIC_IPR_BASE_ADDR                    (0xE000E400UL)
#define NVIC_STIR_BASE_ADDR                   (0xE000EF00UL)

#define NVIC_ISER ((volatile uint32_t *) NVIC_ISER_BASE_ADDR)
#define NVIC_ICER ((volatile uint32_t *) NVIC_ICER_BASE_ADDR)
#define NVIC_IPR  ((volatile uint32_t *) NVIC_IPR_BASE_ADDR)

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE_ADDR                        (0x40000000UL)
#define APB1_PERIPH_BASE_ADDR                   (PERIPH_BASE_ADDR)
#define APB2_PERIPH_BASE_ADDR                   (0x40010000UL)
#define AHB1_PERIPH_BASE_ADDR                   (0x40020000UL)
#define AHB2_PERIPH_BASE_ADDR                   (0x50000000UL)
#define AHB3_PERIPH_BASE_ADDR                   (0xA0000000UL)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define AHB1_PERIPH_OFFSET_ADDR                 (0x00020000UL)
#define USB_OTG_HS_OFFSET_ADDR                  (0x00020000UL)
#define DMA2D_OFFSET_ADDR                       (0x0000B000UL)
#define ETHERNE_MAC_OFFSET_ADDR                 (0x00008000UL)
#define DMA2_OFFSET_ADDR                        (0x00006400UL)
#define DMA1_OFFSET_ADDR                        (0x00006000UL)
#define BKPSRAM_OFFSET_ADDR                     (0x00004000UL)
#define FLASH_INTERFACE_REGISTER_OFFSET_ADDR    (0x00003C00UL)
#define RCC_OFFSET_ADDR                         (0x00003800UL)
#define CRC_OFFSET_ADDR                         (0x00003000UL)
#define GPIOK_OFFSET_ADDR                       (0x00002800UL)
#define GPIOJ_OFFSET_ADDR                       (0x00002400UL)
#define GPIOI_OFFSET_ADDR                       (0x00002000UL)
#define GPIOH_OFFSET_ADDR                       (0x00001C00UL)
#define GPIOG_OFFSET_ADDR                       (0x00001800UL)
#define GPIOF_OFFSET_ADDR                       (0x00001400UL)
#define GPIOE_OFFSET_ADDR                       (0x00001000UL)
#define GPIOD_OFFSET_ADDR                       (0x00000C00UL)
#define GPIOC_OFFSET_ADDR                       (0x00000800UL)
#define GPIOB_OFFSET_ADDR                       (0x00000400UL)
#define GPIOA_OFFSET_ADDR                       (0x00000000UL)

#define USB_OTG_HS_BASE_ADDR                    ((AHB1_PERIPH_BASE_ADDR) + (USB_OTG_HS_OFFSET_ADDR              ))
#define DMA2D_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (DMA2D_OFFSET_ADDR                   ))
#define ETHERNE_MAC_BASE_ADDR                   ((AHB1_PERIPH_BASE_ADDR) + (ETHERNE_MAC_OFFSET_ADDR             ))
#define DMA2_BASE_ADDR                          ((AHB1_PERIPH_BASE_ADDR) + (DMA2_OFFSET_ADDR                    ))
#define DMA1_BASE_ADDR                          ((AHB1_PERIPH_BASE_ADDR) + (DMA1_OFFSET_ADDR                    ))
#define BKPSRAM_BASE_ADDR                       ((AHB1_PERIPH_BASE_ADDR) + (BKPSRAM_OFFSET_ADDR                 ))
#define FLASH_INTERFACE_REGISTER_BASE_ADDR      ((AHB1_PERIPH_BASE_ADDR) + (FLASH_INTERFACE_REGISTER_OFFSET_ADDR))
#define RCC_BASE_ADDR                           ((AHB1_PERIPH_BASE_ADDR) + (RCC_OFFSET_ADDR                     ))
#define CRC_BASE_ADDR                           ((AHB1_PERIPH_BASE_ADDR) + (CRC_OFFSET_ADDR                     ))
#define GPIOK_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOK_OFFSET_ADDR                   ))
#define GPIOJ_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOJ_OFFSET_ADDR                   ))
#define GPIOI_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOI_OFFSET_ADDR                   ))
#define GPIOH_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOH_OFFSET_ADDR                   ))
#define GPIOG_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOG_OFFSET_ADDR                   ))
#define GPIOF_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOF_OFFSET_ADDR                   ))
#define GPIOE_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOE_OFFSET_ADDR                   ))
#define GPIOD_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOD_OFFSET_ADDR                   ))
#define GPIOC_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOC_OFFSET_ADDR                   ))
#define GPIOB_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOB_OFFSET_ADDR                   ))
#define GPIOA_BASE_ADDR                         ((AHB1_PERIPH_BASE_ADDR) + (GPIOA_OFFSET_ADDR                   ))

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define RNG_OFFSET_ADDR                         (0x00060800UL)
#define HASH_OFFSET_ADDR                        (0x00060400UL)
#define CRYP_OFFSET_ADDR                        (0x00060000UL)
#define DCMI_OFFSET_ADDR                        (0x00050000UL)
#define USB_OTG_FS_OFFSET_ADDR                  (0x00000000UL)
#define RNG_BASE_ADDR                           ((AHB2_PERIPH_BASE_ADDR) + (RNG_OFFSET_ADDR))
#define HASH_BASE_ADDR                          ((AHB2_PERIPH_BASE_ADDR) + (HASH_OFFSET_ADDR))
#define CRYP_BASE_ADDR                          ((AHB2_PERIPH_BASE_ADDR) + (CRYP_OFFSET_ADDR))
#define DCMI_BASE_ADDR                          ((AHB2_PERIPH_BASE_ADDR) + (DCMI_OFFSET_ADDR))
#define USB_OTG_FS_BASE_ADDR                    ((AHB2_PERIPH_BASE_ADDR) + (USB_OTG_FS_OFFSET_ADDR))

/*
 * Base addresses of peripherals which are hanging on AHB3 bus
 */

#define FSMC_OFFSET_ADDR                        (0x00000000UL)
#define FSMC_BASE_ADDR                          ((AHB3_PERIPH_BASE_ADDR) + (FSMC_OFFSET_ADDR))

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define UART8_OFFSET_ADDR                       (0x00007C00UL)
#define UART7_OFFSET_ADDR                       (0x00007800UL)
#define DAC_OFFSET_ADDR                         (0x00007400UL)
#define PWR_OFFSET_ADDR                         (0x00007000UL)
#define CAN2_OFFSET_ADDR                        (0x00006800UL)
#define CAN1_OFFSET_ADDR                        (0x00006400UL)
#define I2C3_OFFSET_ADDR                        (0x00005C00UL)
#define I2C2_OFFSET_ADDR                        (0x00005800UL)
#define I2C1_OFFSET_ADDR                        (0x00005400UL)
#define UART5_OFFSET_ADDR                       (0x00005000UL)
#define UART4_OFFSET_ADDR                       (0x00004C00UL)
#define USART3_OFFSET_ADDR                      (0x00004800UL)
#define USART2_OFFSET_ADDR                      (0x00004400UL)
#define I2S3EXT_OFFSET_ADDR                     (0x00004000UL)
#define SPI3_OR_I2S3_OFFSET_ADDR                (0x00003C00UL)
#define SPI2_OR_I2S2_OFFSET_ADDR                (0x00003800UL)
#define I2S2EXT_OFFSET_ADDR                     (0x00003400UL)
#define IWDG_OFFSET_ADDR                        (0x00003000UL)
#define WWDG_OFFSET_ADDR                        (0x00002C00UL)
#define RTC_OFFSET_ADDR                         (0x00002800UL)
#define TIM14_OFFSET_ADDR                       (0x00002000UL)
#define TIM13_OFFSET_ADDR                       (0x00001C00UL)
#define TIM12_OFFSET_ADDR                       (0x00001800UL)
#define TIM7_OFFSET_ADDR                        (0x00001400UL)
#define TIM6_OFFSET_ADDR                        (0x00001000UL)
#define TIM5_OFFSET_ADDR                        (0x00000C00UL)
#define TIM4_OFFSET_ADDR                        (0x00000800UL)
#define TIM3_OFFSET_ADDR                        (0x00000400UL)
#define TIM2_OFFSET_ADDR                        (0x00000000UL)

#define UART8_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (UART8_OFFSET_ADDR       ))
#define UART7_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (UART7_OFFSET_ADDR       ))
#define DAC_BASE_ADDR                           ((APB1_PERIPH_BASE_ADDR) + (DAC_OFFSET_ADDR         ))
#define PWR_BASE_ADDR                           ((APB1_PERIPH_BASE_ADDR) + (PWR_OFFSET_ADDR         ))
#define CAN2_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (CAN2_OFFSET_ADDR        ))
#define CAN1_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (CAN1_OFFSET_ADDR        ))
#define I2C3_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (I2C3_OFFSET_ADDR        ))
#define I2C2_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (I2C2_OFFSET_ADDR        ))
#define I2C1_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (I2C1_OFFSET_ADDR        ))
#define UART5_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (UART5_OFFSET_ADDR       ))
#define UART4_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (UART4_OFFSET_ADDR       ))
#define USART3_BASE_ADDR                        ((APB1_PERIPH_BASE_ADDR) + (USART3_OFFSET_ADDR      ))
#define USART2_BASE_ADDR                        ((APB1_PERIPH_BASE_ADDR) + (USART2_OFFSET_ADDR      ))
#define I2S3EXT_BASE_ADDR                       ((APB1_PERIPH_BASE_ADDR) + (I2S3EXT_OFFSET_ADDR     ))
#define SPI3_OR_I2S3_BASE_ADDR                  ((APB1_PERIPH_BASE_ADDR) + (SPI3_OR_I2S3_OFFSET_ADDR))
#define SPI2_OR_I2S2_BASE_ADDR                  ((APB1_PERIPH_BASE_ADDR) + (SPI2_OR_I2S2_OFFSET_ADDR))
#define I2S2EXT_BASE_ADDR                       ((APB1_PERIPH_BASE_ADDR) + (I2S2EXT_OFFSET_ADDR     ))
#define IWDG_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (IWDG_OFFSET_ADDR        ))
#define WWDG_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (WWDG_OFFSET_ADDR        ))
#define RTC_BASE_ADDR                           ((APB1_PERIPH_BASE_ADDR) + (RTC_OFFSET_ADDR         ))
#define TIM14_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (TIM14_OFFSET_ADDR       ))
#define TIM13_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (TIM13_OFFSET_ADDR       ))
#define TIM12_BASE_ADDR                         ((APB1_PERIPH_BASE_ADDR) + (TIM12_OFFSET_ADDR       ))
#define TIM7_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM7_OFFSET_ADDR        ))
#define TIM6_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM6_OFFSET_ADDR        ))
#define TIM5_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM5_OFFSET_ADDR        ))
#define TIM4_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM4_OFFSET_ADDR        ))
#define TIM3_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM3_OFFSET_ADDR        ))
#define TIM2_BASE_ADDR                          ((APB1_PERIPH_BASE_ADDR) + (TIM2_OFFSET_ADDR        ))

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define LCD_TFT_OFFSET_ADDR                     (0x00006800UL)
#define SAI1_OFFSET_ADDR                        (0x00005800UL)
#define SPI6_OFFSET_ADDR                        (0x00005400UL)
#define SPI5_OFFSET_ADDR                        (0x00005000UL)
#define TIM11_OFFSET_ADDR                       (0x00004800UL)
#define TIM10_OFFSET_ADDR                       (0x00004400UL)
#define TIM9_OFFSET_ADDR                        (0x00004000UL)
#define EXTI_OFFSET_ADDR                        (0x00003C00UL)
#define SYSCFG_OFFSET_ADDR                      (0x00003800UL)
#define SPI4_OFFSET_ADDR                        (0x00003400UL)
#define SPI1_OFFSET_ADDR                        (0x00003000UL)
#define SDIO_OFFSET_ADDR                        (0x00002C00UL)
#define ADC1_OFFSET_ADDR                        (0x00002000UL)
#define USART6_OFFSET_ADDR                      (0x00001400UL)
#define USART1_OFFSET_ADDR                      (0x00001000UL)
#define TIM8_OFFSET_ADDR                        (0x00000400UL)
#define TIM1_OFFSET_ADDR                        (0x00000000UL)

#define LCD_TFT_BASE_ADDR                       ((APB2_PERIPH_BASE_ADDR) + (LCD_TFT_OFFSET_ADDR))
#define SAI1_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SAI1_OFFSET_ADDR   ))
#define SPI6_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SPI6_OFFSET_ADDR   ))
#define SPI5_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SPI5_OFFSET_ADDR   ))
#define TIM11_BASE_ADDR                         ((APB2_PERIPH_BASE_ADDR) + (TIM11_OFFSET_ADDR  ))
#define TIM10_BASE_ADDR                         ((APB2_PERIPH_BASE_ADDR) + (TIM10_OFFSET_ADDR  ))
#define TIM9_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (TIM9_OFFSET_ADDR   ))
#define EXTI_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (EXTI_OFFSET_ADDR   ))
#define SYSCFG_BASE_ADDR                        ((APB2_PERIPH_BASE_ADDR) + (SYSCFG_OFFSET_ADDR ))
#define SPI4_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SPI4_OFFSET_ADDR   ))
#define SPI1_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SPI1_OFFSET_ADDR   ))
#define SDIO_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (SDIO_OFFSET_ADDR   ))
#define ADC1_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (ADC1_OFFSET_ADDR   ))
#define USART6_BASE_ADDR                        ((APB2_PERIPH_BASE_ADDR) + (USART6_OFFSET_ADDR ))
#define USART1_BASE_ADDR                        ((APB2_PERIPH_BASE_ADDR) + (USART1_OFFSET_ADDR ))
#define TIM8_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (TIM8_OFFSET_ADDR   ))
#define TIM1_BASE_ADDR                          ((APB2_PERIPH_BASE_ADDR) + (TIM1_OFFSET_ADDR   ))

/*************************** Peripheral register definition structures for RCC*******************/
/*
 * Peripheral register definition structures for RCC
 */

typedef struct {
	uint32_t CR; /* clock control register Address offset: 0x00 Reset value: 0x0000 XX83 where X is undefined */
	uint32_t PLLCFGR; /* PLL configuration register Address offset: 0x04 Reset value: 0x2400 3010 */
	uint32_t CFGR; /* clock configuration register Address offset: 0x08 Reset value: 0x0000 0000 */
	uint32_t CIR; /* clock interrupt register Address offset: 0x0C Reset value: 0x0000 0000 */
	uint32_t AHB1RSTR; /* AHB1 peripheral reset register Address offset: 0x10 Reset value: 0x0000 0000 */
	uint32_t AHB2RSTR; /* AHB2 peripheral reset register Address offset: 0x14 Reset value: 0x0000 0000 */
	uint32_t AHB3RSTR; /* AHB3 peripheral reset register Address offset: 0x18 Reset value: 0x0000 0000 */
	uint32_t RESERVED0; /*  */
	uint32_t APB1RSTR; /* APB1 peripheral reset register Address offset: 0x20 Reset value: 0x0000 0000 */
	uint32_t APB2RSTR; /* APB2 peripheral reset register Address offset: 0x24 Reset value: 0x0000 0000 */
	uint32_t RESERVED1[2]; /*  */
	uint32_t AHB1ENR; /* AHB1 peripheral clock enable register Address offset: 0x30 Reset value: 0x0010 0000 */
	uint32_t AHB2ENR; /* AHB2 peripheral clock enable register Address offset: 0x34 Reset value: 0x0000 0000 */
	uint32_t AHB3ENR; /* AHB3 peripheral clock enable register Address offset: 0x38 Reset value: 0x0000 0000 */
	uint32_t RESERVED2; /*  */
	uint32_t APB1ENR; /* APB1 peripheral clock enable register Address offset: 0x40 Reset value: 0x0000 0000 */
	uint32_t APB2ENR; /* APB2 peripheral clock enable register Address offset: 0x44 Reset value: 0x0000 0000 */
	uint32_t RESERVED3[2]; /*  */
	uint32_t AHB1LPENR; /*  */
	uint32_t AHB2LPENR; /* AHB2 peripheral clock enable in low power mode register Address offset: 0x54 Reset value: 0x0000 00F1 */
	uint32_t AHB3LPENR; /* AHB3 peripheral clock enable in low power mode register Address offset: 0x58 Reset value: 0x0000 0001 */
	uint32_t RESERVED4; /*  */
	uint32_t APB1LPENR; /* APB1 peripheral clock enable in low power mode register Address offset: 0x60 Reset value: 0xF6FE C9FF */
	uint32_t APB2LPENR; /* APB2 peripheral clock enabled in low power mode register Address offset: 0x64 Reset value: 0x0x0477 7F33 */
	uint32_t RESERVED5[2]; /*  */
	uint32_t BDCR; /* Backup domain control register Address offset: 0x70 Reset value: 0x0000 0000, reset by Backup domain reset. */
	uint32_t CSR; /* clock control & status register Address offset: 0x74 Reset value: 0x0E00 0000, reset by system reset, except reset flags by power reset only. */
	uint32_t RESERVED6[2]; /*  */
	uint32_t SSCGR; /* spread spectrum clock generation register Address offset: 0x80 Reset value: 0x0000 0000 */
	uint32_t PLLI2SCFGR; /* PLLI2S configuration register Address offset: 0x84 Reset value: 0x2400 3000 */
	uint32_t PLLSAICFGR; /* PLL configuration register Address offset: 0x88 Reset value: 0x2400 3000 */
	uint32_t DCKCFGR; /* Dedicated Clock Configuration Register Address offset: 0x8C Reset value: 0x0000 0000 */
} rcc_reg_t;

// RCC base addresses definition typecasted to rcc_reg_t

#define RCC ((rcc_reg_t *) RCC_BASE_ADDR)

/*************************** Register definition structures for EXTI*******************/
/*
 * Peripheral register definition structures for EXTI
 */
typedef struct {
	volatile uint32_t IMR; /*!< Interrupt mask register
	 0: Interrupt request from line x is masked
	 1: Interrupt request from line x is not masked*/

	volatile uint32_t EMR; /*!< Event mask register
	 0: Event request from line x is masked
	 1: Event request from line x is not masked*/

	volatile uint32_t RTSR; /*!< Rising trigger selection register
	 The external wakeup lines are edge triggered, no glitch must be generated on these lines.
	 If a falling edge occurs on the external interrupt line while writing to the EXTI_FTSR register,
	 the pending bit is not set.
	 Rising and falling edge triggers can be set for the same interrupt line. In this configuration,
	 both generate a trigger condition.
	 0: Rising trigger disabled (for Event and Interrupt) for input line
	 1: Rising trigger enabled (for Event and Interrupt) for input line*/

	volatile uint32_t FTSR; /*!< Falling trigger selection register
	 The external wakeup lines are edge triggered, no glitch must be generated on these lines.
	 If a falling edge occurs on the external interrupt line while writing to the EXTI_FTSR register,
	 the pending bit is not set.
	 Rising and falling edge triggers can be set for the same interrupt line. In this configuration,
	 both generate a trigger condition.
	 0: Falling trigger disabled (for Event and Interrupt) for input line
	 1: Falling trigger enabled (for Event and Interrupt) for input line.*/

	volatile uint32_t SWIER; /*!< Software interrupt event register
	 If interrupt are enabled on line x in the EXTI_IMR register,
	 writing '1' to SWIERx bit when it is set at '0'
	 sets the corresponding pending bit in the EXTI_PR register,
	 thus resulting in an interrupt request generation.
	 This bit is cleared by clearing the corresponding bit in EXTI_PR (by writing a 1 to the bit).*/

	volatile uint32_t PR; /*!< Pending register
	 This bit is set when the selected edge event arrives on the external interrupt line.
	 This bit is cleared by programming it to ‘1’*/
} exti_reg_t;

/*
 * Define the RCC_CFGR bit position
 */
#define RCC_CFGR_SW0		(0U)
#define RCC_CFGR_SW1		(1U)
#define RCC_CFGR_SWS0		(2U)
#define RCC_CFGR_SWS1		(3U)
#define RCC_CFGR_HPRE		(4U)
#define RCC_CFGR_PPRE1		(10U)
#define RCC_CFGR_PPRE2		(13U)
#define RCC_CFGR_RTCPRE		(16U)
#define RCC_CFGR_MCO1		(21U)
#define RCC_CFGR_I2SSCR		(23U)
#define RCC_CFGR_MCO1_PRE	(24U)
#define RCC_CFGR_MCO2_PRE	(27U)
#define RCC_CFGR_MCO2		(30U)

/*
 * Define the RCC_CFGR_SW value: System clock switch
 */
#define IS_HSI_SYSTEM_CLK	(0U)
#define IS_HSE_SYSTEM_CLK	(1U)
#define IS_PLL_SYSTEM_CLK	(2U)

/*!< EXTI base addresses definition typecasted to exti_reg_t*/
#define EXTI ((exti_reg_t *) EXTI_BASE_ADDR)

/*!< EXTI irq number definition*/
#define EXTI0_IRQ_NUMBER			(06U)
#define EXTI1_IRQ_NUMBER			(07U)
#define EXTI2_IRQ_NUMBER			(08U)
#define EXTI3_IRQ_NUMBER			(09U)
#define EXTI4_IRQ_NUMBER			(0AU)
#define EXTI9_5_IRQ_NUMBER			(23U)
#define EXTI15_10_IRQ_NUMBER		(40U)

/*************************** Register definition structures for SYSCFG*******************/
/*
 * Peripheral register definition structures for SYSCFG
 */

typedef struct {
	volatile uint32_t MEMRMP; /* memory remap register. Address offset: 0x00 Reset value: 0x0000 000X (X is the memory mode selected by the BOOT pins) */
	volatile uint32_t PMC; /* peripheral mode configuration register Address offset: 0x04 Reset value: 0x0000 0000 */
	volatile uint32_t EXTICR[4]; /* external interrupt configuration register Address offset: 0x08 Reset value: 0x0000 0000 */
	volatile uint32_t CMPCR; /* Compensation cell control register Address offset: 0x20 Reset value: 0x0000 0000 */
} syscfg_reg_t;

/*!< SYSCFG base addresses definition typecasted to syscfg_reg_t*/
#define SYSCFG ((syscfg_reg_t *) SYSCFG_BASE_ADDR)

/*!< Clock Enable Macros for SYSCFG peripherals*/
#define SYSCFG_PCLK_EN() (RCC -> APB2ENR |= (1U << 14U))

/*!< Clock Disable Macros for SYSCFG peripherals*/
#define SYSCFG_PCLK_DI() (RCC -> APB2ENR &= ~(1U << 14U))

/*************************** Register definition structures for GPIO*******************/
/*
 * Peripheral register definition structures for GPIO
 */

typedef struct {
	volatile uint32_t MODER; /* mode register Address offset: 0x00 Reset values: 0xA800 0000 for port A 0x0000 0280 for port B 0x0000 0000 for other ports */
	volatile uint32_t OTYPER; /* port output type register Address offset: 0x04 Reset value: 0x0000 0000 */
	volatile uint32_t OSPEEDR; /* output speed register Address offset: 0x08 Reset values: 0x0C00 0000 for port A 0x0000 00C0 for port B 0x0000 0000 for other ports */
	volatile uint32_t PUPDR; /* pull-up/pull-down register Address offset: 0x0C Reset values: 0x6400 0000 for port A 0x0000 0100 for port B 0x0000 0000 for other ports */
	volatile uint32_t IDR; /* input data register Address offset: 0x10 Reset value: 0x0000 XXXX (where X means undefined) */
	volatile uint32_t ODR; /* output data register Address offset: 0x14 Reset value: 0x0000 0000 */
	volatile uint32_t BSRR; /* bit set/reset register Address offset: 0x18 Reset value: 0x0000 0000 */
	volatile uint32_t LCKR; /* configuration lock register Address offset: 0x1C Reset value: 0x0000 0000 */
	volatile uint32_t AFR[2]; /* alternate function low & high register Address offset: 0x20 Reset value: 0x0000 0000 */
} gpio_reg_t;

/*!< GPIO base addresses definition typecasted to gpio_reg_t*/
#define GPIOA ((gpio_reg_t *) GPIOA_BASE_ADDR)
#define GPIOB ((gpio_reg_t *) GPIOB_BASE_ADDR)
#define GPIOC ((gpio_reg_t *) GPIOC_BASE_ADDR)
#define GPIOD ((gpio_reg_t *) GPIOD_BASE_ADDR)
#define GPIOE ((gpio_reg_t *) GPIOE_BASE_ADDR)
#define GPIOF ((gpio_reg_t *) GPIOF_BASE_ADDR)
#define GPIOG ((gpio_reg_t *) GPIOG_BASE_ADDR)
#define GPIOH ((gpio_reg_t *) GPIOH_BASE_ADDR)
#define GPIOI ((gpio_reg_t *) GPIOI_BASE_ADDR)

/*!< GPIO port number definition*/
#define GPIO_PORTA      (0x00U)
#define GPIO_PORTB      (0x01U)
#define GPIO_PORTC      (0x02U)
#define GPIO_PORTD      (0x03U)
#define GPIO_PORTE      (0x04U)
#define GPIO_PORTF      (0x05U)
#define GPIO_PORTG      (0x06U)
#define GPIO_PORTH      (0x07U)
#define GPIO_PORTI      (0x08U)
#define GPIO_PORT_NONE  (0x09U)

/*!< GPIO macro : convert a base address to port number*/
#define GPIO_ADDR_TO_PORT(addr) ((GPIOA) == (addr) ? GPIO_PORTA : \
		((GPIOB) == (addr) ? GPIO_PORTB : \
				((GPIOC) == (addr) ? GPIO_PORTC : \
						((GPIOD) == (addr) ? GPIO_PORTD : \
								((GPIOE) == (addr) ? GPIO_PORTE : \
										((GPIOF) == (addr) ? GPIO_PORTF : \
												((GPIOG) == (addr) ? GPIO_PORTG : \
														((GPIOH) == (addr) ? GPIO_PORTH : \
																((GPIOI) == (addr) ? GPIO_PORTI : \
																		GPIO_PORT_NONE)))))))))

/*!< Clock Enable Macros for GPIO peripherals*/
#define GPIOA_PCLK_EN() (RCC -> AHB1ENR |= (1U << 0U))
#define GPIOB_PCLK_EN() (RCC -> AHB1ENR |= (1U << 1U))
#define GPIOC_PCLK_EN() (RCC -> AHB1ENR |= (1U << 2U))
#define GPIOD_PCLK_EN() (RCC -> AHB1ENR |= (1U << 3U))
#define GPIOE_PCLK_EN() (RCC -> AHB1ENR |= (1U << 4U))
#define GPIOF_PCLK_EN() (RCC -> AHB1ENR |= (1U << 5U))
#define GPIOG_PCLK_EN() (RCC -> AHB1ENR |= (1U << 6U))
#define GPIOH_PCLK_EN() (RCC -> AHB1ENR |= (1U << 7U))
#define GPIOI_PCLK_EN() (RCC -> AHB1ENR |= (1U << 8U))

/*!< Clock Disable Macros for GPIO peripherals*/
#define GPIOA_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 0U))
#define GPIOB_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 1U))
#define GPIOC_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 2U))
#define GPIOD_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 3U))
#define GPIOE_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 4U))
#define GPIOF_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 5U))
#define GPIOG_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 6U))
#define GPIOH_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 7U))
#define GPIOI_PCLK_DI() (RCC -> AHB1ENR &= ~(1U << 8U))

/*************************** Register definition structures for I2C*******************/
/*
 * Peripheral register definition structures for I2C
 */

typedef struct {
	volatile uint32_t CR1; /* Control register Address offset: 0x00 Reset value: 0x0000 */
	volatile uint32_t CR2; /* Control register Address offset: 0x00 Reset value: 0x0000 */
	volatile uint32_t OAR1; /*  Own address register Address offset: 0x08 Reset value: 0x0000 */
	volatile uint32_t OAR2; /*  Own address register Address offset: 0x08 Reset value: 0x0000 */
	volatile uint32_t DR; /* Data register Address offset: 0x10 Reset value: 0x0000 */
	volatile uint32_t SR1; /* Status register Address offset: 0x14 Reset value: 0x0000 */
	volatile uint32_t SR2; /* Status register Address offset: 0x14 Reset value: 0x0000 */
	volatile uint32_t CCR; /* Clock control register Address offset: 0x1C Reset value: 0x0000 */
	volatile uint32_t TRISE; /* Address offset: 0x20 Reset value: 0x0002 */
} i2c_reg_t;

/*!< I2C base addresses definition typecasted to i2c_reg_t*/

#define I2C1 ((i2c_reg_t *) I2C1_BASE_ADDR)
#define I2C2 ((i2c_reg_t *) I2C2_BASE_ADDR)
#define I2C3 ((i2c_reg_t *) I2C3_BASE_ADDR)

#define I2C1_PCLK_EN() (RCC -> APB1ENR |= (1U << 21U))
#define I2C2_PCLK_EN() (RCC -> APB1ENR |= (1U << 22U))
#define I2C3_PCLK_EN() (RCC -> APB1ENR |= (1U << 23U))

#define I2C1_PCLK_DI() (RCC -> APB1ENR &= ~(1U << 21U))
#define I2C2_PCLK_DI() (RCC -> APB1ENR &= ~(1U << 22U))
#define I2C3_PCLK_DI() (RCC -> APB1ENR &= ~(1U << 23U))

/*************************** Register definition structures for SPI*******************/
/*
 * Peripheral register definition structures for SPI
 */

typedef struct {
	volatile uint32_t CR1; /* SPI control register 1 (SPI_CR1) (not used in I 2 S mode) Address offset: 0x00 Reset value: 0x0000 */
	volatile uint32_t CR2; /* SPI control register 2. Address offset: 0x04 Reset value: 0x0000 */
	volatile uint32_t SR; /* status register Address offset: 0x08 Reset value: 0x0002 */
	volatile uint32_t DR; /* data register Address offset: 0x0C Reset value: 0x0000 */
	volatile uint32_t CRCPR; /* RX CRC register (not used in I 2 S mode) Address offset: 0x14 Reset value: 0x0000 */
	volatile uint32_t RXCRCR; /* RX CRC register (not used in I 2 S mode) Address offset: 0x14 Reset value: 0x0000 */
	volatile uint32_t TXCRCR; /* TX CRC register (not used in I 2 S mode) Address offset: 0x18 Reset value: 0x0000 */
	volatile uint32_t I2SCFGR; /* configuration register Address offset: 0x1C Reset value: 0x0000 */
	volatile uint32_t I2SPR; /* prescaler register Address offset: 0x20 Reset value: 0000 0010 (0x0002) */
} spi_reg_t;

/*!< SPI base addresses definition typecasted to spi_reg_t */
#define SPI1 ((spi_reg_t *) SPI1_BASE_ADDR)
#define SPI2 ((spi_reg_t *) SPI2_OR_I2S2_BASE_ADDR)
#define SPI3 ((spi_reg_t *) SPI3_OR_I2S3_BASE_ADDR)
#define SPI4 ((spi_reg_t *) SPI4_BASE_ADDR)
#define SPI5 ((spi_reg_t *) SPI5_BASE_ADDR)
#define SPI6 ((spi_reg_t *) SPI6_BASE_ADDR)

/*!< Clock Enable Macros for SPI peripheral*/
#define SPI1_PCLK_EN() (RCC -> APB2ENR |= (1U << 12U))
#define SPI2_PCLK_EN() (RCC -> APB1ENR |= (1U << 14U))
#define SPI3_PCLK_EN() (RCC -> APB1ENR |= (1U << 15U))
#define SPI4_PCLK_EN() (RCC -> APB2ENR |= (1U << 13U))
#define SPI5_PCLK_EN() (RCC -> APB2ENR |= (1U << 20U))
#define SPI6_PCLK_EN() (RCC -> APB2ENR |= (1U << 21U))

/*!< Clock Disable Macros for SPI peripherals*/
#define SPI1_PCLK_DI() (RCC -> APB2ENR &= ~(1U << 12U))
#define SPI2_PCLK_DI() (RCC -> APB1ENR &= ~(1U << 14U))
#define SPI3_PCLK_DI() (RCC -> APB1ENR &= ~(1U << 15U))
#define SPI4_PCLK_DI() (RCC -> APB2ENR &= ~(1U << 13U))
#define SPI5_PCLK_DI() (RCC -> APB2ENR &= ~(1U << 20U))
#define SPI6_PCLK_DI() (RCC -> APB2ENR &= ~(1U << 21U))

/*!< SPI irq number definition*/
#define SPI1_IRQ_NUMBER				(35U)
#define SPI2_IRQ_NUMBER				(36U)
#define SPI3_IRQ_NUMBER				(51U)
#define SPI4_IRQ_NUMBER				(84U)
#define SPI5_IRQ_NUMBER				(85U)
#define SPI6_IRQ_NUMBER				(86U)
/*************************** Register definition structures for USART*******************/
/*
 * Peripheral register definition structures for USART
 */

typedef struct {
	volatile uint32_t SR; /* Status register Address offset: 0x00 Reset value: 0x0000 00C0 */
	volatile uint32_t DR; /* Data register Address offset: 0x04 Reset value: 0xXXXX XXXX */
	volatile uint32_t BRR; /* Baud rate register Note: The baud counters stop counting if the TE or RE bits are disabled respectively. Address offset: 0x08 Reset value: 0x0000 0000 */
	volatile uint32_t CR[3]; /* Control register Address offset: 0x0C Reset value: 0x0000 0000 */
	volatile uint32_t GTPR; /* Guard time and prescaler register Address offset: 0x18 Reset value: 0x0000 0000 */
} usart_reg_t;

/*!< USART base addresses definition typecasted to usart_reg_t*/
#define USART1 ((usart_reg_t *) USART1_BASE_ADDR)
#define USART2 ((usart_reg_t *) USART2_BASE_ADDR)
#define USART3 ((usart_reg_t *) USART3_BASE_ADDR)
#define UART4  ((usart_reg_t *) UART4_BASE_ADDR )
#define UART5  ((usart_reg_t *) USART1_BASE_ADDR)
#define USART6 ((usart_reg_t *) USART6_BASE_ADDR)
#define UART7  ((usart_reg_t *) UART7_BASE_ADDR )
#define UART8  ((usart_reg_t *) UART8_BASE_ADDR )

/*!< Clock Enable Macros for USART peripherals*/
#define USART1_PCLK_EN()  (RCC -> APB2ENR |= (1U << 04U))
#define USART2_PCLK_EN()  (RCC -> APB1ENR |= (1U << 17U))
#define USART3_PCLK_EN()  (RCC -> APB1ENR |= (1U << 18U))
#define UART4_PCLK_EN()   (RCC -> APB1ENR |= (1U << 19U))
#define UART5_PCLK_EN()   (RCC -> APB1ENR |= (1U << 20U))
#define USART6_PCLK_EN()  (RCC -> APB2ENR |= (1U << 05U))
#define UART7_PCLK_EN()   (RCC -> APB1ENR |= (1U << 30U))
#define UART8_PCLK_EN()   (RCC -> APB1ENR |= (1U << 31U))

/*!< Clock Disable Macros for USART peripherals*/
#define USART1_PCLK_DI()  (RCC -> APB2ENR &= ~(1U << 04U))
#define USART2_PCLK_DI()  (RCC -> APB1ENR &= ~(1U << 17U))
#define USART3_PCLK_DI()  (RCC -> APB1ENR &= ~(1U << 18U))
#define UART4_PCLK_DI()   (RCC -> APB1ENR &= ~(1U << 19U))
#define UART5_PCLK_DI()   (RCC -> APB1ENR &= ~(1U << 20U))
#define USART6_PCLK_DI()  (RCC -> APB2ENR &= ~(1U << 05U))
#define UART7_PCLK_DI()   (RCC -> APB1ENR &= ~(1U << 30U))
#define UART8_PCLK_DI()   (RCC -> APB1ENR &= ~(1U << 31U))

/*************************** Peripherals reset for AHB1 devices*******************/
#define GPIOA_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x00U)); (RCC -> AHB1RSTR &= ~(SET << 0x00U));} while (0)
#define GPIOB_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x01U)); (RCC -> AHB1RSTR &= ~(SET << 0x01U));} while (0)
#define GPIOC_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x02U)); (RCC -> AHB1RSTR &= ~(SET << 0x02U));} while (0)
#define GPIOD_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x03U)); (RCC -> AHB1RSTR &= ~(SET << 0x03U));} while (0)
#define GPIOE_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x04U)); (RCC -> AHB1RSTR &= ~(SET << 0x04U));} while (0)
#define GPIOF_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x05U)); (RCC -> AHB1RSTR &= ~(SET << 0x05U));} while (0)
#define GPIOG_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x06U)); (RCC -> AHB1RSTR &= ~(SET << 0x06U));} while (0)
#define GPIOH_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x07U)); (RCC -> AHB1RSTR &= ~(SET << 0x07U));} while (0)
#define GPIOI_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x08U)); (RCC -> AHB1RSTR &= ~(SET << 0x08U));} while (0)
#define GPIOJ_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x09U)); (RCC -> AHB1RSTR &= ~(SET << 0x09U));} while (0)
#define GPIOK_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x0AU)); (RCC -> AHB1RSTR &= ~(SET << 0x0AU));} while (0)
#define CRC_RST()     do {(RCC -> AHB1RSTR |= (SET << 0x0CU)); (RCC -> AHB1RSTR &= ~(SET << 0x0CU));} while (0)
#define DMA1_RST()    do {(RCC -> AHB1RSTR |= (SET << 0x15U)); (RCC -> AHB1RSTR &= ~(SET << 0x15U));} while (0)
#define DMA2_RST()    do {(RCC -> AHB1RSTR |= (SET << 0x16U)); (RCC -> AHB1RSTR &= ~(SET << 0x16U));} while (0)
#define DMA2D_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x17U)); (RCC -> AHB1RSTR &= ~(SET << 0x17U));} while (0)
#define ETHMAC_RST()  do {(RCC -> AHB1RSTR |= (SET << 0x19U)); (RCC -> AHB1RSTR &= ~(SET << 0x19U));} while (0)
#define OTGHS_RST()   do {(RCC -> AHB1RSTR |= (SET << 0x1DU)); (RCC -> AHB1RSTR &= ~(SET << 0x1DU));} while (0)

/*************************** Peripherals reset for AHB2 devices*******************/
#define DCMIRST_RST()   do {(RCC -> AHB2RSTR |= (SET << 0x00U)); (RCC -> AHB2RSTR &= ~(SET << 0x00U));} while (0)
#define CRYPRST_RST()   do {(RCC -> AHB2RSTR |= (SET << 0x04U)); (RCC -> AHB2RSTR &= ~(SET << 0x04U));} while (0)
#define HSAHRST_RST()   do {(RCC -> AHB2RSTR |= (SET << 0x05U)); (RCC -> AHB2RSTR &= ~(SET << 0x05U));} while (0)
#define RNGRST_RST()    do {(RCC -> AHB2RSTR |= (SET << 0x06U)); (RCC -> AHB2RSTR &= ~(SET << 0x06U));} while (0)
#define OTGFSRS_RST()   do {(RCC -> AHB2RSTR |= (SET << 0x07U)); (RCC -> AHB2RSTR &= ~(SET << 0x07U));} while (0)

/*************************** Peripherals reset for AHB3 devices*******************/
#define FSMCRST_RST()   do {(RCC -> AHB3RSTR |= (SET << 0x00U)); (RCC -> AHB3RSTR &= ~(SET << 0x00U));} while (0)

/*************************** Peripherals reset for APB1 devices*******************/
#define TIM2_RST()  do {(RCC -> APB1RSTR |= (SET << 0x00U)); (RCC -> APB1RSTR &= ~(SET << 0x00U));} while (0)
#define TIM3_RST()  do {(RCC -> APB1RSTR |= (SET << 0x01U)); (RCC -> APB1RSTR &= ~(SET << 0x01U));} while (0)
#define TIM4_RST()  do {(RCC -> APB1RSTR |= (SET << 0x02U)); (RCC -> APB1RSTR &= ~(SET << 0x02U));} while (0)
#define TIM5_RST()  do {(RCC -> APB1RSTR |= (SET << 0x03U)); (RCC -> APB1RSTR &= ~(SET << 0x03U));} while (0)
#define TIM6_RST()  do {(RCC -> APB1RSTR |= (SET << 0x04U)); (RCC -> APB1RSTR &= ~(SET << 0x04U));} while (0)
#define TIM7_RST()  do {(RCC -> APB1RSTR |= (SET << 0x05U)); (RCC -> APB1RSTR &= ~(SET << 0x05U));} while (0)
#define TIM12_RST() do {(RCC -> APB1RSTR |= (SET << 0x06U)); (RCC -> APB1RSTR &= ~(SET << 0x06U));} while (0)
#define TIM13_RST() do {(RCC -> APB1RSTR |= (SET << 0x07U)); (RCC -> APB1RSTR &= ~(SET << 0x07U));} while (0)
#define TIM14_RST() do {(RCC -> APB1RSTR |= (SET << 0x08U)); (RCC -> APB1RSTR &= ~(SET << 0x08U));} while (0)
#define WWDG_RST()  do {(RCC -> APB1RSTR |= (SET << 0x0BU)); (RCC -> APB1RSTR &= ~(SET << 0x0BU));} while (0)
#define SPI2_RST()  do {(RCC -> APB1RSTR |= (SET << 0x0EU)); (RCC -> APB1RSTR &= ~(SET << 0x0EU));} while (0)
#define SPI3_RST()  do {(RCC -> APB1RSTR |= (SET << 0x0FU)); (RCC -> APB1RSTR &= ~(SET << 0x0FU));} while (0)
#define UART2_RST() do {(RCC -> APB1RSTR |= (SET << 0x11U)); (RCC -> APB1RSTR &= ~(SET << 0x10U));} while (0)
#define UART3_RST() do {(RCC -> APB1RSTR |= (SET << 0x12U)); (RCC -> APB1RSTR &= ~(SET << 0x11U));} while (0)
#define UART4_RST() do {(RCC -> APB1RSTR |= (SET << 0x13U)); (RCC -> APB1RSTR &= ~(SET << 0x12U));} while (0)
#define UART5_RST() do {(RCC -> APB1RSTR |= (SET << 0x14U)); (RCC -> APB1RSTR &= ~(SET << 0x13U));} while (0)
#define I2C1_RST()  do {(RCC -> APB1RSTR |= (SET << 0x15U)); (RCC -> APB1RSTR &= ~(SET << 0x14U));} while (0)
#define I2C2_RST()  do {(RCC -> APB1RSTR |= (SET << 0x16U)); (RCC -> APB1RSTR &= ~(SET << 0x15U));} while (0)
#define I2C3_RST()  do {(RCC -> APB1RSTR |= (SET << 0x17U)); (RCC -> APB1RSTR &= ~(SET << 0x16U));} while (0)
#define CAN1_RST()  do {(RCC -> APB1RSTR |= (SET << 0x19U)); (RCC -> APB1RSTR &= ~(SET << 0x18U));} while (0)
#define CAN2_RST()  do {(RCC -> APB1RSTR |= (SET << 0x1AU)); (RCC -> APB1RSTR &= ~(SET << 0x19U));} while (0)
#define PWRR_RST()  do {(RCC -> APB1RSTR |= (SET << 0x1CU)); (RCC -> APB1RSTR &= ~(SET << 0x1CU));} while (0)
#define DACR_RST()  do {(RCC -> APB1RSTR |= (SET << 0x1DU)); (RCC -> APB1RSTR &= ~(SET << 0x1DU));} while (0)
#define UART7_RST() do {(RCC -> APB1RSTR |= (SET << 0x1EU)); (RCC -> APB1RSTR &= ~(SET << 0x1EU));} while (0)
#define UART8_RST() do {(RCC -> APB1RSTR |= (SET << 0x1FU)); (RCC -> APB1RSTR &= ~(SET << 0x1FU));} while (0)

/*************************** Peripherals reset for APB2 devices*******************/
#define TIM1_RST()   do {(RCC -> APB2RSTR |= (SET << 0x00U)); (RCC -> APB2RSTR &= ~(SET << 0x00U));} while (0)
#define TIM8_RST()   do {(RCC -> APB2RSTR |= (SET << 0x01U)); (RCC -> APB2RSTR &= ~(SET << 0x01U));} while (0)
#define USART1_RST() do {(RCC -> APB2RSTR |= (SET << 0x04U)); (RCC -> APB2RSTR &= ~(SET << 0x04U));} while (0)
#define USART6_RST() do {(RCC -> APB2RSTR |= (SET << 0x05U)); (RCC -> APB2RSTR &= ~(SET << 0x05U));} while (0)
#define ADC_RST()    do {(RCC -> APB2RSTR |= (SET << 0x08U)); (RCC -> APB2RSTR &= ~(SET << 0x08U));} while (0)
#define SDIO_RST()   do {(RCC -> APB2RSTR |= (SET << 0x0BU)); (RCC -> APB2RSTR &= ~(SET << 0x0BU));} while (0)
#define SPI1_RST()   do {(RCC -> APB2RSTR |= (SET << 0x0CU)); (RCC -> APB2RSTR &= ~(SET << 0x0CU));} while (0)
#define SPI4_RST()   do {(RCC -> APB2RSTR |= (SET << 0x0DU)); (RCC -> APB2RSTR &= ~(SET << 0x0DU));} while (0)
#define SYSCFG_RST() do {(RCC -> APB2RSTR |= (SET << 0x0EU)); (RCC -> APB2RSTR &= ~(SET << 0x0EU));} while (0)
#define TIM9_RST()   do {(RCC -> APB2RSTR |= (SET << 0x10U)); (RCC -> APB2RSTR &= ~(SET << 0x10U));} while (0)
#define TIM10_RST()  do {(RCC -> APB2RSTR |= (SET << 0x11U)); (RCC -> APB2RSTR &= ~(SET << 0x11U));} while (0)
#define TIM11_RST()  do {(RCC -> APB2RSTR |= (SET << 0x12U)); (RCC -> APB2RSTR &= ~(SET << 0x12U));} while (0)
#define SPI5_RST()   do {(RCC -> APB2RSTR |= (SET << 0x14U)); (RCC -> APB2RSTR &= ~(SET << 0x14U));} while (0)
#define SPI6_RST()   do {(RCC -> APB2RSTR |= (SET << 0x15U)); (RCC -> APB2RSTR &= ~(SET << 0x15U));} while (0)
#define SAI1_RST()   do {(RCC -> APB2RSTR |= (SET << 0x16U)); (RCC -> APB2RSTR &= ~(SET << 0x16U));} while (0)
#define LTDC_RST()   do {(RCC -> APB2RSTR |= (SET << 0x1AU)); (RCC -> APB2RSTR &= ~(SET << 0x1AU));} while (0)

/*************************** Some generic macros *******************/

#define ONE_BIT		(0x01U)
#define TWO_BIT		(0x03U)
#define THREE_BIT 	(0x07U)
#define FOUR_BIT 	(0x0FU)
#define FIVE_BIT 	(0x1FU)
#define SIX_BIT 	(0x3FU)
#define SEVEN_BIT 	(0x7FU)
#define EIGHT_BIT 	(0xFFU)
#define NINE_BIT 	(0x1FFU)
#define TEN_BIT 	(0x3FFU)
#define ELEVEN_BIT 	(0x7FFU)
#define TWELVEBIT 	(0xFFFU)

#define SHIFT_ONE	(1U)
#define SHIFT_TWO	(2U)
#define SHIFT_THREE	(3U)
#define SHIFT_FOUR	(4U)

#define ENABLE 			(0x01U)
#define DISABLE 		(0x00U)
#define SET 			(ENABLE)
#define RESET 			(DISABLE)
#define GPIO_PIN_SET 	(SET)
#define GPIO_PIN_RESET 	(RESET)
#define FLAG_SET	 	(SET)
#define FLAG_RESET	 	(RESET)

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"

#endif /* INC_STM32F407XX_H_ */
