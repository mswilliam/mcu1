/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Sep 24, 2020
 *      Author: mangwi01
 */

#ifndef STM32F407XX_GPIO_H_
#define STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
	uint16_t pin_number :4; /*!< possible values from @GPIO_PIN_NUMBERS*/
	uint16_t pin_mode :3; /*!< possible values from @GPIO_MODES*/
	uint16_t pin_ospeed :2; /*!< possible values from @GPIO_SPEED*/
	uint16_t pin_otype :1; /*!< possible values from @GPIO_OUTPUT_TYPE*/
	uint16_t pin_pupd :2; /*!< possible values from @GPIO_PUPD*/
	uint16_t pin_afr :4; /*!< possible values from @GPIO_ALTERNATE_FUNCTION*/
} gpio_pin_config_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {
	gpio_reg_t *ptr_sgpio; /*!< base address of the gpio to which the pin belongs */
	gpio_pin_config_t *ptr_sconfig; /*!< gpio pin configuration setting */
} gpio_handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO possible alternate function
 */
#define GPIO_PIN_0  (0x0U)
#define GPIO_PIN_1  (0x1U)
#define GPIO_PIN_2  (0x2U)
#define GPIO_PIN_3  (0x3U)
#define GPIO_PIN_4  (0x4U)
#define GPIO_PIN_5  (0x5U)
#define GPIO_PIN_6  (0x6U)
#define GPIO_PIN_7  (0x7U)
#define GPIO_PIN_8  (0x8U)
#define GPIO_PIN_9  (0x9U)
#define GPIO_PIN_10 (0xAU)
#define GPIO_PIN_11 (0xBU)
#define GPIO_PIN_12 (0xCU)
#define GPIO_PIN_13 (0xDU)
#define GPIO_PIN_14 (0xEU)
#define GPIO_PIN_15 (0xFU)

/*
 * @GPIO_MODES
 * GPIO possible port mode register
 */
#define GPIO_MODER_IN     (0U) /*!< 000: Input (reset state)*/
#define GPIO_MODER_OUT    (1U) /*!< 001: General purpose output mode*/
#define GPIO_MODER_AF     (2U) /*!< 010: Alternate function mode*/
#define GPIO_MODER_ANALOG (3U) /*!< 11: Analog mode*/
#define GPIO_MODER_IT_FT  (4U) /*!< 100: Interrupt falling edge trigger*/
#define GPIO_MODER_IT_RT  (5U) /*!< 101: Interrupt rising edge trigger*/
#define GPIO_MODER_RFT    (6U) /*!< 110: Interrupt falling & rising edge trigger*/

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO possible output type register
 */
#define GPIO_OTYPER_PP (0U) /*!< 0: Output push-pull (reset state)*/
#define GPIO_OTYPER_OD (1U) /*!< 1: Output open-drain*/

/*
 * @GPIO_SPEED
 * GPIO possible port output speed register
 */
#define GPIO_OSPEEDR_LOW        (0U) /*!< 00: Low speed*/
#define GPIO_OSPEEDR_MEDIUM     (1U) /*!< 01: Medium speed*/
#define GPIO_OSPEEDR_HIGH       (2U) /*!< 10: High speed*/
#define GPIO_OSPEEDR_VERY_HIGH  (3U) /*!< 11: Very high speed*/

/*
 * @GPIO_PUPD
 * GPIO possible port pull-up/pull-down register
 */
#define GPIO_PUPDR_NO (0U) /*!< 00: No pull-up, pull-down*/
#define GPIO_PUPDR_PU (1U) /*!< 01: Pull-up*/
#define GPIO_PUPDR_PD (2U) /*!< 10: Pull-down*/

/*
 * @GPIO_ALTERNATE_FUNCTION
 * GPIO possible alternate function
 */
#define GPIO_AF0  (0x0U)
#define GPIO_AF1  (0x1U)
#define GPIO_AF2  (0x2U)
#define GPIO_AF3  (0x3U)
#define GPIO_AF4  (0x4U)
#define GPIO_AF5  (0x5U)
#define GPIO_AF6  (0x6U)
#define GPIO_AF7  (0x7U)
#define GPIO_AF8  (0x8U)
#define GPIO_AF9  (0x9U)
#define GPIO_AF10 (0xAU)
#define GPIO_AF11 (0xBU)
#define GPIO_AF12 (0xCU)
#define GPIO_AF13 (0xDU)
#define GPIO_AF14 (0xEU)
#define GPIO_AF15 (0xFU)

/********************************************************************************************
 *
 * 								API supported by this driver
 *
 ********************************************************************************************/

/*
 * Perioheral Clock setup
 */

/***************************************************************************************
 * @fn					- gpio_peri_clock_control
 *
 * @brief				- Enables or disables peripheral clock for the given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_peri_clock_control(gpio_reg_t *arg_ptr_gpio,
		uint8_t arg_enable_or_disable);

/*
 * Init and De-init
 */

/***************************************************************************************
 * @fn					- gpio_init
 *
 * @brief				- Initialise pin of a given gpio according t a given configuration
 *
 * @param[in]			- address of to a structure of a gpio
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_init(gpio_handle_t *arg_ptr_gpio_handle);

/***************************************************************************************
 * @fn					- gpio_de_init
 *
 * @brief				- De-init a given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_de_init(gpio_reg_t *arg_ptr_gpio);

/*
 * Read and write
 */

/***************************************************************************************
 * @fn					- gpio_read_from_input_pin
 *
 * @brief				- read the pin value of a given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 * @param[in]			- values from @GPIO_PIN_NUMBERS
 *
 * @return				- GPIO_PIN_SET or GPIO_PIN_RESET macro
 *
 * @Note				- none
 */
uint8_t gpio_read_from_input_pin(gpio_reg_t *arg_ptr_gpio,
		uint8_t arg_pin_number);

/***************************************************************************************
 * @fn					- gpio_read_from_input_port
 *
 * @brief				- read the whole value of a given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 *
 * @return				- value of the given gpio
 *
 * @Note				- none
 */
uint16_t gpio_read_from_input_port(gpio_reg_t *arg_ptr_gpio);

/***************************************************************************************
 * @fn					- gpio_write_to_output_pin
 *
 * @brief				- write to the output pin of the given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 * @param[in]			- values from @GPIO_PIN_NUMBERS
 * @param[in]			- GPIO_PIN_SET or GPIO_PIN_RESET macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_write_to_output_pin(gpio_reg_t *arg_ptr_gpio, uint8_t arg_pin_number,
		uint8_t arg_value);

/***************************************************************************************
 * @fn					- gpio_write_to_output_port
 *
 * @brief				- write to the given outpout port
 *
 * @param[in]			- Base address of the gpio peripheral
 * @param[in]			- 16 bits
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_write_to_output_port(gpio_reg_t *arg_ptr_gpio, uint16_t arg_data);

/***************************************************************************************
 * @fn					- gpio_toggle_output_pin
 *
 * @brief				- toggle the output given pin of the given gpio
 *
 * @param[in]			- Base address of the gpio peripheral
 * @param[in]			- values from @GPIO_PIN_NUMBERS
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_toggle_output_pin(gpio_reg_t *arg_ptr_gpio, uint8_t arg_pin_number);

/*
 * IRQ Configuration and ISR handling
 */

/***************************************************************************************
 * @fn					- gpio_irq_config
 *
 * @brief				- Configure the irq handling of a given irq number
 *
 * @param[in]			- given irq number
 * @param[in]			- given priority of the irq
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
		uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- gpio_irq_handling
 *
 * @brief				-
 *
 * @param[in]			- values from @GPIO_PIN_NUMBERS
 *
 * @return				- none
 *
 * @Note				- none
 */
void gpio_irq_handling(uint8_t arg_pin_number);

#endif /* STM32F407XX_GPIO_H_ */

