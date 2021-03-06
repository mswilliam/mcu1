/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 2 nov. 2020
 *      Author: ange
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*************************** Bit position definition for i2c peripheral driver *******************/
/*!< Bit position definition for i2c CR1*/
#define I2C_CR1_PE			(0U)
#define I2C_CR1_SMBUS		(1U)
#define I2C_CR1_SMBTYPE		(3U)
#define I2C_CR1_ENARP		(4U)
#define I2C_CR1_ENPEC		(5U)
#define I2C_CR1_ENGC		(6U)
#define I2C_CR1_NOSTRETCH	(7U)
#define I2C_CR1_START		(8U)
#define I2C_CR1_STOP		(9U)
#define I2C_CR1_ACK			(10U)
#define I2C_CR1_POS			(11U)
#define I2C_CR1_PEC			(12U)
#define I2C_CR1_ALERT		(13U)
#define I2C_CR1_SWRST		(15U)

/*!< Bit position definition for i2c CCR*/
#define I2C_CCR_CCR		(0U)
#define I2C_CCR_DUTY	(14U)
#define I2C_CCR_FS		(15U)

/*!< Bit position definition for i2c CR2*/
#define I2C_CR2_FREQ	(0U)
#define I2C_CR2_ITERREN	(8U)
#define I2C_CR2_ITEVTEN	(9U)
#define I2C_CR2_ITBUFEN	(10U)
#define I2C_CR2_DMAEN	(11U)
#define I2C_CR2_LAST	(12U)

/*!< Bit position definition for i2c OAR1*/
#define I2C_OAR1_ADD0		(0U)
#define I2C_OAR1_ADD71		(1U)
#define I2C_OAR1_ADD98		(8U)
#define I2C_OAR1_RSV14		(14U)
#define I2C_OAR1_ADDMODE	(15U)

/*!< Bit position definition for i2c SR1*/
#define I2C_SR1_SB			(0U)
#define I2C_SR1_ADDR		(1U)
#define I2C_SR1_BTF			(2U)
#define I2C_SR1_ADD10		(3U)
#define I2C_SR1_STOPF		(4U)
#define I2C_SR1_RxNE		(6U)
#define I2C_SR1_TxE			(7U)
#define I2C_SR1_BERR		(8U)
#define I2C_SR1_ARLO		(9U)
#define I2C_SR1_AF			(10U)
#define I2C_SR1_OVR			(11U)
#define I2C_SR1_PECERR		(12U)
#define I2C_SR1_TIMEOUT		(14U)
#define I2C_SR1_SMBALERT	(15U)

/*!< Bit position definition for i2c SR2*/
#define I2C_SR2_MSL			(0U)
#define I2C_SR2_BUSY		(1U)
#define I2C_SR2_TRA			(2U)
#define I2C_SR2_GENCALL		(4U)
#define I2C_SR2_SMBDEFAUL	(5U)
#define I2C_SR2_SMBHOST		(6U)
#define I2C_SR2_DUALF		(7U)
#define I2C_SR2_PEC			(8U)

/*!< I2C FLAGs*/

#define I2C_FLAG_SB			(SET << I2C_SR1_SB)
#define I2C_FLAG_ADD10		(SET << I2C_SR1_ADD10)
#define I2C_FLAG_TXE		(SET << I2C_SR1_TxE)
#define I2C_FLAG_RXNE		(SET << I2C_SR1_RxNE)
#define I2C_FLAG_ADDR		(SET << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(SET << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(SET << I2C_SR1_STOPF)
#define I2C_FLAG_BERR		(SET << I2C_SR1_BERR)
#define I2C_FLAG_ARLO		(SET << I2C_SR1_ARLO)
#define I2C_FLAG_AF			(SET << I2C_SR1_AF)
#define I2C_FLAG_OVR		(SET << I2C_SR1_OVR)
#define I2C_FLAG_PECERR		(SET << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT	(SET << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT	(SET << I2C_SR1_SMBALERT)

#define I2C_REPEAT_START_NO		(0U)
#define I2C_REPEAT_START_YES	(1U)

/*
 * I2C application events
 */
#define I2C_EV_TX_CMPLT		(0U)
#define I2C_EV_RX_CMPLT		(1U)
#define I2C_EV_STOP			(2U)
#define I2C_ERROR_BERR		(3U)
#define I2C_ERROR_ARLO		(4U)
#define I2C_ERROR_AF		(5U)
#define I2C_ERROR_OVR		(6U)
#define I2C_ERROR_PECERR	(7U)
#define I2C_ERROR_TIMEOUT	(8U)
#define I2C_ERROR_SMBALERT	(9U)
#define I2C_EV_DATA_REQ		(10U)
#define I2C_EV_DATA_RCV		(11U)

/*
 * Configuration structure for I2C peripheral
 */
typedef struct {
	uint32_t scl_speed; /*!< possible values from  @scl_speed*/
	uint16_t fm_duty_cycle; /*!< possible values from  @fm_duty_cycle*/
	uint8_t device_addr; /*!< */
	uint8_t ack_control; /*!< possible values from  @ack_control*/
} i2c_config_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {
	i2c_reg_t *m_poi2c_reg; /*!< base address of the i2c >*/
	i2c_config_t *m_poi2c_sconfig; /*!< i2c configuration setting >*/
	uint8_t *m_pu8tx_buffer; /*!<  To store the app tx buffer addr >*/
	uint8_t *m_pu8rx_buffer; /*!<  To store the app rx buffer addr >*/
	uint32_t m_u32tx_len; /*!<  To store the tx len >*/
	uint32_t m_u32rx_len; /*!<  To store the rx len  >*/
	uint8_t m_u8rx_tx_state; /*!<  To store the comm state see @appState >*/
	uint8_t m_u8dev_addr; /*!<  To store the slave/device address >*/
	uint32_t m_u32rx_size; /*!<  To store the rx size >*/
	uint8_t m_u8sr; /*!<  To store the repeated start value >*/

} i2c_handle_t;
/*
 * @appState
 */
#define I2C_READY		(0U)
#define I2C_BUSY_IN_RX	(1U)
#define I2C_BUSY_IN_TX	(2U)

/*
 * @fm_duty_cycle
 */
#define I2C_FM_DUTY_2		(0U)
#define I2C_FM_DUTY_16_9	(1U)
/*
 * @ack_control
 */
#define I2C_ACK_DI	(0U)
#define I2C_ACK_EN	(1U)
/*
 * @scl_speed
 */
#define I2C_SCL_SPEED_SM	(100000U)
#define I2C_SCL_SPEED_FM2K	(200000U)
#define I2C_SCL_SPEED_FM4K	(400000U)

/********************************************************************************************
 *
 * 								API supported by this driver
 *
 ********************************************************************************************/

/*
 * Perioheral Clock setup
 */

/***************************************************************************************
 * @fn					- i2c_peri_clock_control
 *
 * @brief				- Enables or disables peripheral clock for the given i2c
 *
 * @param[in]			- Base address of the i2c peripheral
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_peri_clock_control(i2c_reg_t *arg_ptr_i2c,
		uint8_t arg_u8enable_or_disable);

/*
 * Init and De-init
 */

/***************************************************************************************
 * @fn					- i2c_init
 *
 * @brief				- Initialise i2c according to a given configuration
 *
 * @param[in]			- address of to a structure of the i2c
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_init(i2c_handle_t *arg_ptr_oi2c_handler);

/***************************************************************************************
 * @fn					- i2c_irq_config
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
void i2c_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
		uint8_t arg_enable_or_disable);

void i2c_ev_irq_handler(i2c_handle_t *arg_poi2c_handler);

void i2c_er_irq_handler(i2c_handle_t *arg_poi2c_handler);

void i2c_irq_slave_manage_callback_events (i2c_reg_t *arg_ptr_i2c, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- i2c_enable
 *
 * @brief				- Enable or disable the given i2c
 *
 * @param[in]			- Base address of the i2c peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_control(i2c_reg_t *arg_ptr_i2c, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- i2c_config
 *
 * @brief				- Enable or disable the ssi of the given i2c
 *
 * @param[in]			- Base address of the i2c peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_config(i2c_reg_t *arg_ptr_i2c, uint8_t arg_enable_or_disable);

/*
 * I2C application call back
 */
void i2c_app_callback(i2c_handle_t *arg_poi2c_handler, uint8_t arg_u8evt);

/***************************************************************************************
 * @fn					- i2c_soe_config
 *
 * @brief				- Enable or disable the soe of the given i2c
 *
 * @param[in]			- Base address of the i2c peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_soe_config(i2c_reg_t *arg_ptr_i2c, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- i2c_de_init
 *
 * @brief				- De-init a given i2c
 *
 * @param[in]			- Base address of the i2c peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void i2c_de_init(i2c_reg_t *arg_ptr_i2c);

/***************************************************************************************
 * @fn					- i2c_get_flag_status
 *
 * @brief				- get a given flag status of a given i2c peripheral status register bit
 *
 * @param[in]			- Base address of the i2c peripheral
 * @param[in]			- i2c peripheral status register bit
 *
 * @return				- FLAG_SET or FLAG_RESET
 *
 * @Note				- none
 */
uint8_t i2c_get_flag_status(i2c_reg_t *arg_ptr_i2c, uint32_t arg_u8flag);

/*
 * Data send and receive
 */
/***************************************************************************************
 * @fn					- i2c_send_data
 *
 * @brief				- Send data to a data register
 *
 * @param[in]			- structure containing the i2c and the information relate to the tranceiver
 * @param[in]			- Base address of the data to be sent
 * @param[in]			- size of the data to be sent
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
void i2c_master_send_data(i2c_handle_t *arg_ptr_oi2c_handler,
		uint8_t *arg_ptr_u8tx_buffer, uint32_t arg_u32len,
		uint8_t arg_u8slave_addr, uint8_t arg_u8repeated_start);

/***************************************************************************************
 * @fn					- i2c_receive_data
 *
 * @brief				- Receive data from a data register
 *
 * @param[in]			- structure containing the i2c and the information relate to the tranceiver
 * @param[in]			- Base address for the received data
 * @param[in]			- size of the data to be received
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
void i2c_master_receive_data(i2c_handle_t *arg_ptr_oi2c_handler,
		uint8_t *arg_ptr_u8tx_buffer, uint32_t arg_u32len,
		uint8_t arg_u8slave_addr, uint8_t arg_u8repeated_start);

/***************************************************************************************
 * @fn					- i2c_slave_send_data
 *
 * @brief				- Send data to a data register
 *
 * @param[in]			- Base address of I2C
 * @param[in]			- data to be send
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
void i2c_slave_send_data(i2c_reg_t *arg_poi2c_reg, uint8_t arg_u8data);

/***************************************************************************************
 * @fn					- i2c_slave_receive_data
 *
 * @brief				- Receive data from a data register
 *
 * @param[in]			- Base address of I2C
 *
 * @return				- data read
 *
 * @Note				- This is a blocking call
 */
uint8_t i2c_slave_receive_data(i2c_reg_t *arg_poi2c_reg);

/***************************************************************************************
 * @fn					- i2c_master_send_data_it
 *
 * @brief				- Send data to a data register
 *
 * @param[in]			- structure containing the i2c and the information relate to the tranceiver
 * @param[in]			- Base address of the data to be sent
 * @param[in]			- size of the data to be sent
 *
 * @return				- application state see @appState
 *
 * @Note				- This is a non blocking call
 */
uint8_t i2c_master_send_data_it(i2c_handle_t *arg_poi2c_handler,
		uint8_t *arg_pu8tx_buffer, uint32_t arg_u32len,
		uint8_t arg_u8slave_addr, uint8_t arg_u8repeated_start);

/***************************************************************************************
 * @fn					- i2c_master_receive_data_it
 *
 * @brief				- Receive data from a data register
 *
 * @param[in]			- structure containing the i2c and the information relate to the tranceiver
 * @param[in]			- Base address for the received data
 * @param[in]			- size of the data to be received
 *
 * @return				- application state see @appState
 *
 * @Note				- This is a non blocking call
 */
uint8_t i2c_master_receive_data_it(i2c_handle_t *arg_poi2c_handler,
		uint8_t *arg_pu8rx_buffer, uint32_t arg_u32len,
		uint8_t arg_u8slave_addr, uint8_t arg_u8repeated_start);

/*
 * IRQ configuration & ISR handling
 */

/*
 * Other Peripherals Control APIs
 */

/*
 * IRQ Configuration and ISR handling
 */

/*
 * i2c application callback
 */

void i2c_manage_ack(i2c_reg_t *arg_poi2c_reg, uint8_t arg_u8en_or_di);

void i2c_generate_stop_condition(i2c_reg_t *arg_poi2c_reg);
void i2c_close_send_data(i2c_handle_t *arg_poi2c_handler);
void i2c_close_receive_data(i2c_handle_t *arg_poi2c_handler);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
