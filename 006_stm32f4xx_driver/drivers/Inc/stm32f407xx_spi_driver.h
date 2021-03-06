/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Oct 5, 2020
 *      Author: mangwi01
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*************************** Possible spi application states *******************/
#define SPI_READY		(0U)
#define SPI_BUSY_IN_RX	(1U)
#define SPI_BUSY_IN_TX	(2U)

/*
 * Possible application events
 */
#define SPI_EVENT_RX_CMPLT		(0U)
#define SPI_EVENT_TX_CMPLT		(1U)
#define SPI_EVENT_RX_ERROR		(2U)
#define SPI_EVENT_TX_ERROR		(3U)
#define SPI_EVENT_UNKNOWN_ERROR	(4U)

/*************************** Bit position definition for spi peripheral driver *******************/

#define SPI_CR1_CPHA        (0U)
#define SPI_CR1_CPOL        (1U)
#define SPI_CR1_MSTR        (2U)
#define SPI_CR1_BR          (3U)
#define SPI_CR1_SPE         (6U)
#define SPI_CR1_LSB         (7U)
#define SPI_CR1_SSI         (8U)
#define SPI_CR1_SSM         (9U)
#define SPI_CR1_RXONLY	    (10U)
#define SPI_CR1_DFF	        (11U)
#define SPI_CR1_CRC_NEXT    (12U)
#define SPI_CR1_CRC_EN      (13U)
#define SPI_CR1_BIDIOE	    (14U)
#define SPI_CR1_BIDIMODE	(15U)

#define SPI_CR2_RXDMAEN   (0U)
#define SPI_CR2_TXDMAEN   (1U)
#define SPI_CR2_SSOE      (2U)
#define SPI_CR2_FRF       (4U)
#define SPI_CR2_ERRIE     (5U)
#define SPI_CR2_RXNEIE    (6U)
#define SPI_CR2_TXEIE     (7U)

#define SPI_SR_RXNE     (0U)
#define SPI_SR_TXE      (1U)
#define SPI_SR_CHSIDE   (2U)
#define SPI_SR_UDR      (3U)
#define SPI_SR_CRC      (4U)
#define SPI_SR_MODF     (5U)
#define SPI_SR_OVR      (6U)
#define SPI_SR_BSY      (7U)
#define SPI_SR_FRE      (8U)

/*
 * Handle structure for a SPI pin
 */

/*
 * @SPI_MSTR
 * MSTR (Master selection) possible values
 */
#define SPI_MSTR_SLAVE    (0x0U)	/*!< 0: Slave configuration*/
#define SPI_MSTR_MASTER   (0x1U)	/*!< 1: Master configuration*/

/*
 * @SPI_DIRECTION
 * DIRECTION direction possible values
 */
#define SPI_DIRECTION_FD  (0x0U)	/*!< 0: full duplex*/
#define SPI_DIRECTION_HD  (0x1U)	/*!< 1: half duplex*/
#define SPI_DIRECTION_RX  (0x2U)	/*!< 1: receive only*/

/*
 * @SPI_BIDIMODE
 * BIDIMODE (Bidirectional data mode enable) possible values
 */
#define SPI_BIDIMODE_2LINE  (0x0U)	/*!< 0: 2-line unidirectional data mode selected*/
#define SPI_BIDIMODE_1LINE  (0x1U)	/*!< 1: 1-line bidirectional data mode selected*/

/*
 * @SPI_BIDIOE
 * BIDIOE (Output enable in bidirectional mode) possible values :
 * This bit combined with the BIDImode bit selects the direction of transfer in bidirectional mode.
 * In master mode, the MOSI pin is used while the MISO pin is used in slave mode.
 */
#define SPI_BIDIOE_OUT_DI  (0x0U)	/*!< 0: Output disabled (receive-only mode)*/
#define SPI_BIDIOE_OUT_EN  (0x1U)	/*!< 1: Output enabled (transmit-only mode)*/

/*
 * @SPI_RXONLY
 * RXONLY (Receive onlyde) possible values :
 * This bit combined with the BIDImode bit selects the direction of transfer in 2-line
 * unidirectional mode. This bit is also useful in a multislave system in which this particular
 * slave is not accessed, the output from the accessed slave is not corrupted.
 */
#define SPI_RXONLY_DI  (0x0U)	/*!< 0: Full duplex (Transmit and receive)*/
#define SPI_RXONLY_EN  (0x1U)	/*!< 1: Output disabled (Receive-only mode)*/

/*
 * @SPI_BR
 * BR (Baud rate control) possible values :
 */
#define SPI_BR_DIV2     (0x0U)	/*!< 000: f PCLK / 2*/
#define SPI_BR_DIV4     (0x1U)	/*!< 001: f/PCLK / 4*/
#define SPI_BR_DIV8     (0x2U)	/*!< 010: f/PCLK / 8*/
#define SPI_BR_DIV16    (0x3U)	/*!< 011: f/PCLK / 16*/
#define SPI_BR_DIV32    (0x4U)	/*!< 100: f/PCLK / 32*/
#define SPI_BR_DIV64 	(0x5U)	/*!< 101: f/PCLK / 64*/
#define SPI_BR_DIV128   (0x6U)	/*!< 110: f/PCLK / 128*/
#define SPI_BR_DIV256   (0x7U)	/*!< 111: f/PCLK / 256*/

/*
 * @SPI_DFF
 * DFF (Data frame format) possible values :
 */
#define SPI_DFF_8BITS    (0x0U)	/*!< 0: 8-bit data frame format is selected for transmission/reception*/
#define SPI_DFF_16BITS   (0x1U)	/*!< 1: 16-bit data frame format is selected for transmission/reception*/

/*
 * @SPI_CPOL
 * CPOL (Clock polarity) possible values :
 */
#define SPI_CPOL_CLCK_LOW   (0x0U)	/*!< 0: CK to 0 when idle*/
#define SPI_CPOL_CLCK_HIGH  (0x1U)	/*!< 1: CK to 1 when idle*/

/*
 * @SPI_CPHA
 * CPHA (Clock phase) possible values :
 */
#define SPI_CPHA_CAPTURE_ON_TRANSIT1   (0x0U)	/*!< 0: 8-bit data frame format is selected for transmission/reception*/
#define SPI_CPHA_CAPTURE_ON_TRANSIT2   (0x1U)	/*!< 1: 16-bit data frame format is selected for transmission/reception*/

/*
 * @SPI_SSM
 * SSM (Software slave management) possible values :
 */
#define SPI_SSM_DI (0x0U)	/*!< 0: Software slave management disabled*/
#define SPI_SSM_EN (0x1U)	/*!< 1: Software slave management enabled*/

#define SPI_FLAG_TX_EMPTY	 	(SET << SPI_SR_TXE)
#define SPI_FLAG_RX_NOT_EMPTY	(SET << SPI_SR_RXNE)
#define SPI_FLAG_BSY		 	(SET << SPI_SR_BSY)
#define SPI_FLAG_ERROR		 	(SET << SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRC | SPI_SR_FRE)

typedef struct {
	uint16_t master_or_slave :1; /*!< possible values from @SPI_MSTR*/
	uint16_t direction :2; /*!< possible values from @SPI_DIRECTION*/
	uint16_t baud_rate_ctrl :3; /*!< possible values from @SPI_BR*/
	uint16_t dff :1; /*!< possible values from @SPI_DFF
	 This bit should be written only when SPI is disabled (SPE = ‘0’) for correct operation.*/
	uint16_t cpol :1; /*!< possible values from @SPI_CPOL
	 This bit should not be changed when communication is ongoing*/
	uint16_t cpha :1; /*!< possible values from @SPI_CPHA
	 This bit should not be changed when communication is ongoing*/
	uint16_t ssm :1; /*!< possible values from @SPI_SSM
	 When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.*/
	uint16_t reserved :6; /*!< reserved*/
} spi_config_t;

typedef struct {
	uint8_t *p_rx_tx_buffer; /*!< To store the app rx/tx buffer*/
	uint32_t len; /*!< To store the rx/tx len*/
	uint8_t state; /*!< To store the rx/tx state*/
} spi_tranceiver_t;

typedef struct {
	spi_reg_t *ptr_sspi; /*!< base address of the spi */
	spi_config_t *ptr_sconfig; /*!< spi configuration setting */
} spi_handle_t;

typedef struct {
	spi_reg_t *ptr_sspi; /*!< base address of the spi */
	spi_tranceiver_t *ptr_tranceiver; /*!< spi tranceuver informations */
} spi_tranceiver_handle_t;

/********************************************************************************************
 *
 * 								API supported by this driver
 *
 ********************************************************************************************/

/*
 * Perioheral Clock setup
 */

/***************************************************************************************
 * @fn					- spi_peri_clock_control
 *
 * @brief				- Enables or disables peripheral clock for the given spi
 *
 * @param[in]			- Base address of the spi peripheral
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_peri_clock_control(spi_reg_t *arg_ptr_spi,
		uint8_t arg_u8enable_or_disable);

/*
 * Init and De-init
 */

/***************************************************************************************
 * @fn					- spi_init
 *
 * @brief				- Initialise spi according to a given configuration
 *
 * @param[in]			- address of to a structure of the spi
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_init(spi_handle_t *arg_ptr_sspi_handler);

/***************************************************************************************
 * @fn					- spi_enable
 *
 * @brief				- Enable or disable the given spi
 *
 * @param[in]			- Base address of the spi peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_control(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- spi_enable
 *
 * @brief				- Enable or disable the ssi of the given spi
 *
 * @param[in]			- Base address of the spi peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_ssi_config(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- spi_soe_config
 *
 * @brief				- Enable or disable the soe of the given spi
 *
 * @param[in]			- Base address of the spi peripheral
 *
 * @param[in]			- ENABLE or DISABLE macro
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_soe_config(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- spi_de_init
 *
 * @brief				- De-init a given spi
 *
 * @param[in]			- Base address of the spi peripheral
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_de_init(spi_reg_t *arg_ptr_spi);

/***************************************************************************************
 * @fn					- spi_get_flag_status
 *
 * @brief				- get a given flag status of a given spi peripheral status register bit
 *
 * @param[in]			- Base address of the spi peripheral
 * @param[in]			- spi peripheral status register bit
 *
 * @return				- FLAG_SET or FLAG_RESET
 *
 * @Note				- none
 */
uint8_t spi_get_flag_status(spi_reg_t *arg_ptr_spi, uint32_t arg_u8flag);

/*
 * Data send and receive
 */
/***************************************************************************************
 * @fn					- spi_send_data
 *
 * @brief				- Send data to a data register
 *
 * @param[in]			- Base address of the spi peripheral
 * @param[in]			- Base address of the data to be sent
 * @param[in]			- size of the data to be sent
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
void spi_send_data(spi_reg_t *arg_ptr_spi, uint8_t *arg_ptr_u8tx_buffer,
		uint32_t arg_u32len);

/***************************************************************************************
 * @fn					- spi_receive_data
 *
 * @brief				- Receive data from a data register
 *
 * @param[in]			- Base address of the spi peripheral
 * @param[in]			- Base address for the received data
 * @param[in]			- size of the data to be received
 *
 * @return				- none
 *
 * @Note				- This is a blocking call
 */
void spi_receive_data(spi_reg_t *arg_ptr_spi, uint8_t *arg_ptr_u8rx_buffer,
		uint32_t arg_u32len);

/***************************************************************************************
 * @fn					- spi_send_data
 *
 * @brief				- Send data to a data register
 *
 * @param[in]			- structure containing the spi and the information relate to the tranceiver
 * @param[in]			- Base address of the data to be sent
 * @param[in]			- size of the data to be sent
 *
 * @return				- none
 *
 * @Note				- This is a non blocking call
 */
uint8_t spi_send_data_ti(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t *arg_ptr_u8tx_buffer, uint32_t arg_u32len);

/***************************************************************************************
 * @fn					- spi_receive_data
 *
 * @brief				- Receive data from a data register
 *
 * @param[in]			- structure containing the spi and the information relate to the tranceiver
 * @param[in]			- Base address for the received data
 * @param[in]			- size of the data to be received
 *
 * @return				- none
 *
 * @Note				- This is a non blocking call
 */
uint8_t spi_receive_data_ti(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t *arg_ptr_u8rx_buffer, uint32_t arg_u32len);

/*
 * IRQ configuration & ISR handling
 */

/*
 * Other Peripherals Control APIs
 */

/*
 * IRQ Configuration and ISR handling
 */

/***************************************************************************************
 * @fn					- spi_irq_config
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
void spi_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
		uint8_t arg_enable_or_disable);

/***************************************************************************************
 * @fn					- spi_irq_handling
 *
 * @brief				-
 *
 * @param[in]			- structure containing the spi andthe configuration parameters
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_irq_handling(spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);

/***************************************************************************************
 * @fn					- spi_clear_ovr_flag
 *
 * @brief				- Clear the overrun flag
 *
 * @param[in]			- structure containing the spi and the information relate to the tranceiver
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_clear_ovr_flag(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);

/***************************************************************************************
 * @fn					- spi_close_transmission
 *
 * @brief				- Close the transmission
 *
 * @param[in]			- structure containing the spi and the information relate to the tranceiver
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_close_transmission(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);

/***************************************************************************************
 * @fn					- spi_close_reception
 *
 * @brief				- Close the reception
 *
 * @param[in]			- structure containing the spi and the information relate to the tranceiver
 *
 * @return				- none
 *
 * @Note				- none
 */
void spi_close_reception(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);

/*
 * spi application callback
 */

void spi_application_event_callback(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t arg_u8event);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
