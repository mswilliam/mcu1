/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 5, 2020
 *      Author: mangwi01
 */
#include "stm32f407xx_spi_driver.h"
#include <string.h>

/*
 * declaration of helper functions
 */
static void spi_irq_handling_tx(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);
static void spi_irq_handling_rx(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);
static void spi_irq_handling_err(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler);

/*
 * Perioheral Clock setup
 */
void spi_peri_clock_control(spi_reg_t *arg_ptr_spi,
		uint8_t arg_enable_or_disable) {

	switch ((uint32_t) arg_ptr_spi) {
	case (uint32_t) SPI1: {
		if (ENABLE == arg_enable_or_disable) {
			SPI1_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI1_PCLK_DI();
		}
	}
		break;
	case (uint32_t) SPI2: {
		if (ENABLE == arg_enable_or_disable) {
			SPI2_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI2_PCLK_DI();
		}
	}
		break;
	case (uint32_t) SPI3: {
		if (ENABLE == arg_enable_or_disable) {
			SPI3_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI3_PCLK_DI();
		}
	}
		break;
	case (uint32_t) SPI4: {
		if (ENABLE == arg_enable_or_disable) {
			SPI4_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI4_PCLK_DI();
		}
	}
		break;
	case (uint32_t) SPI5: {
		if (ENABLE == arg_enable_or_disable) {
			SPI5_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI5_PCLK_DI();
		}
	}
		break;
	case (uint32_t) SPI6: {
		if (ENABLE == arg_enable_or_disable) {
			SPI6_PCLK_EN();
		} else if (DISABLE == arg_enable_or_disable) {
			SPI6_PCLK_DI();
		}
	}
		break;
	}
}

void spi_control(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable) {
	if (ENABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR1 |= (SET << SPI_CR1_SPE);
	} else if (DISABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR1 &= ~(SET << SPI_CR1_SPE);
	}
}

void spi_ssi_config(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable) {
	if (ENABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR1 |= (SET << SPI_CR1_SSI);
	} else if (DISABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR1 &= ~(SET << SPI_CR1_SSI);
	}
}

void spi_soe_config(spi_reg_t *arg_ptr_spi, uint8_t arg_enable_or_disable) {
	if (ENABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR2 |= (SET << SPI_CR2_SSOE);
	} else if (DISABLE == arg_enable_or_disable) {
		arg_ptr_spi->CR2 &= ~(SET << SPI_CR2_SSOE);
	}
}

void spi_init(spi_handle_t *arg_ptr_sspi_handler) {
	/*!< Perioheral Clock enable for spi*/
	spi_peri_clock_control(arg_ptr_sspi_handler->ptr_sspi, ENABLE);

	if (SPI_MSTR_MASTER == arg_ptr_sspi_handler->ptr_sconfig->master_or_slave) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_MSTR);
	} else if (SPI_MSTR_SLAVE
			== arg_ptr_sspi_handler->ptr_sconfig->master_or_slave) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_MSTR);
	}

	if (SPI_DIRECTION_FD == arg_ptr_sspi_handler->ptr_sconfig->direction) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_BIDIMODE);
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_RXONLY);
	} else if (SPI_DIRECTION_HD
			== arg_ptr_sspi_handler->ptr_sconfig->direction) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_BIDIMODE);
	} else if (SPI_DIRECTION_RX
			== arg_ptr_sspi_handler->ptr_sconfig->direction) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_BIDIMODE);
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_RXONLY);
	}

	arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(THREE_BIT << SPI_CR1_BR);
	arg_ptr_sspi_handler->ptr_sspi->CR1 |=
			(arg_ptr_sspi_handler->ptr_sconfig->baud_rate_ctrl << SPI_CR1_BR);

	if (SPI_DFF_16BITS == arg_ptr_sspi_handler->ptr_sconfig->dff) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_DFF);
	} else if (SPI_DFF_8BITS == arg_ptr_sspi_handler->ptr_sconfig->dff) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_DFF);
	}

	if (SPI_CPOL_CLCK_HIGH == arg_ptr_sspi_handler->ptr_sconfig->cpol) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_CPOL);
	} else if (SPI_CPOL_CLCK_LOW == arg_ptr_sspi_handler->ptr_sconfig->cpol) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_CPOL);
	}

	if (SPI_CPHA_CAPTURE_ON_TRANSIT2
			== arg_ptr_sspi_handler->ptr_sconfig->cpha) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_CPHA);
	} else if (SPI_CPHA_CAPTURE_ON_TRANSIT1
			== arg_ptr_sspi_handler->ptr_sconfig->cpha) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_CPHA);
	}

	if (SPI_SSM_EN == arg_ptr_sspi_handler->ptr_sconfig->ssm) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 |= (SET << SPI_CR1_SSM);
	} else if (SPI_SSM_DI == arg_ptr_sspi_handler->ptr_sconfig->ssm) {
		arg_ptr_sspi_handler->ptr_sspi->CR1 &= ~(SET << SPI_CR1_SSM);
	}

}

void spi_de_init(spi_reg_t *arg_ptr_spi) {
	switch ((uint32_t) arg_ptr_spi) {
	case (uint32_t) SPI1: {
		SPI1_RST();
	}
		break;
	case (uint32_t) SPI2: {
		SPI2_RST();
	}
		break;
	case (uint32_t) SPI3: {
		SPI3_RST();
	}
		break;
	case (uint32_t) SPI4: {
		SPI4_RST();
	}
		break;
	case (uint32_t) SPI5: {
		SPI5_RST();
	}
		break;
	case (uint32_t) SPI6: {
		SPI6_RST();
	}
		break;
	}
}

uint8_t spi_get_flag_status(spi_reg_t *arg_ptr_spi, uint32_t arg_u8flag) {
	uint8_t loc_u8flag = 0;
	if (arg_ptr_spi->SR & arg_u8flag) {
		loc_u8flag = FLAG_SET;
	} else {
		loc_u8flag = FLAG_RESET;
	}
	return loc_u8flag;
}

void spi_send_data(spi_reg_t *arg_ptr_spi, uint8_t *arg_ptr_u8tx_buffer,
		uint32_t arg_u32len) {
	uint16_t loc_u16data = 0U;
	uint8_t loc_u8data_len = 0U;
	if (SPI_DFF_8BITS == ((arg_ptr_spi->CR1 >> SPI_CR1_DFF) & ENABLE)) {
		loc_u8data_len = 1U;
	} else if (SPI_DFF_16BITS == ((arg_ptr_spi->CR1 >> SPI_CR1_DFF) & ENABLE)) {
		loc_u8data_len = 2U;
	}

	for (uint32_t loc_u32_len_transmitted = 0;
			loc_u32_len_transmitted < arg_u32len; loc_u32_len_transmitted +=
					loc_u8data_len) {
		memcpy(&loc_u16data, arg_ptr_u8tx_buffer + loc_u32_len_transmitted,
				(loc_u8data_len <= arg_u32len - loc_u32_len_transmitted ?
						loc_u8data_len : arg_u32len - loc_u32_len_transmitted));
		while (FLAG_RESET == spi_get_flag_status(arg_ptr_spi, SPI_FLAG_TX_EMPTY))
			;
		arg_ptr_spi->DR = loc_u16data;
	}
}

void spi_receive_data(spi_reg_t *arg_ptr_spi, uint8_t *arg_ptr_u8rx_buffer,
		uint32_t arg_u32len) {
	uint16_t loc_u16data = 0U;
	uint8_t loc_u8data_len = 0U;
	if (SPI_DFF_8BITS == ((arg_ptr_spi->CR1 >> SPI_CR1_DFF) & ENABLE)) {
		loc_u8data_len = 1U;
	} else if (SPI_DFF_16BITS == ((arg_ptr_spi->CR1 >> SPI_CR1_DFF) & ENABLE)) {
		loc_u8data_len = 2U;
	}

	for (uint32_t loc_u32_len_received = 0; loc_u32_len_received < arg_u32len;
			loc_u32_len_received += loc_u8data_len) {
		while (FLAG_RESET
				== spi_get_flag_status(arg_ptr_spi, SPI_FLAG_RX_NOT_EMPTY))
			;
		loc_u16data = arg_ptr_spi->DR;

		memcpy(arg_ptr_u8rx_buffer + loc_u32_len_received, &loc_u16data,
				(loc_u8data_len <= arg_u32len - loc_u32_len_received ?
						loc_u8data_len : arg_u32len - loc_u32_len_received));
	}
}

uint8_t spi_send_data_ti(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t *arg_ptr_u8tx_buffer, uint32_t arg_u32len) {
	uint8_t loc_u8result = SPI_BUSY_IN_TX;
	if (SPI_READY == arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state) {
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer =
				arg_ptr_u8tx_buffer;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len = arg_u32len;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state = SPI_BUSY_IN_TX;
		/*!< TXE interrupt not masked. Used to generate an interrupt request when the TXE flag is set.*/
		arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2 |=
				(SET << SPI_CR2_TXEIE);
		loc_u8result = SPI_READY;
	}
	return loc_u8result;
}

uint8_t spi_receive_data_ti(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t *arg_ptr_u8rx_buffer, uint32_t arg_u32len) {
	uint8_t loc_u8result = SPI_BUSY_IN_RX;
	if (SPI_READY == arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state) {
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer =
				arg_ptr_u8rx_buffer;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len = arg_u32len;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state = SPI_BUSY_IN_RX;
		/*!< RXNE interrupt not masked. Used to generate an interrupt request when the RXNE flag is set.*/
		arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2 |=
				(SET << SPI_CR2_RXNEIE);
		loc_u8result = SPI_READY;
	}
	return loc_u8result;
}

void spi_irq_config(uint8_t arg_irq_number, uint8_t arg_irq_priority,
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

void spi_irq_handling(spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	uint8_t loc_u8is_expected_it = FLAG_SET
			== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2)
					& (SET << SPI_CR2_RXNEIE));
	uint8_t loc_u8is_trigerred_it = FLAG_SET
			== spi_get_flag_status(arg_ptr_sspi_tranceiver_handler->ptr_sspi,
			SPI_FLAG_RX_NOT_EMPTY);

	if (loc_u8is_expected_it && loc_u8is_trigerred_it) {
		spi_irq_handling_rx(arg_ptr_sspi_tranceiver_handler);
	} else {
		loc_u8is_expected_it = FLAG_SET
				== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2)
						& (SET << SPI_CR2_TXEIE));
		loc_u8is_trigerred_it = FLAG_SET
				== spi_get_flag_status(
						arg_ptr_sspi_tranceiver_handler->ptr_sspi,
						SPI_FLAG_TX_EMPTY);
		if (loc_u8is_expected_it && loc_u8is_trigerred_it) {
			spi_irq_handling_tx(arg_ptr_sspi_tranceiver_handler);
		} else {
			loc_u8is_expected_it = FLAG_SET
					== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2)
							& (SET << SPI_CR2_ERRIE));
			loc_u8is_trigerred_it = FLAG_SET
					== spi_get_flag_status(
							arg_ptr_sspi_tranceiver_handler->ptr_sspi,
							SPI_FLAG_ERROR);

			if (loc_u8is_expected_it && loc_u8is_trigerred_it) {

				spi_irq_handling_err(arg_ptr_sspi_tranceiver_handler);
			}
		}
	}
}

void spi_irq_handling_err(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	spi_clear_ovr_flag(arg_ptr_sspi_tranceiver_handler);
	/*!< inform the application*/
	if (SPI_BUSY_IN_RX
			== arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state) {
	} else if (SPI_BUSY_IN_TX
			== arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state) {
		spi_application_event_callback(arg_ptr_sspi_tranceiver_handler,
		SPI_EVENT_TX_ERROR);
	} else {
		spi_application_event_callback(arg_ptr_sspi_tranceiver_handler,
		SPI_EVENT_UNKNOWN_ERROR);
	}
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state = SPI_READY;
}

void spi_irq_handling_rx(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	if (SPI_DFF_8BITS
			== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR1 >> SPI_CR1_DFF)
					& ENABLE)) {
		*(arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer) =
				(uint8_t) arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len--;

	} else if (SPI_DFF_16BITS
			== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR1 >> SPI_CR1_DFF)
					& ENABLE)) {
		if (arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len <= 1U) {
			*(arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer) =
					(uint8_t) arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR;
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len--;
		} else {
			*((uint16_t*) arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer) =
					(uint16_t) arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR;
			(uint16_t*) arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len -= 2U;
		}
	}

	if (0U == arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len) {
		/*!< Close the spi transmission and inform the application that the rx is over*/
		spi_close_reception(arg_ptr_sspi_tranceiver_handler);
		spi_application_event_callback(arg_ptr_sspi_tranceiver_handler,
		SPI_EVENT_RX_CMPLT);
	}
}

void spi_irq_handling_tx(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	if (SPI_DFF_8BITS
			== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR1 >> SPI_CR1_DFF)
					& ENABLE)) {
		arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR =
				*(arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer);
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
		arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len--;
	} else if (SPI_DFF_16BITS
			== ((arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR1 >> SPI_CR1_DFF)
					& ENABLE)) {
		if (arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len <= 1U) {
			arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR =
					*(arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer);
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len--;
		} else {
			arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR =
					*((uint16_t*) arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer);
			(uint16_t*) arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer++;
			arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len -= 2U;
		}
	}
	if (0U == arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len) {
		/*!< Close the spi transmission and inform the application that the tx is over*/
		spi_close_transmission(arg_ptr_sspi_tranceiver_handler);
		spi_application_event_callback(arg_ptr_sspi_tranceiver_handler,
		SPI_EVENT_TX_CMPLT);
	}
}

void spi_clear_ovr_flag(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	/*!< clear the overrun flag by reading Dr & SR registers*/
	if (arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state != SPI_BUSY_IN_TX) {
		uint8_t loc_u8temp = arg_ptr_sspi_tranceiver_handler->ptr_sspi->DR;
		loc_u8temp = arg_ptr_sspi_tranceiver_handler->ptr_sspi->SR;
		(void) loc_u8temp; /*unused variable*/
	}
}

void spi_close_transmission(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {
	arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2 &= ~(SET << SPI_CR2_TXEIE);
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len = 0U;
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state = SPI_READY;
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer = NULL;
}

void spi_close_reception(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler) {

	arg_ptr_sspi_tranceiver_handler->ptr_sspi->CR2 &= ~(SET << SPI_CR2_RXNEIE);
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->len = 0U;
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->state = SPI_READY;
	arg_ptr_sspi_tranceiver_handler->ptr_tranceiver->p_rx_tx_buffer = NULL;
}

__attribute__((weak)) void spi_application_event_callback(
		spi_tranceiver_handle_t *arg_ptr_sspi_tranceiver_handler,
		uint8_t arg_u8event) {
	// This is a weak implementation. The application may override this function
}
