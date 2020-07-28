/*
 * nRF24.c
 *
 * Created: 28/07/2020 15:21:21
 *  Author: robbie
 */ 
#include "nRF24.h"

/**
 * \brief Perform SPI master transfer.
 *
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 */
static void spi_master_transfer(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {
		spi_write(SPI_MASTER_BASE, p_buffer[i], 0, 0);
		/* Wait transfer done. */
		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_RDRF) == 0);
		spi_read(SPI_MASTER_BASE, &data, &uc_pcs);
		p_buffer[i] = data;
	}
}

uint8_t read_register(uint8_t reg, uint8_t* buf)
{
	uint8_t result;
	
	
	spi_read(SPI_MASTER_BASE, p_dBuffer, 0);
	
	return p_dBuffer;
}