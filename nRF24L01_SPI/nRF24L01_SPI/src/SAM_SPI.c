/*
 * SAM_SPI.c
 *
 * Created: 11/09/2020 21:37:39
 *  Author: Design
 */ 

#include "SAM_SPI.h"
#include <asf.h>

/* SPI clock default setting (Hz). */
uint32_t gs_ul_spi_clock = 5000000;

/**
 * \brief Initialize SPI as master.
 */
void spi_master_initialize(void)
{
	puts("-I- Initialize SPI as master\r");

	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI0);
	spi_disable(SPI0);
	spi_reset(SPI0);
	spi_set_lastxfer(SPI0);
	spi_set_master_mode(SPI0);
	spi_disable_mode_fault_detect(SPI0);
	spi_set_peripheral_chip_select_value(SPI0, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI0, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI0, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI0, SPI_CHIP_SEL,
	SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI0, SPI_CHIP_SEL,
	(sysclk_get_peripheral_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI0, SPI_CHIP_SEL, SPI_DLYBS,
	SPI_DLYBCT);
	spi_enable(SPI0);
}

/**
 * \brief Set the specified SPI clock configuration.
 *
 * \param configuration  Index of the configuration to set.
 */
void spi_set_clock_configuration(uint8_t configuration)
{
	//spi_disable_xdmac();
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_SetPriority(SPI_IRQn, 0);
	NVIC_EnableIRQ(SPI_IRQn);

	printf("Setting SPI clock #%lu ... \n\r", (unsigned long)gs_ul_spi_clock);
	spi_master_initialize();
}

/**
 * \brief Perform SPI master transfer.
 *
 * \param p_buf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 * 
 * \brief after function p_buf will contain the received SPI data  
 */
void spi_master_transfer(void *p_buf, uint32_t size)
{
	uint32_t i;
	uint8_t uc_pcs;
	static uint16_t data;

	uint8_t *p_buffer;

	p_buffer = p_buf;

	for (i = 0; i < size; i++) {
		spi_write(SPI0, p_buffer[i], 0, 0);
		
		/* Wait transfer done. */
		while ((spi_read_status(SPI0) & SPI_SR_RDRF) == 0);
		spi_read(SPI0, &data, &uc_pcs);
		p_buffer[i] = data;
	}
	delay_us(5);
}