/*
 * SAM_SPI.h
 *
 * Created: 11/09/2020 21:38:28
 *  Author: Design
 */ 


#ifndef SAM_SPI_H_
#define SAM_SPI_H_

#include "spi_master.h"

#define SPI_Handler     SPI0_Handler
#define SPI_IRQn        SPI0_IRQn

/* chip select */
#define SPI_CHIP_SEL 1
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/*clock polarity*/
#define SPI_CLK_POLARITY 0

/*Clock phase*/
#define SPI_CLK_PHASE 1

/* Delay before SPCK. */
#define SPI_DLYBS 0x10

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x04

/* Number of SPI clock configurations. */
#define NUM_SPCK_CONFIGURATIONS 1

/** spi mode definition*/
#define	MASTER_MODE   0

extern uint32_t gs_ul_spi_clock;

void spi_master_initialize(void);
void spi_set_clock_configuration(uint8_t configuration);
void spi_master_transfer(void *p_buf, uint32_t size);

#endif /* SAM_SPI_H_ */