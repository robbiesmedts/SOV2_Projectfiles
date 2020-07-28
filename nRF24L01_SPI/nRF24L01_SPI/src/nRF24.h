/*
 * nRF24.h
 *
 * Created: 28/07/2020 15:26:25
 *  Author: robbi
 */ 


#ifndef NRF24_H_
#define NRF24_H_

//public functions


//private functions
static uint8_t spi_master_transfer(void *p_buf, uint32_t size);
uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);

#endif /* NRF24_H_ */