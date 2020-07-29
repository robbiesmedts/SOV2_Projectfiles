/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "nRF24L01.h"
//#include "nRF24.h"
/*
#define CYCLES_IN_DLYTICKS_FUNC        8
#define MS_TO_DLYTICKS(ms)          (U32)(F_CPU / 1000 * ms / CYCLES_IN_DLYTICKS_FUNC) // ((float)(F_CPU)) / 1000.0
#define DelayTicks(ticks)            {volatile U32 n=ticks; while(n--);}//takes 8 cycles
#define DelayMs(ms)                    DelayTicks(MS_TO_DLYTICKS(ms))//uses 20bytes
*/
#define _BV(n) (1 << n)

#define CE PIO_PC9_IDX

#define RF24_1MBPS 0
#define RF24_2MBPS 1

typedef enum{
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;

uint8_t payload_size = 32;
uint8_t addr_width = 5;

#define SPI_Handler     SPI0_Handler
#define SPI_IRQn        SPI0_IRQn

/* chip select */
#define SPI_CHIP_SEL 0
#define SPI_CHIP_PCS spi_get_pcs(SPI_CHIP_SEL)

/*clock polarity*/
#define SPI_CLK_POLARITY 0

/*Clock phase*/
#define SPI_CLK_PHASE 0

/* Delay before SPCK. */
#define SPI_DLYBS 0x40

/* Delay between consecutive transfers. */
#define SPI_DLYBCT 0x10

/* Number of SPI clock configurations. */
#define NUM_SPCK_CONFIGURATIONS 1

uint32_t g_uc_role;

/** spi mode definition*/
#define	MASTER_MODE   0
#define SLAVE_MODE	  1

/** The buffer size for transfer  */
#define BUFFER_SIZE          100

/* SPI clock default setting (Hz). */
static uint32_t gs_ul_spi_clock = 10000000;

#define STRING_EOL    "\r"
#define STRING_HEADER "--Spi nRF24L01 Test --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

uint32_t txDelay;

/**
 * \brief Initialize SPI as master.
 */
static void spi_master_initialize(void)
{
	g_uc_role = MASTER_MODE;
	puts("-I- Initialize SPI as master\r");

	/* Configure an SPI peripheral. */
	spi_enable_clock(SPI_MASTER_BASE);
	spi_disable(SPI_MASTER_BASE);
	spi_reset(SPI_MASTER_BASE);
	spi_set_lastxfer(SPI_MASTER_BASE);
	spi_set_master_mode(SPI_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI_MASTER_BASE, SPI_CHIP_PCS);
	spi_set_clock_polarity(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_POLARITY);
	spi_set_clock_phase(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_CLK_PHASE);
	spi_set_bits_per_transfer(SPI_MASTER_BASE, SPI_CHIP_SEL,
			SPI_CSR_BITS_8_BIT);
	spi_set_baudrate_div(SPI_MASTER_BASE, SPI_CHIP_SEL,
			(sysclk_get_peripheral_hz() / gs_ul_spi_clock));
	spi_set_transfer_delay(SPI_MASTER_BASE, SPI_CHIP_SEL, SPI_DLYBS,
			SPI_DLYBCT);
	spi_enable(SPI_MASTER_BASE);
}

/**
 * \brief Set the specified SPI clock configuration.
 *
 * \param configuration  Index of the configuration to set.
 */
static void spi_set_clock_configuration(uint8_t configuration)
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
 * \param pbuf Pointer to buffer to transfer.
 * \param size Size of the buffer.
 * 
 * \brief after function p_buf will contain the received SPI data  
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

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief read a register of the nRF24L01 transceiver
 * 
 * \param reg register to read
 * \return data register
 */
uint8_t nRF24_readRegister(uint8_t reg)
{
	uint8_t cmd[2] = {R_REGISTER | (REGISTER_MASK & reg), 0xFF};
	
	spi_master_transfer(&cmd, sizeof(cmd));
	
	/** contents of cmd after transfer:
	 * [0] contains STATUS register
	 * [1] contains requested register
	*/
	return cmd[1]; 
}

/**
 * \brief write to a register of the nRF24L01 transceiver
 * 
 * \param reg register to write
 * \param value to write
 * \return STATUS register 
 */
uint8_t nRF24_writeRegister(uint8_t reg, uint8_t val)
{
	uint8_t p_buf[2];
	
	p_buf[0] = (W_REGISTER | (REGISTER_MASK & reg));
	p_buf[1] = val;
	
	
	spi_master_transfer(p_buf, sizeof(p_buf));
	/** contents of p_buf
	* [0] Status register
	* [1] unknown data
	*/
	return p_buf[0];
}
 
/**
 * \brief flush the RX buffer of the nRF24L01 transceiver
 * \return STATUS
 */
uint8_t nRF24_FlushRx(void)
{
	uint8_t cmd;
	cmd = FLUSH_RX;
	
	spi_master_transfer(&cmd, sizeof(cmd));
	
	return cmd;
}
 
/**
 * \brief flush the TX buffer of the nRF24L01 transceiver
 * \return STATUS
 */
uint8_t nRF24_FlushTx(void)
{
	uint8_t cmd;
	cmd = FLUSH_TX;
	
	spi_master_transfer(&cmd, sizeof(cmd));
	return cmd;
}

/**
 * \brief Read the Status register of the nRF24L01 transceiver
 * \return STATUS
 */
uint8_t nRF24_getStatus(void)
{
	uint8_t cmd;
	cmd = RF24_NOP;
	
	spi_master_transfer(&cmd, sizeof(cmd));
	return cmd;
}
 
void nRF24_setRetries(uint8_t delay, uint8_t count)
{
	nRF24_writeRegister(SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) <<ARC );
}

bool nRF24_setDataRate(uint8_t speed)
{
	bool result = false;
	uint8_t setup = nRF24_readRegister(RF_SETUP);
	
	if (speed == RF24_1MBPS)
	{
		setup &= setup & 0xF7;
		txDelay = 450;
	}
	else
	{
		setup |= setup | 0x08;
		txDelay = 190;
	}
	nRF24_writeRegister(RF_SETUP, setup);
	
	if(nRF24_readRegister(RF_SETUP) == setup)
		result = true;
	
	return result;
}

void nRF24_setCRCLength(rf24_crclength_e length)
{
	uint8_t config = nRF24_readRegister(NRF_CONFIG) & ~(_BV(CRCO) | _BV(EN_CRC));
	
	if (length == RF24_CRC_DISABLED){
		// do nothing we turned it off above
	} 
	else if (length == RF24_CRC_8){
		config |= _BV(EN_CRC);
	} 
	else
	{
		config |= _BV(EN_CRC);
		config |= _BV(CRCO);
	}
	nRF24_writeRegister(NRF_CONFIG, config);
}

void toggle_features(void)
{
	uint8_t config[2] = {ACTIVATE, 0x73};
	
	spi_master_transfer(config, sizeof(config));
}

void nRF24_setChannel(uint8_t channel)
{
	const uint8_t max_channel = 125;
	if (channel > max_channel)
		nRF24_writeRegister(RF_CH, max_channel);
	else
		nRF24_writeRegister(RF_CH, channel);
}

void nRF24_powerUp(void)
{
	uint8_t config = nRF24_readRegister(NRF_CONFIG);
	
	if (!(config & _BV(PWR_UP))){
		nRF24_writeRegister(NRF_CONFIG, config | _BV(PWR_UP));
		//delay 5ms
	}
	//DelayMs(5);
}

bool nRF24_begin(void)
{
	uint8_t setup = 0;
	ioport_set_pin_dir(CE, IOPORT_DIR_OUTPUT);//ce_pin PC9
	ioport_set_pin_level(CE, 0);
	
	nRF24_writeRegister(NRF_CONFIG, 0x0C);
	nRF24_setRetries(5, 15);
	setup = nRF24_readRegister(RF_SETUP);
	
	nRF24_setDataRate(RF24_1MBPS);
	nRF24_setCRCLength(RF24_CRC_16);
	toggle_features();
	
	nRF24_writeRegister(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	
	nRF24_setChannel(76);
	
	nRF24_FlushRx();
	nRF24_FlushTx();
	
	nRF24_powerUp();
	
	nRF24_writeRegister(NRF_CONFIG, (nRF24_readRegister(NRF_CONFIG)) & ~_BV(PRIM_RX));
	
	return (setup != 0 && setup != 0xFF);
}

void nRF24_openWritingPipe(uint64_t address)
{
	nRF24_writeRegister(RX_ADDR_P0, *(uint8_t *)(&address));
	nRF24_writeRegister(TX_ADDR, *(uint8_t *)(&address));
	
	nRF24_writeRegister(RX_PW_P0, payload_size);
}

void nRF24_setPayloadSize(uint8_t size)
{
	const uint8_t max_size = 32;
	
	if (size > max_size)
		payload_size = 32;
	else
		payload_size = size;
	
}

uint8_t nRF24_getpayloadSize(void)
{
	return payload_size;
}

int main (void)
{
	uint8_t uc_key;

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Initialize the console UART. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);
	/* Insert application code here, after the board has been initialized. */
	
	spi_master_initialize();
	
	while(1)
	{
		
	}
}
