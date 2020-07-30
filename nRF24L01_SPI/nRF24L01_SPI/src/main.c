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
#include "string.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "nRF24L01.h"
//#include "nRF24.h"


#define CE PIO_PC9_IDX

#define RF24_1MBPS 0
#define RF24_2MBPS 1

typedef enum{
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
}rf24_crclength_e;

typedef enum {
	RF_PA_MIN = 0,
	RF_PA_LOW,
	RF_PA_HIGH,
	RF_PA_MAX,
	RF_PA_ERROR
	}rf24_pa_dbm_e;
	
static const uint8_t pipe_s[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t pipe_size_s[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
static const uint8_t pipe_enable_s[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};
static const uint8_t localAddr = 0;
static const uint64_t listeningPipes[5] = {0x3A3A3A3AD2, 0x3A3A3A3AC3, 0x3A3A3A3AB4, 0x3A3A3A3AA5, 0x3A3A3A3A96};

struct dataStruct{
	uint8_t command;
	uint64_t destAddr;
	int datavalue;
	}dataIn, dataOut;

uint32_t txDelay;
uint8_t payload_size = 32;
uint8_t addr_width = 5;
uint8_t pipe0_reading_address[5];

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

void startFastWrite(const void* buf, uint8_t len, const bool multicast);


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

uint8_t nRF_writeRegister(uint8_t reg, const uint8_t* buf, uint8_t length)
{
	uint8_t p_buf[length+1];
	
	p_buf[0] = (W_REGISTER | (REGISTER_MASK & reg));
	
	for (uint8_t i = 1; i< length; i++)
	{
		p_buf[i] = (*buf++);
	}
	spi_master_transfer(p_buf, sizeof(p_buf));
	
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

uint8_t nRF24_writePayload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
	uint8_t s_buff[data_len + 1];
	uint8_t* current = (uint8_t*) buf;
	
	s_buff[0] = writeType;
	for (uint8_t i = 1; i< data_len+1; i++)
	{
		s_buff[i] = current[i-1];
	}
	
	spi_master_transfer(s_buff, sizeof(s_buff));
	
	for (uint8_t i = 0; i< data_len; i++)
	{
		current[i] = s_buff[i+1];
	}
	
	return s_buff[0];
}

void startFastWrite(const void* buf, uint8_t len, const bool multicast)
{
	nRF24_writePayload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);

	ioport_set_pin_level(CE, 1);

}

bool nRFwrite(const void* buf, uint8_t len, const bool multicast)
{
	startFastWrite(buf, len, multicast);
	while(!(nRF24_getStatus() & ((1<<TX_DS) | (1<<MAX_RT))));
	ioport_set_pin_level(CE, 0);
	uint8_t status = nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<RX_DR) | (1<<MAX_RT));
	
	if(status & (1<<MAX_RT)){
		nRF24_FlushTx();
		return 0;
	}
	return 1;
}

void nRF24_closeReadingPipe(uint8_t pipe)
 {
	 nRF24_writeRegister(EN_RXADDR, nRF24_readRegister(EN_RXADDR) & ~(1<< pipe_enable_s[pipe]));
 }

uint8_t nRF24_readPayload(uint8_t* buf, uint8_t data_len)
{
	uint8_t s_buff[data_len+1];
	
	if (data_len > payload_size){
		data_len = payload_size;
	}
	s_buff[0] = R_RX_PAYLOAD;
	
	for (uint8_t i = 1; i< data_len+1; i++)
	{
		s_buff[i] = 0xFF;
	}
	
	spi_master_transfer(s_buff, sizeof(s_buff));
	
	for (uint8_t i = 0; i< data_len; i++)
	{
		buf[i] = s_buff[i+1];
	}
	
	return s_buff[0];
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
	uint8_t config = nRF24_readRegister(NRF_CONFIG) & ~((1<<CRCO) | (1<<EN_CRC));
	
	if (length == RF24_CRC_DISABLED){
		// do nothing we turned it off above
	} 
	else if (length == RF24_CRC_8){
		config |= (1<<EN_CRC);
	} 
	else
	{
		config |= (1<<EN_CRC);
		config |= (1<<CRCO);
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
	
	if (!(config & (1<<PWR_UP))){
		nRF24_writeRegister(NRF_CONFIG, config | (1<<PWR_UP));
		//delay 5ms
	}
	delay_ms(5);
}

void nRF24_powerDown(void)
{
	ioport_set_pin_level(CE, 0);
	nRF24_writeRegister(NRF_CONFIG, nRF24_readRegister(NRF_CONFIG) & ~(1<<PWR_UP));
}

void nRF24_startListening(void)
{
	nRF24_powerUp();
	
	nRF24_writeRegister(NRF_CONFIG, nRF24_readRegister(NRF_CONFIG) | (1<<PRIM_RX)); 
	nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
	
	ioport_set_pin_level(CE, 1);
	
	if (pipe0_reading_address[0] > 0){
		nRF_writeRegister(RX_ADDR_P0, pipe0_reading_address, addr_width);
	} else {
		nRF24_closeReadingPipe(0);
	}
	
	if (nRF24_readRegister(FEATURE) & (1<<EN_ACK_PAY)){
		nRF24_FlushTx();
	}
}

void nRF24_stopListening(void)
{
	ioport_set_pin_level(CE, 0);
	
	delay_us(txDelay);
	if (nRF24_readRegister(FEATURE) & 1<<(EN_ACK_PAY))
	{
		delay_us(txDelay);
		nRF24_FlushTx();
	}
	nRF24_writeRegister(NRF_CONFIG, (nRF24_readRegister(NRF_CONFIG)) & ~(1<<PRIM_RX));
	nRF24_writeRegister(EN_RXADDR, nRF24_readRegister(EN_RXADDR) | (1<< pipe_enable_s[0])); 
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
	
	nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
	
	nRF24_setChannel(76);
	
	nRF24_FlushRx();
	nRF24_FlushTx();
	
	nRF24_powerUp();
	
	nRF24_writeRegister(NRF_CONFIG, (nRF24_readRegister(NRF_CONFIG)) & ~(1<<PRIM_RX));
	
	return (setup != 0 && setup != 0xFF);
}

void nRF24_openWritingPipe(uint64_t address)
{
	nRF_writeRegister(RX_ADDR_P0, (uint8_t *)(&address), addr_width);
	nRF_writeRegister(TX_ADDR, (uint8_t *)(&address), addr_width);
	
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

void nRF24_setAddressWidth(uint8_t width)
{
	if (width -= 2){
		nRF24_writeRegister(SETUP_AW, width % 4);
		addr_width = (width % 4) + 2;
	} else {
		nRF24_writeRegister(SETUP_AW, 0);
		addr_width = 2;
	}
}

void nRF24_openReadingPipe(uint8_t pipe, uint64_t address)
{
	if (pipe == 0){
		memcpy(pipe0_reading_address, &address, addr_width);
	}
	if (pipe <= 5){
		if (pipe < 2){
			nRF_writeRegister(pipe_s[pipe], (uint8_t *)(&address), addr_width);
		} else {
			nRF_writeRegister(pipe_s[pipe], (uint8_t *)(&address), 1);
		}
		nRF24_writeRegister(pipe_size_s[pipe], payload_size);
	}
	nRF24_writeRegister(EN_RXADDR, nRF24_readRegister((EN_RXADDR) | (1 << pipe_enable_s[pipe])));
}

bool nRF24_available( uint8_t* pipe_num)
{
	if (!(nRF24_readRegister(FIFO_STATUS) & (1<<RX_EMPTY)))
	{
		if(pipe_num)
		{
			uint8_t status = nRF24_getStatus();
			*pipe_num = (status >> RX_P_NO) & 0x07;
		}
		return 1;
	}
	return 0;
}

void nRF24_setPALevel(uint8_t level)
{
	uint8_t setup = nRF24_readRegister(RF_SETUP) & 0xF8;
	
	
	if (level > 3) {
		level = (RF_PA_MAX << 1) + 1;
	} else {
		level = (level << 1) + 1;
	}
	nRF24_writeRegister(RF_SETUP, setup |= level);
}

uint8_t nRF24_getPALevel(void)
{
	return (nRF24_readRegister(RF_SETUP) & (1<<(RF_PWR_LOW) | (1<<RF_PWR_HIGH))) >> 1;
}

void nRF24_read(uint8_t* buf, uint8_t len)
{
	nRF24_readPayload(buf, len);
	
	nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<MAX_RT) | (1<<TX_DS));	
}

bool nRF24_write(const void* buf, uint8_t len)
{
	return nRFwrite(buf, len, 0);
}

int main (void)
{
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Initialize the console UART. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);
	/* Insert application code here, after the board has been initialized. */
	
	spi_set_clock_configuration(gs_ul_spi_clock);
	
	nRF24_begin();
	nRF24_openReadingPipe(1, listeningPipes[1]);
	nRF24_openReadingPipe(2, listeningPipes[2]);
	nRF24_openWritingPipe(listeningPipes[localAddr]);
	nRF24_setPALevel(RF_PA_MIN);
	nRF24_startListening();
	
	
	while(1)
	{
		nRF24_stopListening();
		nRF24_openWritingPipe(listeningPipes[1]);
		
		dataOut.command = 1;
		dataOut.destAddr = 0;
		dataOut.datavalue = 0;
		
		for (int i = 0; i< 1024; i++)
		{
			if(!nRF24_write(&dataOut, sizeof(dataOut)))
			{
				printf("transmission failed \n\r");
			}
			delay_ms(10);
		}
		delay_ms(500);
	}
}
