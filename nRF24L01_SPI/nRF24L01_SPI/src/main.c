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
#include "spi_master.h"
#include "nRF24L01.h"
//#include "nRF24.h"

/*
Definition of the debug functions
When disabled all the debug code will ben ignored by the compiler
*/
#define _DEBUG

/* Datapaket standaard.
   datapaketten verzonden binnen dit project zullen dit formaat hanteren om een uniform systeem te vormen
   destAddr     //adres (6x8bits) ontvangen met pakket, zal volgens commando een ontvangend adres worden of een adres waarnaar gezonden word
   dataValue    //variabele (8bits) om binnenkomende/uitgaande data in op te slagen
   command      //commando (8bits) gestuctureerd volgens command table

   command table
   0 = Stop command
   1 = use sensor for own actuator
   2 = send sensor value to other actuator
   3 = receive sensor value for own actuator
*/
struct dataStruct{
	uint32_t destAddr;
	uint16_t datavalue;
	uint8_t command;
}dataIn, dataOut;

/*
Globale variabele gebruikt door de software library
*/
uint32_t txDelay; // delay tussen TX paketten
uint8_t payload_size = 32; // grootte van de payload
uint8_t addr_width; // adres lengte
bool dynamic_payloads_enabled = false;
uint8_t pipe0_reading_address[5]; // dummy locatie voor Pipe0 adres
static const uint8_t pipe_s[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t pipe_size_s[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
static const uint8_t pipe_enable_s[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

static const uint32_t listeningPipes[5] = {0x3A3A3AD2UL, 0x3A3A3AC3UL, 0x3A3A3AB4UL, 0x3A3A3AA5UL, 0x3A3A3A96UL}; //unieke adressen gebruikt door de nodes.
static const uint8_t localAddr = 0; // lokaal adres van de node

#ifdef _DEBUG
static const char rf24_datarate_e_str_0[] = "1MBPS";
static const char rf24_datarate_e_str_1[] = "2MBPS";
static const char rf24_datarate_e_str_2[] = "250KBPS";
static const char * const rf24_datarate_e_str_P[] = {
	rf24_datarate_e_str_0,
	rf24_datarate_e_str_1,
	rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] = "nRF24L01";
static const char rf24_model_e_str_1[] = "nRF24L01+";
static const char * const rf24_model_e_str_P[] = {
	rf24_model_e_str_0,
	rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] = "Disabled";
static const char rf24_crclength_e_str_1[] = "8 bits";
static const char rf24_crclength_e_str_2[] = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] = {
	rf24_crclength_e_str_0,
	rf24_crclength_e_str_1,
	rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] = {
	rf24_pa_dbm_e_str_0,
	rf24_pa_dbm_e_str_1,
	rf24_pa_dbm_e_str_2,
	rf24_pa_dbm_e_str_3,
};
#endif // _DEBUG

/*
Pin definitions
*/
#define CE PIO_PC9_IDX

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


/* SPI clock default setting (Hz). */
static uint32_t gs_ul_spi_clock = 5000000;

#define STRING_EOL    "\r"
#define STRING_HEADER "--Spi nRF24L01 Test --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL
/*Functie declatatie en definities*/

void startFastWrite(const void* buf, uint8_t len, const bool multicast);


/**
 * \brief Initialize SPI as master.
 */
static void spi_master_initialize(void)
{
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
 * \param p_buf Pointer to buffer to transfer.
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
	delay_us(5);
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

uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
	//1x spi zenden niet 2 commando's
	uint8_t status[len+1];
	status[0] = R_REGISTER | (REGISTER_MASK & reg);
	spi_master_transfer(&status, sizeof(status));
	
	for (uint8_t i = 0; i< len; i++)
	{
		buf[i] = status[i+1];
	}
	
	return status[0];
	
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
	/** contents of p_buf before transfer
	* [0] write commando to reg
	* [1] data to write
	*/
	
	spi_master_transfer(p_buf, sizeof(p_buf));
	/** contents of p_buf after transfer
	* [0] Status register
	* [1] unknown data
	*/
	return p_buf[0]; //return STATUS
}

/**
 * \brief write to a register of the nRF24L01 transceiver
 * 
 * \param reg register to write
 * \param buf pointer to data
 * \param length length of data to write
 * \return STATUS register 
 */
uint8_t nRF_writeRegister(uint8_t reg, const uint8_t* buf, uint8_t length)
{
	uint8_t p_buf[length+1];
	
	p_buf[0] = (W_REGISTER | (REGISTER_MASK & reg));
	
	for (uint8_t i = 0; i < length; i++)
	{
		p_buf[i+1] = (*buf++);
		//printf("%d || %02x || %02x\n\r", i, p_buf[i], *buf);
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

/**
 * \brief sets datarate used by the nRF24
 * 
 * \param speed datarate to be used
 * \return true if set
 */
bool nRF24_setDataRate(rf24_datarate_e speed)
{
	bool result = false;
	uint8_t setup = nRF24_readRegister(RF_SETUP);
	setup &= ~((1<<RF_DR));
	
	if (speed == RF24_2MBPS) {
		setup |= (1<<RF_DR);
		#if !defined(F_CPU)
		txDelay = 190;
		#else // 16Mhz Arduino
		txDelay = 65;
		#endif
	}
	nRF24_writeRegister(RF_SETUP, setup);
	
	if(nRF24_readRegister(RF_SETUP) == setup)
	result = true;
	
	return result;
}

/**
 * \brief fetch the used datarate
 * 
 * \return datarate
 */
rf24_datarate_e getDataRate(void)
{
	rf24_datarate_e result;
	uint8_t dr = nRF24_readRegister(RF_SETUP) & ((1<<RF_DR_LOW) | (1<<RF_DR_HIGH));
	
	if (dr == (1<<RF_DR_HIGH)) {
		// '01' = 2MBPS
		result = RF24_2MBPS;
	} else {
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

/**
 * \brief fetch the CRC length
 * 
 * \return CRC length
 */
rf24_crclength_e getCRCLength(void)
{
	rf24_crclength_e result = RF24_CRC_DISABLED;
	
	
	uint8_t config = nRF24_readRegister(NRF_CONFIG) & ((1<<CRCO) | (1<<EN_CRC));
	uint8_t AA = nRF24_readRegister(EN_AA);
	
    if (config & (1<<EN_CRC) || AA) {
	    if (config & (1<<CRCO)) {
		    result = RF24_CRC_16;
		    } else {
		    result = RF24_CRC_8;
	    }
    }

    return result;	
}

/**
 * \brief fetch the PA Level
 * 
 * \return PA level
 */
uint8_t nRF24_getPALevel(void)
{
	return (nRF24_readRegister(RF_SETUP) & (1<<(RF_PWR_LOW) | (1<<RF_PWR_HIGH))) >> 1;
}


bool isPVariant(void)
{
	return true;
}

#ifdef _DEBUG
void print_status (uint8_t status)
{
	printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n", status, (status & (1<<RX_DR)) ? 1 : 0, (status & (1<<TX_DS)) ? 1 : 0, (status & (1<<MAX_RT)) ? 1 : 0, (status & (1<<RX_P_NO)) ? 1 : 0, (status & (1<<TX_FULL)) ? 1 : 0);
}

void print_address_register(const char* name, uint8_t reg, uint8_t qty)
{
	printf("%s\t", name);
	while(qty--){
		uint8_t buffer[addr_width];
		read_register(reg++, buffer, sizeof(buffer));
		
		printf(" 0x");
		uint8_t* bufptr = buffer + sizeof(buffer);
		while(--bufptr >= buffer){
			printf("%02x", *bufptr);
		}
	}
	printf("\r\n");
}

void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
	printf("%s\t", name);
	while (qty--)
	{
		printf(" 0x%02x", nRF24_readRegister(reg++));
	}
	printf("\r\n");
}

void printDetails(void)
{
	printf("SPI Speed\t = %ld MHz\r\n",gs_ul_spi_clock/1000000);
	print_status(nRF24_getStatus());
	print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
	print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
	print_address_register("TX_ADDR\t", TX_ADDR, 1);

	print_byte_register("RX_PW_P0-5", RX_PW_P0, 6);
	print_byte_register("SETUP_AW", SETUP_AW, 1);
	print_byte_register("EN_AA\t", EN_AA, 1);
	print_byte_register("EN_RXADDR", EN_RXADDR, 1);
	print_byte_register("RF_CH\t", RF_CH, 1);
	print_byte_register("RF_SETUP", RF_SETUP, 1);
	print_byte_register("CONFIG\t", NRF_CONFIG, 1);
	print_byte_register("DYNPD/FEATURE", DYNPD, 2);
	 
	printf("Data Rate\t = %s\r\n", rf24_datarate_e_str_P[getDataRate()]);
	printf("Model\t\t = %s\r\n", rf24_model_e_str_P[isPVariant()]);
	printf("CRC Length\t = %s\r\n", rf24_crclength_e_str_P[getCRCLength()]);
	printf("PA Power\t = %s\r\n", rf24_pa_dbm_e_str_P[nRF24_getPALevel()]);
}
#endif

/**
 * \brief writes the payload to the TX buffer, adds extra blank bytes if data_len < payload_size
 * 
 * \param buf pointer to databuffer
 * \param data_len length of the data to be written
 * \param writeType type of TX packet to be send (TX /w ACK of TX zonder ACK)
 * 
 * \return STATUS
 */
uint8_t nRF24_writePayload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
	uint8_t blanklen = dynamic_payloads_enabled ? 0 : payload_size - data_len;
	uint8_t size = data_len + blanklen + 1;
	uint8_t s_buff[size];
	uint8_t* current = (uint8_t*) buf;
/*	
	#ifdef _DEBUG
	printf("[Writing %u bytes] ", data_len);
	#endif
*/	
	s_buff[0] = writeType;
	for (uint8_t i = 1; i< size; i++)
	{
		s_buff[i] = current[i-1];
	}
	
	spi_master_transfer(s_buff, size);

	return s_buff[0];
}

/**
 * \brief Initializes the payload and nRF24L01 for transmission
 * 
 * \param buf pointer to databuffer
 * \param len length of the data to be written
 * \param multicast true or false
 * 
 */
void startFastWrite(const void* buf, uint8_t len, const bool multicast)
{
	nRF24_writePayload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD); // ?: operator a ? b : c // if a, b else c

	ioport_set_pin_level(CE, 1);
}

/**
 * \brief passes payload to TX buffer and read STATUS for TX_DS
 * 
 * \param buf pointer to databuffer
 * \param len length of the data to be written
 * \param multicast true or false
 * 
 * \return true if TX complete
 */
bool nRFwrite(const void* buf, uint8_t len, const bool multicast)
{
	startFastWrite(buf, len, multicast);
	
	while(!(nRF24_getStatus() & ((1<<TX_DS) | (1<<MAX_RT))))
	{
		delay_us(100);
	}
	ioport_set_pin_level(CE, 0);
	uint8_t status = nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
	
	if(status & (1<<MAX_RT)){
		nRF24_FlushTx();
		return 0;
	}
	return 1;
}

/**
 * \brief closes RX pipe
 * 
 * \param pipe RX pipe to close
 *
 */
void nRF24_closeReadingPipe(uint8_t pipe)
 {
	 nRF24_writeRegister(EN_RXADDR, nRF24_readRegister(EN_RXADDR) & ~(1<< pipe_enable_s[pipe]));
 }

/**
 * \brief Read the RX buffer payload
 * 
 * \param buf pointer to databuffer
 * \param len length of the data to be read
 * 
 * \return STATUS
 */
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

/**
 * \brief Set address width
 * 
 * \param width address width
 *
 */
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

/**
 * \brief set the amount the transmitter re-sends TX data
 * 
 * \param delay delay between re transmissions
 * \param count amount of retries permitted
 *
 */
void nRF24_setRetries(uint8_t delay, uint8_t count)
{
	nRF24_writeRegister(SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) <<ARC );
}

/**
 * \brief sets CRC length
 * CRC (De)coding behind an transmission. can be disabled, 1 or 2 bytes
 * 
 * \param length length of the decoding
 *
 */
void nRF24_setCRCLength(rf24_crclength_e length)
{
	uint8_t config = nRF24_readRegister(NRF_CONFIG) & ~((1<<CRCO) | (1<<EN_CRC));
	
	if (length == RF24_CRC_DISABLED){
		// do nothing we turned it off above
	} 
	else if (length == RF24_CRC_8){
		config |= (1<<EN_CRC);
	} 
	else //CRC 16 bit
	{
		config |= (1<<EN_CRC);
		config |= (1<<CRCO);
	}
	nRF24_writeRegister(NRF_CONFIG, config);
}

/**
 * \brief toggels ACK features
 *
 */
void toggle_features(void)
{
	uint8_t config[2] = {ACTIVATE, 0x73};
	
	spi_master_transfer(config, sizeof(config));
}

/**
 * \brief set the frequency channel used for transmission
 * 
 * \param channel ferquency channel used
 *
 */
void nRF24_setChannel(uint8_t channel)
{
	const uint8_t max_channel = 125;
	if (channel > max_channel)
		nRF24_writeRegister(RF_CH, max_channel);
	else
		nRF24_writeRegister(RF_CH, channel);
}

/**
 * \brief power up the internal logic of the nRF24 chip
 * 
 */
void nRF24_powerUp(void)
{
	uint8_t config = nRF24_readRegister(NRF_CONFIG);
	
	if (!(config & (1<<PWR_UP))){
		nRF24_writeRegister(NRF_CONFIG, config | (1<<PWR_UP));
		delay_ms(5);
	}
}

/**
 * \brief Power down the internal logic of the nRF24 chip
 *
 */
void nRF24_powerDown(void)
{
	ioport_set_pin_level(CE, 0);
	nRF24_writeRegister(NRF_CONFIG, nRF24_readRegister(NRF_CONFIG) & ~(1<<PWR_UP));
}

/**
 * \brief use the nRF24 module as receiver and listen for transmissions
 *
 */
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

/**
 * \brief use the nRF24 module as transmitter
 *
 */
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

/**
 * \brief configure I/O to be used by nRF24 module and configure the internal logic of the nRF24 as followed:
 *
 * TX retries(5, 15): delay 1,25ms (5*250us), 15 retries
 * Data rate: 1MBPS
 * CRC: 2 bytes
 * RF channel: 2,476GHz (2400+76MHz)
 * address width: 32 bit / 4 bytes
 * 
 */
bool nRF24_begin(void)
{
	uint8_t setup = 0;
	ioport_set_pin_dir(CE, IOPORT_DIR_OUTPUT);//ce_pin PC9
	ioport_set_pin_level(CE, 0);
	
	nRF24_writeRegister(NRF_CONFIG, 0x0C);
	nRF24_setRetries(5, 15);
	
	nRF24_setDataRate(RF24_1MBPS);
	nRF24_setCRCLength(RF24_CRC_16);
	toggle_features();
	
	//reset current status
	nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<TX_DS) | (1<<MAX_RT));
	
	nRF24_setChannel(76);
	nRF24_setAddressWidth(ADDR_4bytes);
	
	nRF24_FlushRx();
	nRF24_FlushTx();
	
	nRF24_powerUp();
	
	nRF24_writeRegister(NRF_CONFIG, (nRF24_readRegister(NRF_CONFIG)) & ~(1<<PRIM_RX));
	setup = nRF24_readRegister(RF_SETUP);
	
	return (setup != 0 && setup != 0xFF);
}

/**
 * \brief Opens TX pipe with given address
 * this Should be the same as the RX0 pipe of receiver
 * 
 * \param address address of the receiving module
 *
 */
void nRF24_openWritingPipe(uint64_t address)
{
	nRF_writeRegister(RX_ADDR_P0, (uint8_t *)(&address), addr_width);
	nRF_writeRegister(TX_ADDR, (uint8_t *)(&address), addr_width);
	
	nRF24_writeRegister(RX_PW_P0, payload_size);
}

/**
 * \brief sets the RX/TX buffer size 
 * 
 * \param size size of the RX/TX buffers
 *
 */
void nRF24_setPayloadSize(uint8_t size)
{
	const uint8_t max_size = 32;
	
	if (size > max_size)
		payload_size = 32;
	else
		payload_size = size;
	
}

/**
 * \brief request the RX/TX buffer size
 * 
 * \return buffer size
 *
 */
uint8_t nRF24_getpayloadSize(void)
{
	return payload_size;
}

/**
 * \brief Opens RX pipe with given address
 * this Should be the same as the TX pipe of transmitter
 * 
 * \param address address of the transmitting module
 *
 */
void nRF24_openReadingPipe(uint8_t pipe, uint64_t address)
{	
	if (pipe == 0){
		memcpy(pipe0_reading_address, &address, addr_width);
	}
	if (pipe <= 5){
		if (pipe < 2){
			nRF_writeRegister(pipe_s[pipe], (const uint8_t *) (&address), addr_width);
		} else {
			nRF_writeRegister(pipe_s[pipe], (const uint8_t *) (&address), 1);
		}
		nRF24_writeRegister(pipe_size_s[pipe], payload_size);
	}
	nRF24_writeRegister(EN_RXADDR, nRF24_readRegister(EN_RXADDR) | (1 << pipe_enable_s[pipe]));
}

/**
 * \brief Ochecks if data is available
 * 
 * \param pipe_num optional pointer to pipe variable. if given the pipe number is returned through this pointer
 *
 * \return true or false
 */
bool nRF24_available(uint8_t* pipe_num)
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

/**
 * \brief sets PA level
 * PA level indicates the output power of the transmitter
 * 
 * \param level: the desired power output (-18, -12, -6 or 0dBm )
 *
 */
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

/**
 * \brief read payload and clear flags
 * 
 * \param buf: pointer to data buffer
 * \param len: length of the payload to be read
 *
 */
void nRF24_read(uint8_t* buf, uint8_t len)
{
	nRF24_readPayload(buf, len);
	
	nRF24_writeRegister(NRF_STATUS, (1<<RX_DR) | (1<<MAX_RT) | (1<<TX_DS));	
}

/**
 * \brief write commando, starts the transmission
 * This function should be used for transmission
 * 
 * \param buf: pointer to the data buffer
 * \param len: length of the payload to be written
 *
 */
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

	nRF24_setPALevel(RF_PA_MIN);
	nRF24_stopListening();

#ifdef _DEBUG	
	printDetails();
#endif
	
//	while(1)
//	{

		nRF24_openWritingPipe(listeningPipes[1]);

		dataOut.command = 1;
		
#ifdef _DEBUG
		printf("commando %d send to %ld\r\n", dataOut.command, listeningPipes[1]);
#endif
		
		for (int i = 0; i < 10; i++)
		{
			if(!nRF24_write(&dataOut, sizeof(dataOut)))
			{
				printf("transmission failed \n\r");
			}
#ifdef _DEBUG
			printf("commando %d\r\n", dataOut.command);
#endif
			delay_ms(10);
		}

		delay_ms(500);

		nRF24_openWritingPipe(listeningPipes[2]);
		
		dataOut.command = 1;
		
#ifdef _DEBUG
		printf("commando %d send to %ld\r\n", dataOut.command, listeningPipes[2]);
#endif

		for (int i = 0; i < 10; i++)
		{
			if(!nRF24_write(&dataOut, sizeof(dataOut)))
			{
				printf("transmission failed \n\r");
			}
#ifdef _DEBUG
			printf("commando %d\r\n", dataOut.command);
#endif
			delay_ms(10);
		}
		
		delay_s(1);
		
		nRF24_openWritingPipe(listeningPipes[1]);
		//data packet zender
		dataOut.command = 2;
		dataOut.destAddr = listeningPipes[2];
		dataOut.datavalue = 0;
		
#ifdef _DEBUG
		printf("commando %d send to %ld\r\n", dataOut.command, listeningPipes[1]);
#endif		

		for (int i = 0; i< 10; i++)
		{
			if(!nRF24_write(&dataOut, sizeof(dataOut)))
			{
				printf("transmission failed \n\r");
			}
			#ifdef _DEBUG
			printf("commando %d\r\n", dataOut.command);
			#endif
			delay_ms(10);
		}
		
//		delay_s(5);
//	}

}
