/*
 * nRF24.c
 *
 * Created: 28/07/2020 15:21:21
 *  Author: robbie
 */ 
#include <asf.h>
#include "nRF24.h"
#include "SAM_SPI.h"
#include "string.h"


/* Globale variabele gebruikt door de nRF24 library */
uint32_t txDelay; // delay tussen TX paketten
uint8_t payload_size = 32; // grootte van de payload
uint8_t addr_width; // adres lengte
bool dynamic_payloads_enabled = false;
uint8_t pipe0_reading_address[5]; // dummy locatie voor Pipe0 adres

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

static uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len)
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
static uint8_t writeRegister(uint8_t reg, const uint8_t* buf, uint8_t length)
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
 * \brief fetch the PA Level
 * 
 * \return PA level
 */
uint8_t nRF24_getPALevel(void)
{
	return (nRF24_readRegister(RF_SETUP) & (1<<(RF_PWR_LOW) | (1<<RF_PWR_HIGH))) >> 1;
}

static bool isPVariant(void)
{
	return true;
}

static void print_status (uint8_t status)
{
	printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n", status, (status & (1<<RX_DR)) ? 1 : 0, (status & (1<<TX_DS)) ? 1 : 0, (status & (1<<MAX_RT)) ? 1 : 0, (status & (1<<RX_P_NO)) ? 1 : 0, (status & (1<<TX_FULL)) ? 1 : 0);
}

static void print_address_register(const char* name, uint8_t reg, uint8_t qty)
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

static void print_byte_register(const char* name, uint8_t reg, uint8_t qty)
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

/**
 * \brief writes the payload to the TX buffer, adds extra blank bytes if data_len < payload_size
 * 
 * \param buf pointer to databuffer
 * \param data_len length of the data to be written
 * \param writeType type of TX packet to be send (TX /w ACK of TX zonder ACK)
 * 
 * \return STATUS
 */
static uint8_t writePayload(const void* buf, uint8_t data_len, const uint8_t writeType)
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
static void startFastWrite(const void* buf, uint8_t len, const bool multicast)
{
	writePayload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD); // ?: operator a ? b : c // if a, b else c

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
static bool nRFwrite(const void* buf, uint8_t len, const bool multicast)
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
static uint8_t readPayload(uint8_t* buf, uint8_t data_len)
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
		writeRegister(RX_ADDR_P0, pipe0_reading_address, addr_width);
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
 * \return 1 if nRF24 module reacts to data
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
	writeRegister(RX_ADDR_P0, (uint8_t *)(&address), addr_width);
	writeRegister(TX_ADDR, (uint8_t *)(&address), addr_width);
	
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
			writeRegister(pipe_s[pipe], (const uint8_t *) (&address), addr_width);
		} else {
			writeRegister(pipe_s[pipe], (const uint8_t *) (&address), 1);
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
 * \brief read payload and clear flags
 * 
 * \param buf: pointer to data buffer
 * \param len: length of the payload to be read
 *
 */
void nRF24_read(uint8_t* buf, uint8_t len)
{
	readPayload(buf, len);
	
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