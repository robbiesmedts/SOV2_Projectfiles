/*
 * nRF24.h
 *
 * Created: 28/07/2020 15:26:25
 *  Author: robbi
 */ 


#ifndef NRF24_H_
#define NRF24_H_

#include "nRF24L01.h"

#define CE PIO_PC9_IDX

/*data enumeration*/
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

typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS
}rf24_datarate_e;

typedef enum {
	ADDR_3bytes = 3,
	ADDR_4bytes,
	ADDR_5bytes
}addr_width_e;

//extern uint8_t payload_size; // grootte van de payload

static const uint8_t pipe_s[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t pipe_size_s[] = {RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5};
static const uint8_t pipe_enable_s[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

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

//public functions
uint8_t nRF24_readRegister(uint8_t reg);
uint8_t nRF24_writeRegister(uint8_t reg, uint8_t val);
uint8_t nRF24_FlushRx(void);
uint8_t nRF24_FlushTx(void);
uint8_t nRF24_getStatus(void);
bool nRF24_setDataRate(rf24_datarate_e speed);
rf24_datarate_e getDataRate(void);
void nRF24_setCRCLength(rf24_crclength_e length);
rf24_crclength_e getCRCLength(void);
void nRF24_setPALevel(uint8_t level);
uint8_t nRF24_getPALevel(void);
void printDetails(void);
void nRF24_closeReadingPipe(uint8_t pipe);
void nRF24_setAddressWidth(uint8_t width);
void nRF24_setRetries(uint8_t delay, uint8_t count);
void toggle_features(void);
void nRF24_setChannel(uint8_t channel);
void nRF24_powerUp(void);
void nRF24_powerDown(void);
void nRF24_startListening(void);
void nRF24_stopListening(void);
bool nRF24_begin(void);
void nRF24_openWritingPipe(uint64_t address);
void nRF24_setPayloadSize(uint8_t size);
uint8_t nRF24_getpayloadSize(void);
void nRF24_openReadingPipe(uint8_t pipe, uint64_t address);
bool nRF24_available(uint8_t* pipe_num);
void nRF24_read(uint8_t* buf, uint8_t len);
bool nRF24_write(const void* buf, uint8_t len);

/*//private functions
static void startFastWrite(const void* buf, uint8_t len, const bool multicast);
static uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
static uint8_t writeRegister(uint8_t reg, const uint8_t* buf, uint8_t length);
static bool isPVariant(void);
static void print_status (uint8_t status);
static void print_address_register(const char* name, uint8_t reg, uint8_t qty);
static void print_byte_register(const char* name, uint8_t reg, uint8_t qty);
static uint8_t writePayload(const void* buf, uint8_t data_len, const uint8_t writeType);
static bool nRFwrite(const void* buf, uint8_t len, const bool multicast);
static uint8_t readPayload(uint8_t* buf, uint8_t data_len);
*/
#endif /* NRF24_H_ */