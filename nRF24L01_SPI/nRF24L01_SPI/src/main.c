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
//#include "string.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "SAM_SPI.h"
#include "nRF24L01.h"
#include "nRF24.h"

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

static const uint32_t listeningPipes[5] = {0x3A3A3AD2UL, 0x3A3A3AC3UL, 0x3A3A3AB4UL, 0x3A3A3AA5UL, 0x3A3A3A96UL}; //unieke adressen gebruikt door de nodes.
//static const uint8_t localAddr = 0; // lokaal adres van de node

#define STRING_EOL    "\r"
#define STRING_HEADER "--Spi nRF24L01 Test --\r\n" \
"-- "BOARD_NAME" --\r\n" \
"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL


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
