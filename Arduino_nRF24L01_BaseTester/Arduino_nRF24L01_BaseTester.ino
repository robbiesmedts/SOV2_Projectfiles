/*
   Test program for the base receiver Arduino code

   What it should do:

   Send a commando
*/
/*
   Uncomment in need of debugging
   when in use, Arduino send every action to the Serial Com port
   set Com with a baud-rate of 115200
*/
#define DEBUG

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#ifdef DEBUG
#include <printf.h>
#endif

RF24 radio(9, 10); //CE, CSN
const byte localAddr = 0; //node 0 is masternode
const uint64_t listeningPipes[5] = {0x3A3A3A3AD2, 0x3A3A3A3AC3, 0x3A3A3A3AB4, 0x3A3A3A3AA5, 0x3A3A3A3A96};


#ifdef DEBUG
void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}
#endif

/* Datapaket standaard.
   datapaketten verzonden binnen dit project zullen dit formaat hanteren om een uniform systeem te vormen
   command      //commando (8bits) gestuctureerd volgens command table
   destAddr     //adres (6x8bits) ontvangen met pakket, zal volgens commando een ontvangend adres worden of een adres waarnaar gezonden word
   dataValue    //variabele (8bits) om binnenkomende/uitgaande data in op te slagen

   command table
   0 = Stop command
   1 = use sensor for own actuator
   2 = send sensor value to other actuator
   3 = receive sensor value for own actuator
*/
struct dataStruct {
  uint8_t command;
  uint64_t destAddr;
  int dataValue;
} dataIn, dataOut;

volatile byte nRF_Status;
bool send = 0;

int sens_pin = A0; //analog 0
int act_pin = 5; //D5 and D6 are both connected to the Timer0 counter
int interrupt_pin = 2;

void setup() {
  pinMode(sens_pin, INPUT); //if reading a switch with no external pull-up resistor, change it to INPUT_PULLUP
  pinMode(interrupt_pin, INPUT_PULLUP);
  pinMode(act_pin, OUTPUT);

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
#endif
  /*
     Initailisation of the nRF24L01 chip
  */
  radio.begin();
  radio.openReadingPipe(1, listeningPipes[1]);
  radio.openReadingPipe(2, listeningPipes[2]);
  radio.openWritingPipe(listeningPipes[localAddr]);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

#ifdef DEBUG
  //print all settings of nRF24L01
  radio.printDetails();
#endif
}

void loop() {
  radio.stopListening();
  radio.openWritingPipe(listeningPipes[1]);

  dataOut.command = 1;
  dataOut.destAddr = 0;
  dataOut.dataValue = 0;

#ifdef DEBUG
  Serial.println("Commando 1 send to address: ");
  printf("%ld\n\r", listeningPipes[1]);
#endif
  //verzend commando 1
  for (int i = 0; i < 1024; i++)
  {
    if (!radio.write(&dataOut, sizeof(dataStruct)))
    {
      Serial.println("transmission failed");
    }
    delay(10);
  }

  delay(500);

  radio.openWritingPipe(listeningPipes[2]);
  dataOut.command = 1;
  dataOut.destAddr = 0;
  dataOut.dataValue = 0;

#ifdef DEBUG
  Serial.println("Commando 1 send to address: ");
  printf("%ld\n\r", listeningPipes[2]);
#endif
  /*
       verzend commando 1 en wacht
  */
  for (int i = 0; i < 1024; i++)
  {
    if (!radio.write(&dataOut, sizeof(dataStruct)))
    {
      Serial.println("transmission failed");
    }
    delay(10);
  }

  /*
     send commando 2 to one receiver and commando 3 to another and wait for termination
  */
  
    radio.openWritingPipe(listeningPipes[1]);
    //data packet ontvanger
    dataOut.command = 3;
    dataOut.destAddr = listeningPipes[2];
    dataOut.dataValue = 0;

    #ifdef DEBUG
    Serial.println("Commando 3 send to address 2");
    #endif

    if (!radio.write(&dataOut, sizeof(dataStruct)))
    {
      Serial.println("transmission failed");
    }

    //data packet zender
    radio.openWritingPipe(listeningPipes[2]);

    dataOut.command = 2;
    dataOut.destAddr = listeningPipes[1];
    dataOut.dataValue = 0;

    #ifdef DEBUG
    Serial.println("Commando 2 send to address 1");
    #endif

    if (!radio.write(&dataOut, sizeof(dataStruct)))
    {
      Serial.println("transmission failed");
    }

    delay(5000);

  /*
     einde programma, stuur stop commando
  */
  dataOut.command = 0;

#ifdef DEBUG
  Serial.println("Commando 0 send to address 1 & 2");
#endif

  radio.openWritingPipe(listeningPipes[1]);
  if (!radio.write(&dataOut, sizeof(dataStruct)))
  {
    Serial.println("transmission failed");
  }

  radio.openWritingPipe(listeningPipes[2]);
  if (!radio.write(&dataOut, sizeof(dataStruct)))
  {
    Serial.println("transmission failed");
  }

#ifdef DEBUG
  Serial.println("einde programma");
#endif
  delay(5000);
}
