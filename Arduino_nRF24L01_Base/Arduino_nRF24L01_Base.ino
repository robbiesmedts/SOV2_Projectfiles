/*
   Start-up for the Arduino Sensor/Actuator side of the project

   What it should do:
   wait for start action from motherboard thru nRF24L01
   Read sensor value
   1) use value for actuator
   2) Send value at an interval back to motherboard
   2.5) receive sensor value from motherboard for actuator
*/
/*
   Uncomment in need of debugging
   when in use, Arduino send every action to the Serial Com port
   set Com with a baud-rate of 115200
*/
//#define DEBUG

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#ifdef DEBUG
#include <printf.h>
#endif

RF24 radio(9, 10); //CE, CSN
const byte localAddr[6] = "000A1";

/* command table
   0 = use sensor for own actuator
   1 = send sensor value to other actuator
   2 = receive sensor value for own actuator
   3 = Stop doing the current command
*/
volatile byte command;
volatile byte nRF_Status;
bool dataAvailable = 0;

int sens_pin = A0; //analog 0
int act_pin = 5; //D5 and D6 are both connected to the Timer0 counter
int interrupt_pin = 2;

void setup()
{
  pinMode(sens_pin, INPUT); //if reading a switch with no external pull-up resistor, change it to INPUT_PULLUP
  pinMode(interrupt_pin, INPUT_PULLUP);
  pinMode(act_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), nRF_ReadData, LOW);

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
#endif

  radio.begin();
  radio.openReadingPipe(0, localAddr);
  radio.setPALevel(RF24_PA_MIN);
  //radio.maskIRQ(1,1,0); //mask transmission complete and transmit failure interrupts; interrupt only occurs when a payload is received
  radio.startListening();

#ifdef DEBUG
  radio.printDetails();
#endif
}

void loop()
{
  if (dataAvailable)
  {
    
  }
  switch (command)
  {
    case 0:
      do  //read sens_pin and do something on act_pin
      {

      } while (command != 3); //!stop command
      break;
    case 1:
      do //read destination adress, read sens_pin, send data to destination address
      {

      } while (command != 3); //!stop command
      break;
    case 2:
      do  //receive data and use data for own actuator
      {

      } while (command != 3);
      break;
    case 3:
    //stop command, do nothing
    default:
      //do nothing
      break;
  }
}

void nRF_IRQ()
{ 
  noInterrupts();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  nRF_Status = SPI.transfer(RF24_NOP);
  SPI.endTransaction();

  dataAvailable = ((nRF_Status & RX_DR)>>6) //if data available x1xx xxxx, shift right and save
  
  interrupts();
}

void nRF_ReadData()
{

}



  if (radio.available())
  {
    radio.read(&command, sizeof(command));      //read the incomming command
  }
  
