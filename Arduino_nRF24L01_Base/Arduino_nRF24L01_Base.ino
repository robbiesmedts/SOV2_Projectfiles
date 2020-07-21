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
const byte localAddr[6] = "Node1";

struct dataStruct{
  unsigned byte command;
  uint16_t dataValue;
}dataPacket;
/* command table
   0 = use sensor for own actuator
   1 = send sensor value to other actuator
   2 = receive sensor value for own actuator
   3 = Stop doing the current command
*/

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
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), nRF_IRQ, LOW);

#ifdef DEBUG
  Serial.begin(115200);
  printf_begin();
#endif
  /*
     Initailisation of the nRF24L01 chip
  */
  radio.begin();
  radio.openReadingPipe(0, localAddr);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

#ifdef DEBUG
  //print all settings of nRF24L01
  radio.printDetails();
#endif
}

void loop()
{
  uint8_t destAddr[40] = "";
  uint8_t destData;

  switch (command)
  {
    case 0:
      do  //read sens_pin and do something on act_pin
      {
        analogWrite(act_pin, analogRead(sens_pin));
        delay(10)
      } while (command = 0); //!stop command
      break;
    case 1:
      radio.stopListening();
      radio.openWritingPipe(destAddr)//set destination address
      do //read destination adress, read sens_pin, send data to destination address
      {

      } while (command != 3); //!stop command
      break;
    case 2:
      radio.openReadingPipe(1, destAddr);//set address 2
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
  if (radio.available())
  {
    radio.read(&command, sizeof(command));
    dataAvailable = 1;
  }
  interrupts();
}
