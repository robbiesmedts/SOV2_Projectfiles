/*
   Start-up for the Arduino Sensor/Actuator side of the project

   What it should do:

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

#ifdef DEBUG
void printHex(uint8_t num){
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
  byte destAddr[6];
  int dataValue;
} dataIn, dataOut;


volatile byte nRF_Status;
bool dataAvailable = 0;

int sens_pin = A0; //analog 0
int act_pin = 5; //D5 and D6 are both connected to the Timer0 counter
int interrupt_pin = 2;

void setup(){
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

void loop(){
  
#ifdef DEBUG
  Serial.print("Incomming command: ");
  Serial.println(dataIn.command);
#endif

  switch (dataIn.command)
  {
    case 0:
      //stop command, do nothing
      break;

    case 1: //read sensor and use fo own actuator
      analogWrite(act_pin, analogRead(sens_pin));
#ifdef DEBUG
      Serial.println("dataIn.command = 0");
#endif
      delay(10);
      break;

    case 2: // read sensor and send to other actuator
      radio.stopListening();
      radio.openWritingPipe(dataIn.destAddr);//set destination address
      dataOut.command = 2;
      dataOut.dataValue = analogRead(sens_pin);
      radio.write(&dataOut, sizeof(dataStruct));
#ifdef DEBUG
      Serial.print("destination address: ");
      for(int i=0; i<sizeof(dataIn.destAddr); i++){
        printHex(dataIn.destAddr[i]);
      }
      Serial.println();
      Serial.print("data send: ");
      Serial.println(dataOut.dataValue);
#endif
      radio.startListening();
      break;

    case 3: //receive sensor value and use for own actuator
#ifdef DEBUG
      Serial.print("receiving address: ");
        for(int i=0; i<sizeof(dataIn.destAddr); i++){
          printHex(dataIn.destAddr[i]);
        }
      Serial.println();
#endif
      radio.openReadingPipe(1, dataIn.destAddr);//set address 2
#ifdef DEBUG
      Serial.print("received data: ");
      Serial.println(dataIn.dataValue);
#endif
      analogWrite(act_pin, dataIn.dataValue);
      break;

    default:
      //do nothing
      break;
  }
}

void nRF_IRQ(){
  noInterrupts();
  if (radio.available())
  {
    radio.read(&dataIn, sizeof(dataStruct));
  }
  interrupts();
}
