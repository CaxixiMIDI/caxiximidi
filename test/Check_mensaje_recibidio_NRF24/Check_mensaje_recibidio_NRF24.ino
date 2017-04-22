

#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/***      Set the Role 0 receiver or 1 sender        ***/
int roleSET = 0;
bool radioNumber = roleSET;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9,10);
/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = roleSET;
/////////HASTA ACA NRF CONFIG
int inInt;

void setup() {
  Serial.begin(9600);
  delay(100);
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
    // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
  }else{
    radio.openReadingPipe(1,addresses[1]);
    radio.startListening(); 
     }
  delay(100);

}

void loop() {
     if( radio.available()){                      // While there is data ready
        radio.read( &inInt, sizeof(int) );             // Get the payload
        Serial.println(inInt);
     }
     delay(2);

}
