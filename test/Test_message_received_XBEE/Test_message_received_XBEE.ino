/*****************************************************************
XBee_Serial_Passthrough.ino

Set up a software serial port to pass data between an XBee Shield
and the serial monitor.

Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's DOUT and DIN pins to Arduino pins 2 and 3.

*****************************************************************/
// We'll use SoftwareSerial to communicate with the XBee:
#include <SoftwareSerial.h>
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX
char inData[10];
int index;
boolean started = false;
boolean ended = false;
int inInt;

void setup()
{
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  XBee.begin(9600);
  Serial.begin(9600);
}

void loop(){
    while(XBee.available() > 0){
    char aChar = XBee.read();
    //Serial.println(aChar);
    if(aChar == '<'){
      started = true;
      index = 0;
      inData[index] = '\0';
    }else if(aChar == '>'){
      ended = true;
    }else if(started){
      inData[index] = aChar;
      index++;
      inData[index] = '\0';
    }
    if(started && ended){
      // Convert the string to an integer
      inInt = atoi(inData);
      // Use the value
      //Serial.println(inInt);
    Serial.println(inInt);
    
       // Get ready for the next time
      started = false;
      ended = false;

      index = 0;
      inData[index] = '\0';
    }
  }
}
