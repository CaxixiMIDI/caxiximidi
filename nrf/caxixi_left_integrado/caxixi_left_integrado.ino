#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

/*
 * CAXIXI NRF24 CONFIG
 */
int roleSET = 1; //Set the Role 0 receiver or 1 sender right or 2 sender left
int radioNumber = roleSET;
RF24 radio(7,8); //Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */

byte addresses[][6] = {"1Node","2Node"};
// Used to control whether this node is sending or receiving
bool role = roleSET;

#include "CaxixiConfig.h"
#include "CommunicationUtils.h"
#include <Wire.h>
#include "CxCircularBuffer.h"
#include "left.h"
#include "setters.h"
#include "receiverComm.h"
#include "buttons.h"
#include "ccm.h"
#include "digitalSmooth.h"


void setup() {
  //pinMode(OCTAVE_UP_BUTTON_PIN, INPUT);// initialize the button pin as a input:
  pinMode(OCTAVE_DOWN_BUTTON_PIN, INPUT);
  //pinMode(SAMPLER_BUTTON_RECORD_PIN, INPUT);
  pinMode(SAMPLER_BUTTON_CLEAR_PIN, INPUT);
  //Serial.begin(9600);
   radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber==0){
    //radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
   // radio.openReadingPipe(2,addresses[2]);
    radio.startListening(); 
      }
    else if(radioNumber==1){
    radio.openWritingPipe(addresses[1]);
    //radio.openReadingPipe(1,addresses[0]);
    }
    else{
    radio.openWritingPipe(addresses[2]);
    }
    delay(10);
    Wire.begin(); //Fix to powerOn battery ArduinoProMini
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true); 
    accelXBuffer.clear();
    accelYBuffer.clear();
    delay(500);
    GyroZBuffer.clear();
    GyroYBuffer.clear();
    GyroZBuffer.clear();
    delay(5);
    my3IMU.init();
    delay(50);
}

int initialMillis;

void loop() {
  //initialMillis = millis();
  my3IMU.getYawPitchRoll(angles);
  my3IMU.getValues(v);
  SensorRead[SENSOR_ACCEL_X] = (int)v[0];
  SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
  SensorRead[SENSOR_ACCEL_Z] = (int)v[2];
  SensorRead[SENSOR_GYRO_X] = (int)angles[0];
  SensorRead[SENSOR_GYRO_Y] = (int)angles[1];
  SensorRead[SENSOR_GYRO_Z] = (int)angles[2];
    if(wantCCM){
    setCircularBufferCCM();
    if(CCMbufferReady || isCCMBufferReady()){
      runCCM();
    }
  }
  else {
    setCircularBuffer();
    if(bufferReady || isBufferReady()){
      runCaxixi();
    }
  }
  delay(2);
  //int diffMillis = millis() - initialMillis;
  //Serial.print(diffMillis);
  //Serial.println();
}

void runCaxixi() {
  
//DESCOMENTAR SEGUN BOTONES UTILIZADOS
    //ButtonRecord(); //Caxixi Right
    //ButtonOctaveUp(); //Caxixi Right
    ButtonClear();
    ButtonOctaveDown();
    currentAccelX = accelXBuffer.getPreviousElement(1);
    currentAccelY = accelYBuffer.getPreviousElement(1);
    setSlopeStill();
    setAccelXForce();
    prevState = state;
    setState();
    switch (noteOn) {
      case NOTE_FORWARD:
      if(noteReleaseForward()){
        SendNoteOff(CAXIXI_LEFT_FORWARD_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      case NOTE_BACKWARD:
      if(noteReleaseBackward()){
        SendNoteOff(CAXIXI_LEFT_BACKWARD_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      case NOTE_HIT:
      if(noteReleaseHit()){
        SendNoteOff(CAXIXI_LEFT_HIT_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      default:
      break;
    }
    if(noteOn == NOTE_OFF && state == STATE_FORWARD && prevState == STATE_BACKWARD){
      noteOn = NOTE_FORWARD;
      SendNoteOn(CAXIXI_LEFT_FORWARD_NOTEON);
    }
    
    if(noteOn == NOTE_OFF && state == STATE_BACKWARD && prevState == STATE_FORWARD){
      noteOn = NOTE_BACKWARD;
      SendNoteOn(CAXIXI_LEFT_BACKWARD_NOTEON);
    }
    
    if(noteOn == NOTE_OFF && currentAccelY > noteThresholdHit){
      noteOn = NOTE_HIT;
      SendNoteOn(CAXIXI_LEFT_HIT_NOTEON);
    }
}


boolean noteReleaseForward()
{
  if(currentAccelX < NoteRelease[NOTE_FORWARD]){
    return true;
  }else{
    return false;
  }
}

boolean noteReleaseBackward()
{
  if(currentAccelX > NoteRelease[NOTE_BACKWARD]){
    return true;
  }else{
    return false;
  }
}

boolean noteReleaseHit()
{
  if(currentAccelY < NoteRelease[NOTE_HIT]){
    return true;
  }else{
    return false;
  }
}

//TODO use the same function for buttons and a param to define right or left


