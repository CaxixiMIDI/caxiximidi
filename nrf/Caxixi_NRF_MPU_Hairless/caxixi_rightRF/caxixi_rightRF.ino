//#include <ADXL345.h>
//#include <bma180.h>
//#include <HMC58X3.h>
//#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>

/*
CAXIXI XBee
*/
/*
 * CAXIXI NRF24 CONFIG
 */
#include <SPI.h>
#include "RF24.h"

/****************** User Config ***************************/
/***      Set the Role 0 receiver or 1 sender right or 2 sender left      ***/
int roleSET = 1;
int radioNumber = roleSET;
/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/

byte addresses[][6] = {"1Node","2Node"};

// Used to control whether this node is sending or receiving
bool role = roleSET;

#include "CaxixiConfig.h"

#include "CommunicationUtils.h"
//#include "FreeSixIMU.h"
#include <Wire.h>

#include "CxCircularBuffer.h"

CxCircularBuffer accelXBuffer(BUFFER_SIZE);
CxCircularBuffer accelYBuffer(BUFFER_SIZE);
boolean bufferReady = false;
/*
int samples = 0;
int resolution = RESOLUTION;
int iteration = 0;
*/
int  accelXSmooth[filterSamples];
int accelYSmooth[filterSamples];
int smoothAccelX;
int smoothAccelY;

int SensorRead[6] = {0, 0, 0, 0, 0, 0};

int NoteRelease[3] = {
  NOTE_RELEASE_FORWARD,
  NOTE_RELEASE_BACKWARD,
  NOTE_RELEASE_HIT
};

int noteThresholdHit = NOTE_THRESHOLD_HIT;
int canHitDefinition = 1;

int noteOn = NOTE_OFF;

float v[6];
float angles[3];
FreeIMU my3IMU = FreeIMU();

boolean isUpThreshold, isDownThreshold, isUpThresholdRotated, isDownThresholdRotated;
boolean canHit, canHitRotated;

int currentAccelX, currentAccelY;

int slopeStill;
int accelXForce;
int state = STATE_STILL;
int prevState;

/**
 * 1. BUTTONS
 */

// 1.1 Sampler
bool record = false;
int recordButtonState = 0;    // current state of the button
int lastRecordButtonState = 0;  // previous state of the button
int clearButtonState = 0;
int lastClearButtonState = 0;


// 1.2 Octavator
int octaveUpButtonLastState  = 0;
int octaveDownButtonLastState  = 0;
int octaveUpButtonState = 0;  // variable for reading the pushbutton status
int octaveDownButtonState = 0;
int currentOctave = 0;

#include <avr/power.h>
int powerpin = 5;

void setup() {
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
///LO QUE SIGUE ES LO QUE HIZO FUNCIONAR EL ACCEL CUANDO POWER ON
////LO SAQUE DE ES PAGINA http://stackoverflow.com/questions/29371430/powering-mpu-6050-with-arduino-digital-pins
///AL PARECER EL PROBLEMA ES UNA ESPECIE DE OVERFLOW CUANDO POWER ON EN EL I2C
///INVESTIGAR MEJOR

Wire.begin();
Wire.beginTransmission(0x68);
Wire.write(0x6B);  // PWR_MGMT_1 register
Wire.write(0);     // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true); 
  accelXBuffer.clear();
  accelYBuffer.clear();
  delay(500);
  my3IMU.init();
  //my3IMU.acc.setFullResBit(true);
  //my3IMU.acc.setRangeSetting(16);
  delay(50);
}

int initialMillis;

void loop() {
  //initialMillis = millis();
  my3IMU.getValues(v);
  SensorRead[SENSOR_ACCEL_X] = (int)v[0];
  SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
  SensorRead[SENSOR_ACCEL_Z] = (int)v[2];
  setCircularBuffer();
  if(bufferReady || isBufferReady()){///Desde aqui descomentar los botones ue se usaran
    //ButtonRecord(); //Caxixi Right REVISAR!!!!!
    //ButtonOctaveUp(); //Caxixi Right
    //ButtonClear(); //Caxixi Left REVISAR!!!!!
   // ButtonOctaveDown(); //Caxii Left
    currentAccelX = accelXBuffer.getPreviousElement(1);
    currentAccelY = accelYBuffer.getPreviousElement(1);
    setSlopeStill();
    setAccelXForce();
    prevState = state;
    setState();
    switch (noteOn) {
      case NOTE_FORWARD:
      if(noteReleaseForward()){
        SendNoteOff(CAXIXI_RIGHT_FORWARD_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      case NOTE_BACKWARD:
      if(noteReleaseBackward()){
        SendNoteOff(CAXIXI_RIGHT_BACKWARD_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      case NOTE_HIT:
      if(noteReleaseHit()){
        SendNoteOff(CAXIXI_RIGHT_HIT_NOTEOFF);
        noteOn = NOTE_OFF;
      }
      break;
      default:
      break;
    }
    if(noteOn == NOTE_OFF && state == STATE_FORWARD && prevState == STATE_BACKWARD){
      noteOn = NOTE_FORWARD;
      SendNoteOn(CAXIXI_RIGHT_FORWARD_NOTEON);
    }
    
    if(noteOn == NOTE_OFF && state == STATE_BACKWARD && prevState == STATE_FORWARD){
      noteOn = NOTE_BACKWARD;
      SendNoteOn(CAXIXI_RIGHT_BACKWARD_NOTEON);
    }
    
    if(noteOn == NOTE_OFF && currentAccelY > noteThresholdHit){
      noteOn = NOTE_HIT;
      SendNoteOn(CAXIXI_RIGHT_HIT_NOTEON);
    }
  }
  delay(2);
  //int diffMillis = millis() - initialMillis;
  //Serial.print(diffMillis);
  //Serial.println();
}

void SendToReceiver(int msg)
{
  /* ///CAXIXI SEND WITH XBEE OR SERIAL///
  Serial.print("<");
  Serial.print(msg);
  Serial.print(">");
  */
    ///CAXIXI SEND WITH NRF24///
  radio.write( &msg, sizeof(int));
  
    }

void SendNoteOn(int note)
{
  SendToReceiver(note);
}

void SendNoteOff(int note)
{
  SendToReceiver(note);
}

void SendRecordStart()
{
  SendToReceiver(CAXIXI_RECORD_START);
}

void SendRecordStop()
{
  SendToReceiver(CAXIXI_RECORD_STOP);
}

void SendOctaveUp()
{
  SendToReceiver(CAXIXI_OCTAVE_UP);
}

void SendOctaveDown()
{
  SendToReceiver(CAXIXI_OCTAVE_DOWN);
}

void SendClear()
{
  SendToReceiver(CAXIXI_SAMPLER_CLEAR);
}

void setCircularBuffer(){
  smoothAccelX = digitalSmooth(SensorRead[SENSOR_ACCEL_X], accelXSmooth);
  smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], accelYSmooth);
  accelXBuffer.addValue(smoothAccelX);
  accelYBuffer.addValue(smoothAccelY);
}

boolean isBufferReady(){
  if(accelXBuffer.getCount() < BUFFER_SIZE
  || accelYBuffer.getCount() < BUFFER_SIZE){
    return false;
  }else{
    bufferReady = true;
    return true;
  }
}

void setSlopeStill()
{
  if(abs(currentAccelX - accelXBuffer.getPreviousElement(8)) < SLOPE_STILL_X_RANGE
  && abs(currentAccelY - accelYBuffer.getPreviousElement(8)) < SLOPE_STILL_Y_RANGE){
    slopeStill = SLOPE_STILL_STATUS; 
  }else{
    slopeStill = SLOPE_MOVING_STATUS;
  }
}

void setAccelXForce()
{
  if(currentAccelX > FORCE_THRESHOLD_FORWARD || currentAccelY > NOTE_THRESHOLD_HIT){
    accelXForce = FORCE_FORWARD;
  }
  if(currentAccelX < FORCE_THRESHOLD_BACKWARD){
    accelXForce = FORCE_BACKWARD;
  }
  if(currentAccelX < FORCE_STILL_RANGE_FORWARD && currentAccelX > FORCE_STILL_RANGE_BACKWARD){
    accelXForce = FORCE_STILL;
  }
}

void setState()
{
  if(slopeStill == SLOPE_STILL_STATUS && accelXForce == FORCE_STILL){
    state = STATE_STILL;
  }else if(slopeStill == SLOPE_MOVING_STATUS && accelXForce == FORCE_FORWARD){
    state = STATE_FORWARD;
  }else if(slopeStill == SLOPE_MOVING_STATUS && accelXForce == FORCE_BACKWARD){
    state = STATE_BACKWARD;
  }else{
    state = prevState;
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

//CAXIXI RIGHT BUTTONS
void ButtonRecord() {
  recordButtonState = digitalRead(SAMPLER_BUTTON_RECORD_PIN);
  if (recordButtonState != lastRecordButtonState) {// if the state has changed, increment the counter
    if (recordButtonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
      if (record==true){
        record=false;
        SendRecordStop();
      } else {
        record=true;
        SendRecordStart();
      }
    }
    lastRecordButtonState = recordButtonState; // save the current state as the last state, for next time through the loop
  }
}

void ButtonOctaveUp() {
  octaveUpButtonState = digitalRead(OCTAVE_UP_BUTTON_PIN);
  if (octaveUpButtonState != octaveUpButtonLastState) {
    if (octaveUpButtonState == HIGH){
      currentOctave = currentOctave+1; 
      SendOctaveUp();
    }
    octaveUpButtonLastState = octaveUpButtonState;
  }
}
////CAXIXI LEFT BUTTONS
void ButtonOctaveDown() {
  octaveDownButtonState = digitalRead(OCTAVE_DOWN_BUTTON_PIN);
  if (octaveDownButtonState != octaveDownButtonLastState) {
    if (octaveDownButtonState == HIGH){
      currentOctave = currentOctave-1; 
      SendOctaveDown();
    }
    octaveDownButtonLastState = octaveDownButtonState;
  }
}

void ButtonClear() {
  clearButtonState = digitalRead(SAMPLER_BUTTON_CLEAR_PIN);
  if (clearButtonState != lastClearButtonState) {// if the state has changed, increment the counter
    if (clearButtonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
    SendClear();
    }
    lastClearButtonState = clearButtonState; // save the current state as the last state, for next time through the loop
  }
}

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
int j, k, temp, top, bottom;
long total;
static int i;
// static int raw[filterSamples];
static int sorted[filterSamples];
boolean done;

i = (i + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

// Serial.print("raw = ");

for (j=0; j<filterSamples; j++){     // transfer data array into anther array for sorting and averaging
  sorted[j] = sensSmoothArray[j];
}

done = 0;                // flag to know when we're done sorting              
while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
  done = 1;
  for (j = 0; j < (filterSamples - 1); j++){
    if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
      temp = sorted[j + 1];
      sorted [j+1] =  sorted[j] ;
      sorted [j] = temp;
      done = 0;
    }
  }
}

/*
for (j = 0; j < (filterSamples); j++){    // print the array to debug
Serial.print(sorted[j]); 
Serial.print("   "); 
}
Serial.println();
*/

// throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
bottom = max(((filterSamples * 15)  / 100), 1); 
top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
k = 0;
total = 0;
for ( j = bottom; j< top; j++){
  total += sorted[j];  // total remaining indices
  k++; 
  // Serial.print(sorted[j]); 
  // Serial.print("   "); 
}

//  Serial.println();
//  Serial.print("average = ");
//  Serial.println(total/k);
return total / k;    // divide by number of samples
}



