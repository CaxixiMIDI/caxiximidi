#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include "CaxixiConfig.h"
#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>
#include "CxCircularBuffer.h"

CxCircularBuffer accelXBuffer(BUFFER_SIZE);
CxCircularBuffer accelYBuffer(BUFFER_SIZE);
boolean bufferReady = false;
int	accelXSmooth[filterSamples];
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
FreeSixIMU my3IMU = FreeSixIMU();

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
int clearButtonState = 0;
int lastClearButtonState = 0;

// 1.2 Octavator
int octaveDownButtonLastState  = 0;
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = LOW;
unsigned long firstTime; // how long since the button was first pressed
int longPressThreshold = 2000; //when a button press is considered long in millis

int lastDownState = 0;
int octaveDownButtonState = 0;  // variable for reading the pushbutton status
int currentOctave = 0;

// 1.3 CCMessages
bool wantCCM = false;
bool CCMbufferReady = false;
CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
CxCircularBuffer GyroYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroZBuffer(BUFFER_SIZE);
int GyroXSmooth[filterSamples];
int smoothGyroX;
int GyroYSmooth[filterSamples];
int smoothGyroY;
int GyroZSmooth[filterSamples];
int smoothGyroZ;
bool NoteState = true;
int NoteThreshold = 600; //Umbral para mandar midiOn/Off
bool Bloqueo = false;
int msg; //msg to send

boolean isRollingX;
String debugRollingX;
int isRollingXVariation = 1;
boolean isRollingY;
String debugRollingY;
int isRollingYVariation = 1;
boolean isRollingZ;
String debugRollingZ;
int isRollingZVariation = 1;

int initialMillis;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  accelXBuffer.clear();
  accelYBuffer.clear();
  delay(5);
  GyroZBuffer.clear();
  GyroYBuffer.clear();
  GyroZBuffer.clear();
  delay(5);
  my3IMU.init();
  my3IMU.acc.setFullResBit(true);
  my3IMU.acc.setRangeSetting(16);
  delay(5);
}


void loop() {
	//initialMillis = millis();
	my3IMU.getValues(v);
	SensorRead[SENSOR_ACCEL_X] = (int)v[0];
	SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
	SensorRead[SENSOR_ACCEL_Z] = (int)v[2];
  SensorRead[SENSOR_GYRO_X] = (int)v[3];
  SensorRead[SENSOR_GYRO_Y] = (int)v[4];
  SensorRead[SENSOR_GYRO_Z] = (int)v[5];
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
}

void runCaxixi() {
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

void SendToReceiver(int msg)
{
  Serial.print("<");
  Serial.print(msg);
  Serial.print(">");
}

void SendNoteOn(int note)
{
  SendToReceiver(note);
}

void SendNoteOff(int note)
{
  SendToReceiver(note);
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

void setCircularBufferCCM(){
  smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], accelYSmooth);
  accelYBuffer.addValue(smoothAccelY);
  smoothGyroX = digitalSmooth(SensorRead[SENSOR_GYRO_X], GyroXSmooth);
  GyroXBuffer.addValue(smoothGyroX);
  smoothGyroY = digitalSmooth(SensorRead[SENSOR_GYRO_Y], GyroYSmooth);
  GyroYBuffer.addValue(smoothGyroY);
  smoothGyroZ = digitalSmooth(SensorRead[SENSOR_GYRO_Z], GyroZSmooth);
  GyroZBuffer.addValue(smoothGyroZ);
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

void ButtonClear() {
  clearButtonState = digitalRead(SAMPLER_BUTTON_CLEAR_PIN);
  if (clearButtonState != lastClearButtonState) {// if the state has changed, increment the counter
    if (clearButtonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
    SendClear();
    }
    lastClearButtonState = clearButtonState; // save the current state as the last state, for next time through the loop
  }
}

void ButtonOctaveDown() {
  octaveDownButtonState = digitalRead(OCTAVE_DOWN_BUTTON_PIN);
  if (octaveDownButtonState != octaveDownButtonLastState) {
    if (octaveDownButtonState == HIGH){
      firstTime = millis();
      if (!wantCCM) {
        //currentOctave = currentOctave+1; 
        SendOctaveDown();
      }
    }
    millis_held = (millis() - firstTime);
    if (octaveDownButtonState == LOW && millis_held > longPressThreshold) {
        //Serial.print("\nChange wantCCM");
        if (!wantCCM) {
        SendOctaveUp(); //Revisar, para compensar el octaveUp indeseado del if anterior
        }
        wantCCM = !wantCCM;
      }      
    octaveDownButtonLastState = octaveDownButtonState;
  }
}

int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
  // static int raw[filterSamples];
  static int sorted[filterSamples];
  boolean done;
  i = (i + 1) % filterSamples;  // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;   // input new data into the oldest slot
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
  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += sorted[j];  // total remaining indices
    k++; 
  }
  return total / k;    // divide by number of samples
}

///////////////////
// CCM functions //
///////////////////

boolean isCCMBufferReady(){
  if(GyroXBuffer.getCount() < BUFFER_SIZE
  || GyroYBuffer.getCount() < BUFFER_SIZE
  || GyroZBuffer.getCount() < BUFFER_SIZE
  || accelYBuffer.getCount() < BUFFER_SIZE){
    return false;
  } else {
    CCMbufferReady = true;
    return true;
  }
}

void setIsRollingX(){
  int currentValue = GyroXBuffer.getPreviousElement(1);
  int firstValue = GyroXBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingXVariation){
    isRollingX = true;
    return;
  } else {
    isRollingX = false; }
}

void setIsRollingY(){
  int currentValue = GyroYBuffer.getPreviousElement(1);
  int firstValue = GyroYBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingYVariation){
    isRollingY = true;
    return;
  }else{
    isRollingY = false; }
}

void setIsRollingZ(){
  int currentValue = GyroZBuffer.getPreviousElement(1);
  int firstValue = GyroZBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingZVariation){
    isRollingZ = true;
    return;
  }else{
    isRollingZ = false; }
}

void processX() {
  int controlvalueX;
  int x;
  controlvalueX = abs(GyroXBuffer.getPreviousElement(1));
  x = map(controlvalueX, 0, 50, 0, 127); 
  SendToReceiver(formatCCM(x,15));
  //delay(2);
}

void processY() {
  int controlvalueY;
  int y;
  controlvalueY = GyroYBuffer.getPreviousElement(1);
  y = map(controlvalueY, 25, -25, 0, 127); 
  SendToReceiver(formatCCM(y,16));
  //delay(2);
}

void processZ() {
  int controlvalueZ;
  int z;
  controlvalueZ = GyroZBuffer.getPreviousElement(1);
  z = map(controlvalueZ, 25, -25, 0, 127); 
  SendToReceiver(formatCCM(z,17));
  //delay(2);
}

void ccmNotes() {
  if (SensorRead[SENSOR_ACCEL_Y] > NoteThreshold & (!Bloqueo)){
    if (NoteState){
       SendToReceiver(CAXIXI_RIGHT_HIT_NOTEON);
       NoteState = false;
       Bloqueo = true;
       }
    else{
      SendToReceiver(CAXIXI_RIGHT_HIT_NOTEOFF);
      NoteState = true;
      Bloqueo = true;
      }
    }
  if (SensorRead[SENSOR_ACCEL_Y] < 400){
    Bloqueo = false;
    }
} 

void areRolling() {
  setIsRollingX();
  setIsRollingY();
  setIsRollingZ();  
}

void ProcessCCM() {
  if(isRollingX){
    processX();
    }
  if(isRollingY){
    processY();
    }
  if(isRollingZ){
    processZ();
    }
}

void runCCM() {
  ButtonOctaveDown();
  ccmNotes();
  areRolling();
  ProcessCCM();
  }

int formatCCM(int NUM, int CH) {
  msg = CH*1000 + NUM;
  return msg;
}
