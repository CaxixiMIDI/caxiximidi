//Let´s XBee

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

// CAXIXI XBee
#include <SoftwareSerial.h>
SoftwareSerial xbee(2, 3); // RX, TX

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

// MIDI_BEGIN
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>

// Caxixi Config
#include "CaxixiConfig.h"

struct MyMidiSettings : public midi::DefaultSettings
{
   //static const bool UseRunningStatus = false; // Messes with my old equipment!
   static const long DefaultSettings::BaudRate = 9600;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);

#include "CxCircularBuffer.h"

CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
CxCircularBuffer GyroYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroZBuffer(BUFFER_SIZE);

int samples = 0;
int resolution = RESOLUTION;
int iteration = 0;

int GyroXSmooth[filterSamples];
int smoothGyroX;
int GyroYSmooth[filterSamples];
int smoothGyroY;
int GyroZSmooth[filterSamples];
int smoothGyroZ;

bool NoteState = true;
int NoteThreshold = 600; //Umbral para mandar midiOn/Off
bool Bloqueo = false;

float v[6];
float angles[3];
// Set the FreeIMU object
FreeSixIMU my3IMU = FreeSixIMU();

int SensorHitAvg[6] = {0, 0, 0, 0, 0, 0};
int midiChannel = 1;

boolean isRollingX;
boolean isRollingY;
boolean isRollingZ;
int isRollingXVariation = 2;
int isRollingYVariation = 2;
int isRollingZVariation = 2;
boolean bufferReady = false;

void setup() {
	MIDI.begin(1);
	Serial.begin(9600);
	Wire.begin();
	xbee.begin(9600);
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
  unsigned long currentMillis = millis();
	iteration++;
  my3IMU.getYawPitchRoll(angles);
  my3IMU.getValues(v);
	SensorHitAvg[SENSOR_ACCEL_X] = readSensorRaw(SENSOR_ACCEL_X);
	SensorHitAvg[SENSOR_ACCEL_Y] = readSensorRaw(SENSOR_ACCEL_Y);
	SensorHitAvg[SENSOR_ACCEL_Z] = readSensorRaw(SENSOR_ACCEL_Z);
	SensorHitAvg[SENSOR_GYRO_X] = readSensorRaw(SENSOR_GYRO_X);
	SensorHitAvg[SENSOR_GYRO_Y] = readSensorRaw(SENSOR_GYRO_Y);
	SensorHitAvg[SENSOR_GYRO_Z] = readSensorRaw(SENSOR_GYRO_Z);

  if(resolution - iteration == 0){
    iteration = 0;
	  setCircularBuffer();
    setIsRollingX();
    setIsRollingY();
    setIsRollingZ();
    if(isBufferReady()){
      Notes();
      if(isRollingX){
    	  processCCMX();
    		}
      if(isRollingY){
    	  processCCMY();
    		}
      if(isRollingZ){
    	  processCCMZ();
    		}
      }
	}
	delay(5);
}

void Notes() {
  if (SensorHitAvg[SENSOR_ACCEL_Y] > NoteThreshold & (!Bloqueo)){
    if (NoteState) {
      SendNoteOn(66);
      NoteState = false;
      Bloqueo = true;
      }
    else {
      SendNoteOff(66);
      NoteState = true;
      Bloqueo = true;
      }
  }
  if (SensorHitAvg[SENSOR_ACCEL_Y] < 400){
    Bloqueo = false;
  }
}  

void SendNoteOn(int note) {
  MIDI.sendNoteOn(note,127,midiChannel);
  }
  
void SendNoteOff(int note) {
  MIDI.sendNoteOff(note,127,midiChannel);
  }

void setIsRollingX(){
  int currentValue = GyroXBuffer.getPreviousElement(1);
  int firstValue = GyroXBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  //Serial.println(currentValue);
  if(firstVariation > isRollingXVariation){
    isRollingX = true;
	return;
  } 
  else {
    isRollingX = false; 
    }
}

void setIsRollingY(){
  int currentValue = GyroYBuffer.getPreviousElement(1);
  int firstValue = GyroYBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if (firstVariation > isRollingYVariation) {
    isRollingY = true;
	return;
  } else {
    isRollingY = false; 
    }
}

void setIsRollingZ(){
  int currentValue = GyroZBuffer.getPreviousElement(1);
  int firstValue = GyroZBuffer.getPreviousElement(3);
  int firstVariation = abs(firstValue - currentValue);
  if (firstVariation > isRollingZVariation) {
    isRollingZ = true;
	  return;
  } 
  else {
    isRollingZ = false; 
    }
}

void processCCMX() {
  int controlvalueX;
  int x;
  controlvalueX = abs(GyroXBuffer.getPreviousElement(1));
  x = map(controlvalueX, 0, 50, 0, 127); 
  MIDI.sendControlChange(12,x,midiChannel);
  //delay(2);
}

void processCCMY() {
  int controlvalueY;
  int y;
  controlvalueY = GyroYBuffer.getPreviousElement(1);
  y = map(controlvalueY, 25, -25, 0, 127); 
  MIDI.sendControlChange(13,y,midiChannel);
  //delay(2);
}

void processCCMZ() {
  int controlvalueZ;
  int z;
  controlvalueZ = GyroZBuffer.getPreviousElement(1);
  z = map(controlvalueZ, 25, -25, 0, 127); 
  MIDI.sendControlChange(14,z,midiChannel);
  //delay(2);
}

void setCircularBuffer() {
  smoothGyroX = digitalSmooth(SensorHitAvg[SENSOR_GYRO_X], GyroXSmooth);
	GyroXBuffer.addValue(smoothGyroX);
	smoothGyroY = digitalSmooth(SensorHitAvg[SENSOR_GYRO_Y], GyroYSmooth);
	GyroYBuffer.addValue(smoothGyroY);
	smoothGyroZ = digitalSmooth(SensorHitAvg[SENSOR_GYRO_Z], GyroZSmooth);
	GyroZBuffer.addValue(smoothGyroZ);
}

boolean isBufferReady() {
  if (bufferReady) {
    return bufferReady;
  }
	if (GyroXBuffer.getCount() < BUFFER_SIZE || GyroYBuffer.getCount() < BUFFER_SIZE || GyroZBuffer.getCount() < BUFFER_SIZE) {
    bufferReady = false;
	} else {
    bufferReady = true;
	}
  return bufferReady;
}     

int readSensorRaw (int sensor)
{
	int val;
	switch (sensor) {
		case SENSOR_ACCEL_X:
			val = (int)v[0];
			break;
		case SENSOR_ACCEL_Y:
			val = (int)v[1];
			break;
		case SENSOR_ACCEL_Z:
			val = (int)v[2];
			break;
		case SENSOR_GYRO_X:
			val = (int)angles[0];
			break;
		case SENSOR_GYRO_Y:
			val = (int)angles[1];
			break;
		case SENSOR_GYRO_Z:
			val = (int)angles[2];
			break;
	}
	return val;
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
