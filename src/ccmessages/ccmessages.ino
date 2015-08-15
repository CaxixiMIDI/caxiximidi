// incluir librerias
// iniciar buffer
// funcion is buffer ready
// funcion variacion
// enviar cc messagge

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

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
   static const long DefaultSettings::BaudRate = 57600;

};

MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);
// MIDI_END

#include "CxCircularBuffer.h"

CxCircularBuffer GyroYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroZBuffer(BUFFER_SIZE);

int samples = 0;
int resolution = RESOLUTION;
int iteration = 0;

int GyroYSmooth[filterSamples];
int smoothGyroY;
int GyroZSmooth[filterSamples];
int smoothGyroZ;

float v[6];
float angles[3];
// Set the FreeIMU object
FreeSixIMU my3IMU = FreeSixIMU();

int SensorHitAvg[6] = {0, 0, 0, 0, 0, 0};
int midiChannel = 1;

void setup() {
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
	//pinMode(speakerPin, OUTPUT);
	GyroYBuffer.clear();
        GyroZBuffer.clear();
	delay(5);
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
}
boolean isRollingY;
int debugGyroY,debugGyroY5,debugGyroY10;
String debugRollingY;
int isRollingYVariation = 5;

boolean isRollingZ;
int debugGyroZ,debugGyroZ5,debugGyroZ10;
String debugRollingZ;
int isRollingZVariation = 5;

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
          setIsRollingY();
          setIsRollingZ();
	  debugRollingY = "";
          debugRollingZ = "";
          if(isBufferReady()){
                debugGyroY = GyroYBuffer.getPreviousElement(1);//Y es la inclinacion para ade (+) y atras (-)
                debugGyroY5 = GyroYBuffer.getPreviousElement(5);
                debugGyroY10 = GyroYBuffer.getPreviousElement(10);
                debugGyroZ = GyroZBuffer.getPreviousElement(1);//Z es la inclinacion para izq (+) y der (-)
                debugGyroZ5 = GyroZBuffer.getPreviousElement(5);
                debugGyroZ10 = GyroZBuffer.getPreviousElement(10);
                //Serial.print(currentMillis);
                //Serial.print(",");
                //Serial.print(debugGyroY10);
                //Serial.print(",");
                //Serial.print(debugGyroY5);
                //Serial.print(",");
                //Serial.print(debugGyroY);
                //Serial.print(",");
                if(isRollingY){
    		processCCMY();//acá debería mandar el CCMESSAGGE, con processCCM()
    			}
                //Serial.print(debugRollingY);
                //Serial.print(debugGyroZ10);
                //Serial.print(",");
                //Serial.print(debugGyroZ5);
                //Serial.print(",");
                //Serial.print(debugGyroZ);
                if(isRollingZ){
    		processCCMZ();//acá debería mandar el CCMESSAGGE, con processCCM()
    			}
                //Serial.print(",");
                //Serial.print(debugRollingZ);
                //Serial.println();
                	}
	}
	delay(12);
}

void setIsRollingY(){
  int currentValue = GyroYBuffer.getPreviousElement(1);
  //Eval first variation
  int firstValue = GyroYBuffer.getPreviousElement(5);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingYVariation){
        isRollingY = true;
	return;
  }else{
    isRollingY = false; }
}

void setIsRollingZ(){
  int currentValue = GyroZBuffer.getPreviousElement(1);
  //Eval first variation
  int firstValue = GyroZBuffer.getPreviousElement(5);
  int firstVariation = abs(firstValue - currentValue);
  if(firstVariation > isRollingZVariation){
        isRollingZ = true;
	return;
  }else{
    isRollingZ = false; }
}

void processCCMY()
{
  	int controlvalueY;
        int y;
        controlvalueY = GyroYBuffer.getPreviousElement(1);
        // acá hay que escalar los datos que tira el giro a la escala 0-127 del MIDI
        if(controlvalueY>75){
          controlvalueY = 75;}
         if(controlvalueY<-75){
          controlvalueY = -75;}        
        y = map(controlvalueY, -75, 75, 0, 127); 
        MIDI.sendControlChange(13,y,midiChannel);
        //delay(2);
}

void processCCMZ()
{
  	int controlvalueZ;
        int z;
        controlvalueZ = GyroZBuffer.getPreviousElement(1);
        // acá hay que escalar los datos que tira el giro (-80, 80) a la escala 0-127 del MIDI
        if(controlvalueZ>75){
        controlvalueZ = 75;}
        if(controlvalueZ<-75){
        controlvalueZ = -75;} 
        z = map(controlvalueZ, -75, 75, 127, 0); 
        MIDI.sendControlChange(12,z,midiChannel);
        //delay(2);
}

void setCircularBuffer()
{
	smoothGyroY = digitalSmooth(SensorHitAvg[SENSOR_GYRO_Y], GyroYSmooth);
	GyroYBuffer.addValue(smoothGyroY);
	smoothGyroZ = digitalSmooth(SensorHitAvg[SENSOR_GYRO_Z], GyroZSmooth);
	GyroZBuffer.addValue(smoothGyroZ);
}

boolean isBufferReady(){
	if(GyroYBuffer.getCount() < BUFFER_SIZE){
		return false;
	}else{
		return true;
	}
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
