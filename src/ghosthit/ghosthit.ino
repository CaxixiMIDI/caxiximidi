#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

/*
CAXIXI RAW
*/

// Caxixi Config
#include "CaxixiConfig.h"

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
struct MyMidiSettings : public midi::DefaultSettings {
	//static const bool UseRunningStatus = false; // Messes with my old equipment!
	static const long DefaultSettings::BaudRate = 57600;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);

#include "CxCircularBuffer.h"
int midiChannel = MIDI_CHANNEL;

CxCircularBuffer AccelXBuffer(BUFFER_SIZE);
CxCircularBuffer AccelYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
boolean bufferReady = false;
/*
int samples = 0;
int resolution = RESOLUTION;
int iteration = 0;
*/
int AccelXSmooth[filterSamples];
int AccelYSmooth[filterSamples];
int GyroXSmooth[filterSamples];
int smoothAccelX;
int smoothAccelY;
int smoothGyroX;

int SensorRead[6] = {0, 0, 0, 0, 0, 0};
unsigned char SensorNote[4] = {
	SENSOR_NOTE_ADELANTE,
	SENSOR_NOTE_ATRAS,
	SENSOR_NOTE_ADELANTE_ROTADA,
	SENSOR_NOTE_ATRAS_ROTADA
};

int NoteRelease[4] = {
	NOTE_RELEASE_ADELANTE,
	NOTE_RELEASE_ATRAS,
	NOTE_RELEASE_ADELANTE_ROTADA,
	NOTE_RELEASE_ATRAS_ROTADA
};

int NoteThreshold[4] = {
	NOTE_THRESHOLD_ADELANTE,
	NOTE_THRESHOLD_ATRAS,
	NOTE_THRESHOLD_ADELANTE_ROTADA,
	NOTE_THRESHOLD_ATRAS_ROTADA
};

int isMovingThreshold = MOVING_SLOPE_THRESHOLD;

// NoteOn = 5 (NoteOff)
int NoteOn = 5;
/*
NoteOnUp = 1
NoteOnDown = 2
NoteOnUpRotated = 3
NoteOnDownRotated = 4
boolean NoteOnUp = false;
boolean NoteOnDown = false;
boolean NoteOnUpRotated = false;
boolean NoteOnDownRotated = false;
*/

float v[6];
float angles[3];
FreeSixIMU my3IMU = FreeSixIMU();

boolean isMoving = false;
boolean isUpThreshold, isDownThreshold, isUpThresholdRotated, isDownThresholdRotated;
boolean canHit, canHitRotated;

int lastMovingAcceleration;
int ghostHitThreshold = GHOST_HIT_THRESHOLD;
boolean avoidedGhostHit, avoidGhostHit;
int currentAccelX, currentAccelY;

void setup() {
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
	AccelXBuffer.clear();
	AccelYBuffer.clear();
	GyroXBuffer.clear();
	delay(5);
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
}

int initialMillis;

void loop() {
	//initialMillis = millis();
	my3IMU.getAngles(angles);
	my3IMU.getValues(v);
	SensorRead[SENSOR_ACCEL_X] = (int)v[0];
	SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
	SensorRead[SENSOR_ACCEL_Z] = (int)v[2];
	SensorRead[SENSOR_GYRO_X] = (int)angles[0];
	SensorRead[SENSOR_GYRO_Y] = (int)angles[1];
	SensorRead[SENSOR_GYRO_Z] = (int)angles[2];
	setCircularBuffer();
	if(bufferReady || isBufferReady()){
		currentAccelX = AccelXBuffer.getPreviousElement(1);
		currentAccelY = AccelYBuffer.getPreviousElement(1);
		
		setIsMoving(); // checks slope 
		
		setCanHit();
		setCanHitRotated();
		setAvoidGhostHit();
		setThresholds();
		setThresholdsRotated();
		switch (NoteOn) {
			case NotaAdelante:
				if(noteReleaseUp()){
					SendNoteOff(SensorNote[NotaAdelante]);
					NoteOn = 5;
				}
				break;
			case NotaAtras:
				if(noteReleaseDown()){
					SendNoteOff(SensorNote[NotaAtras]);
					NoteOn = 5;
				}
				break;
			case NotaAdelanteRotada:
				if(noteReleaseUpRotated()){
					SendNoteOff(SensorNote[NotaAdelanteRotada]);
					NoteOn = 5;
				}
				break;
			case NotaAtrasRotada:
				if(noteReleaseDownRotated()){
					SendNoteOff(SensorNote[NotaAtrasRotada]);
					NoteOn = 5;
				}
				break;
		}
			
		if(NoteOn == 5 && isUpThreshold && canHit){
			if(avoidGhostHit){
				avoidedGhostHit = true;
				avoidGhostHit = false;
			}else{
				NoteOn = NotaAdelante;
				SendNoteOn(SensorNote[NotaAdelante]);
			}
		}
			
		if(NoteOn == 5 && isDownThreshold && canHit){
			if(avoidGhostHit){
				avoidedGhostHit = true;
				avoidGhostHit = false;
			}else{
				NoteOn = NotaAtras;
				SendNoteOn(SensorNote[NotaAtras]);
			}
		}
		
		if(NoteOn == 5 && isUpThresholdRotated && canHitRotated){
			NoteOn = NotaAdelanteRotada;
			SendNoteOn(SensorNote[NotaAdelanteRotada]);
		}

		if(NoteOn == 5 && isDownThresholdRotated && canHitRotated){
			NoteOn = NotaAtrasRotada;
			SendNoteOn(SensorNote[NotaAtrasRotada]);
		}
	}
	delay(3);
	/*
	int diffMillis = millis() - initialMillis;
	Serial.print(diffMillis);
	Serial.println();
        */
	
}

void SendNoteOn(int note)
{
	MIDI.sendNoteOn(note,127,midiChannel);
}

void SendNoteOff(int note)
{
	MIDI.sendNoteOff(note,127,midiChannel);
}

void setCircularBuffer(){
	smoothAccelX = digitalSmooth(SensorRead[SENSOR_ACCEL_X], AccelXSmooth);
	smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], AccelYSmooth);
	smoothGyroX = digitalSmooth(SensorRead[SENSOR_GYRO_X], GyroXSmooth);
	AccelXBuffer.addValue(smoothAccelX);
	AccelYBuffer.addValue(smoothAccelY);
	GyroXBuffer.addValue(smoothGyroX);
}

boolean isBufferReady(){
	if(AccelXBuffer.getCount() < BUFFER_SIZE
		|| AccelYBuffer.getCount() < BUFFER_SIZE
		|| GyroXBuffer.getCount() < BUFFER_SIZE){
		return false;
	}else{
		bufferReady = true;
		return true;
	}
}

void setIsMoving()
{
	int slope;
	isMoving = false;
	slope = currentAccelX - AccelXBuffer.getPreviousElement(2);
	if(abs(slope) > isMovingThreshold){
		isMoving = true;
		lastMovingAcceleration = currentAccelX;
		return;
	}
	slope = AccelXBuffer.getPreviousElement(2) - AccelXBuffer.getPreviousElement(3);
	if(abs(slope) > isMovingThreshold){
		isMoving = true;
		lastMovingAcceleration = currentAccelX;
		return;
	}
	slope = AccelXBuffer.getPreviousElement(3) - AccelXBuffer.getPreviousElement(4);
	if(abs(slope) > isMovingThreshold){
		isMoving = true;
		lastMovingAcceleration = currentAccelX;
		return;
	}
	slope = AccelXBuffer.getPreviousElement(4) - AccelXBuffer.getPreviousElement(5);
	if(abs(slope) > isMovingThreshold){
		isMoving = true;
		lastMovingAcceleration = currentAccelX;
		return;
	}
}

void setCanHit()
{
	if(NoteOn == NotaAdelante || NoteOn == NotaAtras){
		canHit = false;
		return;
	}
	if(!isMoving){
		canHit = false;
		return;
	}
	canHit = true;
}

void setAvoidGhostHit()
{
	if(!isMoving){
		avoidedGhostHit = false;
		avoidGhostHit = false;
		return;
	}
	if(avoidedGhostHit){
		avoidGhostHit = false;
		return;
	}	
	if(abs(currentAccelX - lastMovingAcceleration) > ghostHitThreshold){
		avoidGhostHit = true;
		return;
	}
}

void setCanHitRotated()
{
	if(NoteOn == NotaAdelanteRotada || NoteOn == NotaAtrasRotada){
		canHitRotated = false;
		return;
	}
	int hitters = 0;

	canHitRotated = false;
}

void setThresholds()
{
	if(currentAccelX > NoteThreshold[NotaAdelante]){
		isUpThreshold = true;
		isDownThreshold = false;
		return;
	}else{
		isUpThreshold = false;
	}
	if(currentAccelX < NoteThreshold[NotaAtras]){
		isDownThreshold = true;		
		return;
	}else{
		isDownThreshold = false;
	}
}

void setThresholdsRotated()
{
	if(currentAccelY > NoteThreshold[NotaAdelanteRotada]){
		isUpThresholdRotated = true;
		isDownThresholdRotated = false;
		return;
	}else{
		isUpThresholdRotated = false;
	}
	if(currentAccelY < NoteThreshold[NotaAtrasRotada]){
		isDownThresholdRotated = true;
		return;
	}else{
		isDownThresholdRotated = false;
	}
}

boolean noteReleaseUp()
{
	if(currentAccelX < NoteRelease[NotaAdelante]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseDown()
{
	if(currentAccelX > NoteRelease[NotaAtras]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseUpRotated()
{
	if(currentAccelY < NoteRelease[NotaAdelanteRotada]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseDownRotated()
{
	if(currentAccelY > NoteRelease[NotaAtrasRotada]){
		return true;
	}else{
		return false;
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

































