#include "CaxixiConfig.h"

/* FreeIMU */
#include "FreeSixIMU.h"
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include "CommunicationUtils.h"
#include <Wire.h>

// MIDI
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
struct MyMidiSettings : public midi::DefaultSettings {
	static const long DefaultSettings::BaudRate = 57600;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);
int midiChannel = MIDI_CHANNEL;


#include "CxCircularBuffer.h"
#include "HitStateMachine.h"

CxCircularBuffer accelXBuffer(BUFFER_SIZE);
CxCircularBuffer accelYBuffer(BUFFER_SIZE);
HitStateMachine caxixiState;
boolean bufferReady = false;

//////////////CAXIXI/////////////////////////
int accelXSmooth[filterSamples];
int accelYSmooth[filterSamples];
int smoothAccelX;
int smoothAccelY;

int sensorRead[6] = {0, 0, 0, 0, 0, 0};
unsigned char sensorNote[4] = {
	SENSOR_NOTE_ADELANTE,
	SENSOR_NOTE_ATRAS,
	SENSOR_NOTE_ADELANTE_ROTADA,
	SENSOR_NOTE_ATRAS_ROTADA
};

int noteRelease[4] = {
	NOTE_RELEASE_ADELANTE,
	NOTE_RELEASE_ATRAS,
	NOTE_RELEASE_ADELANTE_ROTADA,
	NOTE_RELEASE_ATRAS_ROTADA
};

int noteThreshold[4] = {
	NOTE_THRESHOLD_ADELANTE,
	NOTE_THRESHOLD_ATRAS,
	NOTE_THRESHOLD_ADELANTE_ROTADA,
	NOTE_THRESHOLD_ATRAS_ROTADA
};

// NoteOn = 5 (NoteOff)
int noteOn = 5;
/*
NoteOnUp = 1
NoteOnDown = 2
NoteOnUpRotated = 3
NoteOnDownRotated = 4
boolean NoteOnUp = false;
boolean NoteOnDown = false;
boolean NoteOnUpRotated = false;
boolean NoteOnDownRotated/ = false;
*/

float v[6];
FreeSixIMU my3IMU = FreeSixIMU();

int currentAccelX, currentAccelY;
HitEvent event;

////////////////////////////////////////

void setup() {
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
	accelXBuffer.clear();
	accelYBuffer.clear();
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
}

void loop() {
	my3IMU.getValues(v);
	sensorRead[SENSOR_ACCEL_X] = (int)v[0];
	sensorRead[SENSOR_ACCEL_Y] = (int)v[1];
	setCircularBuffer();
	if(bufferReady || isBufferReady()){
		event = caxixiState.update(&accelXBuffer, &accelYBuffer);
		currentAccelX = sensorRead[SENSOR_ACCEL_X];
		currentAccelY = sensorRead[SENSOR_ACCEL_Y];
		
		switch (noteOn) {
			case NotaAdelante:
				if(noteReleaseUp()){
					SendNoteOff(sensorNote[NotaAdelante]);
					noteOn = 5;
				}
				break;
			case NotaAtras:
				if(noteReleaseDown()){
					SendNoteOff(sensorNote[NotaAtras]);
					noteOn = 5;
				}
				break;
			case NotaAdelanteRotada:
				if(noteReleaseUpRotated()){
					SendNoteOff(sensorNote[NotaAdelanteRotada]);
					noteOn = 5;
				}
				break;
			case NotaAtrasRotada:
				if(noteReleaseDownRotated()){
					SendNoteOff(sensorNote[NotaAtrasRotada]);
					noteOn = 5;
				}
				break;
		}

		if(noteOn == 5 && event == HITEVENT_FORWARD){
			noteOn = NotaAdelante;
			SendNoteOn(sensorNote[NotaAdelante]);
		}

		if(noteOn == 5 && event == HITEVENT_BACKWARD){
			noteOn = NotaAtras;
			SendNoteOn(sensorNote[NotaAtras]);
		}
	}
	delay(3);
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
	smoothAccelX = digitalSmooth(sensorRead[SENSOR_ACCEL_X], accelXSmooth);
	smoothAccelY = digitalSmooth(sensorRead[SENSOR_ACCEL_Y], accelYSmooth);
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

///////////////CAXIXI FUNCTIONS/////////////
////////////////////////////////////////////

boolean noteReleaseUp()
{
	if(currentAccelX < noteRelease[NotaAdelante]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseDown()
{
	if(currentAccelX > noteRelease[NotaAtras]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseUpRotated()
{
	if(currentAccelY < noteRelease[NotaAdelanteRotada]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseDownRotated()
{
	if(currentAccelY > noteRelease[NotaAtrasRotada]){
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



