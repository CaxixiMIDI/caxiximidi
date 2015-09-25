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

CxCircularBuffer AccelXBuffer(BUFFER_SIZE);
CxCircularBuffer AccelYBuffer(BUFFER_SIZE);
HitStateMachine HitState();
boolean bufferReady = false;

//////////////CAXIXI/////////////////////////
int AccelXSmooth[filterSamples];
int AccelYSmooth[filterSamples];
int smoothAccelX;
int smoothAccelY;

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
	AccelXBuffer.clear();
	AccelYBuffer.clear();
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
}

void loop() {
	my3IMU.getValues(v);
	SensorRead[SENSOR_ACCEL_X] = (int)v[0];
	SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
	setCircularBuffer();
	if(bufferReady || isBufferReady()){
		event = HitState->update(AccelXBuffer, AccelYBuffer);
		currentAccelX = SensorRead[SENSOR_ACCEL_X];
		currentAccelY = SensorRead[SENSOR_ACCEL_Y];
		
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

		if(NoteOn == 5 && event == HITEVENT_FORWARD){
			NoteOn = NotaAdelante;
			SendNoteOn(SensorNote[NotaAdelante]);
		}

		if(NoteOn == 5 && event == HITEVENT_BACKWARD){
			NoteOn = NotaAtras;
			SendNoteOn(SensorNote[NotaAtras]);
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
	smoothAccelX = digitalSmooth(SensorRead[SENSOR_ACCEL_X], AccelXSmooth);
	smoothAccelY = digitalSmooth(SensorRead[SENSOR_ACCEL_Y], AccelYSmooth);
	AccelXBuffer.addValue(smoothAccelX);
	AccelYBuffer.addValue(smoothAccelY);
}

boolean isBufferReady(){
	if(AccelXBuffer.getCount() < BUFFER_SIZE
	|| AccelYBuffer.getCount() < BUFFER_SIZE){
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
