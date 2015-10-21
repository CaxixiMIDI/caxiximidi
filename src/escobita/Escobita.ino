#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

/*
CAXIXI XBee
*/
#include <SoftwareSerial.h>
SoftwareSerial xbee(2, 3); // RX, TX

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

boolean debug = false;
/////////////////////////////////////////

CxCircularBuffer AccelXBuffer(BUFFER_SIZE);
CxCircularBuffer AccelYBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBufferRotated(BUFFER_SIZE);
boolean bufferReady = false;
/*
int samples = 0;
int resolution = RESOLUTION;
int iteration = 0;
*/
int AccelXSmooth[SMOOTH_FILTER];
int AccelYSmooth[SMOOTH_FILTER];
int GyroXSmooth[SMOOTH_FILTER];
int smoothAccelX;
int smoothAccelY;
int smoothGyroX;

int SensorRead[6] = {0, 0, 0, 0, 0, 0};
unsigned char SensorNote[5] = {
	MIDI_NOTE_FORWARD,
	MIDI_NOTE_BACKWARD,
	MIDI_NOTE_HIT,
	MIDI_NOTE_PIEZO_FRONT,
	MIDI_NOTE_PIEZO_BACK
};

int NoteRelease[5] = {
	NOTE_RELEASE_FORWARD,
	NOTE_RELEASE_BACKWARD,
	NOTE_RELEASE_HIT,
	NOTE_RELEASE_PIEZO_FRONT,
	NOTE_RELEASE_PIEZO_BACK
};

int NoteThreshold[5] = {
	NOTE_THRESHOLD_FORWARD,
	NOTE_THRESHOLD_BACKWARD,
	NOTE_THRESHOLD_HIT,
	NOTE_THRESHOLD_PIEZO_FRONT,
	NOTE_THRESHOLD_PIEZO_BACK
};

int canHitResolution = 5;
int canHitDefinition = 1;

int NoteOn = NOTE_OFF;
boolean piezoFrontNoteOn = false;
boolean piezoBackNoteOn = false;

float v[6];
float angles[3];
FreeSixIMU my3IMU = FreeSixIMU();

boolean isUpThreshold, isDownThreshold, isUpThresholdRotated, isDownThresholdRotated = false;
boolean prevIsUpThreshold, prevIsDownThreshold, prevIsUpThresholdRotated, prevIsDownThresholdRotated = false;
boolean canHitUp, canHitDown;
boolean canHitRotated;

int currentAccelX, currentAccelY;
int iterationNumber;
int hitsThreshold = CAN_HIT_THRESOLD;

boolean debugOutput = false;

void setup() {
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
	xbee.begin(57600);
	CanHitBuffer.clear();
	CanHitBufferRotated.clear();
	initCanHitBuffer();
	initCanHitBufferRotated();
	delay(5);
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
	iterationNumber = 0;
}

int initialMillis;

void loop() {
	if(isReleased() && hasHit()) {
		processHit();
	}
	iterationNumber = iterationNumber + 1;
	//initialMillis = millis();
	my3IMU.getAngles(angles);
	my3IMU.getValues(v);
	SensorRead[SENSORREAD_ACCEL_X] = (int)v[0];
	SensorRead[SENSORREAD_ACCEL_Y] = (int)v[1];
	SensorRead[SENSORREAD_PIEZO_FRONT] = readSensorRaw(SENSOR_PIEZO_FRONT_ANALOG);
	SensorRead[SENSORREAD_PIEZO_BACK] = readSensorRaw(SENSOR_PIEZO_BACK_ANALOG);
		
	if(bufferReady || isBufferReady()){
		currentAccelX = SensorRead[SENSORREAD_ACCEL_X];
		currentAccelY = SensorRead[SENSORREAD_ACCEL_Y];
		prevIsUpThreshold = isUpThreshold;
		prevIsDownThreshold = isDownThreshold;
		prevIsUpThresholdRotated = isUpThresholdRotated;
		prevIsDownThresholdRotated = isDownThresholdRotated;
		setThresholds();
		setThresholdsRotated();
		setCanHitUp();
		setCanHitDown();
		setCanHitRotated();
		switch (NoteOn) {
			case NOTE_FORWARD:
				if(noteReleaseUp()){
					SendNoteOff(SensorNote[NOTE_FORWARD]);
					NoteOn = 5;
				}
				break;
			case NOTE_BACKWARD:
				if(noteReleaseDown()){
					SendNoteOff(SensorNote[NOTE_BACKWARD]);
					NoteOn = 5;
				}
				break;
			case NOTE_HIT:
				if(noteReleaseUpRotated()){
					SendNoteOff(SensorNote[NOTE_HIT]);
					NoteOn = 5;
				}
				break;
		}
			
		if(NoteOn == 5 && isUpThreshold && canHitUp){
			NoteOn = NOTE_FORWARD;
			if(debugOutput){
				Serial.println("HITUP");
			}else{
				SendNoteOn(SensorNote[NOTE_FORWARD]);
			}
		}
			
		if(NoteOn == 5 && isDownThreshold && canHitDown){
			NoteOn = NOTE_BACKWARD;
			if(debugOutput){
				Serial.println("HITDOWN");
			}else{
				SendNoteOn(SensorNote[NOTE_BACKWARD]);
			}
		}
		
		if(NoteOn == 5 && isUpThresholdRotated && canHitRotated){
			NoteOn = NOTE_HIT;
			if(debugOutput){
				Serial.println("ROTATED_HITUP");
			}else{
				SendNoteOn(SensorNote[NOTE_HIT]);	
			}
		}
	}
	delay(4);
}

void SendNoteOn(int note)
{
	MIDI.sendNoteOn(note,127,midiChannel);
}

void SendNoteOff(int note)
{
	MIDI.sendNoteOff(note,127,midiChannel);
}

boolean isBufferReady(){
	if(CanHitBuffer.getCount() < 1
	|| CanHitBufferRotated.getCount() < 1){
		return false;
	}else{
		bufferReady = true;
		return true;
	}
}

void initCanHitBuffer()
{
	CanHitBuffer.addValue(0);
}

void initCanHitBufferRotated()
{
	CanHitBufferRotated.addValue(0);
}

void setCanHitUp()
{
	if(NoteOn != 5){
		canHitUp = false;
		return;
	}
	/*
	 * It must has threshold AND it must be different from previous
	*/
	if(isUpThreshold && (isUpThreshold != prevIsUpThreshold)){
		int lastThreshold = CanHitBuffer.getPreviousElement(1);
		CanHitBuffer.addValue(iterationNumber);
		/*
		String debugCanHit = "EvaluateHitUp: (iterationNumber-lastThreshold) - (";
		debugCanHit += iterationNumber;
		debugCanHit += " - ";
		debugCanHit += lastThreshold;
		debugCanHit += ") = ";
		debugCanHit += abs(iterationNumber - lastThreshold);
		*/
		if(debugOutput){
			if(abs(iterationNumber - lastThreshold) > hitsThreshold){			
					String debugPossibleHit = "PossibleHitUp: ";
					debugPossibleHit += abs(iterationNumber - lastThreshold);
					Serial.println(debugPossibleHit);
			}
		}
		if(abs((iterationNumber - lastThreshold)) < hitsThreshold){
			canHitUp = true;
		}else{
			canHitUp = false;
		}
	}
}

void setCanHitDown()
{
	if(NoteOn != 5){
		canHitDown = false;
		return;
	}
	/*
	 * It must has threshold AND it must be different from previous
	*/
	if(isDownThreshold && (isDownThreshold != prevIsDownThreshold)){
		int lastThreshold = CanHitBuffer.getPreviousElement(1);
		CanHitBuffer.addValue(iterationNumber);
		/*
		String debugCanHit = "Evaluate: (iterationNumber-lastThreshold) - (";
		debugCanHit += iterationNumber;
		debugCanHit += " - ";
		debugCanHit += lastThreshold;
		debugCanHit += ") = ";
		debugCanHit += abs(iterationNumber - lastThreshold);
		Serial.println(debugCanHit);
		*/
		if(debugOutput){
			if(abs(iterationNumber - lastThreshold) > hitsThreshold){
				String debugPossibleHit = "PossibleHitDown: ";
				debugPossibleHit += abs(iterationNumber - lastThreshold);
				Serial.println(debugPossibleHit);
			}
		}
		if(abs((iterationNumber - lastThreshold)) < hitsThreshold){
			canHitDown = true;
		}else{
			canHitDown = false;
		}
	}
}

void setCanHitRotated()
{
	if(NoteOn != 5){
		canHitRotated = false;
		return;
	}
	/*
	 * It must has threshold AND it must be different from previous
	*/
	if(isUpThresholdRotated){
		/*
		int lastThresholdRotated = CanHitBufferRotated.getPreviousElement(1);
		CanHitBufferRotated.addValue(iterationNumber);
		/*
		String debugCanHit = "Evaluate: (iterationNumber-lastThreshold) - (";
		debugCanHit += iterationNumber;
		debugCanHit += " - ";
		debugCanHit += lastThreshold;
		debugCanHit += ") = ";
		debugCanHit += abs(iterationNumber - lastThreshold);
		Serial.println(debugCanHit);
		
		if(abs((iterationNumber - lastThresholdRotated)) < hitsThreshold){
			canHitRotated = true;
		}else{
			canHitRotated = false;
		}
		*/
		canHitRotated = true;
	}
}

void setThresholds()
{
	isUpThreshold = false;
	isDownThreshold = false;
	if(currentAccelX > NoteThreshold[NOTE_FORWARD]){
		isUpThreshold = true;
		isDownThreshold = false;
		return;
	}else{
		isUpThreshold = false;
	}
	if(currentAccelX < NoteThreshold[NOTE_BACKWARD]){
		isDownThreshold = true;		
		return;
	}else{
		isDownThreshold = false;
	}
}

void setThresholdsRotated()
{
	if(currentAccelY > NoteThreshold[NOTE_HIT]){
		isUpThresholdRotated = true;
		isDownThresholdRotated = false;
		return;
	}else{
		isUpThresholdRotated = false;
	}
}

boolean noteReleaseUp()
{
	if(currentAccelX < NoteRelease[NOTE_FORWARD]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseDown()
{
	if(currentAccelX > NoteRelease[NOTE_BACKWARD]){
		return true;
	}else{
		return false;
	}
}

boolean noteReleaseUpRotated()
{
	if(currentAccelY < NoteRelease[NOTE_HIT]){
		return true;
	}else{
		return false;
	}
}

////////////////////////////////////
//////////PIEZO FUNCTIONS////////////
/////////////////////////////////////

boolean isReleased()
{
	if (piezoFrontNoteOn == true) {
		if (SensorRead[SENSORREAD_PIEZO_FRONT] < NoteRelease[NOTE_PIEZO_FRONT]) {
			piezoFrontNoteOn = false;
			MIDI.sendNoteOff(SensorNote[NOTE_PIEZO_FRONT],127,midiChannel);
		}
	}
	if (piezoBackNoteOn == true) {
		if (SensorRead[SENSORREAD_PIEZO_BACK] < NoteRelease[NOTE_PIEZO_BACK]) {
			piezoBackNoteOn = false;
			MIDI.sendNoteOff(SensorNote[NOTE_PIEZO_BACK],127,midiChannel);
		}
	}
	
	if (!piezoFrontNoteOn && !piezoBackNoteOn) {
		return true;
	} else {
		return false;
	}
}

boolean hasHit()
{
	if (SensorRead[SENSORREAD_PIEZO_FRONT] > NoteThreshold[NOTE_PIEZO_FRONT]) {
		piezoFrontNoteOn = true;
	}else if (SensorRead[SENSORREAD_PIEZO_BACK] > NoteThreshold[NOTE_PIEZO_BACK]) {
		piezoBackNoteOn = true;
	}
	if (piezoFrontNoteOn || piezoBackNoteOn) {
		return true;
	} else {
		return false;
	}
}

void processHit()
{
	if (piezoFrontNoteOn) {
		MIDI.sendNoteOn(SensorNote[NOTE_PIEZO_FRONT],127,midiChannel);
	}else if(piezoBackNoteOn) {
		MIDI.sendNoteOn(SensorNote[NOTE_PIEZO_BACK],127,midiChannel);
	}
}


int readSensorRaw(int sensor)
{
	int val;
	switch (sensor) {
		case SENSOR_PIEZO_FRONT_ANALOG:
			val = analogRead(SENSOR_PIEZO_FRONT_ANALOG);
			break;
		case SENSOR_PIEZO_BACK_ANALOG:
			val = analogRead(SENSOR_PIEZO_BACK_ANALOG);
			break;
	}
	return val;
}



/////////////////////////
/////////FILTER//////////
/////////////////////////
int digitalSmooth(int rawIn, int *sensSmoothArray){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, temp, top, bottom;
  long total;
  static int i;
 // static int raw[SMOOTH_FILTER];
  static int sorted[SMOOTH_FILTER];
  boolean done;

  i = (i + 1) % SMOOTH_FILTER;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  sensSmoothArray[i] = rawIn;                 // input new data into the oldest slot

  // Serial.print("raw = ");

  for (j=0; j<SMOOTH_FILTER; j++){     // transfer data array into anther array for sorting and averaging
    sorted[j] = sensSmoothArray[j];
  }

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (SMOOTH_FILTER - 1); j++){
      if (sorted[j] > sorted[j + 1]){     // numbers are out of order - swap
        temp = sorted[j + 1];
        sorted [j+1] =  sorted[j] ;
        sorted [j] = temp;
        done = 0;
      }
    }
  }

/*
  for (j = 0; j < (SMOOTH_FILTER); j++){    // print the array to debug
    Serial.print(sorted[j]); 
    Serial.print("   "); 
  }
  Serial.println();
*/

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((SMOOTH_FILTER * 15)  / 100), 1); 
  top = min((((SMOOTH_FILTER * 85) / 100) + 1  ), (SMOOTH_FILTER - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
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


