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


/////////PIEZO VARIABLES////////////
#include "microsmooth.h"

unsigned char PiezoSensorNote[2] = {36, 38};          // Nota MIDI correspondiente a cada Sensor (Piezo)
int PiezoNoteCutOff[2] = {1020,1020};                // Valor minimo para que sea considerado un "golpe" en cada Sensor (Piezo)
int PiezoNoteRelease[6] = {800, 800};
boolean activeNote[6] = {0,0,0,0,0,0};

boolean VelocityFlag  = false;                  // Velocity ON (true) o OFF (false)
boolean MicroSmoothFilter = false;
boolean specialMicroSmooth = false;
boolean SensorRaw = false;

int A_AdelanteSmoothValues[PIEZO_SMOOTH_SAMPLES];  //Samples de smooth para los piezos! No son los mismo que para el accel
int A_AtrasSmoothValues[PIEZO_SMOOTH_SAMPLES];

int SensorHitAvg[2] = {0, 0};

uint16_t* hA_AdelanteSmoothValues = ms_init(SGA);
boolean debug = false;
/////////////////////////////////////////

CxCircularBuffer AccelXBuffer(BUFFER_SIZE);
CxCircularBuffer AccelYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBufferRotated(BUFFER_SIZE);
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

int canHitResolution = 5;
int canHitDefinition = 1;

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

boolean isUpThreshold, isDownThreshold, isUpThresholdRotated, isDownThresholdRotated = false;
boolean prevIsUpThreshold, prevIsDownThreshold, prevIsUpThresholdRotated, prevIsDownThresholdRotated = false;
boolean canHitUp, canHitDown;
boolean canHitRotated;

int currentAccelX, currentAccelY;
int iterationNumber;
int hitsThreshold = HITS_THRESOLD;

boolean debugOutput = false;

void setup() {
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
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
  	if (SensorRaw) {
		SensorHitAvg[A_PIEZO_ADELANTE] = readSensorRaw(A_PIEZO_ADELANTE);
		SensorHitAvg[A_PIEZO_ATRAS] = readSensorRaw(A_PIEZO_ATRAS);			
	} else {
		SensorHitAvg[A_PIEZO_ADELANTE] = readSensor(A_PIEZO_ADELANTE);
		SensorHitAvg[A_PIEZO_ATRAS] = readSensor(A_PIEZO_ATRAS);			
	}
	if (isReleased() && hasHit()) {
		processHit();
	}
	iterationNumber = iterationNumber + 1;
	//initialMillis = millis();
	my3IMU.getAngles(angles);
	my3IMU.getValues(v);
	SensorRead[SENSOR_ACCEL_X] = (int)v[0];
	SensorRead[SENSOR_ACCEL_Y] = (int)v[1];
	SensorRead[SENSOR_ACCEL_Z] = (int)v[2];
	SensorRead[SENSOR_GYRO_X] = (int)angles[0];
	SensorRead[SENSOR_GYRO_Y] = (int)angles[1];
	SensorRead[SENSOR_GYRO_Z] = (int)angles[2];
	if(bufferReady || isBufferReady()){
		currentAccelX = SensorRead[SENSOR_ACCEL_X];
		currentAccelY = SensorRead[SENSOR_ACCEL_Y];
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
			
		if(NoteOn == 5 && isUpThreshold && canHitUp){
			NoteOn = NotaAdelante;
			if(debugOutput){
				Serial.println("HITUP");
			}else{
				SendNoteOn(SensorNote[NotaAdelante]);
			}
		}
			
		if(NoteOn == 5 && isDownThreshold && canHitDown){
			NoteOn = NotaAtras;
			if(debugOutput){
				Serial.println("HITDOWN");
			}else{
				SendNoteOn(SensorNote[NotaAtras]);
			}
		}
		
		if(NoteOn == 5 && isUpThresholdRotated && canHitRotated){
			NoteOn = NotaAdelanteRotada;
			if(debugOutput){
				Serial.println("ROTATED_HITUP");
			}else{
				SendNoteOn(SensorNote[NotaAdelanteRotada]);	
			}
		}

		if(NoteOn == 5 && isDownThresholdRotated && canHitRotated){
			NoteOn = NotaAtrasRotada;
			if(debugOutput){
				Serial.println("ROTATED_HITDOWN");
			}else{
				SendNoteOn(SensorNote[NotaAtrasRotada]);
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

////////////////////////////////////
//////////PIEZO FUNCTIONS////////////
/////////////////////////////////////

boolean isReleased()
{
	if (activeNote[A_NotaAdelante]) {
		if (SensorHitAvg[A_PIEZO_ADELANTE] < PiezoNoteRelease[A_NotaAdelante]) {
			activeNote[A_NotaAdelante] = false;
			MIDI.sendNoteOff(PiezoSensorNote[A_NotaAdelante],127,midiChannel);
		}
	}
	if (activeNote[A_NotaAtras]) {
		if (SensorHitAvg[A_PIEZO_ATRAS] < PiezoNoteRelease[A_NotaAtras]) {
			activeNote[A_NotaAtras] = false;
			MIDI.sendNoteOff(PiezoSensorNote[A_NotaAtras],127,midiChannel);
		}
	}
	
	if (!activeNote[A_NotaAdelante] && !activeNote[A_NotaAtras]) {
		return true;
	} else {
		return false;
	}
}

boolean hasHit()
{
	if (SensorHitAvg[A_PIEZO_ADELANTE] > PiezoNoteCutOff[A_NotaAdelante]) {
		activeNote[A_NotaAdelante] = true;
	}else if (SensorHitAvg[A_PIEZO_ATRAS] > PiezoNoteCutOff[A_NotaAtras]) {
		activeNote[A_NotaAtras] = true;
	}
	if (
		activeNote[A_NotaAdelante] || activeNote[A_NotaAtras]
	) {
		return true;
	} else {
		return false;
	}
}

void processHit()
{
	if (activeNote[A_NotaAdelante]) {
		MIDI.sendNoteOn(PiezoSensorNote[A_NotaAdelante],127,midiChannel);
	}else if(activeNote[A_NotaAtras]) {
		MIDI.sendNoteOn(PiezoSensorNote[A_NotaAtras],127,midiChannel);
	}
}


int readSensorRaw (int sensor)
{
	int val;
	switch (sensor) {
    case A_PIEZO_ADELANTE:
			val = analogRead(A_PIEZO_ADELANTE);
      break;
		case A_PIEZO_ATRAS:
			val = analogRead(A_PIEZO_ATRAS);
			break;    
 	 }    
	 return val;
}

int readSensor( int sensor)
{
  int valavg;
  int val;

  switch (sensor) {
    case A_PIEZO_ADELANTE:
			val = readSensorRaw(A_PIEZO_ADELANTE);
			if (specialMicroSmooth) {
				valavg = sga_filter(val, hA_AdelanteSmoothValues);
			} else {
				valavg = digitalSmooth(val, A_AdelanteSmoothValues);  // every sensor you use with digitalSmooth needs its own array	
			}
      break;
		case A_PIEZO_ATRAS:
			val = readSensorRaw(A_PIEZO_ATRAS);
			if (MicroSmoothFilter) {
			//	valavg = sga_filter(val, hA_AtrasSmoothValues);
			} else {
				valavg = digitalSmooth(val, A_AtrasSmoothValues);  // every sensor you use with digitalSmooth needs its own array
			}
			break;    
 	 }    
	 return valavg;
}
/////////////////////////
/////////FILTER//////////
/////////////////////////
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
