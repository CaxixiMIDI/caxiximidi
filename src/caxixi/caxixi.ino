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

CxCircularBuffer AccelXBuffer(BUFFER_SIZE);
CxCircularBuffer AccelYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBuffer(BUFFER_SIZE);
CxCircularBuffer CanHitBufferRotated(BUFFER_SIZE);
boolean bufferReady = false;

/////////////OCTAVADOR//////////
const int OctaveDown = 7;		// the number of the pushbutton OctaveDown pin
const int OctaveUp = 6;			// the number of the pushbutton OctaveUp pin
const int ledPin =  13;			// the number of the LED pin
int lastUpState = 0;
int lastDownState = 0;
int UpState = 0;						// variable for reading the pushbutton status
int DownState = 0;
//////////////////////////////////


////////// SAMPLER ///////////////
Buffer samples[800]; //puede almacenar hasta 50 notas (50 on, 50 off)
int bufferI=0; //indice del buffer para record
int bufferJ=0; //indice del buffer para play
// BUTTON RECORD const & var:
const int  buttonPinRecord = 2;    // the pin that the pushbutton is attached to
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int buttonNoteState = 0;
int lastButtonNoteState = 0;

bool record = false;
bool play = false;
bool isSetT0 = false;
bool isSetT1 = false;
bool FirstNote = false;
bool isSetOctavador = true;

long time = 0;		//time = millis() cuenta los milisegundos desde q arranca
long t0 = 0;			//cuando record==True, ajustamos una variable "t0" a esos msec y t = millis()-t0
long t1 = 0;			//cuando record == False, ajustamos una variable "t1", t1 = millis-t0
long reset = 0;		//y ajustamos la variable reset = millis - t0 - t1, que en principio va a ser cero, pero en cada vuelta va a ir incrementándose en el tamaño del loop
///////////////////////////////////////////////

//////////////CAXIXI/////////////////////////
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
////////////////////////////////////////

void setup() {
	pinMode(ledPin, OUTPUT);// initialize the LED as an output:
	pinMode(OctaveUp, INPUT);// initialize the button pin as a input:
	pinMode(OctaveDown, INPUT);
        pinMode(buttonPinRecord, INPUT);
	MIDI.begin(1);
	Serial.begin(57600);
	Wire.begin();
	CanHitBuffer.clear();
	CanHitBufferRotated.clear();
	initCanHitBuffer();
	initCanHitBufferRotated();
	delay(100);
	initSamplerBuffer();
	delay(100);
	my3IMU.init();
	my3IMU.acc.setFullResBit(true);
	my3IMU.acc.setRangeSetting(16);
	delay(5);
	iterationNumber = 0;
}

void loop() {
	time = millis()-(t0 + t1 + reset);
	setReset();
	PlayBuffer();
	ButtonRecord();
	Lights();// turns on the LED every two button pushes by checking the modulo of the button push counter.
	Octavador();
	iterationNumber = iterationNumber + 1;
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
	}else{
		Serial.println("Buffer Not Ready!");
	}
	delay(3);
}

void initCanHitBuffer()
{
	CanHitBuffer.addValue(0);
}

void initCanHitBufferRotated()
{
	CanHitBufferRotated.addValue(0);
}

void initSamplerBuffer(){//Esta funcion asigna time=9999999 a todos los valores del buffer
	for(int k=0; k<800; k++){
		samples[k] = Buffer_default;
	}
}

void Octavador(){
	UpState = digitalRead(OctaveUp);
	DownState = digitalRead(OctaveDown);
	// check if the pushbutton is pressed.
	// if it is, the buttonState is HIGH:
	if (UpState != lastUpState) {
		if (UpState == HIGH) {    
			SendNoteOff(SensorNote[NotaAdelante]);
			SendNoteOff(SensorNote[NotaAtras]);
			SendNoteOff(SensorNote[NotaAdelanteRotada]);
			SensorNote[NotaAdelante] = SensorNote[NotaAdelante]+12;
			SensorNote[NotaAtras] = SensorNote[NotaAtras]+12;
			SensorNote[NotaAdelanteRotada] = SensorNote[NotaAdelanteRotada]+12;
			//Serial.print("Notas: ");
			//Serial.print(SensorNote[NotaAdelante]);
			//Serial.print(SensorNote[NotaAtras]);
			//Serial.println(SensorNote[NotaAdelanteRotada]);   
			digitalWrite(ledPin, HIGH);  
		} else {
			// turn LED off:
			digitalWrite(ledPin, LOW);
		}
		lastUpState = UpState;
	}
  if (DownState != lastDownState){     
		if (DownState == HIGH) {    
			SendNoteOff(SensorNote[NotaAdelante]);
			SendNoteOff(SensorNote[NotaAtras]);
			SendNoteOff(SensorNote[NotaAdelanteRotada]);
			SensorNote[NotaAdelante] = SensorNote[NotaAdelante]-12;
			SensorNote[NotaAtras] = SensorNote[NotaAtras]-12;
			SensorNote[NotaAdelanteRotada] = SensorNote[NotaAdelanteRotada]-12;
			digitalWrite(ledPin, HIGH);  
		} else {
			// turn LED off:
			digitalWrite(ledPin, LOW);
		}
		lastDownState = DownState;
	}
}

void setReset(){
	int a = t1 - time;
	if(t0 != 0 && t1!=0 && a<=0){
		bubbleSort(samples,100);
		reset = millis()-t0-t1;
		bufferJ = 0;
	}
}

void ButtonRecord() {
	buttonState = digitalRead(buttonPinRecord);
	if (buttonState != lastButtonState) {// if the state has changed, increment the counter
		if (buttonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
			buttonPushCounter++;
			if (record==true){
				record=false;
				isSetOctavador=false;
				setT1();
			} else {
				record=true;
			}
		}
		lastButtonState = buttonState; // save the current state as the last state, for next time through the loop
	}
}

void Lights(){
	// turns on the LED every two button pushes by checking the modulo of the button push counter.
	if (buttonPushCounter % 2 == 1) {
		digitalWrite(ledPin, HIGH);
	} else {
		digitalWrite(ledPin, LOW);
	}
}

void bubbleSort(Buffer a[], int size) {
	for(int i=0; i<(size-1); i++) {
		for(int o=0; o<(size-(i+1)); o++) {
			if(a[o].time > a[o+1].time) {
				Buffer t = a[o];
				a[o] = a[o+1];
				a[o+1] = t;
			}
		}
	}
}

void setT0()//A esta fc la llama ButtonRecord()
{
	if (record && !isSetT0 && FirstNote){
		t0 = millis();
	}
	if (record && !isSetT0&& FirstNote){
		isSetT0 = true;
	}
}

void setT1(){
	if(!record && !isSetT1 && t0 != 0){
		t1 = millis() - t0;
		isSetT1 = true;
		play = true;
	}
}


void SendNoteOn(int note)
{
	if(record){
		FirstNote = true;
		setT0();
		time = millis()-(t0 + t1 + reset);  //Esto es para que a la primera nota la guarde con time == 0
		Buffer sample = {note, 1, time};
		samples[bufferI] = sample;
	}
	MIDI.sendNoteOn(note,127,midiChannel);
	if(record){
		bufferI++;  
	}
}

void SendNoteOff(int note)
{
	if(record){
		Buffer sample = {note, 0, time};
		samples[bufferI] = sample;
	}
	MIDI.sendNoteOff(note,127,midiChannel);
	if(record){
		bufferI++;  
	}
}

void PlayBuffer() {
	if(play){
		if(samples[bufferJ].time-time<0 && samples[bufferJ].time-time>-100){//Esto es un hardcodeo (el -100), corregirlo mirando el reloj en la V2, o dejarlo...
			if(samples[bufferJ].encendido){
				MIDI.sendNoteOn(samples[bufferJ].note,127,midiChannel);
			}else{
				MIDI.sendNoteOff(samples[bufferJ].note,127,midiChannel);
			}
			bufferJ++; //J se reinicia cuando da la vuelta, en la fc setReset 
		}
	}
}

///////////////CAXIXI FUNCTIONS/////////////
////////////////////////////////////////////
boolean isBufferReady(){
	if(CanHitBuffer.getCount() < 1
	|| CanHitBufferRotated.getCount() < 1){
		return false;
	}else{
		bufferReady = true;
		return true;
	}
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
