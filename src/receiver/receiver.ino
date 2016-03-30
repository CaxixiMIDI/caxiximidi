#include "CaxixiConfig.h"

#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
struct MyMidiSettings : public midi::DefaultSettings {
	//static const bool UseRunningStatus = false; // Messes with my old equipment!
	static const long DefaultSettings::BaudRate = 9600;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);

#include "CxCircularBuffer.h"
#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

boolean midiMode = false;

char inData[10];
int index;
boolean started = false;
boolean ended = false;
int inInt;

int cxRightForwardNote = CAXIXI_RIGHT_NOTE_FORWARD;
int cxRightBackwardNote = CAXIXI_RIGHT_NOTE_BACKWARD;
int cxRightHitNote = CAXIXI_RIGHT_NOTE_HIT;
int cxLeftForwardNote = CAXIXI_LEFT_NOTE_FORWARD;
int cxLeftBackwardNote = CAXIXI_LEFT_NOTE_BACKWARD;
int cxLeftHitNote = CAXIXI_LEFT_NOTE_HIT;


/////////////OCTAVADOR//////////
int lastUpState = 0;
int lastDownState = 0;
int UpState = 0;	// variable for reading the pushbutton status
int DownState = 0;
int CurrentOctave = 0;
//////////////////////////////////

////////// SAMPLER ///////////////
Buffer samples[SAMPLER_BUFFER_SIZE]; //puede almacenar hasta 50 notas (50 on, 50 off)
int bufferI=0; //indice del buffer para record
int bufferJ=0; //indice del buffer para play
// BUTTON RECORD const & var:
int buttonPushCounter = 0;				// counter for the number of button presses
int buttonState = 0;							// current state of the button
int lastButtonState = 0;					// previous state of the button
int buttonNoteState = 0;
int lastButtonNoteState = 0;
int buttonUndoState = 0;
int lastButtonUndoState = 0;

int layer = 0;

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


void setup()
{
	pinMode(OCTAVE_LED_PIN, OUTPUT);// initialize the LED as an output:
	pinMode(OCTAVE_UP_BUTTON_PIN, INPUT);// initialize the button pin as a input:
	pinMode(OCTAVE_DOWN_BUTTON_PIN, INPUT);
	pinMode(SAMPLER_BUTTON_RECORD_PIN, INPUT);
	pinMode(SAMPLER_BUTTON_UNDO_PIN, INPUT);
	pinMode(OCTAVE_UP_LED_RED_PIN, OUTPUT);
	pinMode(OCTAVE_UP_LED_BLUE_PIN, OUTPUT);
	pinMode(OCTAVE_UP_LED_GREEN_PIN, OUTPUT);
	pinMode(OCTAVE_DOWN_LED_RED_PIN, OUTPUT);
	pinMode(OCTAVE_DOWN_LED_BLUE_PIN, OUTPUT);
	pinMode(OCTAVE_DOWN_LED_GREEN_PIN, OUTPUT);
	analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_GREEN_PIN, 255);
	analogWrite(OCTAVE_UP_LED_RED_PIN, 255);
	analogWrite(OCTAVE_UP_LED_GREEN_PIN, 255);
	delay(500);
	analogWrite(OCTAVE_UP_LED_RED_PIN,255);
	analogWrite(OCTAVE_UP_LED_GREEN_PIN,0);
	delay(500);
	analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_RED_PIN,0);
	delay(500);
	analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,0);
	delay(500);
	analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
	MIDI.begin(1);
	XBee.begin(9600);
	Serial.begin(9600);
	delay(100);
	initSamplerBuffer();
	delay(100);
}

void loop()
{
	time = millis()-(t0 + t1 + reset);
	setReset();
	PlayBuffer();
	ButtonRecord();
	ButtonUndo();
	Lights();// turns on the LED every two button pushes by checking the modulo of the button push counter.
	Octavador();
	while(XBee.available() > 0){
		char aChar = XBee.read();
		if(aChar == '<'){
			started = true;
			index = 0;
			inData[index] = '\0';
		}else if(aChar == '>'){
			ended = true;
		}else if(started){
			inData[index] = aChar;
			index++;
			inData[index] = '\0';
		}
		if(started && ended){
			// Convert the string to an integer
			inInt = atoi(inData);
			// Use the value
			//Serial.println(inInt);
			if (inInt == CAXIXI_RIGHT_FORWARD_NOTEON){
				SendNoteOn(cxRightForwardNote);
			} else if (inInt == CAXIXI_RIGHT_FORWARD_NOTEOFF){
				SendNoteOff(cxRightForwardNote);
			} else if (inInt == CAXIXI_RIGHT_BACKWARD_NOTEON){
				SendNoteOn(cxRightBackwardNote);
			} else if (inInt == CAXIXI_RIGHT_BACKWARD_NOTEOFF){	
				SendNoteOff(cxRightBackwardNote);
			} else if (inInt == CAXIXI_RIGHT_HIT_NOTEON){	
				SendNoteOn(cxRightHitNote);
			} else if (inInt == CAXIXI_RIGHT_HIT_NOTEOFF){	
				SendNoteOff(cxRightHitNote);
			} else if (inInt == CAXIXI_LEFT_FORWARD_NOTEON){
				SendNoteOn(cxLeftForwardNote);
			} else if (inInt == CAXIXI_LEFT_FORWARD_NOTEOFF){
				SendNoteOff(cxLeftForwardNote);
			} else if (inInt == CAXIXI_LEFT_BACKWARD_NOTEON){	
				SendNoteOn(cxLeftBackwardNote);
			} else if (inInt == CAXIXI_LEFT_BACKWARD_NOTEOFF){	
				SendNoteOff(cxLeftBackwardNote);
			} else if (inInt == CAXIXI_LEFT_HIT_NOTEON){	
				SendNoteOn(cxLeftHitNote);
			} else if (inInt == CAXIXI_LEFT_HIT_NOTEOFF){	
				SendNoteOff(cxLeftHitNote);
			}
			// Get ready for the next time
			started = false;
			ended = false;

			index = 0;
			inData[index] = '\0';
		}
	}
}

void bubbleSort(Buffer a[], int bufferI) {//Optimizamos esto reemplazando size por i?
	for(int k=0; k<=bufferI; k++) {
		for(int o=0; o<(bufferI-(k+1)); o++) {
			if(a[o].time > a[o+1].time) {
				Buffer t = a[o];
				a[o] = a[o+1];
				a[o+1] = t;
			}
		}
	}
}

void SendNoteOn(int note)
{
	if(record){
		FirstNote = true;
		setT0();
		time = millis()-(t0 + t1 + reset);  //Esto es para que a la primera nota la guarde con time == 0
		Buffer sample = {note, layer, 1, time};
		samples[bufferI] = sample;
	}
	MIDI.sendNoteOn(note,127,MIDI_CHANNEL);
	if(record){
		bufferI++;  
	}
}

void SendNoteOff(int note)
{
	if(record){
		Buffer sample = {note, layer, 0, time};
		samples[bufferI] = sample;
	}
	MIDI.sendNoteOff(note,127,MIDI_CHANNEL);
	if(record){
		bufferI++;  
	}
}

void initSamplerBuffer(){//Esta funcion asigna time=9999999 a todos los valores del buffer
	for(int k=0; k<SAMPLER_BUFFER_SIZE; k++){
		samples[k] = Buffer_default;
	}
}

void setReset(){
	int a = t1 - time;
	if(t0 != 0 && t1!=0 && a<=0){
		bubbleSort(samples,bufferI);
		reset = millis()-t0-t1;
		bufferJ = 0;
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

void PlayBuffer() {
	if(play){
		if(samples[bufferJ].time-time<0 && samples[bufferJ].time-time>-100){//Esto es un hardcodeo (el -100), corregirlo mirando el reloj en la V2, o dejarlo...
			if(samples[bufferJ].encendido){
				MIDI.sendNoteOn(samples[bufferJ].note,127,MIDI_CHANNEL);
			}else{
				MIDI.sendNoteOff(samples[bufferJ].note,127,MIDI_CHANNEL);
			}
			bufferJ++; //J se reinicia cuando da la vuelta, en la fc setReset 
		}
	}
}

void ButtonRecord() {
	buttonState = digitalRead(SAMPLER_BUTTON_RECORD_PIN);
	if (buttonState != lastButtonState) {// if the state has changed, increment the counter
		if (buttonState == HIGH) {// if the current state is HIGH then the button wend from off to on:
			buttonPushCounter++;
			if (record==true){
				record=false;
				isSetOctavador=false;
				setT1();
			} else {
				record=true;
				layer = layer + 1;
			}
		}
		lastButtonState = buttonState; // save the current state as the last state, for next time through the loop
	}
}

void Undo(Buffer a[], int i, int layer){//Hay que usar la variable global i para que solamente recorra las notas que están "escritas". Lo mismo hay que hacer en el bubblesort
                                       //Faltaría, además, que cuando son eliminadas algunas notas, se resten a i. Es decir, que i siempre de cuenta de la cantidad de notas en el buffer.
                                       //Aunque eso implicaría llevar una cuenta de las notas que elimino.
  for(int h=0; h<=bufferI; h++) {
    if(a[h].layer == layer){
      a[h] = Buffer_default;
    }
  }
}

void ButtonUndo()
{
  buttonUndoState = digitalRead(SAMPLER_BUTTON_UNDO_PIN);
  if (buttonUndoState != lastButtonUndoState) {// if the state has changed, increment the counter
    if (buttonUndoState == HIGH) {// if the current state is HIGH then the button wend from off to on:
      if(layer>1){//Esto es para que nunca borre la capa original
          Undo(samples,bufferI,layer);
          if(layer>2){//Esto es para que no agregue notas a la capa original
          layer = layer-1;
          }
      }
    }
  lastButtonUndoState = buttonUndoState; // save the current state as the last state, for next time through the loop
  }
}

void Lights(){
	// turns on the LED every two button pushes by checking the modulo of the button push counter.
	if (buttonPushCounter % 2 == 1) {
		digitalWrite(OCTAVE_LED_PIN, HIGH);
	} else {
		digitalWrite(OCTAVE_LED_PIN, LOW);
	}
}

void Octavador(){
	UpState = digitalRead(OCTAVE_UP_BUTTON_PIN);
	DownState = digitalRead(OCTAVE_DOWN_BUTTON_PIN);
	if (UpState != lastUpState) {
		if (UpState == HIGH){
			SendNoteOff(cxRightForwardNote);
			SendNoteOff(cxRightBackwardNote);
			SendNoteOff(cxRightHitNote);
			SendNoteOff(cxLeftForwardNote);
			SendNoteOff(cxLeftBackwardNote);
			SendNoteOff(cxLeftHitNote);
			
			cxRightForwardNote = cxRightForwardNote+12;
			cxRightBackwardNote = cxRightBackwardNote+12;
			cxRightHitNote = cxRightHitNote+12;
			cxLeftForwardNote = cxLeftForwardNote+12;
			cxLeftBackwardNote = cxLeftBackwardNote+12;
			cxLeftHitNote = cxLeftHitNote+12;
			CurrentOctave = CurrentOctave+1; 
			digitalWrite(OCTAVE_LED_PIN, HIGH);  
		} else {
			// turn LED off:
			digitalWrite(OCTAVE_LED_PIN, LOW);
		}
		lastUpState = UpState;
	}
  if (DownState != lastDownState){     
		if (DownState == HIGH) {    
			SendNoteOff(cxRightForwardNote);
			SendNoteOff(cxRightBackwardNote);
			SendNoteOff(cxRightHitNote);
			SendNoteOff(cxLeftForwardNote);
			SendNoteOff(cxLeftBackwardNote);
			SendNoteOff(cxLeftHitNote);
			
			cxRightForwardNote = cxRightForwardNote-12;
			cxRightBackwardNote = cxRightBackwardNote-12;
			cxRightHitNote = cxRightHitNote-12;
			cxLeftForwardNote = cxLeftForwardNote-12;
			cxLeftBackwardNote = cxLeftBackwardNote-12;
			cxLeftHitNote = cxLeftHitNote-12;
			CurrentOctave = CurrentOctave-1;
			digitalWrite(OCTAVE_LED_PIN, HIGH);  
		} else {
			// turn LED off:
			digitalWrite(OCTAVE_LED_PIN, LOW);
		}
		lastDownState = DownState;
	}
	showLeds();
}

void showLeds(){
switch(CurrentOctave){
    case 0: analogWrite(OCTAVE_UP_LED_RED_PIN,255);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
   
    break;

    case 1: analogWrite(OCTAVE_UP_LED_RED_PIN,0);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,0);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
   
    break;

    case 2: analogWrite(OCTAVE_UP_LED_RED_PIN,0);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,75);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
   
    break;

    case 3: analogWrite(OCTAVE_UP_LED_RED_PIN,0);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
   
    break;

    
    case -1: analogWrite(OCTAVE_UP_LED_RED_PIN,255);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,0);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,0);
   
    break;

    case -2: analogWrite(OCTAVE_UP_LED_RED_PIN,255);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,0);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,75);
   
    break;

    case -3: analogWrite(OCTAVE_UP_LED_RED_PIN,255);
              analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
              analogWrite(OCTAVE_DOWN_LED_RED_PIN,0);
              analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
   
    break;
}  
}

void TurnOffAll(){//Apaga todos los leds
	analogWrite(OCTAVE_UP_LED_RED_PIN,255);
	analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
	analogWrite(OCTAVE_UP_LED_BLUE_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
	analogWrite(OCTAVE_DOWN_LED_BLUE_PIN,255);
}










