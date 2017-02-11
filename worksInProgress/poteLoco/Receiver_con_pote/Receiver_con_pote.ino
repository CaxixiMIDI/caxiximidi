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
int currentOctave = 0;
//////////////////////////////////

////////// SAMPLER ///////////////
Buffer samples[SAMPLER_BUFFER_SIZE]; //puede almacenar hasta 50 notas (50 on, 50 off)
int bufferI=0; //indice del buffer para record
int bufferJ=0; //indice del buffer para play
//int fraccionPote = 0; //AGREGADO sin usar porque no devuelve numero decimal
//int pot1 = 1; // Analog A2 AGREGADO
// BUTTON RECORD const & var:
int buttonPushCounter = 0;        // counter for the number of button presses

int layer = 0;

bool record = false;
bool play = false;
bool isSetT0 = false;
bool isSetT1 = false;
bool FirstNote = false;

long time = 0;    //time = millis() cuenta los milisegundos desde q arranca
long t0 = 0;    //cuando record==True, ajustamos una variable "t0" a esos msec y t = millis()-t0
long t1 = 0;    //cuando record == False, ajustamos una variable "t1", t1 = millis-t0
long reset = 0;   //y ajustamos la variable reset = millis - t0 - t1, que en principio va a ser cero, pero en cada vuelta va a ir incrementándose en el tamaño del loop
///////////////////////////////////////////////


void setup()
{
  //pinMode(pot1, INPUT); //AGREGADO
  pinMode(RECORD_LED_PIN, OUTPUT);// initialize the LED as an output:
  pinMode(OCTAVE_UP_BUTTON_PIN, INPUT);// initialize the button pin as a input:
  pinMode(OCTAVE_DOWN_BUTTON_PIN, INPUT);
  pinMode(SAMPLER_BUTTON_RECORD_PIN, INPUT);
  pinMode(SAMPLER_BUTTON_CLEAR_PIN, INPUT);
  pinMode(OCTAVE_UP_LED_RED_PIN, OUTPUT);
  pinMode(OCTAVE_UP_LED_GREEN_PIN, OUTPUT);
  pinMode(OCTAVE_DOWN_LED_RED_PIN, OUTPUT);
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
  //time = millis()-(t0 + t1 + reset);
  time = millis() - reset;///AGREGADO
  setReset();
  PlayBuffer();
  showLeds();
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
    if (inInt == CAXIXI_SAMPLER_CLEAR){
      Clear();
    } else if (inInt == CAXIXI_RECORD_START){
      RecordStart();
      } else if (inInt == CAXIXI_RECORD_STOP){
      RecordStop();
      } else if (inInt == CAXIXI_OCTAVE_UP){
      OctaveUp();
      } else if (inInt == CAXIXI_OCTAVE_DOWN){
      OctaveDown();
      } else if (inInt == CAXIXI_RIGHT_FORWARD_NOTEON){
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
  note = note + (currentOctave * 12);
  if(record){
    FirstNote = true;
    setT0();
    //time = millis()-(t0 + t1 + reset);  //Esto es para que a la primera nota la guarde con time == 0
    time = millis() - reset;///AGREGADO
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
  note = note + (currentOctave * 12);
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
    //reset = millis()-t0-t1;
    reset = millis();///AGREGADO
    bufferJ = 0;
  }
}

void setT0()//A esta fc la llama ButtonRecord()
{
  if (record && !isSetT0 && FirstNote){
    t0 = millis();
    reset= millis();///AGREGADO
  }
  if (record && !isSetT0&& FirstNote){
    isSetT0 = true;
  }
}

void setT1(){
  if(!record && !isSetT1 && t0 != 0){
    t1 = millis() - t0;
    isSetT1 = true;
    reset= millis();///AGREGADO
    play = true;
  }
}

void PlayBuffer() {
  //double coef;
  //coef = 100;
  if(play){
    if(samples[bufferJ].time-time<0 && samples[bufferJ].time-time>-100){//Esto es un hardcodeo (el -100), corregirlo mirando el reloj en la V2, o dejarlo...
      if(samples[bufferJ].encendido){
        MIDI.sendNoteOn(samples[bufferJ].note,127,MIDI_CHANNEL);
      }else{
        MIDI.sendNoteOff(samples[bufferJ].note,127,MIDI_CHANNEL);
      }
      bufferJ++; //J se reinicia cuando da la vuelta, en la fc setReset
      /*if(samples[bufferJ].time*coef/100-time<0 && samples[bufferJ].time*coef/100-time>-100){//Esto es un hardcodeo (el -100), corregirlo mirando el reloj en la V2, o dejarlo..
      if(samples[bufferJ].encendido){
        MIDI.sendNoteOn(samples[bufferJ].note,127,MIDI_CHANNEL);
      }else{
        MIDI.sendNoteOff(samples[bufferJ].note,127,MIDI_CHANNEL);
      }
      bufferJ++; //J se reinicia cuando da la vuelta, en la fc setReset//AGREGADO  */
    }
  }
}

void RecordStart() {
  digitalWrite(RECORD_LED_PIN, HIGH);
  // This will trigger setT0 on first Note
  record=true;
  layer = layer + 1;
}

void RecordStop() {
  digitalWrite(RECORD_LED_PIN, LOW);
  record=false;
  setT1();
}

void Clear(){
  record = false;
  play = false;
  isSetT0 = false;
  isSetT1 = false;
  FirstNote = false;
  
  t0 = 0;            //cuando record==True, ajustamos una variable "t0" a esos msec y t = millis()-t0
  t1 = 0;            //cuando record == False, ajustamos una variable "t1", t1 = millis-t0
  reset = 0;        //y ajustamos la variable reset = millis - t0 - t1, que en principio va a ser cero, pero en cada vuelta va
  
  Clear_Buffer(samples,bufferI);
  bufferI=0; //indice del buffer para record
  bufferJ=0; //indice del buffer para play
}


void Clear_Buffer(Buffer a[], int bufferI) {
    for(int k=0; k<=bufferI; k++) {
        for(int o=0; o<(bufferI-(k+1)); o++) {
                    samples[k] = Buffer_default;
        }
    }
}

void OctaveUp()
{
  SendNoteOff(cxRightForwardNote);
  SendNoteOff(cxRightBackwardNote);
  SendNoteOff(cxRightHitNote);
  SendNoteOff(cxLeftForwardNote);
  SendNoteOff(cxLeftBackwardNote);
  SendNoteOff(cxLeftHitNote);
  currentOctave = currentOctave + 1;
  if (currentOctave>6) {
    currentOctave = -3;
  }
}

void OctaveDown()
{
  SendNoteOff(cxRightForwardNote);
  SendNoteOff(cxRightBackwardNote);
  SendNoteOff(cxRightHitNote);
  SendNoteOff(cxLeftForwardNote);
  SendNoteOff(cxLeftBackwardNote);
  SendNoteOff(cxLeftHitNote);
  currentOctave = currentOctave - 1;
  if (currentOctave<-3) {
    currentOctave = 7;
  }
}

void showLeds(){
switch(currentOctave){
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
  analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
  analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
}
