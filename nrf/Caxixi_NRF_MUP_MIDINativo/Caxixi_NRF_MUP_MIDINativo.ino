#include "CaxixiConfig.h"

#include "MIDIUSB.h"
#include <MIDI.h>
#include <midi_Defs.h>
#include <midi_Message.h>
#include <midi_Namespace.h>
#include <midi_Settings.h>
struct MyMidiSettings : public midi::DefaultSettings {
  static const long DefaultSettings::BaudRate = 9600;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial, MIDI, MyMidiSettings);

#include "CxCircularBuffer.h"
#include <SPI.h>
#include "RF24.h"

//**NRF24 Config** Set the Role 0 receiver or 1 sender **/
int roleSET = 0;
bool radioNumber = roleSET;
RF24 radio(9,10);
byte addresses[][6] = {"1Node","2Node"};
bool role = roleSET; // Used to control whether this node is sending or receiving

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
{/*
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
  analogWrite(RECORD_LED_PIN, 255);
  */
  //MIDI.begin(0);
  /*
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */
  delay(100);
  initSamplerBuffer();
  radio.begin();
  // Set the PA Level low to prevent power supply related issues since this is a
  // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    }else{
    radio.openReadingPipe(1,addresses[1]);
    radio.startListening(); 
     }
}

void loop()
{
  /*
  Serial.println("Sending note on");
  noteOn(0, 48, 64);   // Channel 0, middle C, normal velocity
  MidiUSB.flush();
  delay(500);
  Serial.println("Sending note off");
  noteOff(0, 48, 64);  // Channel 0, middle C, normal velocity
  MidiUSB.flush();
  delay(1500);
  */
  time = millis()-(t0 + t1 + reset);
  setReset();
  PlayBuffer();
  showLeds();
  //Prueba
  //SendNoteOn(43);
  //delay(1000);
 
  if( radio.available()){                      // While there is data ready
      radio.read( &inInt, sizeof(int) );             // Get the payload
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
      }   else if (inInt == CAXIXI_LEFT_HIT_NOTEON){  
          SendNoteOn(cxLeftHitNote);
      } else if (inInt == CAXIXI_LEFT_HIT_NOTEOFF){ 
          SendNoteOff(cxLeftHitNote);
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

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
  MidiUSB.flush();
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
  MidiUSB.flush();
}

void SendNoteOn(int note)
{
  note = note + (currentOctave * 12);
  if(record){
    FirstNote = true;
    setT0();
    time = millis()-(t0 + t1 + reset);  //Esto es para que a la primera nota la guarde con time == 0
    Buffer sample = {note, layer, 1, time};
    samples[bufferI] = sample;
  }
  noteOn(MIDI_CHANNEL,note,127);
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
  noteOff(MIDI_CHANNEL,note,127);
  if(record){
    bufferI++;  
  }
}

void initSamplerBuffer(){
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

void RecordStart() {
  digitalWrite(RECORD_LED_PIN, LOW);
  // This will trigger setT0 on first Note
  record=true;
  layer = layer + 1;
}

void RecordStop() {
  digitalWrite(RECORD_LED_PIN, HIGH);
  record=false;
  setT1();
}

void Clear(){
  bool record = false;
  bool play = false;
  bool isSetT0 = false;
  bool isSetT1 = false;
  bool FirstNote = false;
  
  long t0 = 0;            //cuando record==True, ajustamos una variable "t0" a esos msec y t = millis()-t0
  long t1 = 0;            //cuando record == False, ajustamos una variable "t1", t1 = millis-t0
  long reset = 0;        //y ajustamos la variable reset = millis - t0 - t1, que en principio va a ser cero, pero en cada vuelta va
  
  Clear_Buffer(samples,bufferI);
  int bufferI=0; //indice del buffer para record
  int bufferJ=0; //indice del buffer para play
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

void TurnOffAll(){
  analogWrite(OCTAVE_UP_LED_RED_PIN,255);
  analogWrite(OCTAVE_UP_LED_GREEN_PIN,255);
  analogWrite(OCTAVE_DOWN_LED_RED_PIN,255);
  analogWrite(OCTAVE_DOWN_LED_GREEN_PIN,255);
}
