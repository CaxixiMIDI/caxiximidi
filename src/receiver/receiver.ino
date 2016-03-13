
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

#include "CaxixiConfig.h"
#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

boolean midiMode = false;

char inData[10];
int index;
boolean started = false;
boolean ended = false;
int inInt;

void setup()
{
	MIDI.begin(1);
	XBee.begin(9600);
	Serial.begin(9600);
}

void loop()
{
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
				MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_FORWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_RIGHT_FORWARD_NOTEOFF){
				MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_FORWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_RIGHT_BACKWARD_NOTEON){
				MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_BACKWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_RIGHT_BACKWARD_NOTEOFF){	
				MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_BACKWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_RIGHT_HIT_NOTEON){	
				MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_HIT,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_RIGHT_HIT_NOTEOFF){	
				MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_HIT,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_FORWARD_NOTEON){
				MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_FORWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_FORWARD_NOTEOFF){
				MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_FORWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_BACKWARD_NOTEON){	
				MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_BACKWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_BACKWARD_NOTEOFF){	
				MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_BACKWARD,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_HIT_NOTEON){	
				MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_HIT,127,MIDI_CHANNEL);
			} else if (inInt == CAXIXI_LEFT_HIT_NOTEOFF){	
				MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_HIT,127,MIDI_CHANNEL);
			}
			// Get ready for the next time
			started = false;
			ended = false;

			index = 0;
			inData[index] = '\0';
		}
	}
}


/*


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

#include <SoftwareSerial.h>

#include <CaxixiConfig.h>

SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

String serialMessage = "";
boolean messageComplete = false;

void setup()
{
	XBee.begin(57600);
	serialMessage.reserve(10);
}

void loop()
{
	serialEvent();
	if(messageComplete){
		if(serialMessage == CAXIXI_RIGHT_FORWARD_NOTEON){
			MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_FORWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_RIGHT_FORWARD_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_FORWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_RIGHT_BACKWARD_NOTEON){
			MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_BACKWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_RIGHT_BACKWARD_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_BACKWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_RIGHT_HIT_NOTEON){
			MIDI.sendNoteOn(CAXIXI_RIGHT_NOTE_HIT,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_RIGHT_HIT_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_RIGHT_NOTE_HIT,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_FORWARD_NOTEON){
			MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_FORWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_FORWARD_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_FORWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_BACKWARD_NOTEON){
			MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_BACKWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_BACKWARD_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_BACKWARD,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_HIT_NOTEON){
			MIDI.sendNoteOn(CAXIXI_LEFT_NOTE_HIT,127,MIDI_CHANNEL);
		}else if(serialMessage == CAXIXI_LEFT_HIT_NOTEOFF){
			MIDI.sendNoteOff(CAXIXI_LEFT_NOTE_HIT,127,MIDI_CHANNEL);
		}
		serialMessage = "";
		messageComplete = false;
	}
}
	
void serialEvent() {
	while (XBee.available()) {
		char msg = (char)XBee.read();
		if (msg == '\n') {
			messageComplete = true;
		}else{
			serialMessage += msg; 
		}
	}
}


*/






