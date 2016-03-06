
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

#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

boolean midiMode = false;

void setup()
{
	XBee.begin(9600);
	Serial.begin(9600);
}

void loop()
{
	if (XBee.available()){
		Serial.println(XBee.read());
		//MIDI.sendNoteOn(XBee.read(),127,1);
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