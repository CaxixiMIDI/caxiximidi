
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

SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

void setup()
{
	XBee.begin(57600); 
}

void loop()
{
	char msg = XBee.read();
	switch (msg) {
		case "A":
			MIDI.sendNoteOn(,127,midiChannel);
			break;
		case "B":
		
			break;
		case "C":
			default:
		case "D":
			break;
		case "E":
			break;
		case "F":
			break;
	  }
	
	
		
	}