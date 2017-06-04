CxCircularBuffer accelXBuffer(BUFFER_SIZE);
CxCircularBuffer accelYBuffer(BUFFER_SIZE);
boolean bufferReady = false;

int accelXSmooth[filterSamples];
int accelYSmooth[filterSamples];
int smoothAccelX;
int smoothAccelY;

int SensorRead[6] = {0, 0, 0, 0, 0, 0};

int NoteRelease[3] = {
  NOTE_RELEASE_FORWARD,
  NOTE_RELEASE_BACKWARD,
  NOTE_RELEASE_HIT
};

int noteThresholdHit = NOTE_THRESHOLD_HIT;
int canHitDefinition = 1;

int noteOn = NOTE_OFF;

float v[6];
float angles[3];
FreeIMU my3IMU = FreeIMU();

boolean isUpThreshold, isDownThreshold, isUpThresholdRotated, isDownThresholdRotated;
boolean canHit, canHitRotated;

int currentAccelX, currentAccelY;

int slopeStill;
int accelXForce;
int state = STATE_STILL;
int prevState;

/**
 * 1. BUTTONS
 */

// 1.1 Sampler
bool record = false;
int recordButtonState = 0;    // current state of the button
int lastRecordButtonState = 0;  // previous state of the button
int clearButtonState = 0;
int lastClearButtonState = 0;



// 1.2 Octavator
int octaveUpButtonLastState  = 0;
int octaveDownButtonLastState  = 0;
long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = LOW;
unsigned long firstTime; // how long since the button was first pressed
int longPressThreshold = 2000; //when a button press is considered long in millis

int lastDownState = 0;
int octaveUpButtonState = 0;  // variable for reading the pushbutton status
int octaveDownButtonState = 0;
int currentOctave = 0;
#include <avr/power.h> // For proMini Boards
int powerpin = 5;


// 1.3 CCMessages
bool wantCCM = false;
bool CCMbufferReady = false;
CxCircularBuffer GyroXBuffer(BUFFER_SIZE);
CxCircularBuffer GyroYBuffer(BUFFER_SIZE);
CxCircularBuffer GyroZBuffer(BUFFER_SIZE);
int GyroXSmooth[filterSamples];
int smoothGyroX;
int GyroYSmooth[filterSamples];
int smoothGyroY;
int GyroZSmooth[filterSamples];
int smoothGyroZ;
bool NoteState = true;
int NoteThresholdCCM = 2000;//Umbral para mandar midiOn/Off
int NoteReleaseCCM = 1000;
bool Bloqueo = false;
int msg; //msg to send

boolean isRollingX;
String debugRollingX;
int isRollingXVariation = 1;
boolean isRollingY;
String debugRollingY;
int isRollingYVariation = 1;
boolean isRollingZ;
String debugRollingZ;
int isRollingZVariation = 1;