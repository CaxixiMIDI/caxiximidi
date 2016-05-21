const int MIDI_CHANNEL = 1;

const int NOTE_FORWARD = 0;
const int NOTE_BACKWARD = 1;
const int NOTE_HIT = 2;
const int NOTE_OFF = 5;

// OCTAVE
#define OCTAVE_DOWN_BUTTON_PIN 5
#define OCTAVE_UP_BUTTON_PIN 4
#define OCTAVE_UP_LED_RED_PIN 10
#define OCTAVE_UP_LED_BLUE_PIN 12
#define OCTAVE_UP_LED_GREEN_PIN 11
#define OCTAVE_DOWN_LED_RED_PIN 4
#define OCTAVE_DOWN_LED_BLUE_PIN 10
#define OCTAVE_DOWN_LED_GREEN_PIN 5

// SAMPLER
#define RECORD_LED_PIN 13
#define SAMPLER_BUTTON_RECORD_PIN 3
#define SAMPLER_BUTTON_CLEAR_PIN 6
#define SAMPLER_BUFFER_SIZE 100

// SENSOR_BEGIN
const int SENSOR_ACCEL_X = 0;
const int SENSOR_ACCEL_Y = 1;
const int SENSOR_ACCEL_Z = 2;

// SmoothFilter Samples
#define filterSamples 5
#define BUFFER_SIZE 8

const int SLOPE_STILL_X_RANGE = 20;
const int SLOPE_STILL_Y_RANGE = 50;
const int SLOPE_STILL_STATUS = 0;
const int SLOPE_MOVING_STATUS = 1;

const int FORCE_STILL = 0;
const int FORCE_FORWARD = 1;
const int FORCE_BACKWARD = 2;
const int FORCE_THRESHOLD_FORWARD = 400;
const int FORCE_THRESHOLD_BACKWARD = -300;
const int FORCE_STILL_RANGE_FORWARD = 300;
const int FORCE_STILL_RANGE_BACKWARD = -100;

const int STATE_STILL = 0;
const int STATE_FORWARD = 1;
const int STATE_BACKWARD = 2;

const int RESOLUTION = 1;

#define CAXIXI_RIGHT_NOTE_FORWARD 43
#define CAXIXI_RIGHT_NOTE_BACKWARD 45
#define CAXIXI_RIGHT_NOTE_HIT 47
#define CAXIXI_LEFT_NOTE_FORWARD 36
#define CAXIXI_LEFT_NOTE_BACKWARD 38
#define CAXIXI_LEFT_NOTE_HIT 40

const int NOTE_RELEASE_FORWARD = 100;
const int NOTE_RELEASE_BACKWARD = -100;
const int NOTE_RELEASE_HIT = 600;

const int NOTE_THRESHOLD_HIT = 600;

const int CAXIXI_RIGHT_FORWARD_NOTEON = 111;
const int CAXIXI_RIGHT_FORWARD_NOTEOFF = 110;
const int CAXIXI_RIGHT_BACKWARD_NOTEON = 121;
const int CAXIXI_RIGHT_BACKWARD_NOTEOFF = 120;
const int CAXIXI_RIGHT_HIT_NOTEON = 131;
const int CAXIXI_RIGHT_HIT_NOTEOFF = 130;
const int CAXIXI_LEFT_FORWARD_NOTEON = 211;
const int CAXIXI_LEFT_FORWARD_NOTEOFF = 210;
const int CAXIXI_LEFT_BACKWARD_NOTEON = 221;
const int CAXIXI_LEFT_BACKWARD_NOTEOFF = 220;
const int CAXIXI_LEFT_HIT_NOTEON = 231;
const int CAXIXI_LEFT_HIT_NOTEOFF = 230;
const int CAXIXI_OCTAVE_UP = 401;
const int CAXIXI_OCTAVE_DOWN = 400;
const int CAXIXI_RECORD_STOP = 500;
const int CAXIXI_RECORD_START = 501;
const int CAXIXI_SAMPLER_CLEAR = 502;

struct Buffer_s {
  int note;
  int layer;  //capa de sobregrabación. cada vez que se prende "record", suma 1.
  bool encendido;// on/off
  long time;
} Buffer_default = {0,0,0,9999999};

typedef struct Buffer_s Buffer;
