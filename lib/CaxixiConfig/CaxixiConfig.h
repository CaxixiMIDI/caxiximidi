const int MIDI_CHANNEL = 1;

const int NOTE_FORWARD = 0;
const int NOTE_BACKWARD = 1;
const int NOTE_HIT = 2;
const int NOTE_OFF = 5;

// SENSOR_BEGIN
const int SENSOR_ACCEL_X = 0;
const int SENSOR_ACCEL_Y = 1;
const int SENSOR_ACCEL_Z = 2;

// SmoothFilter Samples
const int filterSamples = 5;
const int BUFFER_SIZE = 8;

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

const int CAXIXI_RIGHT_NOTE_FORWARD = 43;
const int CAXIXI_RIGHT_NOTE_BACKWARD = 45;
const int CAXIXI_RIGHT_NOTE_HIT = 47;
const int CAXIXI_LEFT_NOTE_FORWARD = 47;
const int CAXIXI_LEFT_NOTE_BACKWARD = 47;
const int CAXIXI_LEFT_NOTE_HIT = 47;

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