#define MIDI_CHANNEL 1

#define NotaAdelante 0
#define NotaAtras 1
#define NotaAdelanteRotada 2
#define NotaAtrasRotada 3

// SENSOR_BEGIN
#define SENSOR_ACCEL_X 0
#define SENSOR_ACCEL_Y 1
#define SENSOR_ACCEL_Z 2
#define SENSOR_GYRO_Y 3
#define SENSOR_GYRO_X 4
#define SENSOR_GYRO_Z 5

#define BUFFER_SIZE 8

#define RESOLUTION 1

// SmoothFilter Samples
#define filterSamples 11

#define SENSOR_NOTE_ADELANTE 43
#define SENSOR_NOTE_ATRAS 45
#define SENSOR_NOTE_ADELANTE_ROTADA 47
#define SENSOR_NOTE_ATRAS_ROTADA 49

#define NOTE_RELEASE_ADELANTE 0
#define NOTE_RELEASE_ATRAS 0
#define NOTE_RELEASE_ADELANTE_ROTADA 0
#define NOTE_RELEASE_ATRAS_ROTADA -100

#define NOTE_THRESHOLD_ADELANTE 600
#define NOTE_THRESHOLD_ATRAS -400
#define NOTE_THRESHOLD_ADELANTE_ROTADA 550
#define NOTE_THRESHOLD_ATRAS_ROTADA -6000

#define HITS_THRESOLD 20

struct Buffer_s {
  int note;
  bool encendido;// on/off
  long time;
} Buffer_default = {0,0,9999999};

typedef struct Buffer_s Buffer;
