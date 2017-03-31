#define MIDI_CHANNEL 1

#define NotaAdelante 0
#define NotaAtras 1
#define NotaRotadaAdelante 2
#define NotaRotadaAtras 3

// SENSOR_BEGIN
#define SENSOR_ACCEL_X 0
#define SENSOR_ACCEL_Y 1
#define SENSOR_ACCEL_Z 2
#define SENSOR_GYRO_Y 3
#define SENSOR_GYRO_X 4
#define SENSOR_GYRO_Z 5

#define BUFFER_SIZE 20

#define RESOLUTION 1

// SmoothFilter Samples
#define filterSamples 9

#define SENSOR_NOTE_ADELANTE 43
#define SENSOR_NOTE_ATRAS 45
#define SENSOR_NOTE_ROTADA_ADELANTE 47
#define SENSOR_NOTE_ROTADA_ATRAS 49

#define NOTE_CUTOFF_ADELANTE 400
#define NOTE_CUTOFF_ATRAS -315
#define NOTE_CUTOFF_ROTADA_ADELANTE 500
#define NOTE_CUTOFF_ROTADA_ATRAS -315

// Cuando ROTADA
#define ROTATED_OFFSET 300
// Valor minimo de (adelante Atras) para permitir un golpe
#define DIRECTION_OFFSET_ADELANTE 100
#define DIRECTION_OFFSET_ATRAS -100
// Hack para que fuera de estos valores siempre se este moviendo
#define MOVEMENT_OFFSET_ADELANTE 300
#define MOVEMENT_OFFSET_ATRAS -300
// Valor que determina cuando hay movimiento
#define MOVING_VARIATION 20

#define NOTE_RELEASE_ADELANTE 100
#define NOTE_RELEASE_ATRAS -100
