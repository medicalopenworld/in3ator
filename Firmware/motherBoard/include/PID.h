#include "main.h"
#include <PID_v1.h> //

// PID VARIABLES
#define skinPID 0
#define airPID 1
#define humidityPID 2
#define numPID 3

#define PID_TEMPERATURE_SAMPLE_TIME 4000
#define PID_HUMIDITY_SAMPLE_TIME 200

#define KP_AIR 200
#define KI_AIR 1
#define KD_AIR 500
#define AWO_SKIN 2

#define KP_SKIN 200
#define KI_SKIN 1
#define KD_SKIN 500
#define AWO_AIR 2

#define KP_HUMIDITY 200
#define KI_HUMIDITY 2
#define KD_HUMIDITY 20
#define AWO_HUMIDITY 5

