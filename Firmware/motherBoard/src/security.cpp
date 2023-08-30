/*
  MIT License

  Copyright (c) 2022 Medical Open World, Pablo Sánchez Bergasa

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/
#include <Arduino.h>

#include "main.h"

extern TwoWire *wire;
extern MAM_in3ator_Humidifier in3_hum;
extern Adafruit_ILI9341 tft;
extern SHTC3 mySHTC3;  // Declare an instance of the SHTC3 class
extern RotaryEncoder encoder;

extern bool WIFI_EN;
extern long lastDebugUpdate;
extern long loopCounts;
extern int page;

extern double errorTemperature[SENSOR_TEMP_QTY], temperatureCalibrationPoint;
extern double ReferenceTemperatureRange, ReferenceTemperatureLow;
extern double provisionalReferenceTemperatureLow;
extern double fineTuneSkinTemperature;
extern double RawTemperatureLow[SENSOR_TEMP_QTY],
    RawTemperatureRange[SENSOR_TEMP_QTY];
extern double provisionalRawTemperatureLow[SENSOR_TEMP_QTY];
extern int temperature_array_pos;  // temperature sensor number turn to measure
extern float diffSkinTemperature,
    diffAirTemperature;  // difference between measured temperature and user
                         // input real temperature
extern bool humidifierState, humidifierStateChange;
extern int previousHumidity;  // previous sampled humidity
extern float diffHumidity;    // difference between measured humidity and user
                              // input real humidity

extern byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and
// they have different check rates
extern byte encoderRate;
extern byte encoderCount;
extern bool encPulseDetected;
extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;
extern bool digitalCurrentSensorPresent[2];

// room variables;         // desired temperature in heater
extern const float minDesiredTemp[2];  // minimum allowed temperature to be set
extern const float maxDesiredTemp[2];  // maximum allowed temperature to be set
extern const int presetTemp[2];        // preset baby skin temperature

extern boolean A_set;
extern boolean B_set;
extern int encoderpinA;                  // pin  encoder A
extern int encoderpinB;                  // pin  encoder B
extern bool encPulsed, encPulsedBefore;  // encoder switch status
extern bool updateUIData;
extern volatile int EncMove;      // moved encoder
extern volatile int lastEncMove;  // moved last encoder
extern volatile int
    EncMoveOrientation;             // set to -1 to increase values clockwise
extern int last_encoder_move;       // moved encoder
extern long encoder_debounce_time;  // in milliseconds, debounce time in encoder
                                    // to filter signal bounces
extern long last_encPulsed;         // last time encoder was pulsed

// Text Graphic position variables
extern int humidityX;
extern int humidityY;
extern int temperatureX;
extern int temperatureY;
extern int ypos;
extern bool print_text;
extern int initialSensorPosition;
extern bool pos_text[8];

extern bool enableSet;
extern float temperaturePercentage, temperatureAtStart;
extern float humidityPercentage, humidityAtStart;
extern int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX,
    humBarPosY;
extern int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
extern bool autoLock;  // setting that enables backlight switch OFF after a
                       // given time of no user actions
extern long
    lastbacklightHandler;  // last time there was a encoder movement or pulse
extern long sensorsUpdatePeriod;

extern bool selected;
extern char cstring[128];
extern char *textToWrite;
extern char *words[12];
extern char *helpMessage;
extern byte bar_pos;
extern byte menu_rows;
extern byte length;
extern long lastGraphicSensorsUpdate;
extern long lastSensorsUpdate;
extern bool enableSetProcess;
extern long blinking;
extern bool state_blink;
extern bool blinkSetMessageState;
extern long lastBlinkSetMessage;

extern long lastSuccesfullSensorUpdate[SENSOR_TEMP_QTY];

extern double HeaterPIDOutput;
extern double skinControlPIDInput;
extern double airControlPIDInput;
extern double humidityControlPIDOutput;
extern int humidifierTimeCycle;
extern unsigned long windowStartTime;

extern PID airControlPID;
extern PID skinControlPID;
extern PID humidityControlPID;

#define TEMPERATURE_ERROR 1  // 1 degrees difference to trigger alarm
#define HUMIDITY_ERROR 12    // 12 %RH to trigger alarm

#define TEMPERATURE_ERROR_HYSTERESIS \
  0.05                               // 0.05 degrees difference to disable alarm
#define HUMIDITY_ERROR_HYSTERESIS 5  // 5 %RH to disable alarm

#define FAN_TEST_CURRENTDIF_MIN \
  0.2  // when the fan is spinning, heater cools down and consume less current
#define FAN_TEST_PREHEAT_TIME \
  30000  // when the fan is spinning, heater cools down and consume less current

#define ALARM_TIME_DELAY 30  // in mins, time to check alarm
// security config
#define AIR_THERMAL_CUTOUT 38
#define SKIN_THERMAL_CUTOUT 40
#define AIR_THERMAL_CUTOUT_HYSTERESIS 1
#define SKIN_THERMAL_CUTOUT_HYSTERESIS 1
#define enableAlarms true

#define MINIMUM_SUCCESSFULL_SENSOR_UPDATE 30000  // in millis

bool alarmOnGoing[NUM_ALARMS];
long lastAlarmTrigger[NUM_ALARMS];
float alarmSensedValue;

extern in3ator_parameters in3;

void initAlarms() {
  lastAlarmTrigger[AIR_THERMAL_CUTOUT_ALARM] =
      -1 * minsToMillis(ALARM_TIME_DELAY);
  lastAlarmTrigger[SKIN_THERMAL_CUTOUT_ALARM] =
      -1 * minsToMillis(ALARM_TIME_DELAY);
}

bool evaluateAlarm(byte alarmID, float setPoint, float measuredValue,
                   float errorMargin, float hysteresisValue, long alarmTime) {
  if (millis() - alarmTime > minsToMillis(ALARM_TIME_DELAY) ||
      alarmOnGoing[alarmID]) {  // min to millis
    if (errorMargin) {
      if ((abs(setPoint - measuredValue) + hysteresisValue) > errorMargin) {
        if (!alarmOnGoing[alarmID]) {
          setAlarm(alarmID);
          return true;
        }
      } else {
        if (alarmOnGoing[alarmID]) {
          resetAlarm(alarmID);
        }
      }
    } else {
      if ((measuredValue + hysteresisValue) > setPoint) {
        if (!alarmOnGoing[alarmID]) {
          setAlarm(alarmID);
          return true;
        }
      } else {
        if (alarmOnGoing[alarmID]) {
          resetAlarm(alarmID);
        }
      }
    }
  }
  return false;
}

void checkThermalCutOuts() {
  evaluateAlarm(AIR_THERMAL_CUTOUT_ALARM, AIR_THERMAL_CUTOUT,
                in3.temperature[ROOM_DIGITAL_TEMP_SENSOR], false,
                AIR_THERMAL_CUTOUT_HYSTERESIS,
                lastAlarmTrigger[AIR_THERMAL_CUTOUT_ALARM]);
  evaluateAlarm(SKIN_THERMAL_CUTOUT_ALARM, SKIN_THERMAL_CUTOUT,
                in3.temperature[SKIN_SENSOR], false,
                SKIN_THERMAL_CUTOUT_HYSTERESIS,
                lastAlarmTrigger[SKIN_THERMAL_CUTOUT_ALARM]);
}

void checkStatusOfSensor(byte sensor) {
  byte alarmID = false;
  switch (sensor) {
    case ROOM_DIGITAL_TEMP_SENSOR:
      alarmID = AIR_SENSOR_ISSUE_ALARM;
      break;
    case SKIN_SENSOR:
      alarmID = SKIN_SENSOR_ISSUE_ALARM;
      break;
  }
  if (alarmID) {
    if (millis() - lastSuccesfullSensorUpdate[sensor] >
        MINIMUM_SUCCESSFULL_SENSOR_UPDATE) {
      if (!alarmOnGoing[alarmID]) {
        setAlarm(alarmID);
      }
    } else {
      if (alarmOnGoing[alarmID]) {
        resetAlarm(alarmID);
      }
    }
  }
}

void powerFailureAlarm() {}

void sensorHealthMonitor() {
  checkStatusOfSensor(ROOM_DIGITAL_TEMP_SENSOR);
  checkStatusOfSensor(SKIN_SENSOR);
}

void powerMonitor() {
  currentMonitor();
  voltageMonitor();
#if (HW_NUM >= 10)
  if (digitalCurrentSensorPresent &&
      in3.system_voltage < MINIMUM_SYSTEM_VALUE &&
      GPIORead(ON_OFF_SWITCH) == OFF) {
    powerFailureAlarm();
  }
#endif
}

void alarmTimerStart() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    lastAlarmTrigger[i] = millis();
  }
  lastAlarmTrigger[AIR_THERMAL_CUTOUT_ALARM] =
      -1 * minsToMillis(ALARM_TIME_DELAY);
  lastAlarmTrigger[SKIN_THERMAL_CUTOUT_ALARM] =
      -1 * minsToMillis(ALARM_TIME_DELAY);
}

byte activeAlarm() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    if (alarmOnGoing[i]) {
      return (i);
    }
  }
  return false;
}

bool ongoingAlarms() {
  return (alarmOnGoing[TEMPERATURE_ALARM] || alarmOnGoing[HUMIDITY_ALARM] ||
          alarmOnGoing[AIR_THERMAL_CUTOUT_ALARM] ||
          alarmOnGoing[SKIN_THERMAL_CUTOUT_ALARM] ||
          alarmOnGoing[AIR_SENSOR_ISSUE_ALARM] ||
          alarmOnGoing[SKIN_SENSOR_ISSUE_ALARM] ||
          alarmOnGoing[HEATER_ISSUE_ALARM] || alarmOnGoing[FAN_ISSUE_ALARM]);
}

bool ongoingCriticalAlarm() {
  return (!(alarmOnGoing[AIR_THERMAL_CUTOUT_ALARM] ||
            alarmOnGoing[SKIN_THERMAL_CUTOUT_ALARM] ||
            alarmOnGoing[AIR_SENSOR_ISSUE_ALARM] ||
            alarmOnGoing[SKIN_SENSOR_ISSUE_ALARM] ||
            alarmOnGoing[HEATER_ISSUE_ALARM]));
  // return (true);
}

char *alarmIDtoString(byte alarmID) {
  switch (alarmID) {
    case AIR_THERMAL_CUTOUT_ALARM:
    case SKIN_THERMAL_CUTOUT_ALARM:
      return convertStringToChar(cstring, "THERMAL CUTOUT ALARM");
      break;
    case TEMPERATURE_ALARM:
      return convertStringToChar(cstring, "TEMPERATURE ALARM");
      break;
    case HUMIDITY_ALARM:
      return convertStringToChar(cstring, "HUMIDITY ALARM");
      break;
    case AIR_SENSOR_ISSUE_ALARM:
      return convertStringToChar(cstring, "AIR SENSOR ALARM");
      break;
    case SKIN_SENSOR_ISSUE_ALARM:
      return convertStringToChar(cstring, "SKIN SENSOR ALARM");
      break;
    case FAN_ISSUE_ALARM:
      return convertStringToChar(cstring, "FAN ALARM");
      break;
    case HEATER_ISSUE_ALARM:
      return convertStringToChar(cstring, "HEATER ALARM");
      break;
    default:
      return convertStringToChar(cstring, "ALARM");
      break;
  }
}

void setAlarm(byte alarmID) {
  log("[ALARM] ->" + String(alarmIDtoString(alarmID)) + " has been triggered");
  alarmOnGoing[alarmID] = true;
  buzzerConstantTone(buzzerAlarmTone);
  drawAlarmMessage(alarmIDtoString(alarmID));
}

void resetAlarm(byte alarmID) {
  log("[ALARM] ->" + String(alarmIDtoString(alarmID)) + " has been disable");
  alarmOnGoing[alarmID] = false;
  drawHeading(page, in3.serialNumber);
  if (!ongoingAlarms()) {
    shutBuzzer();
  } else {
    drawAlarmMessage(alarmIDtoString(activeAlarm()));
  }
}

void disableAllAlarms() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    if (alarmOnGoing[i]) {
      if (millis() - lastAlarmTrigger[i] > minsToMillis(ALARM_TIME_DELAY)) {
        shutBuzzer();
      }
      lastAlarmTrigger[i] = millis();
    }
  }
}

void checkAlarms() {
  if (page == actuatorsProgressPage) {
    if (in3.temperatureControl) {
      if (in3.controlMode) {
        alarmSensedValue = in3.temperature[ROOM_DIGITAL_TEMP_SENSOR];
      } else {
        alarmSensedValue = in3.temperature[SKIN_SENSOR];
      }
      evaluateAlarm(TEMPERATURE_ALARM, in3.desiredControlTemperature,
                    alarmSensedValue, TEMPERATURE_ERROR,
                    TEMPERATURE_ERROR_HYSTERESIS,
                    lastAlarmTrigger[TEMPERATURE_ALARM]);
    }
    if (in3.humidityControl) {
      evaluateAlarm(HUMIDITY_ALARM, in3.humidity[ROOM_DIGITAL_HUM_SENSOR],
                    in3.desiredControlHumidity, HUMIDITY_ERROR,
                    HUMIDITY_ERROR_HYSTERESIS,
                    lastAlarmTrigger[HUMIDITY_ALARM]);
    }
  }
}

void securityCheck() {
  checkThermalCutOuts();
  checkAlarms();
  sensorHealthMonitor();
}