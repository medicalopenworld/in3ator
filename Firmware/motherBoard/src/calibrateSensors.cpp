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
extern SHTC3 mySHTC3; // Declare an instance of the SHTC3 class
extern RotaryEncoder encoder;
extern Beastdevices_INA3221 digitalCurrentSensor;

extern bool WIFI_EN;
extern long lastDebugUpdate;
extern long loopCounts;
extern int page;
extern int temperature_filter; // amount of temperature samples to filter
extern long lastNTCmeasurement[numNTC];

extern double errorTemperature[numSensors], temperatureCalibrationPoint;
extern double ReferenceTemperatureRange, ReferenceTemperatureLow;
extern double provisionalReferenceTemperatureLow;
extern double fineTuneSkinTemperature, fineTuneAirTemperature;
extern double RawTemperatureLow[numSensors], RawTemperatureRange[numSensors];
extern double provisionalRawTemperatureLow[numSensors];
extern double temperatureMax[numSensors], temperatureMin[numSensors];
extern int temperatureArray[numNTC][analog_temperature_filter]; // variable to handle each NTC with the array of last samples (only for NTC)
extern int temperature_array_pos;                               // temperature sensor number turn to measure
extern float diffSkinTemperature, diffAirTemperature;                                   // difference between measured temperature and user input real temperature
extern bool humidifierState, humidifierStateChange;
extern int previousHumidity; // previous sampled humidity
extern float diffHumidity;   // difference between measured humidity and user input real humidity

extern byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and they have different check rates
extern byte encoderRate;
extern byte encoderCount;
extern bool encPulseDetected;
extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;
extern bool digitalCurrentSensorPresent;

extern float instantTemperature[secondOrder_filter];
extern float previousTemperature[secondOrder_filter];

// room variables
extern boolean B_set;
extern int encoderpinA;                 // pin  encoder A
extern int encoderpinB;                 // pin  encoder B
extern bool encPulsed, encPulsedBefore; // encoder switch status
extern bool updateUIData;
extern volatile int EncMove;            // moved encoder
extern volatile int lastEncMove;        // moved last encoder
extern volatile int EncMoveOrientation; // set to -1 to increase values clockwise
extern int last_encoder_move;           // moved encoder
extern long encoder_debounce_time;      // in milliseconds, debounce time in encoder to filter signal bounces
extern long last_encPulsed;             // last time encoder was pulsed

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
extern int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX, humBarPosY;
extern int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
extern bool autoLock;             // setting that enables backlight switch OFF after a given time of no user actions
extern long lastbacklightHandler; // last time there was a encoder movement or pulse
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



extern double HeaterPIDOutput;
extern double skinControlPIDInput;
extern double airControlPIDInput;
extern double humidityControlPIDOutput;
extern int humidifierTimeCycle;
extern unsigned long windowStartTime;

extern PID airControlPID;
extern PID skinControlPID;
extern PID humidityControlPID;

extern in3ator_parameters in3;

#define CALIBRATION_ERROR 0.05
#define TIME_BETWEEN_SAMPLES 1 //minutes
#define SAMPLES_WITHIN_ERROR 3
#define DEFAULT_CALIBRATION_TEMPERATURE 36

void autoCalibration()
{
  bool exitCalibrationMenu = false;
  byte numWords = 1;
  long lastTemperatureMeasurement = millis();
  int historyLengthPosition = false;
  double referenceSensorHistory[SAMPLES_WITHIN_ERROR];
  double sensorToCalibrateHistory[SAMPLES_WITHIN_ERROR];
  referenceSensorHistory[0] = in3.temperature[digitalTempHumSensor];
  sensorToCalibrateHistory[0] = in3.temperature[skinSensor];
  page = autoCalibrationPage;
  print_text = true;
  tft.setTextSize(1);
  setTextColor(COLOR_MENU_TEXT);
  for (int i = false; i < numWords; i++)
  {
    pos_text[i] = LEFT_MARGIN;
  }
  pos_text[setCalibrationGraphicPosition] = CENTER;
  switch (in3.language)
  {
  case english:
    words[autoCalibrationMessageGraphicPosition] = convertStringToChar("Calibrating...");
    break;
  case spanish:
    words[autoCalibrationMessageGraphicPosition] = convertStringToChar("Calibrating...");
    break;
  case french:
    words[autoCalibrationMessageGraphicPosition] = convertStringToChar("Calibrating...");
    break;
  case portuguese:
    words[autoCalibrationMessageGraphicPosition] = convertStringToChar("Calibrating...");
    break;
  }
  menu_rows = numWords;
  graphics(page, in3.language, print_text, menu_rows, false, false);
  drawHeading(page, in3.serialNumber);
  bar_pos = true;
  ypos = graphicHeight(bar_pos - 1);
  while (!GPIORead(ENC_SWITCH))
  {
    updateData();
  }
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
  autoCalibrationProcess = setupAutoCalibrationPoint;
  while (!exitCalibrationMenu)
  {
    updateData();
    switch (autoCalibrationProcess)
    {
    case setupAutoCalibrationPoint:
      Serial.println("=================================================point 0");
      clearCalibrationValues();
      autoCalibrationProcess = firstAutoCalibrationPoint;
      turnFans(ON);
      break;
    case firstAutoCalibrationPoint:
      if (!GPIORead(ENC_SWITCH) || checkStableTemperatures(referenceSensorHistory, sensorToCalibrateHistory, SAMPLES_WITHIN_ERROR, CALIBRATION_ERROR))
      {
        provisionalReferenceTemperatureLow = in3.temperature[digitalTempHumSensor];
        provisionalRawTemperatureLow[skinSensor] = in3.temperature[skinSensor];
        vTaskDelay(debounceTime / portTICK_PERIOD_MS);
        while (!GPIORead(ENC_SWITCH))
        {
          updateData();
          exitCalibrationMenu = back_mode();
        }
        vTaskDelay(debounceTime / portTICK_PERIOD_MS);
        in3.desiredControlTemperature = DEFAULT_CALIBRATION_TEMPERATURE;
        startPID(airPID);
        // ledcWrite(HEATER_PWM_CHANNEL, HEATER_HALF_PWR * ongoingCriticalAlarm());
        autoCalibrationProcess = secondAutoCalibrationPoint;
        referenceSensorHistory[historyLengthPosition] = false;
        sensorToCalibrateHistory[historyLengthPosition] = false;
        Serial.println("=================================================point 1");
      }
      break;
    case secondAutoCalibrationPoint:
      PIDHandler();
      if (!GPIORead(ENC_SWITCH) || checkStableTemperatures(referenceSensorHistory, sensorToCalibrateHistory, SAMPLES_WITHIN_ERROR, CALIBRATION_ERROR))
      {
        Serial.println("=================================================point 2");
        ReferenceTemperatureLow = provisionalReferenceTemperatureLow;
        RawTemperatureLow[skinSensor] = provisionalRawTemperatureLow[skinSensor];
        ReferenceTemperatureRange = in3.temperature[digitalTempHumSensor] - ReferenceTemperatureLow;
        RawTemperatureRange[skinSensor] = (in3.temperature[skinSensor] - RawTemperatureLow[skinSensor]);
        log("calibration factors: " + String(RawTemperatureLow[skinSensor]) + "," + String(RawTemperatureRange[skinSensor]) + "," + String(ReferenceTemperatureRange) + "," + String(ReferenceTemperatureLow));
        saveCalibrationToEEPROM();
        ledcWrite(HEATER_PWM_CHANNEL, false);
        turnFans(OFF);
        exitCalibrationMenu = true;
        stopPID(airPID);
      }
      break;
    }
    if (millis() - lastTemperatureMeasurement > minsToMillis(TIME_BETWEEN_SAMPLES))
    {
      lastTemperatureMeasurement = millis();
      if (historyLengthPosition == SAMPLES_WITHIN_ERROR)
      {
        historyLengthPosition = false;
      }
      referenceSensorHistory[historyLengthPosition] = in3.temperature[digitalTempHumSensor];
      sensorToCalibrateHistory[historyLengthPosition] = in3.temperature[skinSensor];
      historyLengthPosition++;

      for (int i = 0; i < SAMPLES_WITHIN_ERROR; i++)
      {
        Serial.println(abs(*referenceSensorHistory - *(referenceSensorHistory + i)));
        Serial.println(abs(*sensorToCalibrateHistory - *(sensorToCalibrateHistory + i)));
      }
    }
  }
  UI_mainMenu();
}

void fineTuneCalibration()
{
  byte numWords = 2;
  fineTuneSkinTemperature = false;
  fineTuneAirTemperature = false;
  page = fineTuneCalibrationPage;
  print_text = true;
  tft.setTextSize(1);
  setTextColor(COLOR_MENU_TEXT);
  for (int i = false; i < numWords; i++)
  {
    pos_text[i] = LEFT_MARGIN;
  }
  pos_text[setCalibrationGraphicPosition] = CENTER;
  words[temperatureCalibrationGraphicPosition] = convertStringToChar("Temperature adjust");
  words[setCalibrationGraphicPosition] = convertStringToChar("SET");
  menu_rows = numWords;
  graphics(page, in3.language, print_text, menu_rows, false, false);
  drawHeading(page, in3.serialNumber);
  bar_pos = true;
  ypos = graphicHeight(bar_pos - 1);
  setTextColor(COLOR_MENU_TEXT);
  drawFloat(in3.temperature[skinSensor], 1, valuePosition, ypos, textFontSize);
  while (!GPIORead(ENC_SWITCH))
  {
    updateData();
  }
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
}

void firstPointCalibration()
{
  byte numWords = 2;
  page = firstPointCalibrationPage;
  print_text = true;
  tft.setTextSize(1);
  setTextColor(COLOR_MENU_TEXT);
  for (int i = false; i < numWords; i++)
  {
    pos_text[i] = LEFT_MARGIN;
  }
  pos_text[setCalibrationGraphicPosition] = CENTER;
  switch (in3.language)
  {
  case english:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("First point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case spanish:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("First point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case french:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("First point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case portuguese:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("First point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  }
  menu_rows = numWords;
  graphics(page, in3.language, print_text, menu_rows, false, false);
  drawHeading(page, in3.serialNumber);
  bar_pos = true;
  ypos = graphicHeight(bar_pos - 1);
  setTextColor(COLOR_MENU_TEXT);
  drawFloat(in3.temperature[skinSensor], 1, valuePosition, ypos, textFontSize);
  while (!GPIORead(ENC_SWITCH))
  {
    updateData();
  }
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
}

void secondPointCalibration()
{
  byte numWords = 2;
  page = secondPointCalibrationPage;
  print_text = true;
  tft.setTextSize(1);
  setTextColor(COLOR_MENU_TEXT);
  for (int i = false; i < numWords; i++)
  {
    pos_text[i] = LEFT_MARGIN;
  }
  pos_text[setCalibrationGraphicPosition] = CENTER;
  switch (in3.language)
  {
  case english:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("Second point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case spanish:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("Second point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case french:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("Second point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  case portuguese:
    words[temperatureCalibrationGraphicPosition] = convertStringToChar("Second point");
    words[setCalibrationGraphicPosition] = convertStringToChar("SET");
    break;
  }
  menu_rows = numWords;
  graphics(page, in3.language, print_text, menu_rows, false, false);
  drawHeading(page, in3.serialNumber);
  bar_pos = true;
  ypos = graphicHeight(bar_pos - 1);
  setTextColor(COLOR_MENU_TEXT);
  drawFloat(in3.temperature[skinSensor], 1, valuePosition, ypos, textFontSize);
  while (!GPIORead(ENC_SWITCH))
  {
    updateData();
  }
  vTaskDelay(debounceTime / portTICK_PERIOD_MS);
}

bool checkStableTemperatures(double *referenceSensorHistory, double *sensorToCalibrateHistory, int historyLength, double stabilityError)
{
  for (int i = 0; i < historyLength; i++)
  {
    if (abs(*referenceSensorHistory - *(referenceSensorHistory + i)) > stabilityError)
    {
      return false;
    }
    if (abs(*sensorToCalibrateHistory - *(sensorToCalibrateHistory + i)) > stabilityError)
    {
      return false;
    }
  }
  return true;
}

bool checkStableCurrentConsumption(double *referenceSensorHistory, int historyLength, double stabilityError)
{
  for (int i = 0; i < historyLength; i++)
  {
    if (abs(*referenceSensorHistory - *(referenceSensorHistory + i)) > stabilityError)
    {
      return false;
    }
  }
  return true;
}

void clearCalibrationValues()
{
  RawTemperatureLow[skinSensor] = false;
  RawTemperatureRange[skinSensor] = false;
  RawTemperatureLow[digitalTempHumSensor] = false;
  RawTemperatureRange[digitalTempHumSensor] = false;
  ReferenceTemperatureRange = false;
  ReferenceTemperatureLow = false;
}
