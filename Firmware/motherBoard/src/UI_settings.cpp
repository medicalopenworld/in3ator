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

extern volatile long lastEncPulse;
extern volatile bool statusEncSwitch;

// WIFI
extern bool WIFI_connection_status;

extern bool roomSensorPresent;

// room variables
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

void UI_settings() {
  byte numWords = 6;
  page = SETTIGNS_PAGE;
  print_text = true;
  tft.setTextSize(1);
  setTextColor(COLOR_MENU_TEXT);
  for (int i = false; i < numWords; i++) {
    pos_text[i] = LEFT_MARGIN;
  }
  pos_text[DEFAULT_VALUES_UI_ROW] = CENTER;
  pos_text[HW_TEST_UI_ROW] = CENTER;
  pos_text[CALIBRATION_UI_ROW] = CENTER;
  switch (in3.language) {
    case ENGLISH:
      words[LANGUAGE_UI_ROW] = convertStringToChar("in3.language");
      words[SERIAL_NUMBER_UI_ROW] = convertStringToChar("Serial number");
      words[DEFAULT_VALUES_UI_ROW] =
          convertStringToChar("Set default values");
      words[CALIBRATION_UI_ROW] = convertStringToChar("Calibration");
      break;
    case SPANISH:
      words[LANGUAGE_UI_ROW] = convertStringToChar("Idioma");
      words[SERIAL_NUMBER_UI_ROW] = convertStringToChar("Serial number");
      words[DEFAULT_VALUES_UI_ROW] =
          convertStringToChar("Fijar valores estandar");
      words[CALIBRATION_UI_ROW] = convertStringToChar("Calibracion");
      break;
    case FRENCH:
      words[LANGUAGE_UI_ROW] = convertStringToChar("Langue");
      words[SERIAL_NUMBER_UI_ROW] = convertStringToChar("Serial number");
      words[DEFAULT_VALUES_UI_ROW] =
          convertStringToChar("Definir valeurs default");
      words[CALIBRATION_UI_ROW] = convertStringToChar("Etalonner");
      break;
    case PORTUGUESE:
      words[LANGUAGE_UI_ROW] = convertStringToChar("Idioma");
      words[SERIAL_NUMBER_UI_ROW] = convertStringToChar("Serial number");
      words[DEFAULT_VALUES_UI_ROW] =
          convertStringToChar("Definir valores de fabrica");
      words[CALIBRATION_UI_ROW] = convertStringToChar("Calibrar");
      break;
  }
  words[WIFI_EN_UI_ROW] = convertStringToChar("WIFI");
  words[HW_TEST_UI_ROW] = convertStringToChar("HW Test");
  if (WiFi.status() == WL_CONNECTED) {
    words[WIFI_EN_UI_ROW] = convertStringToChar("WIFI IP->");
  }
  menu_rows = numWords;
  graphics(page, in3.language, print_text, menu_rows, in3.serialNumber,
           WIFI_EN);
  drawHeading(page, in3.serialNumber);
  bar_pos = true;
  selected = false;
  ypos = graphicHeight(bar_pos - 1);
  while (!GPIORead(ENC_SWITCH)) {
  vTaskDelay(pdMS_TO_TICKS(SWITCH_DEBOUNCE_TIME_MS));    
  }
}
