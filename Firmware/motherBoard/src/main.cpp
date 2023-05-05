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

// Firmware version and head title of UI screen

#include "main.h"

#include <Arduino.h>

TwoWire *wire;
MAM_in3ator_Humidifier in3_hum(DEFAULT_ADDRESS);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
SHTC3 mySHTC3;  // Declare an instance of the SHTC3 class
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
RotaryEncoder encoder(ENC_A, ENC_B, RotaryEncoder::LatchMode::TWO03);
Beastdevices_INA3221 digitalCurrentSensor(INA3221_ADDR41_VCC);

bool WIFI_EN = true;
long lastDebugUpdate;
long loopCounts;
int page;
int temperature_filter = analog_temperature_filter;  // amount of temperature samples to filter
long lastNTCmeasurement[NTC_QTY];


int NTC_PIN[NTC_QTY] = {BABY_NTC_PIN};
double errorTemperature[SENSOR_TEMP_QTY], temperatureCalibrationPoint;
double ReferenceTemperatureRange, ReferenceTemperatureLow;
double provisionalReferenceTemperatureLow;
double fineTuneSkinTemperature, fineTuneAirTemperature;
float diffSkinTemperature, diffAirTemperature;  // difference between measured temperature and user input real temperature
double RawTemperatureLow[SENSOR_TEMP_QTY], RawTemperatureRange[SENSOR_TEMP_QTY];
double provisionalRawTemperatureLow[SENSOR_TEMP_QTY];
double temperatureMax[SENSOR_TEMP_QTY], temperatureMin[SENSOR_TEMP_QTY];
int temperatureArray[NTC_QTY][analog_temperature_filter];  // variable to handle each NTC with the array of last samples (only for NTC)
int temperature_array_pos;                                // temperature sensor number turn to measure
bool humidifierState, humidifierStateChange;
int previousHumidity;  // previous sampled humidity
float diffHumidity;    // difference between measured humidity and user input real humidity

byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and they have different check rates
byte encoderRate = true;
byte encoderCount = false;
bool encPulseDetected;
volatile long lastEncPulse;
volatile bool statusEncSwitch;

bool roomSensorPresent = false;
bool ambientSensorPresent = false;
bool digitalCurrentSensorPresent = false;

float instantTemperature[NTC_QTY][secondOrder_filter];
float previousTemperature[NTC_QTY][secondOrder_filter];

float instantCurrent[secondOrder_filter];
float previousCurrent[secondOrder_filter];

// room variables
float minDesiredTemp[2] = {35, 30};    // minimum allowed temperature to be set
float maxDesiredTemp[2] = {37.5, 37};  // maximum allowed temperature to be set
int presetTemp[2] = {36, 32};          // preset baby skin temperature

boolean A_set;
boolean B_set;
int encoderpinA = ENC_A;          // pin  encoder A
int encoderpinB = ENC_B;          // pin  encoder B
bool encPulsed, encPulsedBefore;  // encoder switch status
bool updateUIData;
volatile int EncMove;                  // moved encoder
volatile int lastEncMove;              // moved last encoder
volatile int EncMoveOrientation = -1;  // set to -1 to increase values clockwise
volatile int last_encoder_move;        // moved encoder
long encoder_debounce_time = true;     // in milliseconds, debounce time in encoder to filter signal bounces
long last_encPulsed;                   // last time encoder was pulsed

// Text Graphic position variables
int humidityX;
int humidityY;
int temperatureX;
int temperatureY;
int separatorTopYPos, separatorMidYPos, separatorBotYPos;
int ypos;
bool print_text;
int initialSensorPosition = separatorPosition - letter_width;
bool pos_text[8];

bool enableSet;
float temperaturePercentage, temperatureAtStart;
float humidityPercentage, humidityAtStart;
int barWidth, barHeight, tempBarPosX, tempBarPosY, humBarPosX, humBarPosY;
int screenTextColor, screenTextBackgroundColor;

// User Interface display variables
bool autoLock;              // setting that enables backlight switch OFF after a given time of no user actions
long lastbacklightHandler;  // last time there was a encoder movement or pulse
long sensorsUpdatePeriod = 1000;

bool selected;
char cstring[128];
char *textToWrite;
char *words[12];
char *helpMessage;
byte bar_pos = true;
byte menu_rows;
byte length;
long lastGraphicSensorsUpdate;
long lastSensorsUpdate;
bool enableSetProcess;
long blinking;
bool state_blink;
bool blinkSetMessageState;
long lastBlinkSetMessage;

long lastSuccesfullSensorUpdate[SENSOR_TEMP_QTY];

int ScreenBacklightMode;
long lastRoomSensorUpdate, lastCurrentSensorUpdate;

in3ator_parameters in3;

void GPRS_Task(void *pvParameters) {
  initGPRS();
  GPRS_TB_Init();
  for (;;) {
    if (!WIFIIsConnected()) {
      GPRS_Handler();
    }
    vTaskDelay(GPRS_TASK_PERIOD / portTICK_PERIOD_MS);
  }
}

void GPRS_Task_check(void *pvParameters) {
  for (;;) {
    log("[GPRS] -> Connection is " + String(GPRS_CheckConnection()));
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void Backlight_Task(void *pvParameters) {
  for (;;) {
    backlightHandler();
    vTaskDelay(BACKLIGHT_TASK_PERIOD / portTICK_PERIOD_MS);
  }
}

void sensors_Task(void *pvParameters) {
  for (;;) {
    measureNTCTemperature(SKIN_SENSOR);
    if (millis() - lastRoomSensorUpdate > ROOM_SENSOR_UPDATE_PERIOD) {
      updateRoomSensor();
      updateAmbientSensor();
      lastRoomSensorUpdate = millis();
    }
    if (millis() - lastCurrentSensorUpdate > DIGITAL_CURRENT_SENSOR_PERIOD) {
      powerMonitor();
      lastCurrentSensorUpdate = millis();
    }
    if (ALARM_SYSTEM_ENABLED && in3.alarmsEnabled) {
      securityCheck();
    }
    vTaskDelay(SENSORS_TASK_PERIOD / portTICK_PERIOD_MS);
  }
}

void OTA_Task(void *pvParameters) {
  WIFI_TB_Init();
  for (;;) {
    WifiOTAHandler();
    vTaskDelay(OTA_TASK_PERIOD / portTICK_PERIOD_MS);
  }
}

void buzzer_Task(void *pvParameters) {
  for (;;) {
    buzzerHandler();
    vTaskDelay(BUZZER_TASK_PERIOD / portTICK_PERIOD_MS);
  }
}

void setup() {
  initHardware(false);
  UI_mainMenu();
  // Task generation
  /* Task n#1 - GPRS Handler */
  log("Creating GPRS task ...\n");
  while (xTaskCreatePinnedToCore(GPRS_Task, (const char *)"GPRS", 8192, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  //while (xTaskCreatePinnedToCore(GPRS_Task_check, (const char *)"GPRS_check", 8192, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS);
  log("GPRS task successfully created!\n");

  log("Creating OTA task ...\n");
  while (xTaskCreatePinnedToCore(OTA_Task, (const char *)"OTA", 8192, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  log("OTA task successfully created!\n");
  log("Creating Backlight task ...\n");
  while (xTaskCreatePinnedToCore(Backlight_Task, (const char *)"BACKLIGHT", 4096, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  log("Backlight task successfully created!\n");

  log("Creating buzzer task ...\n");
  while (xTaskCreatePinnedToCore(buzzer_Task, (const char *)"BUZZER", 4096, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  log("Buzzer task successfully created!\n");
  log("Creating sensors task ...\n");
  while (xTaskCreatePinnedToCore(sensors_Task, (const char *)"SENSORS", 4096, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  log("sensors task successfully created!\n");
  /*
  log("Creating time track task ...\n");
  while (xTaskCreatePinnedToCore(time_track_Task, (const char *)"SENSORS", 4096, NULL, 1, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  log("Time track task successfully created!\n");
  */
}

void loop() {
  userInterfaceHandler(page);
  updateData();
  vTaskDelay(LOOP_TASK_PERIOD / portTICK_PERIOD_MS);
}
