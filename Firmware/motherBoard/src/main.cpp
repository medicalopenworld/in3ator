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
// Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
SHTC3 mySHTC3;             // Declare an instance of the SHTC3 class
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
RotaryEncoder encoder(ENC_A, ENC_B, RotaryEncoder::LatchMode::TWO03);
Beastdevices_INA3221 mainDigitalCurrentSensor(INA3221_ADDR41_VCC);
Beastdevices_INA3221 secundaryDigitalCurrentSensor(INA3221_ADDR40_GND);

bool WIFI_EN = true;
long lastDebugUpdate;
long loopCounts;
int page;

double errorTemperature[SENSOR_TEMP_QTY], temperatureCalibrationPoint;
double ReferenceTemperatureRange, ReferenceTemperatureLow;
double provisionalReferenceTemperatureLow;
double fineTuneSkinTemperature, fineTuneAirTemperature;
float diffSkinTemperature,
    diffAirTemperature; // difference between measured temperature and user
                        // input real temperature
double RawTemperatureLow[SENSOR_TEMP_QTY], RawTemperatureRange[SENSOR_TEMP_QTY];
double provisionalRawTemperatureLow[SENSOR_TEMP_QTY];
double temperatureMax[SENSOR_TEMP_QTY], temperatureMin[SENSOR_TEMP_QTY];
int temperature_array_pos; // temperature sensor number turn to measure
bool humidifierState, humidifierStateChange;
int previousHumidity; // previous sampled humidity
float diffHumidity;   // difference between measured humidity and user input real
                      // humidity

byte autoCalibrationProcess;

// Sensor check rate (in ms). Both sensors are checked in same interrupt and
// they have different check rates
byte encoderRate = true;
byte encoderCount = false;

volatile long lastEncPulse;
volatile bool statusEncSwitch;

bool roomSensorPresent = false;
bool ambientSensorPresent = false;
bool digitalCurrentSensorPresent[2];

// room variables
float minDesiredTemp[2] = {35, 20};   // minimum allowed temperature to be set
float maxDesiredTemp[2] = {37.5, 37}; // maximum allowed temperature to be set
int presetTemp[2] = {36, 32};         // preset baby skin temperature

boolean A_set;
boolean B_set;
int encoderpinA = ENC_A;         // pin  encoder A
int encoderpinB = ENC_B;         // pin  encoder B
bool encPulsed, encPulsedBefore; // encoder switch status
bool updateUIData;
volatile int EncMove;                 // moved encoder
volatile int lastEncMove;             // moved last encoder
volatile int EncMoveOrientation = -1; // set to -1 to increase values clockwise
volatile int last_encoder_move;       // moved encoder
long encoder_debounce_time =
    true;            // in milliseconds, debounce time in encoder to filter signal bounces
long last_encPulsed; // last time encoder was pulsed

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
int screenTextColor, screenTextBackgroundColour;

// User Interface display variables
bool goToSettings = false;
bool autoLock;             // setting that enables backlight switch OFF after a given time
                           // of no user actions
long lastbacklightHandler; // last time there was a encoder movement or pulse

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

QueueHandle_t sharedSensorQueue;

void GPRS_Task(void *pvParameters)
{
  long lastPrint;
  initGPRS();
  GPRS_TB_Init();
  for (;;)
  {
    if (!WIFIIsConnected())
    {
      GPRS_Handler();
    }
    vTaskDelay(pdMS_TO_TICKS(GPRS_TASK_PERIOD_MS));
  }
}

void Backlight_Task(void *pvParameters)
{
  for (;;)
  {
    backlightHandler();
    vTaskDelay(pdMS_TO_TICKS(BACKLIGHT_TASK_PERIOD_MS));
  }
}

void sensors_Task(void *pvParameters)
{
  for (;;)
  {
    fanSpeedHandler();
    measureNTCTemperature();
    if (millis() - lastRoomSensorUpdate > ROOM_SENSOR_UPDATE_PERIOD_MS)
    {
      updateRoomSensor();
      updateAmbientSensor();
      lastRoomSensorUpdate = millis();
    }
    if (millis() - lastCurrentSensorUpdate > DIGITAL_CURRENT_SENSOR_PERIOD_MS)
    {
      powerMonitor();
      lastCurrentSensorUpdate = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(SENSORS_TASK_PERIOD_MS));
  }
}

void OTA_WIFI_Task(void *pvParameters)
{
  WIFI_TB_Init();
  for (;;)
  {
    WifiOTAHandler();
    vTaskDelay(pdMS_TO_TICKS(OTA_TASK_PERIOD_MS));
  }
}

void buzzer_Task(void *pvParameters)
{
  for (;;)
  {
    buzzerHandler();
    vTaskDelay(pdMS_TO_TICKS(BUZZER_TASK_PERIOD_MS));
  }
}

void security_Task(void *pvParameters)
{
  for (;;)
  {
    if (ALARM_SYSTEM_ENABLED && in3.alarmsEnabled)
    {
      securityCheck();
    }
    vTaskDelay(pdMS_TO_TICKS(SECURITY_TASK_PERIOD_MS));
  }
}

void UI_Task(void *pvParameters)
{
  if (goToSettings)
  {
    UI_settings();
  }
  else
  {
    UI_mainMenu();
  }
  for (;;)
  {
    userInterfaceHandler(page);
    vTaskDelay(pdMS_TO_TICKS(UI_TASK_PERIOD_MS));
  }
}

void TimeTrack_Task(void *pvParameters)
{
  for (;;)
  {
    timeTrackHandler();
    vTaskDelay(pdMS_TO_TICKS(TIME_TRACK_TASK_PERIOD_MS));
  }
}

void setup()
{

  sharedSensorQueue = xQueueCreate(SENSOR_TEMP_QTY, sizeof(long));
  initGPIO();

  if (!GPIORead(ENC_SWITCH))
  {
    goToSettings = true;
  }

  initHardware(false);

  logI("Creating buzzer task ...\n");
  while (xTaskCreatePinnedToCore(buzzer_Task, (const char *)"BUZZER", 4096,
                                 NULL, BUZZER_TASK_PRIORITY, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("Buzzer task successfully created!\n");
  logI("Creating sensors task ...\n");
  while (xTaskCreatePinnedToCore(sensors_Task, (const char *)"SENSORS", 4096,
                                 NULL, SENSORS_TASK_PRIORITY, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("sensors task successfully created!\n");
  // Task generation
  logI("Creating security task ...\n");
  while (xTaskCreatePinnedToCore(security_Task, (const char *)"SECURITY", 4096,
                                 NULL, SECURITY_TASK_PRIORITY, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("sensors task successfully created!\n");
  logI("Creating GPRS task ...\n");
  while (xTaskCreatePinnedToCore(GPRS_Task, (const char *)"GPRS", 8192, NULL, GPRS_TAST_PRIORITY,
                                 NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  logI("GPRS task successfully created!\n");

  logI("Creating OTA task ...\n");
  while (xTaskCreatePinnedToCore(OTA_WIFI_Task, (const char *)"OTA", 8192, NULL, OTA_TASK_PRIORITY,
                                 NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  logI("OTA task successfully created!\n");

  logI("Creating Backlight task ...\n");
  while (xTaskCreatePinnedToCore(Backlight_Task, (const char *)"BACKLIGHT",
                                 4096, NULL, BACKLIGHT_TASK_PRIORITY, NULL,
                                 CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("Backlight task successfully created!\n");
  logI("Creating time track task ...\n");
  while (xTaskCreatePinnedToCore(TimeTrack_Task, (const char *)"TimeTrack",
                                 4096, NULL, TIME_TRACK_TASK_PRIORITY, NULL,
                                 CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("Time track task successfully created!\n");

  logI("Creating UI task ...\n");
  while (xTaskCreatePinnedToCore(UI_Task, (const char *)"UI", 4096,
                                 NULL, UI_TASK_PRIORITY, NULL, CORE_ID_FREERTOS) != pdPASS)
    ;
  ;
  logI("UI task successfully created!\n");
  EEPROM.writeString(EEPROM_THINGSBOARD_TOKEN, "ckGEZct8wPuHEhfiFC23");
  EEPROM.write(EEPROM_THINGSBOARD_PROVISIONED, true);
  EEPROM.commit();
  digitalWrite(ACTUATORS_EN, HIGH);
}
long lastPWMIncrease = false;
byte PWMSpeed = false;
void loop()
{
  watchdogReload();
  updateData();
  vTaskDelay(pdMS_TO_TICKS(LOOP_TASK_PERIOD_MS));
}