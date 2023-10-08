#ifndef _MAIN_H
#define _MAIN_H

#include <ESPmDNS.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
// include libraries
#include <Beastdevices_INA3221.h>
#include <EEPROM.h>
#include <Filters.h>
#include <RotaryEncoder.h>
#include <Wire.h>

#include <AH/Timing/MillisMicrosTimer.hpp>
#include <Filters/Butterworth.hpp>

#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "Adafruit_SHT4x.h"
#include "Credentials.h"
#include "ESP32_config.h"
#include "GPRS.h"
#include "PID.h"
#include "SPI.h"
#include "SparkFun_SHTC3.h"
#include "TCA9555.h"
#include "Wifi_OTA.h"
#include "board.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "in3ator_humidifier.h"
#include <ESP32_Updater.h>

#define LOG_GPRS false
#define LOG_MODEM_DATA false
#define LOG_INFORMATION false
#define LOG_ERRORS false
#define LOG_ALARMS true

#define WDT_TIMEOUT 45

#define FAN_RPM_CONVERSION 13333333
#define FAN_UPDATE_TIME_MIN 1000

#define ENABLE_WIFI_OTA true  // enable wifi OTA
#define ENABLE_GPRS_OTA true  // enable GPRS OTA
#define ALARM_SYSTEM_ENABLED true
#define FAN_MAX_CURRENT_OVERRIDE false

#define UKRAINE_MODE false

#define CORE_ID_FREERTOS 1

#define HOLD_PRESS_TO_GO_TO_SETTINGS 0

#define BROWN_OUT_BATTERY_MODE 0
#define BROWN_OUT_NORMAL_MODE 0

#define THINGSBOARD_BUFFER_SIZE 4096
#define THINGSBOARD_FIELDS_AMOUNT 64
#define MAX_MESSAGE_SIZE 512
#define THINGSBOARD_QOS false
#define TELEMETRIES_DECIMALS 2
#define FIRMWARE_FAILURE_RETRIES 12
#define FIRMWARE_PACKET_SIZE 4096

#define SN_KEY "SN"
#define HW_NUM_KEY "HW_num"
#define HW_REV_KEY "HW_revision"
#define FW_VERSION_KEY "FW_version"
#define CCID_KEY "CCID"
#define IMEI_KEY "IMEI"
#define APN_KEY "APN"
#define COP_KEY "COP"
#define SYS_CURR_STANDBY_TEST_KEY "SYS_current_stanby_test"
#define HEATER_CURR_TEST_KEY "Heater_current_test"
#define FAN_CURR_TEST_KEY "Fan_current_test"
#define PHOTOTHERAPY_CURR_KEY "Phototherapy_current_test"
#define HUMIDIFIER_CURR_KEY "Humidifier_current_test"
#define DISPLAY_CURR_TEST_KEY "Display_current_test"
#define BUZZER_CURR_TEST_KEY "Buzzer_current_test"
#define HW_TEST_KEY "HW_Test"
#define LOCATION_LONGTITUD_KEY "tri_longitud"
#define LOCATION_LATITUD_KEY "tri_latitud"
#define TRI_ACCURACY_KEY "tri_accuracy"
#define UI_LANGUAGE_KEY "UI_language"
#define SKIN_TEMPERATURE_KEY "Skin_temp"
#define AIR_TEMPERATURE_KEY "Air_temp"
#define AMBIENT_TEMPERATURE_KEY "Amb_temp"
#define HUMIDITY_ROOM_KEY "Humidity"
#define HUMIDITY_AMBIENT_KEY "Amb_humidity"
#define SYSTEM_CURRENT_KEY "SYS_current"
#define SYSTEM_VOLTAGE_KEY "SYS_voltage"
#define CELL_SIGNAL_QUALITY_KEY "CSQ"
#define HEATER_CURRENT_KEY "Heater_current"
#define FAN_CURRENT_KEY "Fan_current"
#define V5_CURRENT_KEY "V5_current"
#define V5_VOLTAGE_KEY "V5_voltage"
#define BAT_CURRENT_KEY "BAT_current"
#define BAT_VOLTAGE_KEY "BAT_voltage"
#define CONTROL_ACTIVE_KEY "Control_active"
#define CONTROL_MODE_KEY "Control_mode"
#define DESIRED_TEMPERATURE_KEY "Temp_desired"
#define DESIRED_HUMIDITY_ROOM_KEY "Hum_desired"
#define HUMIDIFIER_CURRENT_KEY "Humidifier_current"
#define HUMIDIFIER_VOLTAGE_KEY "Humidifier_voltage"
#define PHOTOTHERAPY_CURRENT_KEY "Phototherapy_current"
#define PHOTOTHERAPY_ACTIVE_KEY "Phototherapy_active"
#define CALIBRATED_SENSOR_KEY "Calibrated_sensor"
#define STANBY_TIME_KEY "Standby_time"
#define CONTROL_ACTIVE_TIME_KEY "Control_active_time"
#define HEATER_ACTIVE_TIME_KEY "Heater_active_time"
#define FAN_ACTIVE_TIME_KEY "Fan_active_time"
#define PHOTHERAPY_ACTIVE_TIME_KEY "Phototherapy_active_time"
#define HUMIDIFIER_ACTIVE_TIME_KEY "Humidifier_active_time"
#define GPRS_CONNECTIVITY_KEY "GPRS_connection"
#define WIFI_CONNECTIVITY_KEY "WIFI_connection"

#define ANALOGREAD_ADC 0
#define MILLIVOTSREAD_ADC 1

#define ADC_READ_FUNCTION MILLIVOTSREAD_ADC

#define ON true
#define OFF false
#define BASIC_CONTROL false
#define PID_CONTROL true
#define SKIN_CONTROL false
#define AIR_CONTROL true

// buzzer variables
#define buzzerStandbyPeriod \
  10000  // in millis, there will be a periodic tone when regulating baby's
         // constants
#define buzzerStandbyTone 500         // in micros, tone freq
#define buzzerAlarmTone 500           // in micros, tone freq
#define buzzerRotaryEncoderTone 2200  // in micros, tone freq
#define buzzerStandbyToneDuration 50  // in micros, tone freq
#define buzzerSwitchDuration 10       // in micros, tone freq
#define buzzerStandbyToneTimes 1      // in micros, tone freq

// EEPROM variables
#define EEPROM_SIZE 256
#define EEPROM_CHECK_STATUS 0
#define EEPROM_FIRST_TURN_ON 10
#define EEPROM_AUTO_LOCK 20
#define EEPROM_LANGUAGE 30
#define EEPROM_SERIAL_NUMBER 40
#define EEPROM_WIFI_EN 50
#define EEPROM_CONTROL_MODE 70
#define EEPROM_DESIRED_CONTROL_MODE 80
#define EEPROM_DESIRED_CONTROL_HUMIDITY 90
#define EEPROM_RAW_SKIN_TEMP_LOW_CORRECTION 100
#define EEPROM_RAW_SKIN_TEMP_RANGE_CORRECTION 110
#define EEPROM_REFERENCE_TEMP_RANGE 170
#define EEPROM_REFERENCE_TEMP_LOW 180
#define EEPROM_FINE_TUNE_TEMP_SKIN 190
#define EEPROM_FINE_TUNE_TEMP_AIR 194
#define EEPROM_THINGSBOARD_PROVISIONED 200
#define EEPROM_THINGSBOARD_TOKEN 205
#define EEPROM_STANDBY_TIME 226
#define EEPROM_CONTROL_ACTIVE_TIME 230
#define EEPROM_HEATER_ACTIVE_TIME 234
#define EEPROM_FAN_ACTIVE_TIME 238
#define EEPROM_PHOTOTHERAPY_ACTIVE_TIME 242
#define EEPROM_HUMIDIFIER_ACTIVE_TIME 246

// configuration variables
#define debounceTime 30  // encoder debouncing time
#define timePressToSettings \
  3000  // in millis, time to press to go to settings window in UI
#define debugUpdatePeriod 1000  // in millis,

// pages number in UI. Configuration and information will be displayed depending
// on the page number

#define mainMenuPage 1
#define actuatorsProgressPage 2
#define settingsPage 3
#define calibrateSensorsPage 4
#define firstPointCalibrationPage 5
#define secondPointCalibrationPage 6
#define autoCalibrationPage 7
#define fineTuneCalibrationPage 8
// languages numbers that will be called in language variable
#define spanish 0
#define english 1
#define french 2
#define portuguese 3
#define numLanguages 4
#define defaultLanguage \
  english  // Preset number configuration when booting for first time
#define DEFAULT_CONTROL_MODE AIR_CONTROL

#define NTC_MEASUREMENT_PERIOD 1  // in millis
#define NTC_SAMPLES_TEST 100
#define CURRENT_UPDATE_PERIOD 500  // in millis
#define VOLTAGE_UPDATE_PERIOD 50   // in millis

#define setupAutoCalibrationPoint 0
#define firstAutoCalibrationPoint 1
#define secondAutoCalibrationPoint 2

// GPRS variables to transmit
#define turnedOn 0      // transmit first turned ON with hardware verification
#define room 1          // transmit room variables
#define aliveRefresh 2  // message to let know that incubator is still ON

#define digitalCurrentSensorReadPeriod 500
#define externalADCReadPeriod 30

// sensor variables
#define defaultCurrentSamples 30
#define defaultTestingSamples 8000
#define Rsense 3000  // 3 microohm as shunt resistor

#define MAIN 0
#define SECUNDARY 1
// I2C addresses
#define MAIN_DIGITAL_CURRENT_SENSOR_I2C_ADDRESS 0x41
#define SECUNDARY_DIGITAL_CURRENT_SENSOR_I2C_ADDRESS 0x40
#define AMBIENT_SENSOR_I2C_ADDRESS 0x44
#define ROOM_SENSOR_I2C_ADDRESS 0x70

// #define system constants
#define humidifierDutyCycleMax \
  100                             // maximum humidity cycle in heater to be set
#define humidifierDutyCycleMin 0  // minimum humidity cycle in heater to be set

#define stepTemperatureIncrement 0.1  // maximum allowed temperature to be set
#define stepHumidityIncrement 5       // maximum allowed temperature to be set
#define presetHumidity 60             // preset humidity
#define maxHum 90                     // maximum allowed humidity to be set
#define minHum 20                     // minimum allowed humidity to be set

// Encoder variables
#define NUMENCODERS 1  // number of encoders in circuit
#if (HW_NUM == 6)
#define ENCODER_TICKS_DIV 1
#else
#define ENCODER_TICKS_DIV 0
#endif
#define encPulseDebounce 50

// User Interface display constants
#define brightenRate 10  // intro brighten speed (Higher value, faster)
#define valuePosition 245
#define separatorPosition 240
#define unitPosition 315
#define textFontSize 2  // text default size
#define width_select 20
#define height_heading 34
#define width_indentation 4
#define width_back 50
#define side_gap 4
#define letter_height 26
#define letter_width 14
#define logo 40
#define arrow_height 6
#define arrow_tail 5
#define headint_text_height height_heading / 5
#define initialSensorsValue "XX"
#define barThickness 3
#define blinkTimeON 1000  // displayed text ON time
#define blinkTimeOFF 100  // displayed text OFF time
#define time_back_draw 255
#define time_back_wait 255

#define DEFAULT_AUTOLOCK ON

typedef enum {
  NTC_BABY_MIN_ERROR = 0,
  NTC_BABY_MAX_ERROR,
  DIG_TEMP_ROOM_MIN_ERROR,
  DIG_TEMP_ROOM_MAX_ERROR,
  DIG_HUM_ROOM_MIN_ERROR,
  DIG_HUM_ROOM_MAX_ERROR,
  DIGITAL_SENSOR_NOTFOUND,
  HEATER_CONSUMPTION_MIN_ERROR,
  FAN_CONSUMPTION_MIN_ERROR,
  PHOTOTHERAPY_CONSUMPTION_MIN_ERROR,
  HUMIDIFIER_CONSUMPTION_MIN_ERROR,
  HEATER_CONSUMPTION_MAX_ERROR,
  FAN_CONSUMPTION_MAX_ERROR,
  PHOTOTHERAPY_CONSUMPTION_MAX_ERROR,
  HUMIDIFIER_CONSUMPTION_MAX_ERROR,
  STANDBY_CONSUMPTION_MAX_ERROR,
  DEFECTIVE_SCREEN,
  DEFECTIVE_BUZZER,
  DEFECTIVE_CURRENT_SENSOR,
  UNCALIBRATED_SENSOR,
} HW_ERROR_ID;

typedef enum {
  NO_ALARMS = 0,
  HUMIDITY_ALARM,
  TEMPERATURE_ALARM,
  AIR_THERMAL_CUTOUT_ALARM,
  SKIN_THERMAL_CUTOUT_ALARM,
  AIR_SENSOR_ISSUE_ALARM,
  SKIN_SENSOR_ISSUE_ALARM,
  FAN_ISSUE_ALARM,
  HEATER_ISSUE_ALARM,
  POWER_SUPPLY_ALARM,
  NUM_ALARMS,
} ALARMS_ID;

typedef enum {
  EVENT_2G = 0,
  EVENT_WIFI,
  EVENT_SERVER_CONNECTION,
  EVENT_OTA_ONGOING,
} UI_EVENTS_ID;

typedef enum {
  EVENT_2G_UI_POS = 5,
  EVENT_SERVER_CONNECTION_UI_POS = EVENT_2G_UI_POS + 2 * letter_width,
  EVENT_WIFI_UI_POS = EVENT_SERVER_CONNECTION_UI_POS + letter_width,
  EVENT_OTA_ONGOING_UI_POS = EVENT_WIFI_UI_POS + letter_width,
} UI_EVENTS_ID_POS;

// Graphic variables
#define ERASE false
#define DRAW true

// graphic text configurations
#define graphicTextOffset 1  // bar pos is counted from 1, but text from 0
#define CENTER true
#define LEFT_MARGIN false

// below are all the different variables positions that will be displayed in
// user interface mainMenu
#define controlModeGraphicPosition 0
#define temperatureGraphicPosition 1
#define humidityGraphicPosition 2
#define LEDGraphicPosition 3
#define settingsGraphicPosition 5
#define startGraphicPosition 4
// settings

#define serialNumberGraphicPosition 0
#define languageGraphicPosition 1
#define WifiENGraphicPosition 2
#define calibrateGraphicPosition 3
#define setdefaultValuesGraphicPosition 4
#define HWTestGraphicPosition 5

// calibration menu
#define autoCalibrationGraphicPosition 0
#define fineTuneCalibrationGraphicPosition 1
#define twoPointCalibrationGraphicPosition 2
#define restartCalibrationGraphicPosition 3

// 2p calibration
#define temperatureCalibrationGraphicPosition 0
#define setCalibrationGraphicPosition 1

// auto calibration
#define autoCalibrationMessageGraphicPosition 0

// color options
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define COLOR_WARNING_TEXT ILI9341_ORANGE
#define COLOR_MENU BLACK
#define COLOR_BAR BLACK
#define COLOR_MENU_TEXT WHITE
#define COLOR_SELECTED WHITE
#define COLOR_CHOSEN BLUE
#define COLOR_HEADING BLUE
#define COLOR_ARROW BLACK
#define COLOR_BATTERY BLACK
#define COLOR_BATTERY_LEFT BLACK
#define COLOR_FRAME_BAR WHITE
#define COLOR_LOADING_BAR RED
#define COLOR_COMPLETED_BAR GREEN
#define introBackColor WHITE
#define introTextColor BLACK
#define transitionEffect BLACK

#define BACKLIGHT_NO_INTERACTION_TIME \
  12000  // time to decrease backlight display if no user actions

#define GPRS_TASK_PERIOD 1
#define OTA_TASK_PERIOD 0
#define SENSORS_TASK_PERIOD 1
#define ROOM_SENSOR_UPDATE_PERIOD 500
#define DIGITAL_CURRENT_SENSOR_PERIOD 5
#define BUZZER_TASK_PERIOD 1
#define BACKLIGHT_TASK_PERIOD 100
#define FAN_TASK_PERIOD 1
#define LOOP_TASK_PERIOD 1

#define BACKLIGHT_DELAY 2
#define INIT_TFT_DELAY 300

#define TIME_TRACK_UPDATE_PERIOD 900000  // 15 minutes

typedef struct {
  double temperature[SENSOR_TEMP_QTY];
  double humidity[SENSOR_HUM_QTY];
  double desiredControlTemperature = false;
  double desiredControlHumidity = false;

  double system_current_standby_test = false;
  double heater_current_test = false;
  double fan_current_test = false;
  double phototherapy_current_test = false;
  double humidifier_current_test = false;
  double display_current_test = false;
  double buzzer_current_test = false;
  bool HW_critical_error = false;
  double HW_test_error_code = false;

  double system_current = false;
  double system_voltage = false;
  double heater_current = false;
  double fan_current = false;
  double humidifier_current = false;
  double humidifier_voltage = false;
  double phototherapy_current = false;
  double USB_current = false;
  double USB_voltage = false;
  double BATTERY_current = false;
  double BATTERY_voltage = false;
  int serialNumber = false;

  bool controlMode = DEFAULT_CONTROL_MODE;
  bool temperatureControl = false;
  bool humidityControl = false;
  bool phototherapy = false;

  bool calibrationError = false;

  long last_check_time = false;
  float standby_time = false;
  float control_active_time = false;
  float heater_active_time = false;
  float fan_active_time = false;
  float phototherapy_active_time = false;
  float humidifier_active_time = false;

  bool alarmsEnabled = true;

  float fan_rpm = false;
  bool fanEncoderUpdate = false;
  long fanEncoderPeriod[2] = {false, false};

  byte language;

} in3ator_parameters;

void logE(String dataString);
void logAlarm(String dataString);
void logI(String dataString);
void logCon(String dataString);
void logModemData(String dataString);
long secsToMillis(long timeInMillis);
long minsToMillis(long timeInMillis);
float millisToHours(long timeInMillis);
void initHardware(bool printOutputTest);
void UI_mainMenu();
void userInterfaceHandler(int UI_page);
void updateData();
void buzzerHandler();
void buzzerTone(int beepTimes, int timevTaskDelay, int freq);

void shutBuzzer();
double measureMeanConsumption(bool, int);
float measureMeanVoltage(bool, int);
void WIFI_TB_Init();
void WifiOTAHandler(void);
void securityCheck();
void buzzerConstantTone(int freq);
void drawAlarmMessage(char *alertMessage);
void drawHeading(int UI_page, int UI_serialNumber);
void updateHeadingEvent(byte Event, bool event_status);
char *convertStringToChar(String input);
char *convertStringToChar(char *arrayInput, String input);
int16_t drawCentreString(char *string, int16_t dX, int16_t poY, int16_t size);
void eraseBar(int UI_menu_rows, int bar_pos);
void UI_updateConnectivityEvents();
void updateBar(int UI_menu_rows, int bar_pos);
void graphics(uint8_t UI_page, uint8_t UI_language, uint8_t UI_print_text,
              uint8_t UI_menu_rows, uint8_t UI_var_0, uint8_t UI_var_1);
int graphicHeight(int position);
int16_t drawFloat(float floatNumber, int16_t decimal, int16_t poX, int16_t poY,
                  int16_t size);
void setTextColor(int16_t color);
int16_t getBackgroundColor();

void turnFans(bool mode);
void alarmTimerStart();

bool ongoingCriticalAlarm();
void setAlarm(byte alarmID);
void resetAlarm(byte alarmID);

void PIDInit();
void PIDHandler();
void startPID(byte var);
void stopPID(byte var);

bool encoderContinuousPress(int UI_page);

void updateLoadingTemperatureBar(float prev, float actual);
void updateLoadingHumidityBar(float prev, float actual);
void drawSelectedTemperature(float temperatureToDraw,
                             float previousTemperatureDrawn);
void drawUnselectedTemperature(float temperatureToDraw,
                               float previousTemperatureDrawn);
void drawHumidity(int UI_humidity, int UI_previousHumdity);
int16_t drawRightString(char *string, int16_t dX, int16_t poY, int16_t size);
void drawStartMessage(bool UI_enableSet, int UI_menu_rows,
                      char *UI_helpMessage);
void drawCentreNumber(int n, int x, int i);
void drawRightNumber(int n, int x, int i);
void drawBack();
void drawActuatorsSeparators();
void drawStop();
void drawHelpMessage(byte UI_language);
void printLoadingTemperatureBar(double UI_desiredControlTemperature);
void printLoadingHumidityBar(int UI_desiredControlHumidity);
void blinkGoBackMessage();
bool ongoingAlarms();
byte activeAlarm();
void disableAllAlarms();
int alarmPendingToDisplay();
int alarmPendingToClear();
void clearDisplayedAlarm(byte alarm);
void clearAlarmPendingToClear(byte alarm);
char *alarmIDtoString(byte alarmID);

void checkSetMessage(int UI_page);

bool updateRoomSensor();
bool updateAmbientSensor();
void updateDisplaySensors();

void UI_settings();
void UI_actuatorsProgress();

void wifiInit(void);
void wifiDisable();

void loaddefaultValues();
void UI_calibration();
void firstPointCalibration();
void fineTuneCalibration();
void autoCalibration();
void recapVariables();
void clearCalibrationValues();
void secondPointCalibration();
void saveCalibrationToEEPROM();
int getYpos(int UI_menu_rows, byte row);
bool back_mode();
void setSensorsGraphicPosition(int UI_page);

void basicHumidityControl();
bool checkStableTemperatures(double *referenceSensorHistory,
                             double *sensorToCalibrateHistory,
                             int historyLength, double stabilityError);
void initRoomSensor();
void initAmbientSensor();
void powerMonitor();
void currentMonitor();
void voltageMonitor();

double roundSignificantDigits(double value, int numberOfDecimals);

void initGPIO();
void initEEPROM();
void drawHardwareErrorMessage(long error, bool criticalError,
                              bool calibrationError);
void initAlarms();
void IRAM_ATTR encSwitchHandler();
void IRAM_ATTR encoderISR();
void IRAM_ATTR fanEncoderISR();
void IRAM_ATTR ON_OFF_Switch_ISR();
void backlightHandler();

void fanSpeedHandler();
bool measureNTCTemperature();
void loadlogo();

void initPin(uint8_t GPIO, uint8_t Mode);
bool GPIORead(uint8_t GPIO);
void GPIOWrite(uint8_t GPIO, uint8_t Mode);

#endif